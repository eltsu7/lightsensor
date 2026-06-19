# Reference

Serial protocol, driver API, and calibration model. For the high-level overview,
architecture, and contributor guidance see [`../CLAUDE.md`](../CLAUDE.md); the
Python API is also documented in docstrings in `lightmeter/sensor.py`.

## Serial protocol

Single-char commands at 115200 baud (`raw` = signed 16-bit 0–32767; flags 0/1):

| Command | Description | Response |
|---------|-------------|----------|
| `r` / `r<n>\n` | Read once / averaging `n` samples (clamped 1–1000) | `raw,sensor_sat,adc_sat\n` |
| `g<n>` | Set gain index 0–5 | `ok` / `err <code>` |
| `G` | Query gain index | integer |
| `p` | Ping (no I2C) | `pong` |
| `I` | Identity / version handshake | `lightsensor proto=1 fw=… id=<MAC> sps=860 vsat=3.20 gains=6.144,…` |
| `W<n>\n`+bytes | Write calibration blob | `ok <crc32>` / `err <code>` |
| `C` | Read calibration | `<size> <crc32>\n` + bytes (`0 0` if none) |
| `H` / `X` | Cal size / erase | size / `ok` / `err <code>` |

**Gain index → range / saturation:** 0 ±6.144 V, 1 ±4.096 V (default), 2 ±2.048 V,
3 ±1.024 V, 4 ±0.512 V, 5 ±0.256 V. Indices 0–1 saturate at the sensor (3.266 V);
2–5 saturate the ADC (32767).

**Error codes** (`err <code>`, mirrored in `sensor.py` `ERR_MESSAGES`): 1 bad arg,
2 bad length, 3 out of memory, 4 transfer timeout/short read, 5 fs open failed,
6 write size mismatch, 7 erase failed.

### Protocol contracts (non-obvious)

- **Firmware is the source of truth** for the gain table and saturation voltage;
  it reports them in `I`. The driver verifies its mirrored `GAIN_VOLTAGES` /
  `SATURATION_VOLTAGE` on connect and warns on drift. Bump `PROTO_VERSION` (both
  sides) on any breaking command/response change.
- **Calibration transfer is CRC32-verified** (matches `binascii.crc32`). The host
  **must throttle** the upload in small flushed chunks — a fast burst overruns the
  device USB-CDC RX buffer while it writes flash, silently dropping bytes; the
  firmware buffers the whole blob in RAM before writing for the same reason.
  `write_calibration` returns `True` only on a CRC match; `read_calibration`
  returns `None` on mismatch. Trust those rather than re-reading.
- **Resync after desync:** every device-side read self-times-out (≤5 s for `W`), so
  the device never blocks forever. On connect the driver drains input → pings to
  `pong` → reads identity. If desynced (e.g. interrupted `W`), it goes **silent**
  past the device timeout to let the stuck command self-abort — it must *not* keep
  pinging, since each byte feeds the pending read and resets its timeout. A `W`
  abort discards only the partial upload; stored calibration is preserved.

## Driver API (`lightmeter/sensor.py`)

### Constants
`GAIN_LABELS`, `GAIN_VOLTAGES` (`[6.144, …, 0.256]`), `DEFAULT_GAIN` (1, ±4.096 V),
`SATURATION_VOLTAGE` (3.2 V — OPA323 ceiling).

`best_gain(max_voltage, headroom=0.85)` — pure function; highest gain index that
keeps `max_voltage` below the saturation threshold with the given headroom.

### `Reading` dataclass
`value` (% of ADC full-scale, 0–100), `sensor_sat`, `adc_sat`. The two saturation
flags are mutually exclusive on this hardware (see gain mapping above).

### `LightSensor`
| Member | Description |
|--------|-------------|
| `read()` | Returns `Reading` or `None` |
| `gain` / `set_gain(i)` / `get_gain()` | Applied gain index (tracked locally) / set / query device |
| `autogain` / `autogain_interval` / `autogain_window` | Continuous autogain inside `read()` and its timing |
| `autogain_oneshot(n=100)` | Collect n samples, apply best gain, return index |
| `zero(n=50)` / `clear_zero()` / `is_zeroed` / `zero_offset` | Dark-offset (volts) measure / clear / state |
| `average` | ADC samples the firmware averages per `read()` (default 1; ~√n noise, ×n time) |
| `info` / `ping()` / `identify()` / `connected` | Identity from handshake / link check / query / port state |
| `reconnect(attempts=5, backoff=0.5)` | Re-detect port + reopen + handshake; returns bool, never raises |
| `auto_reconnect` | Opt-in (default `False`): `read()` reconnects on link error instead of raising |
| `reading_voltage(reading)` | Dark-corrected voltage for a `Reading` at the current gain |
| `read_physical(source=None)` | Read once and convert to a physical value via cached calibration |
| `load_calibration()` / `read_calibration()` / `write_calibration(text)` / `write_calibration_file(path)` | Cal fetch+cache / read / write (CRC-verified) |
| `has_calibration()` / `clear_calibration()` | Stored cal size / erase |

Behavioral contracts (thread-safety, reconnect, dark-offset ordering) are in
CLAUDE.md's invariants; the dark offset is stored as volts so it stays correct
across gain changes, and `read()` subtracts it last (display only) — `sensor_sat`
/ `adc_sat` still reflect the true raw level.

## Calibration & physical units (`Calibration`)

The device stores a spectral responsivity curve `R(λ)` + metadata (`scale_factor`,
`scale_units`, `device_id`, `cal_date`, …) as a CSV in LittleFS; it's dumb storage,
all unit math is host-side.

| Member | Description |
|--------|-------------|
| `scale_factor` / `scale_units` | Absolute scale (physical_unit per volt) and its unit string |
| `responsivity_at(wl)` | `R(λ)` by linear interpolation; `0` outside the measured band |
| `source_weighted_responsivity(wl, intensity)` | `R̄ = ∫sR dλ / ∫s dλ` for a source (trapezoidal) |
| `voltage_to_value(voltage, source=None)` | Convert a dark-corrected voltage to a physical value |

**Conversion model:** `physical = scale_factor · V / R̄_source`. The sensor
integrates incident light over its spectral response, so the same voltage means
different physical levels for different spectra — passing `source=(wavelengths,
intensities)` applies that correction; `source=None` ⇒ `R̄=1.0` (measuring the same
spectrum the absolute scale was calibrated against). Returns `None` when
uncalibrated or the source doesn't overlap the band.

**Not yet absolutely calibrated:** `scale_factor` is a placeholder (1.0) until a
reference measurement is taken, and `R(λ)` is dummy data (`data/calibration_dummy.csv`)
until a monochromator sweep. The pipeline is built and unit-tested; only the numbers
are pending. Lux output would add photopic V(λ) weighting on top.

## Arduino CLI setup

```bash
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit ADS1X15"
```

`CDCOnBoot=cdc` is baked into the justfile FQBN (needed for native USB Serial).
Linux serial access: `sudo usermod -aG dialout $USER`. A stuck CDC port clears with
an RTS pulse or replug; a clean rebuild (`arduino-cli compile --clean`) fixes
occasional stale-cache upload hangs.
