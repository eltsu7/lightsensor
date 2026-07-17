# Reference

Serial protocol, driver API, and calibration model. For the high-level overview,
architecture, and contributor guidance see [`../CLAUDE.md`](../CLAUDE.md); the
Python API is also documented in docstrings in `lightmeter/sensor.py`.

## Serial protocol

Single-char commands at 115200 baud (`raw` = signed 16-bit 0–32767; flags 0/1).
**Protocol version 2.**

| Command | Description | Response |
|---------|-------------|----------|
| `r` / `r<n>\n` | Read once / averaging `n` samples (clamped 1–1000); autoexposes first if autogain is on | `raw,sensor_sat,adc_sat,gain\n` |
| `g<n>` | Set gain index 0–5 (turns autogain **off**) | `ok` / `err <code>` |
| `G` | Query gain index | integer |
| `a0` / `a1` | Disable / enable autogain (autoexposure) | `ok` / `err <code>` |
| `A` | Query autogain state + current gain | `<0\|1> <gain>` |
| `p` | Ping (no I2C) | `pong` |
| `I` | Identity / version handshake | `lightsensor proto=2 fw=… id=<MAC> sps=860 vsat=3.20 dark=… gains=6.144,…` |
| `d<volts>\n` / `D` | Persist / query device electrical dark correction (±0.25 V) | `ok` / `err <code>` / volts |
| `W<n>\n`+bytes | Write calibration blob | `ok <crc32>` / `err <code>` |
| `C` | Read calibration | `<size> <crc32>\n` + bytes (`0 0` if none) |
| `H` / `X` | Cal size / erase | size / `ok` / `err <code>` |

**Autogain (autoexposure)** lives in the firmware. When on, each `r` read takes
a sample and, while over-exposed (saturated or ≥90 % of full scale) or
under-exposed (<40 %), steps the gain one notch (wider range / more sensitive)
and re-reads, until the signal is in-band or a gain rail (0 or 5) is hit; then it
averages `n` at the settled gain. The `r` reply's 4th field is that gain — the
host records it (raw is gain-relative). Any manual `g<n>` turns autogain off.
The 40 %/90 % band is wider than the 2× adjacent-gain ratio, so one step always
lands in-band (no oscillation).

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

- **Electrical dark correction:** firmware persists a per-device offset in
  LittleFS and reports it as `dark` in `I`. The default is the calculated
  R1/R3 divider baseline, `3.3 × 270 / (13000 + 270) = 0.067144 V`.
  `d<volts>\n` updates the value after validating the ±0.25 V range; `D`
  reads it. This is not an automatic darkness measurement.

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
| `gain` / `set_gain(i)` / `get_gain()` | Applied gain index (tracked locally, from the device's `r` reply) / set (also disables autogain on the device) / query device |
| `autogain` / `set_autogain(enabled)` / `get_autogain()` | Local mirror of firmware autoexposure state / enable-disable (`a1`/`a0`) / query device state + gain (`A`) |
| `device_dark_offset_v` / `set_device_dark_offset(v)` / `reset_device_dark_offset()` | Persisted device electrical correction / update / restore the calculated 0.067144 V divider baseline |
| `calibrate_device_dark_offset(n=200)` | Measure the covered sensor uncorrected and persist that device value; returns volts or `None` |
| `zero(n=50)` / `clear_zero()` / `session_zero_offset_v` / `zero_offset` | Temporary background zero / clear it / session value / active correction; clearing restores the device offset |
| `average` | ADC samples the firmware averages per `read()` (default 1; ~√n noise, ×n time) |
| `info` / `ping()` / `identify()` / `connected` | Identity from handshake / link check / query / port state |
| `reconnect(attempts=5, backoff=0.5)` | Re-detect port + reopen + handshake; returns bool, never raises |
| `auto_reconnect` | Opt-in (default `False`): `read()` reconnects on link error instead of raising |
| `reading_voltage(reading)` | Dark-corrected voltage for a `Reading` at the current gain |
| `read_physical(source=None)` | Read once and convert to a physical value via cached calibration |
| `load_calibration()` / `read_calibration()` / `write_calibration(text)` / `write_calibration_file(path)` | Cal fetch+cache / read / write (CRC-verified) |
| `has_calibration()` / `clear_calibration()` | Stored cal size / erase |

Behavioral contracts (thread-safety, reconnect, dark-offset ordering) are in
CLAUDE.md's invariants. Both dark offsets are stored as volts so they stay
correct across gain changes. A session zero overrides the persisted device
offset, and `read()` subtracts the active value last (display only) —
`sensor_sat` / `adc_sat` still reflect the true raw level. Autoexposure itself
is firmware-side (see the protocol table above); the driver only mirrors the
resulting gain.

## Rust driver (`rust/`)

A from-scratch Rust port of this driver (`lightmeter` crate, MIT/Apache-2.0),
built for the [pointcamera](https://github.com/eltsu7/pointcamera) turret rig.
Same protocol, including persisted device dark correction and temporary
zeroing, behind a `Transport` trait so a `SimTransport` can emulate the
firmware line-by-line for hardware-free tests and GUI development. Calibration
transfer and the spectral/photometric conversion pipeline are intentionally not
ported yet — raw values and volts are what a consumer needs today; see the
crate's own rustdoc (`cargo doc --open` in `rust/`) for the full API.

## Calibration & physical units (`Calibration`)

The device stores a spectral responsivity curve `R(λ)` + metadata (`scale_factor`,
`scale_units`, `device_id`, `cal_date`, …) as a CSV in LittleFS; it's dumb storage,
all unit math is host-side.

| Member | Description |
|--------|-------------|
| `scale_factor` / `scale_units` | Absolute scale (physical_unit per volt) and its unit string |
| `provenance` / `is_nominal` | `'measured'` (real cal, default) vs `'datasheet-typical'` (bundled fallback, shape only) |
| `responsivity_at(wl)` | `R(λ)` by linear interpolation; `0` outside the measured band |
| `source_weighted_responsivity(wl, intensity)` | `R̄ = ∫sR dλ / ∫s dλ` for a source (trapezoidal) |
| `voltage_to_value(voltage, source=None)` | Convert a dark-corrected voltage to a physical value |

**Bundled default (`default_calibration()`):** when the device has no stored cal,
`load_calibration()` falls back to the packaged BPW34 datasheet-typical curve
(`lightmeter/data/calibration_bpw34_typical.csv`, from Vishay doc 81521 Fig. 7).
It carries the real *spectral shape* `R(λ)` (peak 900 nm, 10% points 430/1100 nm)
plus a **nominal** absolute scale `scale_factor ≈ 0.103 W/m²/V`, derived purely from
the datasheet — `1/(R_peak·A·R_f)` with `R_peak ≈ 0.646 A/W` (from `I_k = 47 µA @
1 mW/cm², 950 nm`, renormalized to the peak), `A = 7.5 mm²`, and the board's
`R_f = 2 MΩ`. Good to ~datasheet tolerance (±20%), not a measured cal — flagged
`provenance='datasheet-typical'` (`is_nominal=True`) so callers never mistake it for
one. `source=None` assumes monochromatic light at the 900 nm peak. Re-derive
`scale_factor` if `R_f` changes; replace the whole file with a measured cal when
available. Pass `load_calibration(use_default=False)` to opt out of the fallback.

**Daylight weighting (`daylight_spectrum(temp_k=6500, …)`):** returns
`(wavelengths_nm, relative_intensities)` for a Planck blackbody across the BPW34
band (a blackbody, not the D65 table, because D65 stops at 830 nm while the sensor
sees to ~1100 nm). The GUI's **W/m² units mode** passes this as `source` to weight
`R(λ)`, so the displayed irradiance assumes a daylight spectrum; with the default
cal that works out to ≈ 0.21 W/m²/V (R̄ ≈ 0.49). It's a nominal estimate — change
the source, or set a real `scale_factor`, for anything better.

**Photometric / lux (`luminous_efficacy(wl, intensity)`):** returns the source's
luminous efficacy `K = 683 · V̄` (lm/W), where `V̄` is the spectrum weighted by the
CIE photopic `V(λ)` (`PHOTOPIC_V`, 380–780 nm). Illuminance (lux) = `K ·`
irradiance, so the GUI's **lux units mode** = W/m² factor × `K(daylight)` ≈
0.21 × 144 ≈ 30 lux/V. Lux (illuminance, lm/m²) is the honest unit for a flat
detector; luminance (cd/m²) would need a solid-angle / diffuse-field assumption and
isn't exposed.

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
