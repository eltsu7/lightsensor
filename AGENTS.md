# LightSensor — Project Summary

## Overview
Precision calibrated light sensor using a custom PCB with OPA323 op-amp and ADS1115 16-bit ADC over I2C, read by an ESP32-C3 SuperMini. Includes a live debug GUI.

## Hardware
- **MCU:** ESP32-C3 SuperMini — native USB-CDC (no USB-UART bridge, no DTR/RTS reset quirks). Port auto-detected (typically `/dev/ttyACM0` on Linux, a `COM` port on Windows). Flash via `just flash`.
- **I2C pins:** SDA→GPIO10, SCL→GPIO21. ADS1115 ALERT/RDY tied to ground (no data-ready interrupt).
- **ADC:** ADS1115 on the sensor PCB — ADDR→GND (0x48), VDD→3.3V. Runs at 100 kHz I2C and 860 SPS data rate. 400 kHz I2C failed (all zeros) with an earlier breakout's pull-ups over jumper wires.
- **Sensor:** Custom PCB using OPA323 op-amp, powered from 3.3V
- **OPA323 saturation:** ~3.266V (~34 mV below 3.3V rail); both sensor and ADC saturation reported per reading
- **ADC absolute max input:** VDD + 0.3V = 3.6V — do not exceed regardless of gain setting

## Files
| File | Description |
|------|-------------|
| `lightsensor/lightsensor.ino` | Arduino sketch (ESP32-C3) — device interface over serial |
| `lightsensor.py` | Sensor driver — `LightSensor` class, `Reading` dataclass, `best_gain()`, autogain |
| `port_detect.py` | Cross-platform serial-port auto-detection; importable and runnable (`uv run python port_detect.py` prints the port — used by the justfile when flashing) |
| `main.py` | Debug GUI — Tkinter, threaded sampler, live plot |
| `test_read.py` | Smoke test — auto-connect, read 10 samples, print values + sample rate |
| `test_calibration.py` | Unit tests for the volts→units conversion math (pure, no hardware) |
| `calibration_dummy.csv` | Placeholder spectral responsivity curve (400 pts) until monochromator data exists |
| `justfile` | `just compile`, `just upload`, `just flash` (port auto-detected) |
| `docs/` | ADS1115, OPA323, Soldered 333095 breakout datasheets |
| `TODO_v2.md` | Plans for v2 (ESP32-C3 SuperMini, on-board ADC, new cable) |

## Usage

**Flash firmware:**
```bash
just flash
```

**Run debug GUI:**
```bash
uv run main.py                 # auto-detects port
uv run main.py --port COM5     # or specify explicitly
```

**Use driver in code:**
```python
from lightsensor import LightSensor

with LightSensor() as sensor:      # port auto-detected if omitted
    sensor.set_gain(2)             # ±2.048V
    reading = sensor.read()        # Reading(value, sensor_sat, adc_sat)
    print(reading.value)           # light level as % of ADC full-scale
    print(reading.sensor_sat)      # op-amp at rail
    print(reading.adc_sat)         # ADC raw hit 32767

    # autogain
    sensor.autogain_oneshot(100)   # sample 100 readings, set best gain
    sensor.autogain = True         # continuous autogain inside read()
```

## Driver API (`lightsensor.py`)

### Constants
| Name | Description |
|------|-------------|
| `GAIN_LABELS` | Display strings for each gain index (`["±6.144V", …]`) |
| `GAIN_VOLTAGES` | Full-scale voltages (`[6.144, 4.096, …, 0.256]`) |
| `DEFAULT_GAIN` | `1` (±4.096V) |
| `SATURATION_VOLTAGE` | `3.2` V — OPA323 output ceiling with 3.3V supply |

### `best_gain(max_voltage, headroom=0.85)`
Pure function. Returns the highest gain index that keeps `max_voltage` below the saturation threshold with the given headroom factor.

### `Reading` dataclass
| Field | Type | Description |
|-------|------|-------------|
| `value` | `float` | Light level as % of ADC full-scale (0–100) |
| `sensor_sat` | `bool` | Op-amp output near supply rail |
| `adc_sat` | `bool` | ADC raw hit 32767 |

Sensor and ADC saturation are mutually exclusive with this hardware: sensor_sat only occurs at low gain settings (full-scale > 3.266V), adc_sat only at high gain settings (full-scale < 3.266V).

### `LightSensor`
| Member | Description |
|--------|-------------|
| `gain` | Currently applied gain index (locally tracked) |
| `autogain` | Enable continuous autogain inside `read()` |
| `autogain_interval` | Gain evaluation interval in seconds (default 0.25) |
| `autogain_window` | History window for evaluation in seconds (default 0.5) |
| `read()` | Returns `Reading` or `None` |
| `set_gain(index)` | Sets gain, updates `self.gain`, returns `True` on success |
| `get_gain()` | Queries current gain index from device |
| `autogain_oneshot(n=100)` | Collect n samples, apply best gain, return gain index |
| `zero(n=50)` | Measure dark offset over n samples; subtract from future reads; returns offset (V) |
| `clear_zero()` | Remove the dark offset |
| `is_zeroed` / `zero_offset` | Whether an offset is active / its value in volts |
| `average` | Number of ADC samples the firmware averages per `read()` (default 1) |
| `info` | `DeviceInfo` from the connect handshake (`product`, `proto`, `fw`, `id`) |
| `ping()` / `identify()` | Link health check / query identity (`DeviceInfo`) |
| `reading_voltage(reading)` | Absolute dark-corrected voltage for a `Reading` at the current gain |
| `read_physical(source=None)` | Read once and convert to a physical value via the cached calibration |
| `load_calibration()` | Fetch device calibration into `self.calibration` (cache) |
| `read_calibration()` / `write_calibration(text)` / `write_calibration_file(path)` | Low-level cal transfer (CRC-verified) |
| `has_calibration()` / `clear_calibration()` | Stored cal size / erase |

Firmware-side averaging: `read()` sends `r<n>` and the device averages `n` raw ADC samples, returning one `Reading`. Reduces noise by ~√n at the cost of proportionally slower reads.

The dark offset is stored as a voltage (gain-independent) so it stays correct across gain changes. `read()` subtracts it from `value`; `sensor_sat`/`adc_sat` still reflect the true raw level.

### Calibration & physical units (`Calibration`)
The device stores a spectral responsivity curve `R(λ)` + metadata (`scale_factor`, `scale_units`, `device_id`, `cal_date`, …); all unit conversion happens host-side. `LightSensor.read_calibration()` returns a `Calibration`:

| Member | Description |
|--------|-------------|
| `scale_factor` / `scale_units` | Absolute scale (physical_unit per volt) and its unit string |
| `responsivity_at(wl)` | `R(λ)` by linear interpolation; `0` outside the measured band |
| `source_weighted_responsivity(wl, intensity)` | `R̄ = ∫sR dλ / ∫s dλ` for a source spectrum (trapezoidal) |
| `voltage_to_value(voltage, source=None)` | Convert a dark-corrected voltage to a physical value |

**Conversion model:** `physical = scale_factor · V / R̄_source`. The sensor integrates the incident light over its spectral response, so the same voltage means different physical levels for different source spectra — supplying `source` (a `(wavelengths, intensities)` pair) applies that spectral correction. `source=None` ⇒ `R̄_source = 1.0`, i.e. you're measuring the same spectrum the absolute scale was calibrated against. Returns `None` when there's no `scale_factor` (uncalibrated) or the source doesn't overlap the calibrated band.

**Not yet absolutely calibrated:** `scale_factor` is a placeholder (1.0) until a reference measurement against a known source/meter is taken, and `R(λ)` is dummy data until measured on a monochromator. The conversion *pipeline* is implemented and unit-tested (`test_calibration.py`); only the numbers are pending. Convolving `R(λ)` against a real source spectrum (and photopic weighting for lux) stays a host-side step on top of this.

## Device Interface (Serial)

Commands sent over serial at 115200 baud:

| Command | Description | Response |
|---------|-------------|----------|
| `r` | Read ADC once (1 sample) | `raw,sensor_sat,adc_sat\n` — e.g. `15031,0,0` |
| `r<n>` | Read ADC, averaging `n` samples on the device (newline-terminated) | one `raw,s,a\n` line |
| `g<n>` | Set gain index 0–5 | `ok` or `err <code>` |
| `G` | Query current gain index | integer + newline |
| `p` | Ping (CDC link health check, no I2C) | `pong` |
| `I` | Identity / version handshake | `lightsensor proto=1 fw=1.0.0 id=<MAC> sps=860 ngains=6` |
| `W<n>\n`+bytes | Write calibration blob (n bytes) | `ok <crc32>` or `err <code>` |
| `C` | Read calibration | `<size> <crc32>\n` then `<size>` bytes (`0 0` if none) |
| `H` | Calibration size (has-cal check) | size or `0` |
| `X` | Erase calibration | `ok` or `err <code>` |

`raw` is the signed 16-bit ADC value (0–32767). `sensor_sat` and `adc_sat` are 0 or 1. The averaging count is clamped to 1–1000 on the device.

### Protocol contract
- **`I` (identity):** product token + space-separated `key=value` pairs. `proto` is the protocol version (bump on any breaking command/response change); `fw` the firmware version; `id` the 48-bit eFuse MAC as hex, usable as a per-unit serial number. The driver runs this on connect (`LightSensor.info`) and warns on a `proto` mismatch.
- **Error codes** (`err <code>`): 1 bad arg, 2 bad length, 3 out of memory, 4 transfer timeout / short read, 5 filesystem open failed, 6 write size mismatch, 7 erase failed. Mirrored in `lightsensor.py` `ERR_MESSAGES`.
- **Resync / recovery:** every device-side read self-times-out (≤5 s for `W`), so the device never blocks forever. On connect the driver handshakes: drain input → ping until `pong` → read identity. If desynced (e.g. an interrupted `W`), the driver goes **silent** for longer than the device timeout to let the stuck command self-abort — it must not keep pinging, since each byte feeds the pending read and resets its timeout. A `W` that aborts on timeout discards only the partial upload; stored calibration is preserved.

**Read speed (ESP32-C3, single-shot ADC, 860 SPS, 100 kHz I2C):** ~330 reads/s (1 sample). Averaging multiplies the per-read time by `n`. The ~2.5 ms/sample floor is ADC conversion + I2C; lowering it needs continuous-conversion mode + ALERT/RDY on a GPIO (currently tied to ground).

### Gain index mapping
| Index | Range | Saturation limit |
|-------|-------|-----------------|
| 0 | ±6.144V | sensor (3.266V) |
| 1 | ±4.096V (default) | sensor (3.266V) |
| 2 | ±2.048V | ADC (32767) |
| 3 | ±1.024V | ADC (32767) |
| 4 | ±0.512V | ADC (32767) |
| 5 | ±0.256V | ADC (32767) |

## Debug GUI (`main.py`)

Threaded sampler reads as fast as the device allows (decoupled from ~33 fps display). Values are stored as actual voltage (V) so data is gain-independent and preserved across gain changes.

Controls are arranged in a right-hand sidebar grouped into sections.

### View
| Control | Description |
|---------|-------------|
| Follow latest | X axis follows the newest points; auto-disabled when the user pans/zooms in time |
| Auto Y-scale | Autoscale Y axis to visible window; auto-disabled when the user zooms the Y axis |
| Absolute scale | Y axis in V (default); unchecked shows % of current gain range |

### Overlays
| Control | Description |
|---------|-------------|
| Window average | Dashed line at mean of visible window |
| Line fit | Linear regression over visible window |
| Noise band | ±σ shaded band; legend shows σ, relative σ, peak-to-peak |

### Gain
| Control | Description |
|---------|-------------|
| Gain − / combobox / + | Manual gain selection; stops continuous autogain |
| One-shot gain | Collect 100 samples, apply best gain |
| Auto gain ● | Continuous autogain; ● indicates active |
| Zero (dark) / Clear zero | Measure dark offset over 50 samples and subtract from reads; button shows `Zeroed ●` when active. Status shows `zeroing…` during measurement |

### Acquisition
| Control | Description |
|---------|-------------|
| Scan interval | Target ms between samples (0 = as fast as possible) |
| Stop / Start | Pause and resume sampling (decoupled from view interaction) |
| Avg samples | Number of ADC samples the firmware averages per reading (Apply averaging button); reduces noise by ~√n, slower by n |
| Clear | Clear the plot buffer |
| ● Record / ■ Stop recording | Capture all samples to an unbounded buffer; on stop, save CSV and open a standalone zoom/pan plot |

Stats overlays operate over the actually-visible x-range (read from the axis), not a fixed window. User pan/zoom via the matplotlib toolbar drops Follow latest / Auto Y-scale automatically but never stops capturing. Saturation reference line shown in red dashes at the OPA323 ceiling. Status bar (sidebar bottom) shows `⚠ SENSOR SAT` or `⚠ ADC SAT` when the latest reading is saturated, and `● REC <n>` while recording.

### Recordings
Recording captures every sample independent of the rolling display buffer. On stop, data is written to `recordings/rec_YYYY-MM-DD_HH-MM-SS.csv` (named by start time, git-ignored) with columns `time_s, voltage_v, sensor_sat, adc_sat`, then opened in a standalone matplotlib window. Two reusable module functions back this: `save_recording(times, values, sensor_sat, adc_sat, started_at)` returns the file path, and `open_recording_plot(parent, path)` opens any such CSV in a zoom/pan viewer — designed so a future "previous measurements" selector only needs to call `open_recording_plot` with a chosen file.

## Arduino CLI setup
```bash
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit ADS1X15"
```
Flash with `CDCOnBoot=cdc` (already baked into the justfile FQBN) so native USB Serial works.

## Notes
- Arduino sketch filename must match its directory name (`lightsensor/lightsensor.ino`)
- User needs serial access: `sudo usermod -aG dialout $USER` (Linux)
- Serial baud: 115200 (USB-CDC ignores the rate, but it's set for consistency)
- Native USB CDC: a plain port open works on Linux and Windows (no DTR/RTS auto-reset workaround needed). A stuck device (e.g. after many killed scripts left unread bytes) clears with an RTS pulse or replug.
- A clean rebuild may be needed after edits if uploads behave oddly: `arduino-cli compile --clean ...` (stale build cache once caused identical-looking code to "hang").
- ADS1115 input hard-limited to VDD+0.3V = 3.6V; do not apply 5V signals or power the sensor from a higher supply without checking I2C pull-up voltage
- Slow downward drift observed — likely thermal warmup of ADC reference or op-amp offset drift
