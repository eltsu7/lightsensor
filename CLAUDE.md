# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. It is the authoritative project doc; exhaustive per-control GUI behaviour and the full driver member list live in the code (`lightmeter/gui.py`, `lightmeter/sensor.py`).

## Overview

Precision calibrated light sensor: a custom PCB (OPA323 op-amp + ADS1115 16-bit ADC over I2C) read by an ESP32-C3 SuperMini, with a Python driver and a live Tkinter debug GUI. The firmware returns raw ADC counts; all physics/unit conversion happens host-side.

> Work in progress — firmware and driver are still evolving, and the absolute calibration numbers are placeholders pending a reference measurement (see `TODO.md`).

## Commands

```bash
just flash                        # compile + upload firmware (port auto-detected)
just compile                      # compile only
uv run python -m lightmeter.gui   # debug GUI (or: lightmeter). --port to override
uv run tests/test_read.py         # smoke test: connect, read 10 samples (needs hardware)
uv run tests/test_calibration.py  # pure unit tests for the conversion math (no hardware)
uv run ruff check                 # lint (config in pyproject.toml)
uv build                          # build the wheel
```

`tests/` uses a hand-rolled runner, not pytest. `test_read.py` needs the ESP32-C3 connected; `test_calibration.py` is pure and runs anywhere.

## Architecture

Three layers over one USB-CDC serial link at 115200 baud:

1. **Firmware** (`firmware/lightsensor/lightsensor.ino`, Arduino/ESP32-C3) — owns the ADS1115 over I2C and a LittleFS partition. A flat single-char command loop. Returns raw ADC counts only; no unit conversion. (The `.ino` must live in a directory of the same name — Arduino constraint.)
2. **Driver** (`lightmeter/sensor.py`) — the `LightSensor` class wraps the serial protocol. All derivation lives here: counts→voltage, gain bookkeeping, dark-offset (`zero`), autogain, calibration parse/conversion. `lightmeter/port_detect.py` does cross-platform port autodetection (also used by the justfile).
3. **GUI** (`lightmeter/gui.py`) — Tkinter app, background sampler thread decoupled from the ~33 fps plot. Records to `recordings/rec_*.csv` (git-ignored); `save_recording` / `open_recording_plot` are reusable for a future "previous measurements" picker.

### Key invariants (easy to break)

- **Voltage is the canonical unit**, not percent or counts. Driver, GUI, and recordings store actual volts so data stays valid across gain changes. Convert with `value * GAIN_VOLTAGES[gain] / 100`; don't carry gain-relative numbers.
- **Dark offset is applied last** and never feeds gain/saturation logic. `read()` runs autogain and saturation flags on the true (un-zeroed) level, then subtracts the offset for display only. Preserve that ordering. The offset is stored as volts (gain-independent).
- **Two mutually-exclusive saturation modes**, set by gain: `sensor_sat` (op-amp at rail, gains 0–1, full-scale > 3.266 V) vs `adc_sat` (counts hit 32767, gains 2–5). Both come from the firmware per reading.
- **Thread-safe / reconnect:** every serial transaction is guarded by a re-entrant lock. A cleanly-closed port is transparently reopened. On a real link error, `auto_reconnect` off (default) re-raises so a caller with its own loop (the GUI sampler) handles it; on, `read()` calls `reconnect()` and returns `None`. `reconnect()` re-detects the port (the device can re-enumerate after replug).

## Hardware

- **MCU:** ESP32-C3 SuperMini, native USB-CDC (no DTR/RTS reset quirks; a plain open works). Port auto-detected (`/dev/ttyACM0` on Linux, a `COM` port on Windows).
- **I2C:** SDA→GPIO10, SCL→GPIO21, 100 kHz (400 kHz failed on this wiring). ADS1115 ALERT/RDY tied to ground (no data-ready interrupt).
- **ADC:** ADS1115 on the sensor PCB, ADDR→GND (0x48), VDD→3.3 V, 860 SPS. **Absolute max input VDD+0.3 = 3.6 V — never exceed.**
- **Sensor:** OPA323 op-amp, 3.3 V supply; output saturates ~3.266 V (~34 mV below rail).
- **Read speed:** ~330 reads/s single-shot (ADC conversion + I2C bound). Averaging `n` multiplies per-read time by `n`, cuts noise ~√n. Beating the ceiling needs continuous-conversion mode + ALERT/RDY on a GPIO (hardware-gated).
- **Drift:** slow downward drift observed (likely thermal warmup of ADC reference / op-amp offset) — uncharacterized; see `TODO.md`.

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

**Gain index → range / saturation:** 0 ±6.144 V, 1 ±4.096 V (default), 2 ±2.048 V, 3 ±1.024 V, 4 ±0.512 V, 5 ±0.256 V. Indices 0–1 saturate at the sensor (3.266 V); 2–5 saturate the ADC (32767).

**Error codes** (`err <code>`, mirrored in `sensor.py` `ERR_MESSAGES`): 1 bad arg, 2 bad length, 3 out of memory, 4 transfer timeout/short read, 5 fs open failed, 6 write size mismatch, 7 erase failed.

### Protocol contracts (non-obvious)

- **Firmware is the source of truth** for the gain table and saturation voltage; it reports them in `I`. The driver verifies its mirrored `GAIN_VOLTAGES` / `SATURATION_VOLTAGE` on connect and warns on drift. Bump `PROTO_VERSION` (both sides) on any breaking command/response change.
- **Calibration transfer is CRC32-verified** (matches `binascii.crc32`). The host **must throttle** the upload in small flushed chunks — a fast burst overruns the device USB-CDC RX buffer while it writes flash, silently dropping bytes; the firmware buffers the whole blob in RAM before writing for the same reason. `write_calibration` returns `True` only on a CRC match; `read_calibration` returns `None` on mismatch. Trust those rather than re-reading.
- **Resync after desync:** every device-side read self-times-out (≤5 s for `W`), so the device never blocks forever. On connect the driver drains input → pings to `pong` → reads identity. If desynced (e.g. interrupted `W`), it goes **silent** past the device timeout to let the stuck command self-abort — it must *not* keep pinging, since each byte feeds the pending read and resets its timeout. A `W` abort discards only the partial upload; stored calibration is preserved.

## Calibration & physical units

The device stores a spectral responsivity curve `R(λ)` + metadata (`scale_factor`, `scale_units`, `device_id`, `cal_date`, …) as a CSV in LittleFS; it's dumb storage, all unit math is host-side. `read_calibration()` / `read_physical()` give you the data and conversion.

**Conversion model:** `physical = scale_factor · V / R̄_source`, where `V` is the dark-corrected voltage and `R̄_source` is the source-weighted mean responsivity (`∫sR/∫s`). The sensor integrates incident light over its spectral response, so the same voltage means different physical levels for different spectra — passing a `source=(wavelengths, intensities)` applies that correction; `source=None` ⇒ `R̄=1.0` (measuring the same spectrum the absolute scale was calibrated against). Returns `None` when uncalibrated or the source doesn't overlap the band.

**Not yet absolutely calibrated:** `scale_factor` is a placeholder (1.0) until a reference measurement is taken, and `R(λ)` is dummy data (`data/calibration_dummy.csv`) until a monochromator sweep. The pipeline is built and unit-tested; only the numbers are pending. Lux output would add photopic V(λ) weighting on top.

## Arduino CLI setup

```bash
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "Adafruit ADS1X15"
```

`CDCOnBoot=cdc` is baked into the justfile FQBN (needed for native USB Serial). Linux serial access: `sudo usermod -aG dialout $USER`. A stuck CDC port clears with an RTS pulse or replug; a clean rebuild (`arduino-cli compile --clean`) fixes occasional stale-cache upload hangs.
