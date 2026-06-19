# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

`AGENTS.md` is the authoritative project spec — hardware details, serial protocol, gain mapping, driver API, and GUI controls. Read it first. This file covers only commands and the cross-file architecture not obvious from a single file.

## Commands

```bash
just flash                        # compile + upload firmware (port auto-detected)
just compile                      # compile only
uv run python -m lightmeter.gui   # debug GUI (or: lightmeter). --port to override
uv run tests/test_read.py         # smoke test: connect, read 10 samples, print rate
uv run tests/test_calibration.py  # pure unit tests for the conversion math (no hardware)
uv run ruff check                 # lint (ruff is the linter; config in pyproject.toml)
```

`tests/test_read.py` is a standalone hardware smoke test (needs the ESP32-C3 connected); `tests/test_calibration.py` is a pure unit suite that runs without hardware.

## Architecture

Three layers, talking over one USB-CDC serial link at 115200 baud:

1. **Firmware** (`firmware/lightsensor/lightsensor.ino`, Arduino/ESP32-C3) — owns the ADS1115 over I2C and a LittleFS partition. A flat single-char command loop (`r`/`g`/`G`/`p`/`I` for reads/gain/identity, `W`/`C`/`H`/`X` for calibration storage). It returns raw ADC counts only; it does no unit conversion.
2. **Driver** (`lightmeter/sensor.py`) — the `LightSensor` class wraps the serial protocol. All physics/derivation lives here: counts→voltage, gain bookkeeping, dark-offset (`zero`), autogain, and calibration parse/conversion. `lightmeter/port_detect.py` supplies cross-platform port autodetection (also used by the justfile).
3. **GUI** (`lightmeter/gui.py`) — Tkinter app with a background sampler thread decoupled from the ~33 fps plot.

### Key invariants (easy to break)

- **Voltage is the canonical unit, not percent or counts.** The driver, GUI, and recordings all store actual volts so data stays valid across gain changes. When adding features, convert to volts (`value * GAIN_VOLTAGES[gain] / 100`) rather than carrying gain-relative numbers.
- **Dark offset is applied last and never feeds gain/saturation logic.** `read()` runs autogain and saturation flags on the true (un-zeroed) level, then subtracts the offset for display only. Preserve that ordering.
- **Two mutually-exclusive saturation modes**, set by gain: `sensor_sat` (op-amp at rail, gains 0–1) vs `adc_sat` (counts hit 32767, gains 2–5). Both come from the firmware per reading.

### Calibration storage (host ⇄ device)

The device is dumb storage for a spectral responsivity CSV (~400 points + metadata header); all convolution/unit math is intended to stay host-side. Transfers are CRC32-verified (matches Python `binascii.crc32`):

- The host **must throttle** the upload (small flushed chunks) — a single fast burst overruns the device USB-CDC RX buffer while it writes flash, silently dropping bytes. `write_calibration` already does this; don't "optimize" it into one big write.
- The firmware buffers the whole blob in RAM before writing flash for the same reason.
- `write_calibration` returns `True` only when the device-computed CRC matches the host's; `read_calibration` returns `None` on CRC mismatch. Trust those booleans rather than re-reading to verify.

### Hardware gotchas

100 kHz I2C (400 kHz failed on this wiring), 860 SPS, ~330 reads/s ceiling (ADC conversion + I2C bound; ALERT/RDY is tied to ground so no continuous-conversion path). Never exceed 3.6 V on an ADS1115 input. A stuck CDC port clears with an RTS pulse or replug; a clean rebuild (`arduino-cli compile --clean`) fixes occasional stale-cache upload hangs.
