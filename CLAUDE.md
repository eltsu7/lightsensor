# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

The serial protocol, driver API, and calibration model are documented in [`docs/reference.md`](docs/reference.md); the Python API also has docstrings in `lightmeter/sensor.py`. This file is the high-level orientation.

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

## Key invariants (easy to break)

- **Voltage is the canonical unit**, not percent or counts. Driver, GUI, and recordings store actual volts so data stays valid across gain changes. Convert with `value * GAIN_VOLTAGES[gain] / 100`; don't carry gain-relative numbers.
- **Dark offset is applied last** and never feeds gain/saturation logic. `read()` runs autogain and saturation flags on the true (un-zeroed) level, then subtracts the offset for display only. Preserve that ordering. The offset is stored as volts (gain-independent).
- **Two mutually-exclusive saturation modes**, set by gain: `sensor_sat` (op-amp at rail, gains 0–1, full-scale > 3.266 V) vs `adc_sat` (counts hit 32767, gains 2–5). Both come from the firmware per reading.
- **Thread-safe / reconnect:** every serial transaction is guarded by a re-entrant lock. A cleanly-closed port is transparently reopened. On a real link error, `auto_reconnect` off (default) re-raises so a caller with its own loop (the GUI sampler) handles it; on, `read()` calls `reconnect()` and returns `None`.
- **Calibration transfer must be throttled and is CRC-verified.** Don't "optimize" `write_calibration` into one big burst — it overruns the device RX buffer. See `docs/reference.md` for why, plus the resync and firmware-is-source-of-truth contracts.

## Hardware gotchas

100 kHz I2C (400 kHz failed on this wiring), 860 SPS, ~330 reads/s single-shot ceiling (ALERT/RDY tied to ground, so no continuous-conversion path). **Never exceed 3.6 V on an ADS1115 input.** OPA323 saturates ~3.266 V. Native USB-CDC (no DTR/RTS quirks); a stuck port clears with an RTS pulse or replug. Slow downward drift observed (uncharacterized thermal warmup — see `TODO.md`). Full hardware notes and Arduino CLI setup in `docs/reference.md`.
