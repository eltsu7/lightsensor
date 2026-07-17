# Repository Guidelines

## Project Overview

`lightmeter` is a calibrated light-sensor system: a BPW34 photodiode and OPA323 front end feed an ADS1115 ADC on an ESP32-C3. The repository contains ESP32 firmware, Python driver/Tkinter debug GUI, and a separate Rust protocol-v2 driver.

The firmware returns gain-relative raw ADC data over native USB-CDC; Python owns voltage and spectral/physical-unit conversion. Absolute calibration remains nominal until a reference measurement replaces the bundled datasheet-derived shape.

## Architecture & Data Flow

```text
BPW34 → OPA323 → ADS1115 → ESP32-C3 firmware → USB-CDC protocol v2
                                                     ├─ Python LightSensor → GUI / calibration math
                                                     └─ Rust driver
```

- `firmware/lightsensor/lightsensor.ino` is the wire-protocol source of truth. It samples the ADS1115, performs autogain/averaging, reports raw counts plus saturation flags/gain, and stores calibration blobs in LittleFS.
- `lightmeter/sensor.py` serializes host transactions, converts readings to canonical volts, provides dark-zero/autogain/calibration APIs, and validates calibration CRCs.
- `lightmeter/gui.py` runs serial I/O in `SensorSampler`'s daemon thread; Tk only consumes locked snapshots and redraws independently.
- Python and Rust share protocol version 2. A breaking wire change requires a `PROTO_VERSION` bump and matching driver package major versions.

### Protocol and measurement invariants

- USB-CDC uses 115200 baud and a compact single-character protocol; see `docs/reference.md`.
- Firmware owns the gain table and saturation voltage; hosts mirror them only for conversion and mismatch warnings.
- Preserve two distinct saturation modes: sensor rail saturation at high full-scale gains versus ADC-count saturation at lower full-scale gains.
- Firmware-side autogain is authoritative: with autogain enabled, each read settles in the 40–90% full-scale band before averaging; a manual gain command disables it. Hosts record the returned gain and never calculate exposure steps.
- Store/record **volts**, never gain-relative percent. Apply the active dark correction last for display; it must not affect autogain or saturation decisions. Firmware persists a device baseline (default `0.067144 V` from R1/R3); session `zero()` overrides it and `clear_zero()` restores it.
- Calibration transfers are CRC32-verified and throttled: Python sends flushed 128-byte chunks with short pauses. Do not replace this with one bulk write.
- On an interrupted/desynchronized calibration write, remain silent through the device timeout (up to 5 s); a ping resets the firmware payload timeout.

## Key Directories

- `lightmeter/` — Python package: serial driver, port detection, Tkinter GUI, bundled calibration data.
- `firmware/lightsensor/` — Arduino sketch; directory name must match the `.ino` file.
- `rust/` — independent Rust 2024 protocol-v2 driver crate.
- `tests/` — pure calibration contracts and hardware-connected read smoke script.
- `data/` — calibration parser fixture.
- `docs/` — protocol, hardware setup, calibration limitations, and component references.
- `recordings/` — GUI output; ignored by Git.

## Development Commands

```bash
uv sync
uv run python -m lightmeter.gui        # or installed: lightmeter
uv run tests/test_calibration.py       # pure calibration/math checks
uv run tests/test_read.py              # connected sensor required
uv run ruff check
uv build

just compile                           # ESP32-C3 firmware compile
just upload                            # port detection + upload
just flash                             # compile then upload

cd rust && cargo test
```

`just` uses `esp32:esp32:esp32c3:CDCOnBoot=cdc`; native USB CDC is required. For Arduino CLI setup and Linux serial permissions, follow `docs/reference.md`.

## Code Conventions & Common Patterns

### Python

- Python >=3.13; Ruff line length is 100. Use 4-space indentation, `snake_case` functions/attributes, `PascalCase` classes/dataclasses, `UPPER_SNAKE_CASE` constants, and direct type annotations.
- Keep the public surface explicit in `lightmeter/__init__.py`. Reuse module-level helpers such as `best_gain`, `parse_calibration`, and `autodetect_port`; avoid dependency-injection frameworks or new abstraction layers.
- `LightSensor` owns a `threading.RLock`; every serial transaction must remain protected. Nested high-level calls rely on reentrancy.
- Return `None`/`False` for malformed replies and expected protocol failures, with DEBUG/WARNING logging as appropriate. Link errors raise unless `auto_reconnect=True`; `reconnect()` returns `bool`.
- `SensorSampler` is the sole GUI-side serial owner. Put requested state changes on its worker path, discard the transition sample after gain/autogain/zero changes, and protect UI snapshots with its lock. Session zeroing must not erase the persisted device baseline.
- Invalidate cached calibration after successful device writes/erases. Validate complete length and CRC before parsing downloaded calibration.

### Firmware

- Use existing Arduino style: two-space indentation, `camelCase` functions/state, `UPPER_SNAKE_CASE` constants, and single-threaded global state.
- Preserve command framing/timeouts and reply formats. `g`/`a` commands require their value byte; `d<volts>\n` persists a validated device dark correction and `D` queries it; reads must keep their `raw,sensor_sat,adc_sat,gain` response shape.
- Flash calibration writes intentionally buffer the complete payload before writing LittleFS so serial RX does not overrun during flash activity.

## Important Files

- `lightmeter/sensor.py` — `LightSensor`, `Reading`, `Calibration`, spectral conversion, protocol constants, reconnect and calibration transfer.
- `lightmeter/gui.py` — `SensorSampler`, `SensorApp`, recording/plot helpers, CLI `main()`.
- `lightmeter/port_detect.py` — VID/PID/hint/sole-port serial detection; runnable directly.
- `lightmeter/__init__.py` — supported Python exports.
- `firmware/lightsensor/lightsensor.ino` — protocol loop, ADS1115 acquisition/autogain, LittleFS calibration storage.
- `docs/reference.md` — canonical protocol, calibration-transfer, hardware setup, and measurement-model details.
- `README.md` — current project overview, quick start, and hardware safety limits.
- `pyproject.toml`, `uv.lock`, `.python-version` — Python package/runtime policy.
- `justfile` — firmware compile/upload/flash recipes.
- `rust/Cargo.toml` — Rust driver manifest.

## Runtime/Tooling Preferences

- Use **uv** for Python environments and commands. Python is pinned to 3.13 (`.python-version`) and requires >=3.13.
- Python package build backend is Hatchling; runtime dependencies are `pyserial` and `matplotlib`.
- Use `arduino-cli` through `just` for firmware. Required board core/library setup is documented in `docs/reference.md`; firmware depends on `Adafruit ADS1X15`.
- Rust is a separate edition-2024 crate under `rust/`; use Cargo from that directory.
- Do not treat the bundled BPW34 calibration scale as a measured absolute calibration. Preserve provenance (`measured` versus `datasheet-typical`) and spectrum-dependent conversion behavior.
- Current wiring and operational constraints belong in `README.md`/firmware; the exploratory LCD pin note is not authoritative for current sensor wiring.

- Hardware limits: keep I2C at 100 kHz, never exceed 3.6 V at an ADS1115 input, and expect roughly 330 reads/s in single-shot mode because ALERT/RDY is grounded. Thermal downward drift is observed but not characterized.

## Testing & QA

- Tests use a hand-rolled runner, not pytest. Add deterministic module-level `test_<behavior>()` functions with plain `assert`s to `tests/test_calibration.py`; run it with `uv run tests/test_calibration.py`.
- Use the local `approx()` helper for floating-point contracts and preserve `data/calibration_dummy.csv`'s parseable calibration-v1 metadata/schema.
- `tests/test_read.py` opens hardware at module top level and takes 10 samples. Run only with a connected compatible device; do not blindly import or generic-discover it.
- Test observable boundaries: interpolation endpoints/out-of-band behavior, zero response, gain-independent voltage, dark-offset ordering, CRC failures, and protocol/package-version compatibility.
- Before yielding a permanent Python change, run the focused test path and `uv run ruff check`; compile firmware with `just compile` when modifying the sketch. Run `cd rust && cargo test` for Rust changes.
