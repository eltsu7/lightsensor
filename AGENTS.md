# Repository Guidelines

## Project overview

`lightmeter` is the production software for the ordered LightSensor PD Amp v3.0
board. A BPW34/OPA323 fixed 2 MΩ transimpedance stage feeds an ADS1220; a bare
RP2040 streams measurements over native USB CDC. Python provides the protocol-3
driver, Tkinter GUI, and CSV recording.

The immutable `v2-final` Git tag preserves the retired ESP32-C3/ADS1115 protocol-2
system. Do not add protocol-2 compatibility to the active tree. Rust, spectral
calibration, physical-unit conversion, and temperature compensation are deferred.

## Architecture and data flow

```text
BPW34 → OPA323 → ADS1220 → RP2040 protocol 3 → Python LightSensor
                                                       ├─ SensorSampler → Tk GUI
                                                       └─ versioned stream CSV
TMP117 ────────────────────────────────────────────────────────────────┘
```

- `firmware/lightsensor/lightsensor.ino` is the protocol and measurement source
  of truth. It owns ADS1220 configuration, DRDY acquisition, autogain, sliding
  averaging, clipping flags, temperature reads, and atomic dark persistence.
- `lightmeter/sensor.py` owns COBS/CRC framing, command correlation, handshake,
  typed samples/events, UTC synchronization, and reconnect behavior.
- `lightmeter/gui.py` keeps `SensorSampler` as the sole serial owner. Tk consumes
  locked snapshots and never performs transport I/O.
- `docs/reference.md` defines protocol 3 and the public Python workflow.
- `docs/v3/hardware.md` is the ordered-board electrical contract and measured
  bring-up record. Do not recover pins from memory.

## Measurement and protocol invariants

- USB CDC uses binary protocol 3: COBS-delimited frames, fixed version/type/length
  header, little-endian payloads, and CRC32. Maximum decoded frame is 256 bytes.
- Connect with `HELLO`, `TIME_SYNC`, and `LIST_PROFILES`. Every request ID is
  nonzero and unique among active commands. A stream emits no samples until its
  exact start token is acknowledged within one second.
- Stream configuration is atomic. `START_STREAM` carries format, mode, profile,
  gain, autogain, averaging window, and finite output count. Changing acquisition
  or dark state replaces the stream and produces a complete new header.
- Sequence numbers restart at zero per stream. Every sample carries RP2040 time,
  actual gain, status, and TMP117 temperature. Any protocol, ADC, temperature,
  timing, overflow, or storage error stops acquisition.
- Profiles are firmware-owned measured contracts: filtered 19.958 SPS,
  interactive 327.876 SPS, and turbo 1949.3 SPS. Do not substitute ADS1220
  datasheet nominal rates for the measured values.
- Gains are 1×–128× (`gain = 2**gain_index`). Autogain is firmware-authoritative,
  volts-only, one step per conversion, and targets 40–85% of full scale.
- Preserve separate status mechanisms: positive/negative ADC full scale,
  measured positive TIA clipping at 1.64 V differential, and autogain rail flags.
  Negative TIA clipping remains unmeasured and must not be asserted.
- Canonical voltage is signed `AIN0 - AIN1`, normalized independently for every
  conversion before a sliding mean. Apply the active dark correction last. Dark
  correction must never affect autogain or clipping.
- Raw streams are gain-dependent evidence and cannot autogain. Store voltage for
  normal measurements; never store gain-relative percentages.
- Session zero overrides the persisted device baseline. `clear_zero()` restores
  the device baseline. `save_baseline()` is the explicit atomic flash write.
  Missing storage means `0.0 V`; corrupt/mount-failed storage is an explicit fault
  and may only be repaired by device-ID-confirmed `RESET_STORAGE`.
- TMP117 temperature is evidence only. No compensation or warm-up threshold is
  approved.

## Hardware constraints

- RP2040 runs at 133 MHz from the ordered 12 MHz crystal and W25Q32 4 MiB QSPI
  flash. The production build uses a 2 MiB sketch / 2 MiB LittleFS split.
- ADS1220 SPI0 pins: GPIO0 MISO/DOUT, GPIO1 CS, GPIO2 SCLK, GPIO3 MOSI, GPIO4
  DRDY. Use SPI mode 1 at 1 MHz, internal 2.048 V reference, continuous mode,
  dedicated active-low DRDY, and `AIN0 - AIN1`.
- TMP117 uses I2C1 on GPIO10/GPIO11 at 400 kHz, address `0x48`; ALERT is not
  connected. A failed scheduled temperature read terminates the stream.
- The analog front end is fixed 2 MΩ / 3.3 pF. There is no switched feedback
  resistor and no external ADC reference.
- Firmware USB identity is manufacturer `LightSensor`, product `LightSensor v3`,
  VID:PID `2e8a:f00a`; USB serial and protocol ID are the flash UID.

## Key paths

- `firmware/lightsensor/lightsensor.ino` — production RP2040 firmware.
- `lightmeter/sensor.py` — protocol constants, dataclasses, framing, and driver.
- `lightmeter/gui.py` — worker-thread acquisition, plot, controls, and CSV writer.
- `lightmeter/port_detect.py` — USB identity/device-ID discovery.
- `lightmeter/__init__.py` — supported Python exports.
- `tests/test_protocol.py` — hardware-free hand-rolled protocol contracts.
- `tests/test_read.py` — connected ten-sample smoke script; do not discover/import.
- `docs/reference.md` — canonical protocol and host reference.
- `docs/v3/hardware.md` — electrical facts and measured characterization.
- `justfile` — Arduino-Pico setup, compile, and UF2 upload recipes.

## Development commands

```bash
uv sync
uv run tests/test_protocol.py
uv run tests/test_read.py          # connected v3 board only
uv run ruff check
uv build
uv run python -m lightmeter.gui

just setup                         # install Arduino-Pico core once
just compile
just upload                        # SENSOR_PORT defaults to /dev/ttyACM0
just flash
```

Hold `BOOT`, pulse `RESET`, then release `BOOT` for manual ROM UF2 recovery.

## Code conventions

### Python

- Python >=3.13; Ruff line length 100. Use 4 spaces, direct type annotations,
  `snake_case` functions, `PascalCase` classes/dataclasses, and uppercase constants.
- Keep the public surface explicit in `lightmeter/__init__.py`. Prefer module
  helpers and dataclasses over new framework layers.
- `LightSensor` owns a `threading.RLock`; every serial command/read remains under
  it. Preserve partial received frame bytes across serial timeouts.
- Validate exact payload sizes, enum ranges, sequence semantics, COBS boundaries,
  and CRC before accepting data. Device-reported errors are typed and must not be
  hidden as empty samples.
- Link reconnect performs a fresh handshake/time sync/profile read, remains
  stopped, and stays bound to the original flash UID. Never silently resume the
  prior stream or select another device.
- `SensorSampler` is the GUI's sole serial owner. Queue controls onto its worker,
  treat stream replacements as new sessions, and protect every UI snapshot.
- Bound live rendering independently from acquisition history and recording:
  use canvas-width chronological min/max bins at 20 Hz, reduced to 10 Hz for
  turbo acquisition. Paused plots must re-bin raw history without changing user
  pan/zoom limits.
- Record one CSV per stream. Include the full effective start/config row and UTC,
  device time, sequence, value, gain, status, and temperature for every sample.

### Firmware

- Use existing Arduino style: two-space indentation, `camelCase` functions/state,
  uppercase constants, and single-threaded global state plus the minimal DRDY ISR.
- Keep acquisition interrupt work bounded: only count events and capture time.
  SPI, USB, filtering, state changes, and flash stay in the main loop.
- No dynamic allocation in protocol/acquisition paths. Preserve the 256-byte
  decoded limit, 258-byte COBS-body limit, and 1024-conversion window ceiling.
- Do not silently drop samples or DRDY events. A gap/overflow is a terminal error.
- Persistent writes stop/restart streams, write a CRC-protected temporary record,
  flush, and atomically rename it. Never auto-format on mount/integrity failure.
- A breaking wire change requires a protocol bump and matching Python package
  major version.

## Testing and verification

- Tests use a hand-rolled runner, not pytest. Add deterministic module-level
  `test_<behavior>()` functions with plain asserts to `tests/test_protocol.py`.
- Protocol tests must defend observable framing, CRC, bounds, typed sample fields,
  stream-state rules, and package/protocol-major compatibility.
- `tests/test_read.py` opens hardware at module top level; run it only with the
  production v3 board connected.
- Before yielding a Python change, run `uv run tests/test_protocol.py` and
  `uv run ruff check`. Exercise driver/GUI worker changes against hardware when
  available. Compile firmware changes with `just compile`; flash and run the
  changed acquisition path for behavioral firmware changes.
