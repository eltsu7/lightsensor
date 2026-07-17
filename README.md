# lightmeter

A calibrated light sensor: custom PCB (OPA323 transimpedance amplifier + ADS1115
16-bit ADC, I2C) read by an ESP32-C3 SuperMini over native USB. Firmware streams
raw counts and can autoexpose; a Python driver + Tkinter GUI and a Rust driver
both speak the same serial protocol.

> **Work in progress.** The absolute calibration numbers are placeholders
> pending a reference measurement — see [Calibration status](#calibration-status)
> and [`TODO.md`](TODO.md).

## Features

- **Firmware-side autogain** — when enabled, the device autoexposes itself
  (steps gain, re-reads, until in-band) before every averaged read; the host never guesses.
- **Gain-independent data.** Readings convert to volts, not raw counts or
  gain-relative percentages, so a session survives gain changes intact.
- **Device dark correction + session zeroing.** Firmware persists a per-device
  electrical baseline (default: the calculated 67.144 mV R1/R3 divider value);
  a temporary zero can override it for the current background. Saturation flags
  (op-amp rail vs. ADC overflow) remain firmware-reported and mutually exclusive.
- **On-device calibration storage**: a spectral responsivity curve travels
  with the sensor (LittleFS, CRC32-verified transfer). The Python driver uses it
  for physical-unit conversion; the Rust driver currently exposes raw values and volts.
- **Two drivers, one protocol** — Python (`lightmeter/sensor.py`) and Rust
  (`rust/`, crate `lightmeter`), kept in lock-step by a semver rule (below).
- **Live Tkinter GUI** — real-time plot with window-average / line-fit
  overlays, unit switching (%, V, W/m², lux), CSV recording + reopenable plots.

## Hardware

| Part | Role |
|---|---|
| ESP32-C3 SuperMini | MCU, native USB-CDC serial (no DTR/RTS reset quirks) |
| ADS1115 | 16-bit ADC, I2C @ 0x48, single-shot @ 860 SPS |
| OPA323 | Transimpedance amplifier off a BPW34 photodiode |

I2C on GPIO4 (SDA) / GPIO3 (SCL) at 100 kHz (400 kHz was unreliable on this
wiring). **Never exceed 3.6 V on an ADS1115 input**. The OPA323 output reaches
about 3.266 V; firmware conservatively reports sensor saturation from 3.20 V.
Datasheets for all three parts plus the enclosure are under `docs/`.

## Layout

```
firmware/lightsensor/lightsensor.ino   Arduino/ESP32-C3 firmware
lightmeter/                            Python driver + Tkinter GUI
rust/                                  Rust driver (crate `lightmeter`)
docs/reference.md                      Serial protocol, driver API, calibration model
docs/                                  Component datasheets
tests/                                 Hand-rolled Python test scripts
AGENTS.md                              Architecture, invariants, hardware gotchas
```

## Quick start (Python)

```bash
uv sync                     # or: pip install -e .
just flash                  # compile + upload firmware (port auto-detected)
uv run python -m lightmeter.gui   # debug GUI (or just: lightmeter)
```

```python
from lightmeter import LightSensor

with LightSensor() as sensor:        # port auto-detected
    print(sensor.info)               # device identity / firmware
    sensor.set_autogain(True)        # firmware autoexposes each read()
    reading = sensor.read()          # Reading(value, sensor_sat, adc_sat)
    print(sensor.gain, sensor.reading_voltage(reading))
```

## Quick start (Rust)

```rust
use lightmeter::{LightSensor, SerialTransport};

let transport = SerialTransport::open(None)?; // port autodetected
let mut sensor = LightSensor::new(transport)?;
sensor.set_autogain(true)?;
if let Some(reading) = sensor.read()? {
    println!("{:.3} V (gain {})", sensor.reading_voltage(reading), sensor.gain);
}
```

Hardware-free development and tests run against `lightmeter::sim::SimTransport`,
which emulates the firmware's autoexposure line-by-line — no device needed.
Built for the [pointcamera](https://github.com/eltsu7/pointcamera) turret rig;
not yet published to crates.io, so consume it as a path or git dependency.

**Versioning contract:** each driver's package **major** version is pinned to
the protocol version it speaks (`lightmeter==2.*` / `lightmeter = "2"` ⇒
proto 2). A test in each driver enforces this — a `PROTO_VERSION` bump that
forgets the matching package bump fails CI instead of shipping silently.

## Protocol

Single-char commands at 115200 baud over USB-CDC — read/average (`r`), manual
or auto gain (`g`/`a`/`A`), identity handshake (`I`), device dark correction
(`d`/`D`), and calibration blob transfer (`W`/`C`/`H`/`X`). Full command table,
response formats, and the non-obvious contracts (resync-after-desync, throttled
calibration upload, firmware-is-source-of-truth) are in
[`docs/reference.md`](docs/reference.md).

## Calibration status

The Python conversion pipeline (spectral responsivity → volts → physical units,
daylight/lux weighting) is built and unit-tested, but the absolute scale is still
a datasheet-derived estimate (~±20%), not a measured calibration, and the bundled
spectral curve is Vishay's typical BPW34 data, not a monochromator sweep of this
specific sensor. `Calibration.provenance` always tells you which you have
(`'measured'` vs `'datasheet-typical'`) — see
[`docs/reference.md`](docs/reference.md). The Rust driver intentionally supports
only raw values and volts today.

## Development

```bash
uv run tests/test_calibration.py   # pure unit tests (no hardware)
uv run tests/test_read.py          # smoke test: connect, read 10 samples (needs hardware)
uv run ruff check                  # lint
cd rust && cargo test              # Rust driver + sim tests (no hardware)
just compile                       # firmware compile only
```

`tests/` uses a hand-rolled runner (functions named `test_*`), not pytest.
See [`AGENTS.md`](AGENTS.md) for architecture and the invariants that are easy
to break (voltage as the canonical unit, dark-offset ordering, thread-safety).

## Roadmap

See [`TODO.md`](TODO.md): absolute calibration reference measurement, thermal
drift characterization, and a higher sample rate path (currently ~330
reads/s, single-shot ADC mode).

## License

No repository-wide license file yet. The Rust crate (`rust/`) opts into
`MIT OR Apache-2.0` via its `Cargo.toml`.
