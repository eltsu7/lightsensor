# lightmeter

LightSensor v3 is a USB light-measurement instrument built around a BPW34
photodiode, OPA323 transimpedance amplifier, ADS1220 24-bit ADC, TMP117 board
temperature sensor, and bare RP2040. The RP2040 streams raw ADC codes or
canonical differential volts over native USB CDC. A Python driver and Tkinter
GUI provide control, plotting, dark zeroing, and CSV recording.

The ordered-board electrical contract and measured bring-up results are in
[`docs/v3/hardware.md`](docs/v3/hardware.md). Absolute optical calibration and
temperature compensation are not implemented yet.

## Features

- Three measured acquisition profiles: filtered 20 SPS, interactive 330 SPS,
  and maximum-rate 2 kSPS.
- Continuous and finite binary streams with sequence numbers, RP2040 monotonic
  timestamps, gain, clipping/autogain status, and temperature in every sample.
- PGA gains 1× through 128×. Firmware-authoritative autogain keeps voltage
  streams near 40–85% of ADC full scale when hardware range permits.
- Canonical signed voltage output. Voltage normalization is gain-independent;
  the active dark correction is applied last and never affects autogain or
  clipping decisions.
- Temporary session zero and an explicitly saved per-device flash baseline.
  Persistent replacement is atomic and CRC32-checked.
- Versioned protocol-3 COBS frames with payload length, request correlation,
  CRC32, bounded frame size, and explicit stream start/stop/error events.
- The GUI keeps serial ownership on one worker thread and writes one versioned
  CSV per recorded stream.

## Hardware

| Part | Role |
|---|---|
| RP2040 | Acquisition, USB CDC, protocol, and persistence |
| W25Q32JV | 4 MiB QSPI firmware/LittleFS flash and device identity |
| ADS1220 | 24-bit differential ADC over SPI0 with dedicated DRDY |
| TMP117 | Board temperature over I2C at `0x48` |
| OPA323 + 2 MΩ / 3.3 pF | Fixed-gain photodiode transimpedance stage |

The ADC measures `VOUT - 1.65 V BIAS` on `AIN0 - AIN1` using its internal
2.048 V reference. ADC SPI is mode 1 at 1 MHz; TMP117 I2C runs at 400 kHz.
See [`docs/v3/hardware.md`](docs/v3/hardware.md) before probing or changing pin
assignments.

## Layout

```text
firmware/lightsensor/lightsensor.ino  RP2040 production firmware
lightmeter/sensor.py                  Python protocol-3 driver
lightmeter/gui.py                     Tkinter streaming GUI and CSV recorder
docs/reference.md                     Protocol and host API reference
docs/v3/hardware.md                   Ordered-board electrical contract/results
tests/test_protocol.py                Hardware-free protocol contracts
tests/test_read.py                    Connected ten-sample smoke test
```

The final protocol-2 implementation is preserved by the immutable `v2-final`
Git tag. It is not maintained in the active v3 tree.

## Quick start

Install the Arduino-Pico core once, then build or flash:

```bash
just setup
just compile
just flash                       # defaults to /dev/ttyACM0
SENSOR_PORT=/dev/ttyACM1 just flash
```

Install/run Python with `uv`:

```bash
uv sync
uv run python -m lightmeter.gui  # or installed command: lightmeter
uv run tests/test_protocol.py
uv run tests/test_read.py        # connected v3 board required
uv run ruff check
```

The firmware USB descriptor is `LightSensor v3`, VID:PID `2e8a:f00a`. The flash
UID is exposed as the 16-hex-digit USB serial number and protocol device ID, so
`LightSensor(device_id="...")` can select one of several connected boards.

## Python example

```python
from lightmeter import LightSensor, StreamConfig, StreamMode, StreamStopped, VoltageSample

with LightSensor() as sensor:
    print(sensor.info)
    print(sensor.profiles)
    sensor.start_stream(
        StreamConfig(
            mode=StreamMode.FINITE,
            profile_id=1,
            autogain=True,
            window=8,
            output_count=100,
        )
    )
    while True:
        event = sensor.read_event(5.0)
        if isinstance(event, VoltageSample):
            print(event.value, event.gain, event.temperature_c, event.status)
        elif isinstance(event, StreamStopped):
            break
```

`LightSensor.zero(n)` acquires `n` uncorrected finite voltage samples at gain
128×, installs their mean as the session dark correction, and returns the mean.
`save_baseline()` is the explicit flash write. `clear_zero()` restores the
persisted device baseline (or zero when no record exists).

## Recording format

While recording is enabled, every stream gets a separate CSV under
`recordings/`. A `stream_start` row records the complete effective profile,
registers, rate, gain/autogain, averaging window, timestamps, and dark source.
Every `sample` row records host UTC, RP2040 time, sequence, volts/raw code, gain,
status, and temperature. Applying settings or changing dark correction causes
an atomic stream replacement and therefore starts a new CSV.

## Current limits

- Absolute optical responsivity is uncalibrated; there is no W/m² or lux output.
- The measured positive TIA clipping threshold is 1.64 V differential. Negative
  TIA clipping remains unmeasured, so firmware does not assert that flag.
- Warm-up drift is real but not modeled. TMP117 values are recorded as evidence,
  not used for compensation.
- Firmware updates use RP2040 ROM UF2 (`just flash`); there is no application OTA
  or rollback slot.
- The Rust protocol-2 driver was removed from the active tree. A protocol-3 Rust
  driver remains deferred.

See [`TODO.md`](TODO.md) for calibration and remaining characterization work.
