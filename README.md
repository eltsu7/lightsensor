# lightmeter

Driver, Rust port, and debug GUI for a calibrated light sensor built on an
ESP32-C3 SuperMini with an OPA323 op-amp and ADS1115 16-bit ADC (I2C).

> Work in progress — the firmware and Python API are still evolving, and the
> absolute calibration numbers are placeholders pending a reference measurement.

## Install

```bash
uv sync          # or: pip install -e .
```

## Use

```bash
just flash                  # build + upload firmware (port auto-detected)
uv run python -m lightmeter.gui   # debug GUI  (or just: lightmeter)
```

```python
from lightmeter import LightSensor

with LightSensor() as sensor:        # port auto-detected
    print(sensor.info)               # device identity / firmware
    sensor.set_autogain(True)        # firmware autoexposes each read()
    reading = sensor.read()          # Reading(value, sensor_sat, adc_sat)
    print(sensor.gain, sensor.reading_voltage(reading))
```

A Rust port (`rust/`, crate `lightmeter`) speaks the same protocol for the
[pointcamera](https://github.com/eltsu7/pointcamera) turret rig.

See [`docs/reference.md`](docs/reference.md) for the serial protocol, driver API,
and calibration model.
