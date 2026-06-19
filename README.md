# lightmeter

Driver and debug GUI for a calibrated light sensor built on an ESP32-C3 SuperMini
with an OPA323 op-amp and ADS1115 16-bit ADC (I2C).

> Work in progress — the firmware and Python API are still evolving, and the
> absolute calibration numbers are placeholders pending a reference measurement.

## Install

```bash
uv sync          # or: pip install -e .
```

## Use

```bash
just flash       # build + upload firmware (port auto-detected)
uv run main.py   # debug GUI  (or: lightmeter)
```

```python
from lightsensor import LightSensor

with LightSensor() as sensor:        # port auto-detected
    print(sensor.info)               # device identity / firmware
    sensor.set_gain(2)
    reading = sensor.read()          # Reading(value, sensor_sat, adc_sat)
    print(sensor.reading_voltage(reading))
```

See [`AGENTS.md`](AGENTS.md) for the full hardware notes, serial protocol, and
driver API.
