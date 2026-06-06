# LightSensor — Project Summary

## Overview
Precision calibrated light sensor using a custom PCB with OPA323 op-amp and ADS1115 16-bit ADC over I2C. Includes an ESP8266-based reader with a live debug plotter.

## Hardware
- **MCU:** ESP8266 (NodeMCU v2)
- **ADC:** ADS1115 — connected via I2C (SDA→D2, SCL→D1, ADDR→GND, VDD→3.3V)
- **Sensor:** Custom PCB using OPA323 op-amp, powered from 3.3V
- **ADC gain:** `GAIN_ONE` (±4.096V) default — sensor (OPA323) saturates at ~3.266V (~34 mV below 3.3V rail)

## Files
| File | Description |
|------|-------------|
| `lightsensor/lightsensor.ino` | Arduino sketch — responds to `r` over serial with raw ADS1115 reading |
| `lightsensor.py` | `LightSensor` class — wraps serial communication (auto-detects CP210x port), returns values as % (0–100); `set_gain`/`get_gain` for ADC gain |
| `main.py` | Tkinter GUI with threaded sampler, live plot, gain selector, autoscale/average/line-fit overlays, adjustable scan interval |
| `justfile` | `just compile`, `just upload`, `just flash` |

## Usage

**Flash firmware:**
```bash
just flash
```

**Run plotter:**
```bash
uv run main.py                 # auto-detects port
uv run main.py --port COM5     # or specify explicitly
```

**Use sensor in code:**
```python
from lightsensor import LightSensor

with LightSensor() as sensor:  # port auto-detected if omitted
    sensor.set_gain(2)         # ±2.048V
    print(sensor.read())       # returns float % or None
```

## Arduino CLI setup
```bash
arduino-cli config add board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli lib install "Adafruit ADS1X15"
```

## Device Interface (Serial)

Commands sent over serial at 9600 baud:

| Command | Description | Response |
|---------|-------------|----------|
| `r` | Read raw ADC value (0–32767) | integer + newline |
| `g<n>` | Set gain index 0–5 | `ok` or `err` |
| `G` | Query current gain index | integer + newline |

Gain index mapping:

| Index | Range |
|-------|-------|
| 0 | ±6.144V |
| 1 | ±4.096V (default) |
| 2 | ±2.048V |
| 3 | ±1.024V |
| 4 | ±0.512V |
| 5 | ±0.256V |

## Notes
- Arduino sketch filename must match its directory name (`lightsensor/lightsensor.ino`)
- ESP8266 needs to be in `dialout` group: `sudo usermod -aG dialout $USER`
- Serial baud: 9600. `time.sleep(2)` on connect gives ESP8266 time to reboot
- ADS1115 input hard-limited to VDD+0.3V = 3.3V; sensor powered at 3.3V to stay within range
- Slow downward drift observed — likely thermal warmup of ADC reference or op-amp offset drift
- `blit=False` required for autoscaling Y axis labels to update correctly
