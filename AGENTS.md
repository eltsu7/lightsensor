# LightSensor — Project Summary

## Overview
Precision calibrated light sensor using a custom PCB with OPA323 op-amp and ADS1115 16-bit ADC over I2C. Includes an ESP8266-based reader with a live debug GUI.

## Hardware
- **MCU:** ESP8266 (NodeMCU v2)
- **ADC:** ADS1115 — connected via I2C (SDA→D2, SCL→D1, ADDR→GND, VDD→3.3V)
- **Sensor:** Custom PCB using OPA323 op-amp, powered from 3.3V
- **OPA323 saturation:** ~3.266V (~34 mV below 3.3V rail); both sensor and ADC saturation reported per reading
- **ADC absolute max input:** VDD + 0.3V = 3.6V — do not exceed regardless of gain setting

## Files
| File | Description |
|------|-------------|
| `lightsensor/lightsensor.ino` | Arduino sketch — device interface over serial |
| `lightsensor.py` | Sensor driver — `LightSensor` class, `Reading` dataclass, `best_gain()`, autogain |
| `main.py` | Debug GUI — Tkinter, threaded sampler, live plot |
| `justfile` | `just compile`, `just upload`, `just flash` |
| `docs/` | ADS1115, OPA323, ESP8266 datasheets |

## Usage

**Flash firmware:**
```bash
just flash
```

**Run debug GUI:**
```bash
uv run main.py                 # auto-detects port
uv run main.py --port COM5     # or specify explicitly
```

**Use driver in code:**
```python
from lightsensor import LightSensor

with LightSensor() as sensor:      # port auto-detected if omitted
    sensor.set_gain(2)             # ±2.048V
    reading = sensor.read()        # Reading(value, sensor_sat, adc_sat)
    print(reading.value)           # light level as % of ADC full-scale
    print(reading.sensor_sat)      # op-amp at rail
    print(reading.adc_sat)         # ADC raw hit 32767

    # autogain
    sensor.autogain_oneshot(100)   # sample 100 readings, set best gain
    sensor.autogain = True         # continuous autogain inside read()
```

## Driver API (`lightsensor.py`)

### Constants
| Name | Description |
|------|-------------|
| `GAIN_LABELS` | Display strings for each gain index (`["±6.144V", …]`) |
| `GAIN_VOLTAGES` | Full-scale voltages (`[6.144, 4.096, …, 0.256]`) |
| `DEFAULT_GAIN` | `1` (±4.096V) |
| `SATURATION_VOLTAGE` | `3.2` V — OPA323 output ceiling with 3.3V supply |

### `best_gain(max_voltage, headroom=0.85)`
Pure function. Returns the highest gain index that keeps `max_voltage` below the saturation threshold with the given headroom factor.

### `Reading` dataclass
| Field | Type | Description |
|-------|------|-------------|
| `value` | `float` | Light level as % of ADC full-scale (0–100) |
| `sensor_sat` | `bool` | Op-amp output near supply rail |
| `adc_sat` | `bool` | ADC raw hit 32767 |

Sensor and ADC saturation are mutually exclusive with this hardware: sensor_sat only occurs at low gain settings (full-scale > 3.266V), adc_sat only at high gain settings (full-scale < 3.266V).

### `LightSensor`
| Member | Description |
|--------|-------------|
| `gain` | Currently applied gain index (locally tracked) |
| `autogain` | Enable continuous autogain inside `read()` |
| `autogain_interval` | Gain evaluation interval in seconds (default 0.25) |
| `autogain_window` | History window for evaluation in seconds (default 0.5) |
| `read()` | Returns `Reading` or `None` |
| `set_gain(index)` | Sets gain, updates `self.gain`, returns `True` on success |
| `get_gain()` | Queries current gain index from device |
| `autogain_oneshot(n=100)` | Collect n samples, apply best gain, return gain index |

## Device Interface (Serial)

Commands sent over serial at 9600 baud:

| Command | Description | Response |
|---------|-------------|----------|
| `r` | Read ADC | `raw,sensor_sat,adc_sat\n` — e.g. `15031,0,0` |
| `g<n>` | Set gain index 0–5 | `ok` or `err` |
| `G` | Query current gain index | integer + newline |

`raw` is the signed 16-bit ADC value (0–32767). `sensor_sat` and `adc_sat` are 0 or 1.

### Gain index mapping
| Index | Range | Saturation limit |
|-------|-------|-----------------|
| 0 | ±6.144V | sensor (3.266V) |
| 1 | ±4.096V (default) | sensor (3.266V) |
| 2 | ±2.048V | ADC (32767) |
| 3 | ±1.024V | ADC (32767) |
| 4 | ±0.512V | ADC (32767) |
| 5 | ±0.256V | ADC (32767) |

## Debug GUI (`main.py`)

Threaded sampler reads as fast as the device allows (decoupled from ~33 fps display). Values are stored as actual voltage (V) so data is gain-independent and preserved across gain changes.

### Controls
| Control | Description |
|---------|-------------|
| Auto Y-scale | Autoscale Y axis to visible window |
| Window average | Dashed line at mean of visible window |
| Line fit | Linear regression over visible window |
| Noise band | ±σ shaded band; legend shows σ, relative σ, peak-to-peak |
| Absolute scale | Y axis in V (default); unchecked shows % of current gain range |
| Gain − / combobox / + | Manual gain selection; stops continuous autogain |
| One-shot gain | Collect 100 samples, apply best gain |
| Auto gain ● | Continuous autogain; ● indicates active |
| Scan interval | Target ms between samples (0 = as fast as possible) |
| Stop / Start | Pause and resume sampling |
| Clear | Clear the plot buffer |

Saturation reference line shown in red dashes at the OPA323 ceiling. Status bar shows `⚠ SENSOR SAT` or `⚠ ADC SAT` when the latest reading is saturated.

## Arduino CLI setup
```bash
arduino-cli config add board_manager.additional_urls https://arduino.esp8266.com/stable/package_esp8266com_index.json
arduino-cli core update-index
arduino-cli core install esp8266:esp8266
arduino-cli lib install "Adafruit ADS1X15"
```

## Notes
- Arduino sketch filename must match its directory name (`lightsensor/lightsensor.ino`)
- ESP8266 needs to be in `dialout` group: `sudo usermod -aG dialout $USER`
- Serial baud: 9600
- On Linux, do NOT set RTS/DTR before opening the port — any transition triggers the NodeMCU auto-reset circuit and causes a full USB disconnect/reconnect. On Windows, deassert both before open to avoid WriteFile error 22.
- ADS1115 input hard-limited to VDD+0.3V = 3.6V; do not apply 5V signals or power the sensor from a higher supply without checking I2C pull-up voltage
- Slow downward drift observed — likely thermal warmup of ADC reference or op-amp offset drift
