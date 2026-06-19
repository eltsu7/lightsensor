# LightSensor — TODO

The v2 hardware migration (ESP32-C3 SuperMini, ADS1115 integrated on the sensor
PCB, native USB-CDC) is done and shipped — see CLAUDE.md for the current design.
What's left:

## Measurement quality
- [ ] **Absolute calibration** — take one reference measurement against a known
      source/meter to set `scale_factor`, and a monochromator sweep to replace
      `data/calibration_dummy.csv` with the real `R(λ)`. The conversion pipeline
      is already built and unit-tested; only the numbers are missing.
- [ ] **Thermal drift** — slow downward drift observed (ADC reference / op-amp
      offset warmup). Characterize it; consider a temperature reading or
      compensation, or at least a documented warmup time.

## Higher sample rate (hardware-gated)
- [ ] Free the ADS1115 ALERT/RDY pin (currently tied to ground) onto a spare
      GPIO, then use continuous-conversion mode + data-ready interrupt to get
      past the ~330 reads/s single-shot ceiling. Note: discard 1–2 samples after
      each gain change.

## Optional / later
- [ ] Lux / illuminance output (photopic V(λ) weighting on top of the existing
      irradiance pipeline; needs the source spectrum).
- [ ] Selectable ADS1115 ADDR (solder-jumper) so multiple sensors can share one
      I2C bus.
- [ ] Firmware update path for end users (currently `just flash` + arduino-cli,
      a developer workflow).
