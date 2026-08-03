# LightSensor — TODO

The RP2040/ADS1220 v3 firmware, protocol-3 Python driver, Tk GUI, dark
persistence, and recording path are implemented and hardware-smoke-tested. The
immutable `v2-final` Git tag preserves the retired protocol-2 system.

## Calibration and measurement quality

- [ ] **Absolute optical calibration** — characterize detector responsivity and
  establish a reference scale against traceable optical equipment before adding
  W/m² or lux conversion. Protocol 3 intentionally carries volts/raw evidence,
  not provisional physical units.
- [ ] **Warm-up drift** — measure dark differential voltage and TMP117 temperature
  from cold start under controlled conditions. Establish repeatable warm-up
  guidance before considering compensation.
- [ ] **Temperature compensation** — only after a measured, independently
  validated model exists. Keep raw temperature in recordings regardless.
- [ ] **Negative TIA clipping** — inject/observe the negative-direction analog
  limit. Firmware currently flags only the measured positive 1.64 V threshold
  plus both ADC/PGA full-scale codes.
- [ ] **Reference-grade gain/offset calibration** — characterize per-gain ADS1220
  and analog coefficients with a precision source if required by the optical
  calibration error budget.

## Product and tooling

- [ ] **Protocol-3 Rust driver** — implement only when a Rust consumer is needed;
  do not revive the protocol-2 API from the active tree.
- [ ] **Calibration capture/tooling** — define the versioned optical calibration
  model and provenance first, then add capture/import and physical-unit output.
- [ ] **End-user firmware update workflow** — current recovery is developer UF2
  (`just flash`) or manual BOOT/RESET. Decide whether application-assisted ROM
  reboot is worth adding; there is no OTA/rollback partition.
- [ ] **Long-duration soak** — record multi-hour turbo and filtered streams to
  validate USB stability, sequence continuity, temperature history, and atomic
  baseline persistence over repeated power cycles.
