# LightSensor v2 — TODO

## Goals
Move to a tighter, higher-integrity hardware design: ESP32 MCU, ADC integrated
on the sensor PCB, and a small purpose-chosen cable between sensor and MCU.

## 1. MCU: ESP32-C3 SuperMini (replace ESP8266 NodeMCU)
- [ ] Switch firmware target to ESP32 (ESP32-C3 SuperMini).
- [ ] Update `arduino-cli` core: install `esp32:esp32`, select correct board/FQBN.
- [ ] Update `justfile` (compile/upload/flash) for the new board + port.
- [ ] Re-check serial: native USB-CDC on C3 (no CH340/CP210x auto-reset quirks).
      Revisit the RTS/DTR notes in AGENTS.md — likely no longer needed.
- [ ] Confirm I2C pins (C3 default SDA=GPIO8, SCL=GPIO9; remap if needed).
- [ ] Bring a spare GPIO to the ADS1115 ALERT/RDY pin (for future continuous-mode
      data-ready interrupt; gets past the ~177 SPS polling ceiling).
- [ ] Verify 3.3V regulator on SuperMini can supply ADC + op-amp over the cable.

## 2. ADC directly on the sensor PCB
- [ ] Place ADS1115 next to the OPA323 — keep the analog path on-board, only
      digital I2C leaves the board.
- [ ] Add own I2C pull-ups on the sensor board: 2.2–3.3 kΩ (stronger than the
      breakout's defaults, for ~1 m cable margin). Do NOT duplicate pull-ups
      elsewhere on the bus.
- [ ] Decoupling at ADS1115 VDD: 100 nF + 1–10 µF bulk, close to the pin
      (far end of the power cable will be noisier).
- [ ] Bring ADDR pin to a solder-jumper / selectable, so multiple sensors can
      share one bus later.
- [ ] Keep ADS1115 input within abs-max VDD+0.3V = 3.6V (sensor still 3.3V-fed).
- [ ] Stay at 100 kHz I2C (400 kHz was marginal even with the breakout).

## 3. Cable (replace breakout + jumper wires)
- [ ] Target length ~1 m, small/thin, shielded.
- [ ] Preferred: thin 4-core shielded signal cable OR repurposed USB 2.0 cable
      (twisted D+/D− pair → SDA/SCL, VBUS → 3.3V, GND → GND, shield to GND at
      MCU end only).
- [ ] Pair/twist SDA & SCL with a ground; tie shield to GND at MCU end only.
- [ ] Connector: JST-PH 4-pin footprint at board edge (polarized, serviceable),
      OR direct-solder pads for minimum size.
- [ ] Add strain relief: two holes for a zip-tie around the cable jacket so
      tugs don't load the connector / solder joints.
- [ ] Optional: footprint for a PCA9615 differential I2C extender in case cable
      grows beyond ~1 m later.

## Reference: what the passives on an ADS1115 breakout do
(So I remember what to replicate on the integrated board.)

- **I2C pull-up resistors** (2, on SDA & SCL, tied to VDD, ~4.7k–10k on the
  breakout): I2C can only actively pull lines LOW; the pull-ups restore them to
  HIGH. Required or the bus doesn't work. Too-weak (high-value) pull-ups can't
  charge cable capacitance fast enough — this is why 400 kHz failed. On v2 use
  2.2–3.3 kΩ for ~1 m cable margin.
- **Decoupling / bypass capacitor** (~100 nF across VDD–GND next to the chip,
  sometimes + a bulk cap): supplies instant current and filters supply noise so
  the ADC's voltage reference stays clean. Critical for a 16-bit part — noise
  here shows up directly in readings.
- **ADDR pull-down/strap**: sets the I2C address. ADDR→GND = 0x48 (our case).
- **ALERT/RDY pull-up** (if ALERT is broken out): the pin is open-drain, so it
  needs a pull-up to read it.
- **Optional input series resistor / RC filter** on the analog inputs for
  protection/anti-alias. Many boards omit it.

All of the above must be replicated on the v2 PCB (covered in sections 2 & 3).

## 4. Firmware / software follow-ups
- [ ] Consider continuous-conversion mode + ALERT/RDY interrupt for higher
      sample rate (note: must discard 1–2 samples after each gain change).
- [ ] Update AGENTS.md hardware section for the new MCU/ADC/cable.
