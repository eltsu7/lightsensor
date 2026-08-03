# V3 hardware contract

## Status and authority

The PD Amp v3.0 PCB was ordered on 2026-07-26. The first assembled board passed
digital bring-up on 2026-08-03. This document contains the electrical facts and
measured behavior needed by firmware and host software; it does not define the
production protocol.

The final EasyEDA project and fabrication netlist remain authoritative if this
document disagrees with a manufactured board:

| Artifact | SHA-256 |
| --- | --- |
| `ProPrj_Photodiode Amplifier_2026-07-24.epro2` | `63a140c3b19b3098f17f92fb0aaa239fae5c9e5b29dba41fd451a3ac3c005655` |
| `BOM_PD Amp v3.0_V3.0_2026-07-24.xlsx` | `25d9ddce2905073e3bc72bfb3ae46f28ec9ccdb69446f6cee7464dcc7de8d0d4` |
| `Gerber_V3.0_2026-07-24.zip` | `cdc0624283d224a493655d79a84aa2a233447e0fe576cbbb03e77a61a3c92e2f` |
| `PCB_V3.0_2026-07-24.zip` | `7f4f68970694b9b3b4a9c69b6ad3b08306900666cea32443a7b2d61b9b7f4738` |
| `InteractiveBOM_V3.0_2026-7-24.html` | `482ce9402cf5a3bc1a42691c2353f061eb3946d16e7ac7882c589f448248d7d6` |

The ordered artifacts currently live in `/home/eeli/Downloads/pd amd v3/`.

## Architecture

```text
VBPW34S -> OPA323, 2 Mohm TIA -> ADS1220 -> SPI0 -> RP2040 -> native USB-C
                  ^                  |          |
                1.65 V               |          +-> W25Q32JV 4 MiB QSPI
                                     +-> DRDY
TMP117 board temperature -> I2C1 ----------------^
```

The board is USB-powered sensor hardware only. It has no motor interface,
general-purpose header, or status LED.

| Part | Device | Software-visible role |
| --- | --- | --- |
| `PD1` | Vishay VBPW34S | Silicon photodiode |
| `U1` | OPA323 | Fixed-gain transimpedance amplifier |
| `U3` | RP2040 | MCU, USB device, SPI/I2C controller |
| `U4` | ADS1220 | 24-bit differential ADC with PGA |
| `U5` | W25Q32JVUUIQ | 4 MiB boot and data flash |
| `U7` | TMP117 | Board-area temperature sensor |

## RP2040 interface map

These assignments come from the ordered fabrication netlist:

| RP2040 signal | Connected device | Electrical detail |
| --- | --- | --- |
| GPIO0 / SPI0 RX | ADS1220 `DOUT/DRDY` | 47 ohm series resistor |
| GPIO1 / SPI0 CSn | ADS1220 active-low `CS` | 47 ohm series resistor |
| GPIO2 / SPI0 SCK | ADS1220 `SCLK` | 47 ohm series resistor |
| GPIO3 / SPI0 TX | ADS1220 `DIN` | 47 ohm series resistor |
| GPIO4 | ADS1220 dedicated `DRDY` | Active low, 47 ohm series resistor |
| GPIO10 / I2C1 SDA | TMP117 `SDA` | 5.1 kohm pull-up to 3.3 V |
| GPIO11 / I2C1 SCL | TMP117 `SCL` | 5.1 kohm pull-up to 3.3 V |
| `SWCLK`, `SWDIO` | Labelled test pads | 3.3 V and GND pads are nearby |
| `RUN` | Reset button | 10 kohm pull-up; button shorts to GND |
| USB D-/D+ | USB-C through USBLC6 | 27 ohm series resistors |
| Dedicated QSPI pins | W25Q32 | Not available as GPIO |

GPIO5-GPIO9 and GPIO12-GPIO29 are unconnected and are not routed to a header.
The RP2040 uses the fitted 12 MHz crystal.

## Boot, USB, and flash

- Hold `BOOT`, press and release `RESET`, then release `BOOT` to enter the RP2040
  ROM UF2 bootloader.
- Native USB 2.0 Full Speed is wired directly to the RP2040; there is no UART
  bridge. Production firmware exposes one CDC interface carrying protocol 3.
- The labelled test pads expose SWD but no dedicated `RUN` pad.
- The fitted W25Q32 reports JEDEC ID `EF4016` and 4 MiB capacity. Firmware builds
  must declare 4 MiB.
- The first board's flash unique ID is `DE657814573A0C29`; do not treat it as a
  model-wide constant.
- The production build uses the proven 2 MiB sketch / 2 MiB LittleFS split.

## ADS1220 contract

### Connections

| ADS1220 signal | Connection |
| --- | --- |
| `AIN0/REFP1` | OPA323 `VOUT` |
| `AIN1` | `1.65V BIAS` |
| `AIN2`, `AIN3/REFN1` | Unconnected |
| `REFP0`, `REFN0` | Unconnected |
| `CLK` | GND; use the internal oscillator |
| `AVDD`, `DVDD` | 3.3 V |
| `AVSS`, `DGND`, exposed pad | GND |
| `DRDY` | RP2040 GPIO4 |
| `DOUT/DRDY`, `DIN`, `SCLK`, `CS` | RP2040 SPI0, GPIO0-GPIO3 |

### Required behavior

- Configure SPI mode 1. Bring-up was stable at 1 MHz SCLK.
- Measure signed `AIN0 - AIN1 = VOUT - VBIAS`; do not hard-code 1.65 V as zero.
- Select the internal 2.048 V reference. External reference 0 is invalid because
  its pins are unconnected.
- Use the dedicated active-low DRDY signal for conversion timing.
- PGA gains are 1, 2, 4, 8, 16, 32, 64, and 128.
- For a signed 24-bit code, nominal differential voltage is:

```text
Vdiff = raw * 2.048 V / (2^23 * gain)
```

- PGA common-mode operation was verified at gains 1-128 on the first board.
- The first conversion after adjacent gain changes was within steady-state noise
  at filtered 20, normal 45, normal 330, and turbo 2000 SPS. No post-change
  conversion discard is required for those production settings.
- Keep analog/TIA clipping distinct from positive or negative ADC/PGA clipping.
  Positive TIA clipping was measured; negative TIA clipping remains unmeasured.

The tested baseline registers are `00,04,00,00`: AIN0-AIN1, gain 1, normal
20 SPS, continuous conversion, internal reference, and dedicated DRDY. Reset
values `00,00,00,00` and baseline configuration write/readback were verified.

### Measured conversion rates

Each point used 128 continuous conversions and lost no DRDY events:

| Mode | Nominal rates, SPS | Observed rates, SPS |
| --- | --- | --- |
| Normal | 20, 45, 90, 175, 330, 600, 1000 | 19.957, 44.849, 88.593, 172.696, 327.879, 592.633, 986.232 |
| Turbo | 40, 90, 180, 350, 660, 1200, 2000 | 39.449, 88.651, 175.120, 341.366, 648.108, 1171.435, 1949.408 |

The internal oscillator ran 0.2-2.7% below the nominal table values. Use measured
timestamps rather than deriving elapsed time from the nominal rate.

At the fastest setting, USB CDC delivered all 4096 diagnostic text samples at
1949.6 samples/s with contiguous sequence numbers and no missed conversions.
Observed DRDY intervals were 505-522 us. A second fastest-rate run included
400 kHz TMP117 reads every 100 ms: all 4096 conversions arrived at 1949.455
samples/s, with 22 temperature reads and a worst I2C transaction of 172 us.

## Analog front end

- VBPW34S anode is grounded; its cathode connects to the OPA323 summing node.
- OPA323 `IN+` and ADS1220 `AIN1` share `1.65V BIAS`.
- The feedback network is fixed at 2 Mohm, 0.1%, in parallel with 3.3 pF C0G.
- The nominal uncalibrated transfer is `VOUT - VBIAS = Iphoto * 2 Mohm`.
- Increasing illumination should produce a positive differential ADC result.
- The 1.65 V node is a 10 kohm / 10 kohm divider with 10 uF bypassing. Its
  actual voltage is measured differentially and must not be a firmware constant.

Controlled illumination produced positive differential readings. Gains 1-32
agreed within 0.7% at approximately 57.5 mV while sequential source drift was
present; gains 1-4 agreed within 0.035% at approximately 329.8 mV. Higher gains
then reached exact positive ADC full scale.

At stronger illumination, gain 1 plateaued at `+1.649637 V` differential with
only `19.7 uV RMS` variation while remaining below its `+2.048 V` ADC range.
This is measured positive TIA/output-rail clipping. Firmware may conservatively
flag positive TIA clipping at `+1.64 V`. No negative TIA threshold was measured.

Two covered full-rate/gain matrices and one internally shorted matrix lost no
DRDY events or conversions. Representative covered input-referred RMS noise:

| Setting | Covered RMS noise |
| --- | --- |
| Normal 20 SPS, unfiltered | median 5.68 uV |
| Normal 45 SPS | median 4.75 uV |
| Normal 330 SPS | median 32.4 uV |
| Turbo 2000 SPS | median 35.8 uV |
| Normal 20 SPS, simultaneous 50/60 Hz rejection | 1.2-5.4 uV by gain |

The stable filtered-20 repeat measured `5.39 uV` at gain 1, `3.13 uV` at gain 2,
and `1.2-2.1 uV` at gains 4-128. Internally shorted offset ranged approximately
`+6.8 uV` to `-2.4 uV` across gains.

Covered differential offset moved from approximately `1.09 mV` to `0.33 mV`
while TMP117 moved from `30.84 C` to `28.72 C`. This proves material drift but
does not establish a compensation coefficient because temperature and elapsed
time were not independently controlled.

The VBPW34S and changed ADC path still require absolute optical calibration.
Manufacturer BPW34-family curves are only datasheet-typical guidance and are not
a measured v3 calibration.

## TMP117 contract

- I2C1 on GPIO10/GPIO11; 100 kHz and production 400 kHz were verified.
- `ADD0 = GND` selects 7-bit address `0x48`.
- `ALERT` is unconnected; firmware must poll over I2C.
- Device ID register is `0x0117`; the first board returned `0x0117`.
- Temperature data is signed 16-bit with `0.0078125 degrees C/LSB`.
- First-board readings were plausible, approximately 27.2-28.4 degrees C during
  bring-up.

TMP117 measures the board/analog area, not photodiode junction temperature. No
temperature-compensation model is approved.

## Remaining hardware characterization

Do not turn these into production constants before measurement:

- 3.3 V, 1.1 V, and 1.65 V rail values and ripple;
- SWD access;
- negative-direction TIA clipping;
- a controlled temperature-versus-offset compensation model;
- measured optical and absolute calibration.
