# V3 software architecture record

## Status

Approved and implemented on 2026-08-03 for the ordered RP2040/ADS1220 board.
This replaces the former decision worksheet. Electrical facts and measured
bring-up results remain authoritative in [`hardware.md`](hardware.md); the active
wire/API reference is [`../reference.md`](../reference.md).

The final protocol-2 implementation is preserved by the immutable `v2-final` Git
tag. The active tree is v3-only; no dual-generation compatibility layer remains.

## Delivered architecture

### Platform and source layout

- Arduino-Pico is the firmware platform.
- `firmware/lightsensor/lightsensor.ino` is the production sketch.
- RP2040 uses 133 MHz, generic W25Q32 boot2, native Pico SDK USB CDC, and the
  4 MiB flash option split into 2 MiB sketch / 2 MiB LittleFS.
- ROM UF2 is the supported update/recovery path. SWD remains available for
  development. There is no application image slot, OTA updater, or rollback.

### USB and protocol

- One CDC interface carries binary request/reply and streaming traffic.
- USB identity: Arduino-Pico/Raspberry Pi generic VID:PID `2e8a:f00a`, product
  `LightSensor v3`, manufacturer `LightSensor`, and flash UID as USB serial.
- Protocol 3 frames use COBS plus a zero delimiter. Decoded frames contain
  version, message type, little-endian payload length, payload, and CRC32.
- Maximum decoded frames are 256 bytes and COBS bodies are 258 bytes. Larger
  future objects require an explicitly designed chunked transfer; none exists.
- Every host command has a nonzero `u32` request ID. Streams have a separate
  `u64` start token and `u32` sample sequence.
- Start emits the complete effective configuration and waits up to one second for
  `ACK_STREAM`; firmware emits no samples before acknowledgement.
- Any command, frame, ADC, temperature, timing, storage, or overflow error stops
  acquisition and emits a terminal typed error.

### Measurement representation

- Streams select raw signed `i32` ADC codes or canonical signed `f32` volts.
- Every sample includes sequence, DRDY-time RP2040 microseconds, actual gain,
  status, value, and TMP117 temperature.
- Voltage is normalized per conversion before sliding averaging, so gain changes
  do not change its scale. Active dark correction is subtracted last.
- Raw streams are gain-dependent and cannot autogain.
- Absolute optical calibration, spectral conversion, W/m², and lux are absent.

### Acquisition profiles and autogain

Production profiles are measured rather than inferred from the ADS1220 nominal
oscillator:

| ID | Profile | Measured rate | Purpose |
|---:|---|---:|---|
| 0 | `normal_20_50_60` | 19.958 SPS | precision and simultaneous 50/60 Hz rejection |
| 1 | `normal_330` | 327.876 SPS | interactive GUI |
| 2 | `turbo_2000` | 1949.3 SPS | maximum-rate capture |

All profiles permit PGA gains 1×–128×. The averaging window is a firmware sliding
mean of 1–1024 conversions. Characterization found no required post-change
conversion discard for these exact settings.

Autogain is firmware-authoritative and voltage-stream-only. It changes one gain
step per conversion, targets 40–85% of absolute full scale, and reports under- or
over-range when it reaches a gain rail. A clipped transition conversion is not
added to the averaging window.

ADC positive and negative full-scale flags remain separate from analog clipping.
The measured positive TIA threshold is 1.64 V differential. Negative TIA clipping
is deferred and therefore unflagged.

### Concurrency and overflow

- One RP2040 core runs control, USB, SPI reads, filtering, temperature, and flash.
- The DRDY ISR only increments a bounded event count and captures the timestamp.
- More than one pending DRDY event is a terminal overflow; no sample is silently
  dropped or overwritten.
- Measured USB CDC delivery sustained every conversion at the 1949.3 SPS profile.

### Time and temperature

- Host `TIME_SYNC` supplies UTC microseconds and captures RP2040 monotonic time.
- Every stream header records both clock origins; every sample records device
  time. Reconnect performs a new synchronization and remains stopped.
- TMP117 runs at 400 kHz I2C and refreshes every 100 ms. Every sample carries the
  latest successful value. A failed refresh terminates the stream.
- Temperature is recorded evidence only; no warm-up enforcement or compensation
  model is approved.

### Dark correction and storage

- A missing record is normal and means device baseline `0.0 V`.
- A session baseline overrides the persisted baseline. Clearing session zero
  restores the device value. Saving promotes the session value atomically and
  removes the redundant session layer.
- Values are finite `f32` volts bounded to ±0.25 V.
- The LittleFS record is kind/schema-versioned and CRC32-protected. Replacement
  uses a flushed temporary file and rename.
- Mount or record-integrity failure is an explicit storage-fault state; firmware
  never auto-formats. Repair requires `RESET_STORAGE` with exact device ID plus
  the ASCII confirmation `ERASE`.

### Host delivery

- Python package major version 3 implements discovery, framing, handshake,
  profiles, commands, typed stream events, session zero, persistence, and reset.
- `LightSensor.read_event(timeout)` is the blocking synchronous stream API. The
  driver owns an `RLock`; callers choose their own worker policy.
- The Tk GUI retains one `SensorSampler` daemon thread as sole serial owner.
  Profile, gain, autogain, and averaging changes apply atomically.
- Recording uses one CSV per stream. Each file begins with complete effective
  stream metadata and stores every sample's UTC, device time, sequence, volts/raw
  value, gain, status, and temperature.
- Rust support and calibration tools are deferred; the old protocol-2 Rust crate
  is not part of the active v3 tree.

## Verification completed

- RP2040 ROM boot, UF2 upload, native CDC identity, W25Q32 UID/capacity, and
  2 MiB LittleFS mount/read/write/rename were exercised on the assembled board.
- ADS1220 register readback, DRDY timing, all gains, all production profiles,
  filtered 20 SPS behavior, autogain, finite/continuous streams, acknowledgement
  timeout, stream replacement, and terminal errors were exercised.
- TMP117 identity and periodic samples were exercised at 400 kHz I2C.
- A 4096-sample 2 kSPS USB diagnostic and a 1000-sample production stream
  completed with contiguous sequences and no missed DRDY events.
- Atomic session-dark save, reboot persistence, and confirmed storage reset were
  exercised.
- The Python driver acquired raw and voltage streams, autogain, zero, save/reset,
  and reconnect metadata against hardware. The GUI worker acquired, applied a
  new profile, recorded separate stream CSVs, zeroed/cleared, paused, and resumed
  against hardware.

## Deferred evidence

Absolute optical calibration, negative TIA clipping, controlled warm-up drift,
reference-grade per-gain coefficients, and a temperature-compensation model
remain unresolved by design. They must be measured before becoming production
constants; see [`../../TODO.md`](../../TODO.md).
