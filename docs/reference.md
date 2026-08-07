# LightSensor v3 reference

This document defines protocol 3, the Python host API, and measurement semantics.
The firmware implementation in `firmware/lightsensor/lightsensor.ino` is the wire
source of truth. The ordered-board pinout is in [`v3/hardware.md`](v3/hardware.md).

## USB identity and transport

The production board exposes one native USB CDC port:

- VID:PID: `2e8a:f00a` (Arduino-Pico/Raspberry Pi generic IDs)
- manufacturer: `LightSensor`
- product: `LightSensor v3`
- serial number: uppercase 16-hex-digit W25Q32 unique ID
- nominal CDC baud: 115200 (USB CDC does not derive timing from this value)

The same flash UID is returned as the protocol device ID. Python discovery
requires this identity; it does not fall back to an arbitrary sole serial port.

## Frame format

Every message is a COBS-encoded frame followed by `0x00`. The decoded frame is:

| Field | Type | Meaning |
|---|---:|---|
| version | `u8` | `3` |
| message type | `u8` | table below |
| payload length | `u16` | payload bytes only |
| payload | bytes | type-specific schema |
| CRC32 | `u32` | standard reflected CRC32 over header and payload |

All integers and IEEE-754 `f32` values are little-endian. Maximum decoded frame
size is 256 bytes; maximum payload is 248 bytes; the COBS body is at most 258
bytes, followed by its delimiter. Hosts must validate version, exact decoded
length, and CRC before interpreting a payload.

Every host request begins with a nonzero `request_id:u32`. Direct responses echo
it. Unsolicited terminal events use request ID zero. One host command is active at
a time; the Python driver serializes them with an `RLock`.

## Message types

Field lists below omit the common frame header and CRC. Request payloads include
the leading request ID shown.

| ID | Name | Payload |
|---:|---|---|
| `01` | `HELLO` | `request_id:u32` |
| `02` | `TIME_SYNC` | `request_id:u32, utc_us:u64` |
| `03` | `PING` | `request_id:u32` |
| `04` | `GET_STATUS` | `request_id:u32` |
| `05` | `LIST_PROFILES` | `request_id:u32` |
| `10` | `START_STREAM` | `request_id:u32, format:u8, mode:u8, profile:u8, gain_index:u8, autogain:u8, window:u16, output_count:u32` |
| `11` | `ACK_STREAM` | `request_id:u32, stream_start_device_us:u64` |
| `12` | `STOP_STREAM` | `request_id:u32` |
| `20` | `SET_SESSION_DARK` | `request_id:u32, volts:f32` |
| `21` | `CLEAR_SESSION_DARK` | `request_id:u32` |
| `22` | `SAVE_SESSION_DARK` | `request_id:u32` |
| `30` | `RESET_STORAGE` | `request_id:u32, device_id:u64, ASCII "ERASE"` |
| `81` | `HELLO_REPLY` | `request_id:u32, fw_major:u8, fw_minor:u8, fw_patch:u8, hardware_major:u8, capabilities:u32, max_decoded_frame:u16, device_id:u64, time_synced:u8, storage_state:u8` |
| `82` | `TIME_SYNCED` | `request_id:u32, supplied_utc_us:u64, captured_device_us:u64` |
| `83` | `PONG` | `request_id:u32` |
| `84` | `STATUS` | `request_id:u32, state:u8, time_synced:u8, storage_state:u8, dark_source:u8, device_dark:f32, session_dark_or_NaN:f32, temperature_or_NaN:f32, device_us:u64, last_error:u16` |
| `85` | `PROFILES` | `request_id:u32, count:u8`, then repeated profile records |
| `90` | `STREAM_STARTED` | stream header described below |
| `91` | `SAMPLE_RAW` | `sequence:u32, device_us:u64, gain_index:u8, status:u8, mean_code:i32, temperature:f32` |
| `92` | `SAMPLE_VOLTS` | `sequence:u32, device_us:u64, gain_index:u8, status:u8, volts:f32, temperature:f32` |
| `93` | `STREAM_STOPPED` | `request_id:u32, stream_start_device_us:u64, delivered_outputs:u32, reason:u8` |
| `A0` | `OK` | `request_id:u32, operation_type:u8` |
| `FF` | `ERROR` | `request_id:u32, stream_start_device_us:u64, last_sequence_or_FFFFFFFF:u32, code:u16, detail:u16` |

A `PROFILES` record is `id:u8, name_length:u8, ASCII name, measured_millisps:u32,
registers:u8[4], allowed_gain_mask:u8, settling_discard_count:u16`.

### Stream header

`STREAM_STARTED` contains:

```text
request_id:u32
stream_start_device_us:u64
stream_start_utc_us:u64
format:u8, mode:u8, profile:u8, gain_index:u8, autogain:u8
window:u16, output_count:u32
measured_millisps:u32
ADS1220_registers:u8[4]
settling_discard_count:u16
allowed_gain_mask:u8
autogain_low_q15:u16, autogain_high_q15:u16, autogain_hysteresis_q15:u16
dark_source:u8, active_dark_volts:f32
temperature_period_us:u32
```

The client must acknowledge the exact `stream_start_device_us` token within one
second. Firmware emits no samples before the matching `ACK_STREAM`, then replies
`OK`. A missing or wrong acknowledgement stops acquisition with an error.
Sequence zero and the token identify a new stream; both reset on every start or
atomic replacement. `device_us` marks the ADS1220 DRDY interrupt.

### Enumerations and flags

- stream format: raw `0`, volts `1`
- stream mode: continuous `0`, finite `1`
- device state: stopped `0`, awaiting acknowledgement `1`, streaming `2`
- storage state: empty `0`, valid `1`, fault `2`
- stop reason: requested `0`, finite complete `1`, replaced `2`
- dark source: persisted/default device baseline `0`, session baseline `1`

Sample status bits:

| Bit | Meaning |
|---:|---|
| `0` | positive ADC/PGA full-scale code |
| `1` | negative ADC/PGA full-scale code |
| `2` | measured positive TIA clipping threshold reached |
| `4` | autogain remains over range at gain 1× |
| `5` | autogain remains under range at gain 128× |

Negative TIA clipping is not characterized and has no asserted flag. Bit 3 and
bits 6–7 are reserved.

## Acquisition profiles

| ID | Name | ADS1220 registers | Measured conversion rate | Intended use |
|---:|---|---|---:|---|
| 0 | `normal_20_50_60` | `00 04 10 00` | 19.958 SPS | precision, simultaneous 50/60 Hz rejection |
| 1 | `normal_330` | `00 84 00 00` | 327.876 SPS | interactive GUI |
| 2 | `turbo_2000` | `00 D4 00 00` | 1949.3 SPS | maximum-rate capture |

All profiles use continuous conversion, internal 2.048 V reference, dedicated
DRDY, PGA enabled, and gains 1×–128×. Gain index `i` means gain `2**i`.
Characterization found no required conversion discard for these profiles.

`window` is a sliding conversion mean from 1 through 1024. No output is emitted
until the window fills; then one output is emitted for every accepted conversion.
For a finite stream, `output_count` counts emitted outputs, not ADC conversions.
Continuous streams require `output_count=0`; finite streams require it to be
nonzero.

Raw streams return signed, gain-dependent mean ADC codes and cannot autogain.
Voltage streams normalize every conversion before averaging:

```text
volts_before_dark = normalized_code * 2.048 / (8388608 * 128)
volts = volts_before_dark - active_dark_volts
```

Here `normalized_code = raw_code * 2**(7 - gain_index)`. This preserves one
canonical voltage scale across gain changes. Store volts, never gain-relative
percentages.

## Autogain and clipping

Autogain is firmware-owned and valid only for voltage streams. It uses absolute
ADC magnitude and changes one gain step per conversion:

- below 40% full scale: increase gain;
- above 85% full scale: decrease gain;
- otherwise retain gain.

At gain rails, status bits report under/over range. A clipped conversion that
causes a gain decrease is not added to the averaging window. The returned sample
always carries the gain actually used. Dark correction is subtracted only after
normalization/averaging, so it cannot change autogain or clipping decisions.

ADC/PGA clipping is exact signed full scale. Positive analog/TIA clipping is
flagged at 1.64 V differential, measured at gain 1×. Keep these mechanisms
distinct.

## State and failure rules

Connect sequence is `HELLO`, `TIME_SYNC`, `LIST_PROFILES`. Time synchronization
is required before a stream. It maps RP2040 monotonic microseconds to host UTC;
the stream header captures both clocks.

`START_STREAM` while active first emits `STREAM_STOPPED(reason=replaced)`, then a
complete new header. Dark set/clear/save behaves the same while active. There are
no piecemeal gain/profile/window commands. `TIME_SYNC` and `RESET_STORAGE` also
stop active acquisition. Reconnect performs identity/time/profile setup but does
not resume a prior stream.

Any command, frame, ADC, temperature, timing, storage, or overflow error stops
acquisition and emits one `ERROR`. No stale-temperature continuation exists.

| Code | Meaning |
|---:|---|
| 1 | malformed/oversized frame |
| 2 | unsupported protocol version |
| 3 | payload schema mismatch |
| 4 | CRC failure |
| 5 | unknown message type |
| 6 | invalid argument |
| 7 | invalid device state |
| 8 | time not synchronized |
| 9 | stream acknowledgement timeout |
| 10 | ADS1220 configuration failure |
| 11 | ADS1220 DRDY timeout |
| 12 | acquisition/USB overflow |
| 13 | TMP117 failure |
| 14 | LittleFS mount failure |
| 15 | persistent record integrity failure |
| 16 | persistent write failure |
| 17 | storage reset confirmation mismatch |
| 18 | firmware invariant failure |

## Dark correction and persistence

A missing persistent record is normal and means a `0.0 V` device baseline. A
valid session baseline overrides it. `CLEAR_SESSION_DARK` restores the device
baseline without erasing it. `SAVE_SESSION_DARK` atomically promotes the session
value to the device baseline and clears the redundant session layer. Values must
be finite and within ±0.25 V.

The LittleFS record is schema-versioned and CRC32-protected. Firmware writes a
temporary file, flushes it, then renames it over the active record. Mount failure
or an invalid existing record produces explicit storage-fault state; firmware
does not auto-format. `RESET_STORAGE` is the only repair operation. It requires
the exact device ID plus `ERASE`, formats application storage, and returns the
baseline to zero.

Protocol 3 has no spectral/physical calibration storage or conversion. Absolute
optical calibration remains deferred.

## Python API

`LightSensor` owns transport framing, request serialization, handshake, and typed
message parsing. Typical flow:

```python
from lightmeter import LightSensor, StreamConfig, VoltageSample

with LightSensor(device_id="DE657814573A0C29") as sensor:
    header = sensor.start_stream(StreamConfig(profile_id=1, window=8))
    while True:
        event = sensor.read_event(2.0)
        if isinstance(event, VoltageSample):
            consume(event)
```

Important methods:

- `connect()` / `close()` / `reconnect()`
- `get_status()`, `list_profiles()`, `ping()`, `synchronize_time()`
- `start_stream(StreamConfig)`, `read_event(timeout)`, `stop_stream()`
- `zero(n, profile_id=0, gain_index=7)`
- `set_session_dark(volts)`, `clear_zero()`, `save_baseline()`
- `reset_storage(confirm_device_id)`

`read_event()` returns `RawSample`, `VoltageSample`, `StreamStopped`, or
`ErrorEvent`. Matching command errors raise `DeviceError`; malformed host-side
frames raise `ProtocolError`; serial link failures raise `ConnectionError`.
Reconnects repeat the full handshake and remain bound to the first connected
flash UID; they never select a different LightSensor or resume acquisition.

The Tk GUI's `SensorSampler` is the sole serial owner. Controls enqueue work onto
its daemon thread; Tk only reads locked snapshots. Applying acquisition or dark
settings causes a complete stream replacement. CSV files use a fixed column
schema with one `stream_start` metadata row followed by `sample` rows.
If opening, writing, flushing, or closing a CSV fails, recording is disabled and
the GUI reports the error while acquisition continues.

## Firmware build and recovery

Install Arduino-Pico and build through `just`:

```bash
just setup
just compile
just flash
SENSOR_PORT=/dev/ttyACM1 just flash
```

The FQBN selects a 4 MiB generic RP2040 board with a 2 MiB sketch / 2 MiB
LittleFS split, 133 MHz CPU, generic Winbond boot2, Pico SDK USB, and UF2 upload.
For manual ROM recovery: hold `BOOT`, press/release `RESET`, then release `BOOT`.
`just flash` resets an application CDC port into ROM UF2 automatically. SWD pads
remain the development recovery path; no application update or rollback slot is
implemented.
