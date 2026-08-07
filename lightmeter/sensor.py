from collections import deque
from dataclasses import dataclass
from enum import IntEnum, IntFlag
import binascii
import logging
import math
import struct
import threading
import time

import serial

from lightmeter.port_detect import _normalize_device_id, autodetect_port

log = logging.getLogger(__name__)

PROTO_VERSION = 3
HARDWARE_MAJOR = 3
BAUD_RATE = 115200
MAX_DECODED_FRAME = 256
MAX_ENCODED_FRAME = 258
MAX_WINDOW = 1024
MAX_DARK_OFFSET_V = 0.25

MSG_HELLO = 0x01
MSG_TIME_SYNC = 0x02
MSG_PING = 0x03
MSG_GET_STATUS = 0x04
MSG_LIST_PROFILES = 0x05
MSG_START_STREAM = 0x10
MSG_ACK_STREAM = 0x11
MSG_STOP_STREAM = 0x12
MSG_SET_SESSION_DARK = 0x20
MSG_CLEAR_SESSION_DARK = 0x21
MSG_SAVE_SESSION_DARK = 0x22
MSG_RESET_STORAGE = 0x30

MSG_HELLO_REPLY = 0x81
MSG_TIME_SYNCED = 0x82
MSG_PONG = 0x83
MSG_STATUS = 0x84
MSG_PROFILES = 0x85
MSG_STREAM_STARTED = 0x90
MSG_SAMPLE_RAW = 0x91
MSG_SAMPLE_VOLTS = 0x92
MSG_STREAM_STOPPED = 0x93
MSG_OK = 0xA0
MSG_ERROR = 0xFF

ERROR_MESSAGES = {
    1: "malformed or oversized frame",
    2: "unsupported protocol version",
    3: "message schema mismatch",
    4: "CRC failure",
    5: "unknown message type",
    6: "invalid argument",
    7: "invalid device state",
    8: "time not synchronized",
    9: "stream acknowledgement timeout",
    10: "ADS1220 configuration failure",
    11: "ADS1220 data-ready timeout",
    12: "acquisition or USB overflow",
    13: "TMP117 failure",
    14: "LittleFS mount failure",
    15: "persistent record integrity failure",
    16: "persistent write failure",
    17: "storage reset confirmation mismatch",
    18: "firmware invariant failure",
}

CAP_RAW_STREAM = 1 << 0
CAP_VOLTS_STREAM = 1 << 1
CAP_FINITE_STREAM = 1 << 2
CAP_VOLTS_AUTOGAIN = 1 << 3
CAP_TEMPERATURE = 1 << 4
CAP_SESSION_DARK = 1 << 5
CAP_PERSISTENT_DARK = 1 << 6
CAP_STORAGE_RESET = 1 << 7
REQUIRED_CAPABILITIES = (
    CAP_RAW_STREAM
    | CAP_VOLTS_STREAM
    | CAP_FINITE_STREAM
    | CAP_VOLTS_AUTOGAIN
    | CAP_TEMPERATURE
    | CAP_SESSION_DARK
    | CAP_PERSISTENT_DARK
    | CAP_STORAGE_RESET
)


class StreamFormat(IntEnum):
    RAW = 0
    VOLTS = 1


class StreamMode(IntEnum):
    CONTINUOUS = 0
    FINITE = 1


class DeviceState(IntEnum):
    STOPPED = 0
    AWAITING_ACK = 1
    STREAMING = 2


class StorageState(IntEnum):
    EMPTY = 0
    VALID = 1
    FAULT = 2


class StopReason(IntEnum):
    REQUESTED = 0
    FINITE_COMPLETE = 1
    REPLACED = 2


class SampleStatus(IntFlag):
    NONE = 0
    ADC_POSITIVE_CLIP = 1 << 0
    ADC_NEGATIVE_CLIP = 1 << 1
    TIA_POSITIVE_CLIP = 1 << 2
    AUTOGAIN_OVERRANGE = 1 << 4
    AUTOGAIN_UNDERRANGE = 1 << 5

SAMPLE_STATUS_MASK = (
    SampleStatus.ADC_POSITIVE_CLIP
    | SampleStatus.ADC_NEGATIVE_CLIP
    | SampleStatus.TIA_POSITIVE_CLIP
    | SampleStatus.AUTOGAIN_OVERRANGE
    | SampleStatus.AUTOGAIN_UNDERRANGE
)


@dataclass(frozen=True)
class DeviceInfo:
    firmware_version: tuple[int, int, int]
    hardware_major: int
    capabilities: int
    maximum_decoded_frame: int
    device_id: str
    time_synchronized: bool
    storage_state: StorageState


@dataclass(frozen=True)
class Profile:
    id: int
    name: str
    measured_sps: float
    registers: tuple[int, int, int, int]
    allowed_gain_mask: int
    settling_discard_count: int


@dataclass(frozen=True)
class StreamConfig:
    format: StreamFormat = StreamFormat.VOLTS
    mode: StreamMode = StreamMode.CONTINUOUS
    profile_id: int = 1
    gain_index: int = 0
    autogain: bool = True
    window: int = 1
    output_count: int = 0

    def __post_init__(self):
        if not isinstance(self.format, StreamFormat):
            raise TypeError("format must be StreamFormat")
        if not isinstance(self.mode, StreamMode):
            raise TypeError("mode must be StreamMode")
        if type(self.profile_id) is not int:
            raise TypeError("profile_id must be int")
        if not 0 <= self.profile_id <= 255:
            raise ValueError("profile_id must fit in u8")
        if type(self.gain_index) is not int:
            raise TypeError("gain_index must be int")
        if not 0 <= self.gain_index <= 7:
            raise ValueError("gain_index must be from 0 through 7")
        if type(self.autogain) is not bool:
            raise TypeError("autogain must be bool")
        if type(self.window) is not int:
            raise TypeError("window must be int")
        if not 1 <= self.window <= MAX_WINDOW:
            raise ValueError(f"window must be from 1 through {MAX_WINDOW}")
        if type(self.output_count) is not int:
            raise TypeError("output_count must be int")
        if self.format is StreamFormat.RAW and self.autogain:
            raise ValueError("raw streams cannot use autogain")
        if self.mode is StreamMode.CONTINUOUS and self.output_count != 0:
            raise ValueError("continuous streams require output_count=0")
        if self.mode is StreamMode.FINITE and not 1 <= self.output_count <= 0xFFFFFFFF:
            raise ValueError("finite streams require output_count=1..2^32-1")

    @property
    def gain(self):
        return 1 << self.gain_index


@dataclass(frozen=True)
class StreamHeader:
    request_id: int
    stream_start_device_us: int
    stream_start_utc_us: int
    config: StreamConfig
    measured_sps: float
    registers: tuple[int, int, int, int]
    settling_discard_count: int
    allowed_gain_mask: int
    autogain_low_fraction: float
    autogain_high_fraction: float
    autogain_hysteresis_fraction: float
    dark_source: int
    active_dark_volts: float
    temperature_period_us: int


@dataclass(frozen=True)
class RawSample:
    sequence: int
    device_timestamp_us: int
    gain_index: int
    status: SampleStatus
    value: int
    temperature_c: float

    @property
    def gain(self):
        return 1 << self.gain_index


@dataclass(frozen=True)
class VoltageSample:
    sequence: int
    device_timestamp_us: int
    gain_index: int
    status: SampleStatus
    value: float
    temperature_c: float

    @property
    def gain(self):
        return 1 << self.gain_index


@dataclass(frozen=True)
class StreamStopped:
    request_id: int
    stream_start_device_us: int
    delivered_outputs: int
    reason: StopReason


@dataclass(frozen=True)
class ErrorEvent:
    request_id: int
    stream_start_device_us: int
    last_sample_sequence: int | None
    code: int
    detail: int

    @property
    def message(self):
        return ERROR_MESSAGES.get(self.code, f"unknown device error {self.code}")


@dataclass(frozen=True)
class DeviceStatus:
    request_id: int
    state: DeviceState
    time_synchronized: bool
    storage_state: StorageState
    dark_source: int
    device_dark_volts: float
    session_dark_volts: float | None
    temperature_c: float | None
    device_monotonic_us: int
    last_error_code: int


class ProtocolError(RuntimeError):
    pass


class DeviceError(RuntimeError):
    def __init__(self, event):
        self.event = event
        super().__init__(f"{event.message} (code={event.code}, detail={event.detail})")


def _protocol_enum(enum_type, value, field):
    try:
        return enum_type(value)
    except ValueError as exc:
        raise ProtocolError(f"invalid {field} value {value}") from exc


def _cobs_encode(data):
    output = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def _cobs_decode(data):
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ProtocolError("truncated COBS frame")
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def _encode_frame(message_type, payload):
    if len(payload) > MAX_DECODED_FRAME - 8:
        raise ValueError("payload exceeds protocol frame limit")
    decoded = struct.pack("<BBH", PROTO_VERSION, message_type, len(payload)) + payload
    decoded += struct.pack("<I", binascii.crc32(decoded) & 0xFFFFFFFF)
    return _cobs_encode(decoded) + b"\0"


def _decode_frame(encoded):
    decoded = _cobs_decode(encoded)
    if len(decoded) < 8:
        raise ProtocolError("decoded frame is too short")
    version, message_type, payload_length = struct.unpack_from("<BBH", decoded)
    if version != PROTO_VERSION:
        raise ProtocolError(f"device protocol {version}, driver protocol {PROTO_VERSION}")
    if payload_length > MAX_DECODED_FRAME - 8 or len(decoded) != payload_length + 8:
        raise ProtocolError("frame payload length mismatch")
    expected_crc = struct.unpack_from("<I", decoded, len(decoded) - 4)[0]
    actual_crc = binascii.crc32(decoded[:-4]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ProtocolError("frame CRC mismatch")
    return message_type, decoded[4:-4]


class _PayloadReader:
    def __init__(self, payload):
        self.payload = payload
        self.offset = 0

    def unpack(self, format_):
        size = struct.calcsize(format_)
        if self.offset + size > len(self.payload):
            raise ProtocolError("short message payload")
        values = struct.unpack_from(format_, self.payload, self.offset)
        self.offset += size
        return values

    def take(self, length):
        if self.offset + length > len(self.payload):
            raise ProtocolError("short message payload")
        value = self.payload[self.offset : self.offset + length]
        self.offset += length
        return value

    def finish(self):
        if self.offset != len(self.payload):
            raise ProtocolError("trailing message payload bytes")


class LightSensor:
    def __init__(self, port=None, *, timeout=2.0, auto_reconnect=False, device_id=None):
        self.port = port
        self._explicit_port = port is not None
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect
        self.requested_device_id = _normalize_device_id(device_id)
        self.ser = None
        self.info = None
        self.profiles = ()
        self.stream_header = None
        self._request_id = 0
        self._rx = bytearray()
        self._events = deque()
        self._lock = threading.RLock()
        self._reconnecting = False

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open and self.info is not None

    def _next_request_id(self):
        self._request_id = (self._request_id + 1) & 0xFFFFFFFF
        if self._request_id == 0:
            self._request_id = 1
        return self._request_id

    def _write_request(self, message_type, request_id, payload=b""):
        frame = _encode_frame(message_type, struct.pack("<I", request_id) + payload)
        try:
            written = self.ser.write(frame)
            self.ser.flush()
        except serial.SerialException as exc:
            self._link_failed(exc)
        if written != len(frame):
            raise ConnectionError(f"short serial write: {written}/{len(frame)} bytes")

    def _link_failed(self, exc):
        if self.auto_reconnect and self.reconnect():
            raise ConnectionError("serial link failed; reconnected in stopped state") from exc
        self.close()
        raise ConnectionError("serial link failed") from exc

    def _read_encoded(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            delimiter = self._rx.find(0)
            if delimiter >= 0:
                if delimiter > MAX_ENCODED_FRAME:
                    del self._rx[: delimiter + 1]
                    raise ProtocolError("encoded frame exceeds protocol limit")
                frame = bytes(self._rx[:delimiter])
                del self._rx[: delimiter + 1]
                if frame:
                    return frame
                continue
            if len(self._rx) > MAX_ENCODED_FRAME:
                self._rx.clear()
                raise ProtocolError("encoded frame exceeds protocol limit")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("timed out waiting for protocol frame")
            try:
                self.ser.timeout = min(remaining, 0.05)
                chunk = self.ser.read(max(self.ser.in_waiting, 1))
            except serial.SerialException as exc:
                self._link_failed(exc)
            if chunk:
                self._rx.extend(chunk)

    def _read_message(self, timeout=None):
        encoded = self._read_encoded(self.timeout if timeout is None else timeout)
        return _decode_frame(encoded)

    def _parse_error(self, payload):
        if len(payload) != 20:
            raise ProtocolError("invalid ERROR payload length")
        request_id, token, last_sequence, code, detail = struct.unpack("<IQIHH", payload)
        return ErrorEvent(
            request_id,
            token,
            None if last_sequence == 0xFFFFFFFF else last_sequence,
            code,
            detail,
        )

    def _parse_sample(self, message_type, payload):
        if len(payload) != 22:
            raise ProtocolError("invalid sample payload length")
        if message_type not in (MSG_SAMPLE_RAW, MSG_SAMPLE_VOLTS):
            raise ProtocolError(f"invalid sample message type 0x{message_type:02X}")
        sequence, timestamp, gain_index, status = struct.unpack_from("<IQBB", payload)
        temperature = struct.unpack_from("<f", payload, 18)[0]
        if gain_index > 7:
            raise ProtocolError(f"invalid sample gain index {gain_index}")
        if status & ~int(SAMPLE_STATUS_MASK):
            raise ProtocolError(f"invalid sample status bits 0x{status:02X}")
        if not math.isfinite(temperature) or not -55.0 <= temperature <= 150.0:
            raise ProtocolError(f"invalid sample temperature {temperature}")
        sample_status = SampleStatus(status)
        if message_type == MSG_SAMPLE_RAW:
            value = struct.unpack_from("<i", payload, 14)[0]
            return RawSample(sequence, timestamp, gain_index, sample_status, value, temperature)
        value = struct.unpack_from("<f", payload, 14)[0]
        if not math.isfinite(value):
            raise ProtocolError(f"invalid sample voltage {value}")
        return VoltageSample(sequence, timestamp, gain_index, sample_status, value, temperature)

    def _parse_stopped(self, payload):
        if len(payload) != 17:
            raise ProtocolError("invalid STREAM_STOPPED payload length")
        request_id, token, count, reason = struct.unpack("<IQIB", payload)
        return StreamStopped(request_id, token, count, _protocol_enum(StopReason, reason, "stop reason"))

    def _parse_header(self, payload):
        reader = _PayloadReader(payload)
        request_id, token, start_utc = reader.unpack("<IQQ")
        format_, mode, profile_id, gain_index, autogain, window, count = reader.unpack(
            "<BBBBBHI"
        )
        measured_millisps = reader.unpack("<I")[0]
        registers = reader.unpack("<BBBB")
        discard = reader.unpack("<H")[0]
        gain_mask = reader.unpack("<B")[0]
        low, high, hysteresis = reader.unpack("<HHH")
        dark_source = reader.unpack("<B")[0]
        dark_volts = reader.unpack("<f")[0]
        temperature_period = reader.unpack("<I")[0]
        reader.finish()
        if autogain not in (0, 1):
            raise ProtocolError(f"invalid stream autogain value {autogain}")
        if dark_source not in (0, 1):
            raise ProtocolError(f"invalid stream dark source {dark_source}")
        if measured_millisps == 0:
            raise ProtocolError("stream measured rate must be positive")
        if gain_mask == 0 or gain_index > 7 or not gain_mask & (1 << gain_index):
            raise ProtocolError("stream gain is not permitted by its profile")
        if not 0 <= low < high <= 32768 or hysteresis > 32768:
            raise ProtocolError("invalid stream autogain thresholds")
        if not math.isfinite(dark_volts) or abs(dark_volts) > MAX_DARK_OFFSET_V:
            raise ProtocolError(f"invalid stream dark correction {dark_volts}")
        if temperature_period == 0:
            raise ProtocolError("stream temperature period must be positive")
        try:
            config = StreamConfig(
                _protocol_enum(StreamFormat, format_, "stream format"),
                _protocol_enum(StreamMode, mode, "stream mode"),
                profile_id,
                gain_index,
                bool(autogain),
                window,
                count,
            )
        except (TypeError, ValueError) as exc:
            raise ProtocolError(f"invalid stream configuration: {exc}") from exc
        return StreamHeader(
            request_id,
            token,
            start_utc,
            config,
            measured_millisps / 1000.0,
            registers,
            discard,
            gain_mask,
            low / 32768.0,
            high / 32768.0,
            hysteresis / 32768.0,
            dark_source,
            dark_volts,
            temperature_period,
        )

    def _parse_status(self, payload):
        if len(payload) != 30:
            raise ProtocolError("invalid STATUS payload length")
        (
            request_id,
            state,
            time_synchronized,
            storage_state,
            dark_source,
            device_dark,
            session_dark_value,
            temperature_value,
            monotonic_us,
            last_error,
        ) = struct.unpack("<IBBBBfffQH", payload)
        if time_synchronized not in (0, 1):
            raise ProtocolError(f"invalid time synchronization value {time_synchronized}")
        if dark_source not in (0, 1):
            raise ProtocolError(f"invalid status dark source {dark_source}")
        if not math.isfinite(device_dark) or abs(device_dark) > MAX_DARK_OFFSET_V:
            raise ProtocolError(f"invalid device dark correction {device_dark}")
        session_dark = None if math.isnan(session_dark_value) else session_dark_value
        if session_dark is not None and (
            not math.isfinite(session_dark) or abs(session_dark) > MAX_DARK_OFFSET_V
        ):
            raise ProtocolError(f"invalid session dark correction {session_dark}")
        temperature = None if math.isnan(temperature_value) else temperature_value
        if temperature is not None and (
            not math.isfinite(temperature) or not -55.0 <= temperature <= 150.0
        ):
            raise ProtocolError(f"invalid status temperature {temperature}")
        return DeviceStatus(
            request_id,
            _protocol_enum(DeviceState, state, "device state"),
            bool(time_synchronized),
            _protocol_enum(StorageState, storage_state, "storage state"),
            dark_source,
            device_dark,
            session_dark,
            temperature,
            monotonic_us,
            last_error,
        )

    def _event_from_message(self, message_type, payload):
        if message_type in (MSG_SAMPLE_RAW, MSG_SAMPLE_VOLTS):
            return self._parse_sample(message_type, payload)
        if message_type == MSG_STREAM_STOPPED:
            return self._parse_stopped(payload)
        if message_type == MSG_ERROR:
            return self._parse_error(payload)
        raise ProtocolError(f"unexpected asynchronous message type 0x{message_type:02X}")

    def _wait_for(self, request_id, accepted_types, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"timed out waiting for request {request_id}")
            message_type, payload = self._read_message(remaining)
            if message_type == MSG_ERROR:
                event = self._parse_error(payload)
                if event.request_id in (0, request_id):
                    self.stream_header = None
                    raise DeviceError(event)
                self._events.append(event)
                continue
            if len(payload) < 4:
                raise ProtocolError(
                    f"response type 0x{message_type:02X} has no complete request ID"
                )
            if message_type in accepted_types:
                response_id = struct.unpack_from("<I", payload)[0]
                if response_id == request_id:
                    return message_type, payload
            self._events.append(self._event_from_message(message_type, payload))

    def connect(self):
        with self._lock:
            if self.connected:
                return self.info
            port = self.port or autodetect_port(self.requested_device_id)
            try:
                self.ser = serial.Serial(
                    port,
                    BAUD_RATE,
                    timeout=0.05,
                    write_timeout=self.timeout,
                )
                self.port = port
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self._rx.clear()
                self._events.clear()
                request_id = self._next_request_id()
                self._write_request(MSG_HELLO, request_id)
                _, payload = self._wait_for(request_id, {MSG_HELLO_REPLY})
                if len(payload) != 24:
                    raise ProtocolError("invalid HELLO_REPLY payload length")
                values = struct.unpack("<IBBBBIHQBB", payload)
                if values[8] not in (0, 1):
                    raise ProtocolError(f"invalid HELLO time synchronization value {values[8]}")
                if values[7] == 0:
                    raise ProtocolError("invalid all-zero device ID")
                info = DeviceInfo(
                    (values[1], values[2], values[3]),
                    values[4],
                    values[5],
                    values[6],
                    f"{values[7]:016X}",
                    bool(values[8]),
                    _protocol_enum(StorageState, values[9], "HELLO storage state"),
                )
                if info.hardware_major != HARDWARE_MAJOR:
                    raise ProtocolError(f"unsupported hardware generation {info.hardware_major}")
                if info.firmware_version[0] != PROTO_VERSION:
                    raise ProtocolError(
                        f"unsupported firmware major {info.firmware_version[0]}"
                    )
                if info.maximum_decoded_frame != MAX_DECODED_FRAME:
                    raise ProtocolError(
                        "device maximum decoded frame does not match protocol contract"
                    )
                missing_capabilities = REQUIRED_CAPABILITIES & ~info.capabilities
                if missing_capabilities:
                    raise ProtocolError(
                        f"device lacks required capabilities 0x{missing_capabilities:02X}"
                    )
                if self.requested_device_id and info.device_id != self.requested_device_id:
                    raise ProtocolError(
                        f"connected device {info.device_id}, requested {self.requested_device_id}"
                    )
                self.requested_device_id = info.device_id
                self.info = info
                self.synchronize_time()
                self.info = DeviceInfo(
                    info.firmware_version,
                    info.hardware_major,
                    info.capabilities,
                    info.maximum_decoded_frame,
                    info.device_id,
                    True,
                    info.storage_state,
                )
                self.profiles = self.list_profiles()
                return self.info
            except Exception:
                self.close()
                raise

    open = connect

    def synchronize_time(self):
        with self._lock:
            request_id = self._next_request_id()
            utc_us = time.time_ns() // 1000
            self._write_request(MSG_TIME_SYNC, request_id, struct.pack("<Q", utc_us))
            _, payload = self._wait_for(request_id, {MSG_TIME_SYNCED})
            if len(payload) != 20:
                raise ProtocolError("invalid TIME_SYNCED payload length")
            response_id, echoed_utc, device_us = struct.unpack("<IQQ", payload)
            if response_id != request_id or echoed_utc != utc_us:
                raise ProtocolError("time synchronization reply mismatch")
            self.stream_header = None
            self._events.clear()
            return echoed_utc, device_us

    def ping(self):
        with self._lock:
            request_id = self._next_request_id()
            self._write_request(MSG_PING, request_id)
            _, payload = self._wait_for(request_id, {MSG_PONG})
            if payload != struct.pack("<I", request_id):
                raise ProtocolError("invalid PONG payload")
            return True

    def list_profiles(self):
        with self._lock:
            request_id = self._next_request_id()
            self._write_request(MSG_LIST_PROFILES, request_id)
            _, payload = self._wait_for(request_id, {MSG_PROFILES})
            reader = _PayloadReader(payload)
            response_id = reader.unpack("<I")[0]
            count = reader.unpack("<B")[0]
            if response_id != request_id:
                raise ProtocolError("profile reply request mismatch")
            profiles = []
            profile_ids = set()
            for _ in range(count):
                profile_id, name_length = reader.unpack("<BB")
                if profile_id in profile_ids:
                    raise ProtocolError(f"duplicate profile ID {profile_id}")
                try:
                    name = reader.take(name_length).decode("ascii")
                except UnicodeDecodeError as exc:
                    raise ProtocolError("profile name is not ASCII") from exc
                measured_millisps = reader.unpack("<I")[0]
                registers = reader.unpack("<BBBB")
                gain_mask = reader.unpack("<B")[0]
                discard = reader.unpack("<H")[0]
                if not name:
                    raise ProtocolError(f"profile {profile_id} has an empty name")
                if measured_millisps == 0:
                    raise ProtocolError(f"profile {profile_id} has a zero measured rate")
                if gain_mask == 0:
                    raise ProtocolError(f"profile {profile_id} permits no gains")
                profile_ids.add(profile_id)
                profiles.append(
                    Profile(
                        profile_id,
                        name,
                        measured_millisps / 1000.0,
                        registers,
                        gain_mask,
                        discard,
                    )
                )
            reader.finish()
            if not profiles:
                raise ProtocolError("device reported no acquisition profiles")
            return tuple(profiles)

    def get_status(self):
        with self._lock:
            request_id = self._next_request_id()
            self._write_request(MSG_GET_STATUS, request_id)
            _, payload = self._wait_for(request_id, {MSG_STATUS})
            status = self._parse_status(payload)
            if status.request_id != request_id:
                raise ProtocolError("status reply request mismatch")
            return status

    def _ack_header(self, header):
        request_id = self._next_request_id()
        self._write_request(
            MSG_ACK_STREAM,
            request_id,
            struct.pack("<Q", header.stream_start_device_us),
        )
        _, payload = self._wait_for(request_id, {MSG_OK})
        if payload != struct.pack("<IB", request_id, MSG_ACK_STREAM):
            raise ProtocolError("invalid ACK_STREAM reply")
        self.stream_header = header
        return header

    def start_stream(self, config):
        with self._lock:
            if not isinstance(config, StreamConfig):
                raise TypeError("config must be StreamConfig")
            request_id = self._next_request_id()
            payload = struct.pack(
                "<BBBBBHI",
                config.format,
                config.mode,
                config.profile_id,
                config.gain_index,
                config.autogain,
                config.window,
                config.output_count,
            )
            self._write_request(MSG_START_STREAM, request_id, payload)
            _, response = self._wait_for(request_id, {MSG_STREAM_STARTED})
            header = self._parse_header(response)
            self._events.clear()
            return self._ack_header(header)

    def stop_stream(self):
        with self._lock:
            request_id = self._next_request_id()
            self._write_request(MSG_STOP_STREAM, request_id)
            _, payload = self._wait_for(request_id, {MSG_STREAM_STOPPED})
            event = self._parse_stopped(payload)
            self.stream_header = None
            self._events.clear()
            self._rx.clear()
            self.ser.reset_input_buffer()
            return event

    def read_event(self, timeout=None):
        with self._lock:
            if self._events:
                event = self._events.popleft()
            else:
                message_type, payload = self._read_message(timeout)
                event = self._event_from_message(message_type, payload)
            if isinstance(event, ErrorEvent) or (
                isinstance(event, StreamStopped)
                and self.stream_header is not None
                and event.stream_start_device_us == self.stream_header.stream_start_device_us
            ):
                self.stream_header = None
            return event

    def _dark_command(self, message_type, payload=b""):
        request_id = self._next_request_id()
        self._write_request(message_type, request_id, payload)
        response_type, response = self._wait_for(request_id, {MSG_OK, MSG_STREAM_STARTED})
        if response_type == MSG_OK:
            if response != struct.pack("<IB", request_id, message_type):
                raise ProtocolError("invalid dark command reply")
            return None
        header = self._parse_header(response)
        self._events.clear()
        return self._ack_header(header)


    def _set_storage_state(self, state):
        if self.info is not None:
            self.info = DeviceInfo(
                self.info.firmware_version,
                self.info.hardware_major,
                self.info.capabilities,
                self.info.maximum_decoded_frame,
                self.info.device_id,
                self.info.time_synchronized,
                state,
            )


    def set_session_dark(self, volts):
        with self._lock:
            if not math.isfinite(volts) or abs(volts) > MAX_DARK_OFFSET_V:
                raise ValueError(f"dark correction must be within ±{MAX_DARK_OFFSET_V} V")
            return self._dark_command(MSG_SET_SESSION_DARK, struct.pack("<f", volts))

    def clear_zero(self):
        with self._lock:
            return self._dark_command(MSG_CLEAR_SESSION_DARK)

    def save_baseline(self):
        with self._lock:
            header = self._dark_command(MSG_SAVE_SESSION_DARK)
            self._set_storage_state(StorageState.VALID)
            return header

    def zero(self, sample_count, *, profile_id=0, gain_index=7):
        with self._lock:
            if not 1 <= sample_count <= 0xFFFFFFFF:
                raise ValueError("sample_count must be 1..2^32-1")
            status = self.get_status()
            if status.state is not DeviceState.STOPPED:
                self.stop_stream()
            self.set_session_dark(0.0)
            config = StreamConfig(
                format=StreamFormat.VOLTS,
                mode=StreamMode.FINITE,
                profile_id=profile_id,
                gain_index=gain_index,
                autogain=False,
                window=1,
                output_count=sample_count,
            )
            self.start_stream(config)
            values = []
            while True:
                event = self.read_event(max(self.timeout, sample_count / 10.0))
                if isinstance(event, VoltageSample):
                    values.append(event.value)
                elif isinstance(event, ErrorEvent):
                    raise DeviceError(event)
                elif isinstance(event, StreamStopped):
                    break
            if len(values) != sample_count:
                raise ProtocolError(f"zero acquired {len(values)}/{sample_count} samples")
            baseline = math.fsum(values) / len(values)
            self.set_session_dark(baseline)
            return baseline

    def reset_storage(self, confirm_device_id):
        with self._lock:
            if self.info is None:
                raise RuntimeError("sensor is not connected")
            normalized = _normalize_device_id(confirm_device_id)
            if normalized != self.info.device_id:
                raise ValueError(f"confirmation must equal device ID {self.info.device_id}")
            request_id = self._next_request_id()
            payload = struct.pack("<Q", int(normalized, 16)) + b"ERASE"
            self._write_request(MSG_RESET_STORAGE, request_id, payload)
            _, response = self._wait_for(request_id, {MSG_OK})
            if response != struct.pack("<IB", request_id, MSG_RESET_STORAGE):
                raise ProtocolError("invalid RESET_STORAGE reply")
            self.stream_header = None
            self._events.clear()
            self._set_storage_state(StorageState.EMPTY)
            return True

    def reconnect(self):
        with self._lock:
            if self._reconnecting:
                return False
            self._reconnecting = True
            try:
                port = self.port if self._explicit_port else None
                self.close()
                self.port = port
                try:
                    self.connect()
                except (ConnectionError, OSError, ProtocolError, serial.SerialException):
                    log.warning("LightSensor reconnect failed", exc_info=True)
                    return False
                return True
            finally:
                self._reconnecting = False

    def close(self):
        with self._lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                finally:
                    self.ser = None
            self.info = None
            self.profiles = ()
            self.stream_header = None
            self._rx.clear()
            self._events.clear()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()
