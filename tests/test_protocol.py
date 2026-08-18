import math
import struct
from pathlib import Path
import tomllib
from types import SimpleNamespace

import lightmeter.gui as gui
import lightmeter.port_detect as port_detect

import lightmeter.sensor as protocol


def assert_raises(exception_type, function):
    try:
        function()
    except exception_type:
        return
    raise AssertionError(f"expected {exception_type.__name__}")


def stream_header():
    return protocol.StreamHeader(
        1,
        1000,
        2000,
        protocol.StreamConfig(),
        327.876,
        (0, 132, 0, 0),
        0,
        0xFF,
        0.4,
        0.85,
        0.05,
        0,
        0.0,
        100_000,
    )



def stream_header_payload(*, format_=1, autogain=1, dark_source=0, dark_volts=0.0):
    return b"".join(
        (
            struct.pack("<IQQ", 1, 1000, 2000),
            struct.pack("<BBBBBHI", format_, 0, 1, 0, autogain, 1, 0),
            struct.pack("<I", 327876),
            struct.pack("<BBBB", 0, 132, 0, 0),
            struct.pack("<H", 0),
            struct.pack("<B", 0xFF),
            struct.pack("<HHH", 13107, 27853, 1638),
            struct.pack("<B", dark_source),
            struct.pack("<f", dark_volts),
            struct.pack("<I", 100_000),
        )
    )


def test_cobs_round_trip_boundaries():
    payloads = [
        b"",
        b"\0",
        b"abc\0def",
        bytes(range(1, 255)),
        bytes(range(256)) * 2,
    ]
    for payload in payloads:
        assert protocol._cobs_decode(protocol._cobs_encode(payload)) == payload


def test_frame_round_trip_and_crc_rejection():
    payload = b"\0binary\0payload"
    encoded = protocol._encode_frame(protocol.MSG_PING, payload)
    assert encoded.endswith(b"\0")
    assert protocol._decode_frame(encoded[:-1]) == (protocol.MSG_PING, payload)

    decoded = bytearray(protocol._cobs_decode(encoded[:-1]))
    decoded[4] ^= 1
    damaged = protocol._cobs_encode(decoded)
    assert_raises(protocol.ProtocolError, lambda: protocol._decode_frame(damaged))


def test_frame_version_and_size_rejection():
    frame = protocol._encode_frame(protocol.MSG_PING, b"")
    decoded = bytearray(protocol._cobs_decode(frame[:-1]))
    decoded[0] = protocol.PROTO_VERSION - 1
    damaged = protocol._cobs_encode(decoded)
    assert_raises(protocol.ProtocolError, lambda: protocol._decode_frame(damaged))
    assert_raises(
        ValueError,
        lambda: protocol._encode_frame(
            protocol.MSG_PING, b"x" * (protocol.MAX_DECODED_FRAME - 7)
        ),
    )


def test_encoded_frame_limit_applies_before_delimiter():
    sensor = protocol.LightSensor("unused")
    sensor._rx.extend(b"x" * (protocol.MAX_ENCODED_FRAME + 1) + b"\0")
    assert_raises(protocol.ProtocolError, lambda: sensor._read_encoded(0.01))


def test_device_id_and_usb_identity_validation():
    assert port_detect._normalize_device_id("de657814573a0c29") == "DE657814573A0C29"
    assert_raises(ValueError, lambda: port_detect._normalize_device_id("DE657814573A0C2"))
    assert_raises(ValueError, lambda: port_detect._normalize_device_id("DE657814573A0C2Z"))
    assert_raises(ValueError, lambda: protocol.LightSensor("unused", device_id="not-a-uid"))

    expected = SimpleNamespace(
        vid=0x2E8A,
        pid=0xF00A,
        product="LightSensor v3",
        description="LightSensor v3",
    )
    wrong_vid = SimpleNamespace(
        vid=0x1234,
        pid=0x5678,
        product="LightSensor v3",
        description="LightSensor v3",
    )
    assert port_detect._is_lightsensor(expected)
    assert not port_detect._is_lightsensor(wrong_vid)


def test_terminal_device_error_clears_stream_state():
    sensor = protocol.LightSensor("unused")
    sensor.stream_header = object()
    payload = struct.pack("<IQIHH", 7, 1234, 0xFFFFFFFF, 7, 0)
    sensor._read_message = lambda timeout: (protocol.MSG_ERROR, payload)
    assert_raises(protocol.DeviceError, lambda: sensor._wait_for(7, {protocol.MSG_OK}))
    assert sensor.stream_header is None


def test_short_correlated_response_is_protocol_error():
    sensor = protocol.LightSensor("unused")
    sensor._read_message = lambda timeout: (protocol.MSG_OK, b"")
    assert_raises(protocol.ProtocolError, lambda: sensor._wait_for(1, {protocol.MSG_OK}))


def test_reconnect_failure_does_not_recurse():
    class FailingWriteSerial:
        def __init__(self, *args, **kwargs):
            self.is_open = True

        def reset_input_buffer(self):
            pass

        def reset_output_buffer(self):
            pass

        def write(self, frame):
            raise protocol.serial.SerialException("write failed")

        def close(self):
            self.is_open = False

    real_serial = protocol.serial.Serial
    log_disabled = protocol.log.disabled
    protocol.serial.Serial = FailingWriteSerial
    protocol.log.disabled = True
    sensor = protocol.LightSensor("unused", auto_reconnect=True)
    try:
        assert_raises(ConnectionError, sensor.connect)
    finally:
        protocol.serial.Serial = real_serial
        protocol.log.disabled = log_disabled
        sensor.close()


def test_connect_reset_failure_closes_serial_handle():
    instances = []

    class FailingResetSerial:
        def __init__(self, *args, **kwargs):
            self.is_open = True
            instances.append(self)

        def reset_input_buffer(self):
            raise protocol.serial.SerialException("reset failed")

        def close(self):
            self.is_open = False

    real_serial = protocol.serial.Serial
    protocol.serial.Serial = FailingResetSerial
    sensor = protocol.LightSensor("unused")
    try:
        assert_raises(protocol.serial.SerialException, sensor.connect)
    finally:
        protocol.serial.Serial = real_serial
    assert sensor.ser is None
    assert not instances[0].is_open


def test_stream_config_contracts():
    continuous = protocol.StreamConfig()
    assert continuous.gain == 1
    finite = protocol.StreamConfig(
        format=protocol.StreamFormat.RAW,
        mode=protocol.StreamMode.FINITE,
        gain_index=7,
        autogain=False,
        window=1024,
        output_count=1,
    )
    assert finite.gain == 128
    assert_raises(
        ValueError,
        lambda: protocol.StreamConfig(format=protocol.StreamFormat.RAW, autogain=True),
    )
    assert_raises(ValueError, lambda: protocol.StreamConfig(window=0))
    assert_raises(
        ValueError,
        lambda: protocol.StreamConfig(mode=protocol.StreamMode.FINITE, output_count=0),
    )
    assert_raises(ValueError, lambda: protocol.StreamConfig(output_count=1))



def test_stream_config_rejects_wrong_field_types_and_bounds():
    assert_raises(TypeError, lambda: protocol.StreamConfig(format=0))
    assert_raises(TypeError, lambda: protocol.StreamConfig(mode=0))
    assert_raises(ValueError, lambda: protocol.StreamConfig(profile_id=256))
    assert_raises(ValueError, lambda: protocol.StreamConfig(gain_index=8))
    assert_raises(TypeError, lambda: protocol.StreamConfig(autogain=1))
    assert_raises(TypeError, lambda: protocol.StreamConfig(window=1.0))
    assert_raises(TypeError, lambda: protocol.StreamConfig(output_count=1.0))

def test_sample_payloads_preserve_metadata():
    sensor = protocol.LightSensor("unused")
    raw_payload = struct.pack("<IQBBif", 7, 123456, 3, 5, -1234, 29.5)

    raw = sensor._parse_sample(protocol.MSG_SAMPLE_RAW, raw_payload)
    assert raw.sequence == 7
    assert raw.device_timestamp_us == 123456
    assert raw.gain == 8
    assert raw.status == (
        protocol.SampleStatus.ADC_POSITIVE_CLIP | protocol.SampleStatus.TIA_POSITIVE_CLIP
    )
    assert raw.value == -1234
    assert raw.temperature_c == 29.5

    volts_payload = struct.pack("<IQBBff", 8, 123999, 7, 32, -0.00125, 30.0)
    volts = sensor._parse_sample(protocol.MSG_SAMPLE_VOLTS, volts_payload)
    assert volts.sequence == 8
    assert volts.gain == 128
    assert volts.status == protocol.SampleStatus.AUTOGAIN_UNDERRANGE
    assert math.isclose(volts.value, -0.00125, rel_tol=1e-6)


def test_stream_header_payload_validates_wire_semantics():
    sensor = protocol.LightSensor("unused")
    header = sensor._parse_header(stream_header_payload())
    assert header.config == protocol.StreamConfig()
    assert math.isclose(header.measured_sps, 327.876)
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_header(stream_header_payload(format_=0, autogain=1)),
    )
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_header(stream_header_payload(dark_source=2)),
    )
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_header(stream_header_payload(dark_volts=math.inf)),
    )




def test_sample_payload_rejects_invalid_enum_bits_and_floats():
    sensor = protocol.LightSensor("unused")
    bad_gain = struct.pack("<IQBBff", 1, 2, 8, 0, 0.0, 25.0)
    bad_status = struct.pack("<IQBBff", 1, 2, 0, 0x08, 0.0, 25.0)
    bad_voltage = struct.pack("<IQBBff", 1, 2, 0, 0, math.inf, 25.0)
    bad_temperature = struct.pack("<IQBBff", 1, 2, 0, 0, 0.0, math.nan)
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_sample(protocol.MSG_SAMPLE_VOLTS, bad_gain),
    )
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_sample(protocol.MSG_SAMPLE_VOLTS, bad_status),
    )
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_sample(protocol.MSG_SAMPLE_VOLTS, bad_voltage),
    )
    assert_raises(
        protocol.ProtocolError,
        lambda: sensor._parse_sample(protocol.MSG_SAMPLE_VOLTS, bad_temperature),
    )


def test_invalid_status_and_stop_enums_are_protocol_errors():
    sensor = protocol.LightSensor("unused")
    bad_status = struct.pack(
        "<IBBBBfffQH",
        1,
        9,
        1,
        protocol.StorageState.EMPTY,
        0,
        0.0,
        math.nan,
        math.nan,
        1,
        0,
    )
    bad_stop = struct.pack("<IQIB", 0, 1, 0, 9)
    assert_raises(protocol.ProtocolError, lambda: sensor._parse_status(bad_status))
    assert_raises(protocol.ProtocolError, lambda: sensor._parse_stopped(bad_stop))


def test_synchronize_time_clears_stale_stream_state():
    sensor = protocol.LightSensor("unused")
    sensor.stream_header = object()
    sensor._events.append(object())
    sent = {}

    def write_request(message_type, request_id, payload):
        sent["utc"] = struct.unpack("<Q", payload)[0]

    def wait_for(request_id, accepted_types):
        return (
            protocol.MSG_TIME_SYNCED,
            struct.pack("<IQQ", request_id, sent["utc"], 1234),
        )

    sensor._write_request = write_request
    sensor._wait_for = wait_for
    assert sensor.synchronize_time()[1] == 1234
    assert sensor.stream_header is None
    assert not sensor._events

def test_status_nan_fields_become_none():
    sensor = protocol.LightSensor("unused")
    payload = struct.pack(
        "<IBBBBfffQH",
        42,
        protocol.DeviceState.STOPPED,
        1,
        protocol.StorageState.EMPTY,
        0,
        0.0,
        math.nan,
        math.nan,
        987654,
        0,
    )
    status = sensor._parse_status(payload)
    assert status.request_id == 42
    assert status.time_synchronized
    assert status.session_dark_volts is None
    assert status.temperature_c is None


def test_package_major_matches_protocol():
    metadata = tomllib.loads((Path(__file__).parent.parent / "pyproject.toml").read_text())
    package_major = int(metadata["project"]["version"].split(".", 1)[0])
    assert package_major == protocol.PROTO_VERSION


def test_error_payload_maps_sequence_sentinel():
    sensor = protocol.LightSensor("unused")
    payload = struct.pack("<IQIHH", 9, 1234, 0xFFFFFFFF, 11, 2)
    event = sensor._parse_error(payload)
    assert event.request_id == 9
    assert event.stream_start_device_us == 1234
    assert event.last_sample_sequence is None
    assert event.message == "ADS1220 data-ready timeout"


def test_gui_reconnect_remains_bound_to_first_device():
    device_id = "DE657814573A0C29"
    info = protocol.DeviceInfo(
        (3, 0, 0),
        3,
        protocol.REQUIRED_CAPABILITIES,
        protocol.MAX_DECODED_FRAME,
        device_id,
        True,
        protocol.StorageState.EMPTY,
    )
    requested_ids = []
    sampler = gui.SensorSampler()

    class FakeSensor:
        def __init__(self, port, *, timeout, device_id=None):
            requested_ids.append(device_id)
            self.profiles = ()

        def connect(self):
            if len(requested_ids) == 2:
                sampler._running.clear()
            return info

        def start_stream(self, config):
            return stream_header()

        def read_event(self, timeout):
            raise ConnectionError("disconnected")

        def close(self):
            pass

    real_sensor = gui.LightSensor
    gui.LightSensor = FakeSensor
    sampler._running.set()
    try:
        sampler._run()
    finally:
        gui.LightSensor = real_sensor
    assert requested_ids == [None, device_id]


def test_actual_gain_display_uses_sample_not_starting_gain():
    header = SimpleNamespace(config=SimpleNamespace(gain=1))
    no_stream = SimpleNamespace(latest=None, header=None)
    starting = SimpleNamespace(latest=None, header=header)
    autogained = SimpleNamespace(latest=SimpleNamespace(gain=128), header=header)

    assert gui._actual_gain_text(no_stream) == "—"
    assert gui._actual_gain_text(starting) == "1×"
    assert gui._actual_gain_text(autogained) == "128×"


def test_plot_refresh_slows_only_during_turbo_acquisition():
    normal_header = SimpleNamespace(config=SimpleNamespace(profile_id=1))
    turbo_header = SimpleNamespace(
        config=SimpleNamespace(profile_id=gui.TURBO_PROFILE_ID)
    )
    normal = SimpleNamespace(acquiring=True, header=normal_header)
    turbo = SimpleNamespace(acquiring=True, header=turbo_header)
    paused = SimpleNamespace(acquiring=False, header=turbo_header)

    assert gui._refresh_interval(normal) == gui.DEFAULT_REFRESH_MS
    assert gui._refresh_interval(turbo) == gui.TURBO_REFRESH_MS
    assert gui._refresh_interval(paused) == gui.DEFAULT_REFRESH_MS


def test_plot_bins_span_window_and_preserve_extrema_and_clipping():
    times = tuple(index / 1000.0 for index in range(20_000))
    values = [0.0] * len(times)
    statuses = [0] * len(times)
    values[12_000] = 10.0
    values[15_000] = -8.0
    statuses[17_000] = int(protocol.SampleStatus.ADC_POSITIVE_CLIP)

    plotted_times, plotted_values, plotted_statuses = gui._prepare_plot_points(
        times,
        values,
        statuses,
        10.0,
        20.0,
        500,
    )

    assert len(plotted_times) <= 1000
    assert plotted_times[0] == 10.0
    assert plotted_times[-1] == times[-1]
    assert max(plotted_values) == 10.0
    assert min(plotted_values) == -8.0
    assert any(status & gui.CLIP_STATUS_MASK for status in plotted_statuses)

    narrow = gui._prepare_plot_points(times, values, statuses, 12.0, 12.5, 500)
    assert narrow[0] == times[12_000:12_501]
    assert narrow[1] == tuple(values[12_000:12_501])
    assert narrow[2] == tuple(statuses[12_000:12_501])


def test_paused_plot_update_preserves_limits_and_rebins_selected_range():
    figure = gui.Figure()
    app = gui.SensorApp.__new__(gui.SensorApp)
    app.axes = figure.add_subplot(111)
    (app.line,) = app.axes.plot([], [])
    (app.clipped_line,) = app.axes.plot([], [])
    app.axes.set_xlim(12.0, 13.0)
    app.axes.set_ylim(-2.0, 3.0)
    snapshot = SimpleNamespace(
        times=tuple(index / 1000.0 for index in range(20_000)),
        values=tuple((index % 100) / 100.0 for index in range(20_000)),
        statuses=(0,) * 20_000,
        acquiring=False,
    )

    app._update_plot(snapshot)
    assert app.axes.get_xlim() == (12.0, 13.0)
    assert app.axes.get_ylim() == (-2.0, 3.0)
    assert min(app.line.get_xdata()) >= 12.0
    assert max(app.line.get_xdata()) <= 13.0

    snapshot.acquiring = True
    app._update_plot(snapshot)
    assert app.axes.get_xlim() == (
        snapshot.times[-1] - gui.WINDOW_SECONDS,
        snapshot.times[-1],
    )
    assert app.axes.get_ylim() != (-2.0, 3.0)


def test_recording_write_failure_disables_recording_without_raising():
    class FailingWriter:
        def writerow(self, row):
            raise OSError("disk full")

    class RecordingFile:
        closed = False

        def close(self):
            self.closed = True

    sampler = gui.SensorSampler("unused")
    sampler._recording = True
    sampler._record_file = RecordingFile()
    sampler._record_writer = FailingWriter()
    sampler._header = stream_header()
    sample = protocol.VoltageSample(0, 1000, 0, protocol.SampleStatus.NONE, 0.1, 25.0)
    sampler._record_sample(sample)
    snapshot = sampler.snapshot()
    assert not snapshot.recording
    assert snapshot.recording_path is None
    assert "disk full" in snapshot.status


if __name__ == "__main__":
    tests = [
        value
        for name, value in sorted(globals().items())
        if name.startswith("test_") and callable(value)
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"{len(tests)} protocol tests passed")
