import math
import struct
from pathlib import Path
import tomllib

import lightmeter.sensor as protocol


def assert_raises(exception_type, function):
    try:
        function()
    except exception_type:
        return
    raise AssertionError(f"expected {exception_type.__name__}")


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
