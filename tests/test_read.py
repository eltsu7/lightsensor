"""Connected LightSensor v3 smoke test. Run directly; do not test-discover."""

from lightmeter import (
    ErrorEvent,
    LightSensor,
    StreamConfig,
    StreamMode,
    StreamStopped,
    VoltageSample,
)


with LightSensor(timeout=3.0) as sensor:
    print(sensor.info)
    for profile in sensor.profiles:
        print(profile)
    sensor.start_stream(StreamConfig(mode=StreamMode.FINITE, output_count=10))
    samples = []
    while True:
        event = sensor.read_event(5.0)
        if isinstance(event, VoltageSample):
            samples.append(event)
            print(event)
        elif isinstance(event, ErrorEvent):
            raise RuntimeError(event)
        elif isinstance(event, StreamStopped):
            assert event.delivered_outputs == 10
            break

assert len(samples) == 10
assert [sample.sequence for sample in samples] == list(range(10))
