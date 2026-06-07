"""Quick smoke test: auto-connect, read N samples, print them with timing."""
import time
from lightsensor import LightSensor, GAIN_LABELS

N = 10

with LightSensor() as sensor:
    print(f"Connected. gain={sensor.gain} ({GAIN_LABELS[sensor.gain]})")
    t0 = time.perf_counter()
    last = t0
    count = 0
    for i in range(N):
        r = sensor.read()
        now = time.perf_counter()
        dt = (now - last) * 1000
        last = now
        if r is None:
            print(f"[{i}] read failed (None)")
            continue
        count += 1
        print(f"[{i}] value={r.value:6.2f}%  sensor_sat={r.sensor_sat}  "
              f"adc_sat={r.adc_sat}  dt={dt:6.1f} ms")
    total = time.perf_counter() - t0
    if count:
        print(f"\n{count}/{N} ok in {total*1000:.0f} ms  "
              f"-> {count/total:.1f} samples/s")
