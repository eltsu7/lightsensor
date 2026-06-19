"""Unit tests for the calibration / volts→units conversion math.

Pure functions only — no hardware required. Run with: uv run tests/test_calibration.py
"""

import os

from lightmeter.sensor import parse_calibration

# Repo root, so the dummy data path is independent of the caller's cwd.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def approx(a, b, tol=1e-9):
    return abs(a - b) < tol


def make_cal(scale="2.0", units="W/m^2"):
    """A tiny triangular responsivity peaking at 1.0 at 550 nm."""
    text = (
        f"# scale_factor: {scale}\n"
        f"# scale_units: {units}\n"
        "wavelength_nm,responsivity\n"
        "500,0.0\n"
        "550,1.0\n"
        "600,0.0\n"
    )
    return parse_calibration(text)


def test_parse_metadata():
    cal = make_cal()
    assert cal.scale_factor == 2.0
    assert cal.scale_units == "W/m^2"
    assert cal.wavelengths == [500, 550, 600]
    assert cal.responsivity == [0.0, 1.0, 0.0]


def test_responsivity_interpolation():
    cal = make_cal()
    assert approx(cal.responsivity_at(550), 1.0)
    assert approx(cal.responsivity_at(525), 0.5)   # halfway up the ramp
    assert approx(cal.responsivity_at(575), 0.5)   # halfway down
    # Outside the measured band -> no calibrated response.
    assert cal.responsivity_at(400) == 0.0
    assert cal.responsivity_at(700) == 0.0
    # Exact endpoints.
    assert cal.responsivity_at(500) == 0.0
    assert cal.responsivity_at(600) == 0.0


def test_source_weighted_responsivity():
    cal = make_cal()
    # A narrow source centered on the peak should weight close to R=1.
    src_wl = [549, 550, 551]
    src_i = [1.0, 1.0, 1.0]
    r_bar = cal.source_weighted_responsivity(src_wl, src_i)
    assert 0.97 < r_bar <= 1.0
    # A flat source across the whole band: triangular R averages to ~0.5.
    flat_wl = [500, 550, 600]
    flat_i = [1.0, 1.0, 1.0]
    r_flat = cal.source_weighted_responsivity(flat_wl, flat_i)
    assert approx(r_flat, 0.5, tol=1e-6)


def test_voltage_to_value_no_source():
    cal = make_cal(scale="2.0")
    # No source -> R̄ defaults to 1.0 -> physical = scale * V.
    assert approx(cal.voltage_to_value(1.5), 3.0)
    assert approx(cal.voltage_to_value(0.0), 0.0)


def test_voltage_to_value_with_source():
    cal = make_cal(scale="2.0")
    # Flat source -> R̄ = 0.5 -> physical = scale * V / 0.5 = 4 * V.
    flat = ([500, 550, 600], [1.0, 1.0, 1.0])
    assert approx(cal.voltage_to_value(1.0, source=flat), 4.0, tol=1e-5)


def test_uncalibrated_returns_none():
    # No scale_factor in metadata.
    cal = parse_calibration("wavelength_nm,responsivity\n550,1.0\n560,1.0\n")
    assert cal.scale_factor is None
    assert cal.voltage_to_value(1.0) is None


def test_source_outside_band_returns_none():
    cal = make_cal()
    # Source entirely outside the calibrated band -> R̄ = 0 -> None.
    far = ([700, 750, 800], [1.0, 1.0, 1.0])
    assert cal.voltage_to_value(1.0, source=far) is None


def test_dummy_csv_loads():
    # The shipped dummy file parses and has the expected shape.
    with open(os.path.join(ROOT, "data", "calibration_dummy.csv")) as f:
        cal = parse_calibration(f.read())
    assert len(cal.wavelengths) == 400
    assert cal.scale_factor == 1.0
    assert cal.metadata.get("device_id") == "dummy-0001"


def run():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)} passed")


if __name__ == "__main__":
    run()
