"""Unit tests for the calibration / volts→units conversion math.

Pure functions only — no hardware required. Run with: uv run tests/test_calibration.py
"""

import os
import tomllib

from lightmeter.sensor import (
    DEFAULT_DARK_OFFSET_V,
    DEFAULT_GAIN,
    GAIN_VOLTAGES,
    LightSensor,
    PROTO_VERSION,
    daylight_spectrum,
    default_calibration,
    luminous_efficacy,
    parse_calibration,
)

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


def test_provenance_defaults_to_measured():
    # A cal without an explicit provenance is assumed real.
    cal = make_cal()
    assert cal.provenance == "measured"
    assert cal.is_nominal is False


def test_bundled_default_calibration():
    # The packaged BPW34 fallback: real spectral shape, flagged nominal, with a
    # datasheet-derived (not measured) absolute scale for R_f = 2 MΩ.
    cal = default_calibration()
    assert cal is not None
    assert cal.provenance == "datasheet-typical"
    assert cal.is_nominal is True
    assert cal.scale_units == "W/m^2"
    # Nominal scale: 1/(R_peak*A*R_f) with R_peak≈0.646 A/W, A=7.5e-6 m², R_f=2e6 Ω.
    assert approx(cal.scale_factor, 0.103, tol=0.005)
    # source=None ⇒ assume the 900 nm peak ⇒ physical = scale_factor * V.
    assert approx(cal.voltage_to_value(2.0), 2 * cal.scale_factor, tol=1e-9)
    # Peak near 900 nm, falls off to the 10% points at the band edges.
    assert approx(cal.responsivity_at(900), 1.0, tol=1e-6)
    assert cal.responsivity_at(660) > cal.responsivity_at(450)
    assert cal.responsivity_at(1200) == 0.0


def test_daylight_spectrum_shape():
    wl, inten = daylight_spectrum()
    # Spans the BPW34 band, ascending, all positive.
    assert wl[0] == 380 and wl[-1] == 1100
    assert all(b > 0 for b in inten)
    assert wl == sorted(wl)
    # 6500 K blackbody peaks in the visible (~450 nm), not the IR.
    assert wl[inten.index(max(inten))] < 600


def test_default_daylight_conversion():
    # The GUI's volts→W/m² path: default cal weighted by daylight. Daylight is
    # blue-heavy where BPW34 is weak, so R̄ < 1 and the factor exceeds the
    # peak-monochromatic scale_factor.
    cal = default_calibration()
    day = daylight_spectrum()
    r_bar = cal.source_weighted_responsivity(*day)
    assert 0.3 < r_bar < 0.7
    factor = cal.voltage_to_value(1.0, source=day)
    assert factor > cal.scale_factor
    assert approx(factor, cal.scale_factor / r_bar, tol=1e-9)


def test_luminous_efficacy():
    # A monochromatic source near the photopic peak -> close to 683 lm/W
    # (the 10 nm table tops out at V=0.995 around 550–560 nm).
    peak = ([554, 555, 556], [1.0, 1.0, 1.0])
    assert 675 < luminous_efficacy(*peak) <= 683
    # Pure near-IR (outside the photopic band) carries no luminous flux.
    ir = ([1000, 1010, 1020], [1.0, 1.0, 1.0])
    assert luminous_efficacy(*ir) == 0.0
    # Daylight: broadband incl. IR -> efficacy well below the 683 peak.
    k = luminous_efficacy(*daylight_spectrum())
    assert 80 < k < 250


def test_lux_factor_from_default():
    # The GUI's volts→lux path: radiometric factor × daylight luminous efficacy.
    cal = default_calibration()
    day = daylight_spectrum()
    phys = cal.voltage_to_value(1.0, source=day)  # W/m² per V
    lux = phys * luminous_efficacy(*day)  # lux per V
    assert lux > 0
    # Low-light 2 MΩ front end: full-scale (~3.27 V) is a modest indoor level.
    assert 20 < lux * 3.266 < 400



def bare_sensor(device_offset=DEFAULT_DARK_OFFSET_V):
    """Construct only the state used by dark-offset unit tests; no serial port."""
    sensor = LightSensor.__new__(LightSensor)
    sensor._device_dark_offset_v = device_offset
    sensor._session_zero_offset_v = None
    sensor.autogain = False
    sensor.auto_reconnect = False
    sensor.gain = DEFAULT_GAIN
    return sensor


def test_default_dark_offset_matches_schematic_divider():
    assert approx(DEFAULT_DARK_OFFSET_V, 3.3 * 270 / (13_000 + 270))


def test_session_zero_overrides_and_clears_to_device_offset():
    sensor = bare_sensor(0.067)
    sensor._measure_uncorrected_offset = lambda n: 0.100
    assert approx(sensor.zero(5), 0.100)
    assert approx(sensor.session_zero_offset_v, 0.100)
    sensor.clear_zero()
    assert sensor.session_zero_offset_v is None
    assert approx(sensor.zero_offset, 0.067)


def test_dark_offset_is_applied_last_without_changing_saturation_flags():
    sensor = bare_sensor(0.067)
    raw = 10_000
    sensor._read_raw = lambda: (raw, True, False, DEFAULT_GAIN)
    reading = sensor.read()
    expected = raw / 32767 * 100 - 0.067 / GAIN_VOLTAGES[DEFAULT_GAIN] * 100
    assert approx(reading.value, expected)
    assert reading.sensor_sat is True
    assert reading.adc_sat is False


def test_device_dark_calibration_uses_uncorrected_measurement():
    sensor = bare_sensor(0.067)
    sensor._session_zero_offset_v = 0.100
    sensor._measure_uncorrected_offset = lambda n: 0.080
    saved = []
    sensor.set_device_dark_offset = lambda offset: saved.append(offset) or True
    assert approx(sensor.calibrate_device_dark_offset(200), 0.080)
    assert saved == [0.080]
    assert approx(sensor.session_zero_offset_v, 0.100)

def test_package_major_matches_protocol():
    """The package's semver MAJOR is pinned to the wire protocol version it
    speaks (same contract as the Rust crate, see rust/tests/sensor.rs):
    `lightmeter==2.*` promises proto-2 compatibility. A PROTO_VERSION bump
    without a matching pyproject.toml major bump (or vice versa) breaks that
    promise silently -- catch it here instead."""
    with open(os.path.join(ROOT, "pyproject.toml"), "rb") as f:
        pkg_version = tomllib.load(f)["project"]["version"]
    major = int(pkg_version.split(".")[0])
    assert major == PROTO_VERSION, (
        f"pyproject.toml major version ({major}) must equal PROTO_VERSION "
        f"({PROTO_VERSION}) -- bump both together"
    )


def run():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"  ok  {t.__name__}")
    print(f"\n{len(tests)} passed")


if __name__ == "__main__":
    run()
