//! Contract tests for the light-sensor driver against `SimTransport`
//! (the firmware emulator implementing protocol v2, docs/reference.md).
//!
//! `LightSensor::new` takes the transport by value, so to poke the sim
//! mid-test (light level, gain inspection) the tests wrap it in a shared
//! handle. `SharedSim` is a test-local type, so implementing the crate's
//! `Transport` for it is fine.

use std::cell::RefCell;
use std::rc::Rc;
use std::time::Instant;

use lightmeter::sim::SimTransport;
use lightmeter::transport::Result as TResult;
use lightmeter::{
    DARK_OFFSET_WRITE_TOLERANCE_V, DEFAULT_DARK_OFFSET_V, DEFAULT_GAIN, GAIN_VOLTAGES, LightSensor,
    PROTO_VERSION, Reading, SATURATION_VOLTAGE, Transport, best_gain, parse_identity,
};

const HEADROOM: f64 = 0.85;

/// Shared handle to a `SimTransport` so tests keep access after handing
/// the transport to the driver.
#[derive(Clone)]
struct SharedSim(Rc<RefCell<SimTransport>>);

impl SharedSim {
    fn new(sim: SimTransport) -> Self {
        Self(Rc::new(RefCell::new(sim)))
    }
    fn set_level(&self, volts: f64) {
        self.0.borrow_mut().level_volts = volts;
    }
    fn gain(&self) -> usize {
        self.0.borrow().gain()
    }
    fn autogain(&self) -> bool {
        self.0.borrow().autogain()
    }

    fn dark_offset_writes(&self) -> usize {
        self.0.borrow().dark_offset_writes()
    }
}

impl Transport for SharedSim {
    fn send(&mut self, bytes: &[u8]) -> TResult<()> {
        self.0.borrow_mut().send(bytes)
    }
    fn read_line(&mut self) -> TResult<Option<String>> {
        self.0.borrow_mut().read_line()
    }
    fn drain(&mut self) {
        self.0.borrow_mut().drain();
    }
}

/// Driver over a clean sim at `level_volts` / `noise_volts`, plus the shared
/// sim handle for mid-test inspection and level changes.
fn sensor_with(level_volts: f64, noise_volts: f64) -> (LightSensor<SharedSim>, SharedSim) {
    let mut raw = SimTransport::default();
    raw.level_volts = level_volts;
    raw.noise_volts = noise_volts;
    let sim = SharedSim::new(raw);
    let sensor = LightSensor::new(sim.clone()).expect("handshake against sim must succeed");
    (sensor, sim)
}

fn read_value(s: &mut LightSensor<SharedSim>) -> f64 {
    s.read()
        .expect("link ok")
        .expect("sim always answers")
        .value
}

//
// best_gain
//

/// The chosen index always keeps the input strictly below its own ceiling
/// (full_scale.min(3.2) * headroom), except when even the widest range (0)
/// cannot hold the signal.
#[test]
fn best_gain_result_keeps_ceiling_above_input() {
    for v in [0.0, 0.05, 0.1, 0.2, 0.4, 0.8, 1.5, 2.0, 2.7] {
        let i = best_gain(v, HEADROOM);
        let ceiling = GAIN_VOLTAGES[i].min(SATURATION_VOLTAGE) * HEADROOM;
        assert!(
            v < ceiling,
            "best_gain({v}) = {i}, but ceiling {ceiling} <= input"
        );
        // And it is the *tightest* such index: the next-narrower range
        // (if any) must NOT fit.
        if i + 1 < GAIN_VOLTAGES.len() {
            let narrower = GAIN_VOLTAGES[i + 1].min(SATURATION_VOLTAGE) * HEADROOM;
            assert!(
                v >= narrower,
                "best_gain({v}) = {i} is not the tightest fit"
            );
        }
    }
}

#[test]
fn best_gain_picks_expected_indices() {
    // 0.1 V fits the narrowest range (0.256 * 0.85 = 0.2176).
    assert_eq!(best_gain(0.1, HEADROOM), 5);
    // 3.0 V: indices 0-1 have their ceiling capped at 3.2 * 0.85 = 2.72,
    // so nothing fits -> fall back to the widest range.
    assert_eq!(best_gain(3.0, HEADROOM), 0);
    // Way beyond any range.
    assert_eq!(best_gain(10.0, HEADROOM), 0);
}

/// Exactly at a ceiling the comparison is strict (`<`), so the signal
/// spills to the next wider gain.
#[test]
fn best_gain_exact_threshold_goes_wider() {
    for i in 1..GAIN_VOLTAGES.len() {
        let ceiling = GAIN_VOLTAGES[i].min(SATURATION_VOLTAGE) * HEADROOM;
        let at = best_gain(ceiling, HEADROOM);
        assert!(
            at < i,
            "input == ceiling of {i} must pick a wider gain, got {at}"
        );
        // Just below the ceiling still fits index i.
        assert_eq!(best_gain(ceiling * 0.999, HEADROOM), i);
    }
}

//
// parse_identity
//

#[test]
fn parse_identity_full_line() {
    let line = "lightsensor proto=1 fw=abc id=AA:BB sps=860 vsat=3.20 \
                gains=6.144,4.096,2.048,1.024,0.512,0.256";
    let info = parse_identity(line).expect("valid identity line");
    assert_eq!(info.name, "lightsensor");
    assert_eq!(info.proto, 1);
    assert_eq!(info.firmware, "abc");
    assert_eq!(info.id, "AA:BB");
    assert_eq!(info.fields.get("sps").map(String::as_str), Some("860"));
    assert_eq!(info.fields.get("vsat").map(String::as_str), Some("3.20"));
    assert_eq!(
        info.fields.get("gains").map(String::as_str),
        Some("6.144,4.096,2.048,1.024,0.512,0.256")
    );
}

#[test]
fn parse_identity_rejects_garbage_and_missing_proto() {
    assert_eq!(parse_identity("!!! total garbage ,,, 123"), None);
    assert_eq!(parse_identity("lightsensor fw=abc id=AA:BB"), None); // no proto
    assert_eq!(parse_identity("lightsensor proto=banana fw=abc"), None); // unparsable proto
    assert_eq!(parse_identity(""), None);
}

//
// handshake
//

#[test]
fn handshake_populates_identity() {
    let (s, _) = sensor_with(1.0, 0.0);
    let info = s.info.as_ref().expect("sim reports identity");
    assert_eq!(info.proto, 2);
    assert_eq!(info.name, "lightsensor");
    assert_eq!(s.gain, DEFAULT_GAIN);
}

//
// read()
//

#[test]
fn read_reports_percent_of_full_scale() {
    let (mut s, sim) = sensor_with(1.0, 0.0);
    let expected = (1.0 - DEFAULT_DARK_OFFSET_V) / GAIN_VOLTAGES[DEFAULT_GAIN] * 100.0;
    let got = read_value(&mut s);
    assert!(
        (got - expected).abs() < 0.02,
        "got {got}, expected ~{expected}"
    );

    // Changing the simulated light level changes the reading proportionally.
    sim.set_level(2.0);
    let brighter = read_value(&mut s);
    let expected_brighter = (2.0 - DEFAULT_DARK_OFFSET_V) / GAIN_VOLTAGES[DEFAULT_GAIN] * 100.0;
    assert!(
        (brighter - expected_brighter).abs() < 0.02,
        "got {brighter}, expected ~{expected_brighter}",
    );
}

//
// saturation semantics
//

#[test]
fn low_gain_rail_saturation_sets_sensor_sat_only() {
    for gain in [0usize, 1] {
        let (mut s, _) = sensor_with(SATURATION_VOLTAGE, 0.0);
        assert!(s.set_gain(gain).unwrap());
        let r = s.read().unwrap().unwrap();
        assert!(
            r.sensor_sat,
            "gain {gain}, level 3.2 V: op-amp rail saturation expected"
        );
        assert!(!r.adc_sat, "gain {gain}: flags must be mutually exclusive");
    }
}

#[test]
fn high_gain_adc_clip_sets_adc_sat_only() {
    for gain in [4usize, 5] {
        // Well above full scale for this gain; full_scale < 3.2 here, so
        // sensor_sat can never fire — the flags stay mutually exclusive.
        let level = GAIN_VOLTAGES[gain] * 2.0;
        let (mut s, _) = sensor_with(level, 0.0);
        assert!(s.set_gain(gain).unwrap());
        let r = s.read().unwrap().unwrap();
        assert!(r.adc_sat, "gain {gain}, level {level} V: ADC clip expected");
        assert!(
            !r.sensor_sat,
            "gain {gain}: flags must be mutually exclusive"
        );
        let expected = 100.0 - DEFAULT_DARK_OFFSET_V / GAIN_VOLTAGES[gain] * 100.0;
        assert!(
            (r.value - expected).abs() < 0.01,
            "clipped raw must retain dark correction"
        );
    }
}

//
// set_gain / get_gain
//

#[test]
fn set_gain_round_trip() {
    let (mut s, sim) = sensor_with(1.0, 0.0);
    for gain in 0..GAIN_VOLTAGES.len() {
        assert!(s.set_gain(gain).unwrap(), "device must ack gain {gain}");
        assert_eq!(s.gain, gain, "driver-side mirror");
        assert_eq!(sim.gain(), gain, "device-side state");
        assert_eq!(s.get_gain().unwrap(), Some(gain), "G query");
    }
}

#[test]
fn set_gain_invalid_index_is_rejected_without_state_change() {
    let (mut s, sim) = sensor_with(1.0, 0.0);
    let before = s.gain;
    assert_eq!(s.set_gain(9).unwrap(), false, "device must nak index 9");
    assert_eq!(s.gain, before, "driver gain unchanged after nak");
    assert_eq!(sim.gain(), before, "device gain unchanged after nak");
}

//
// zero()
//

#[test]
fn zero_measures_offset_and_subtracts_it() {
    let (mut s, _) = sensor_with(1.0, 0.0);
    let offset = s.zero(5).unwrap();
    assert!(
        (offset - 1.0).abs() < 1e-3,
        "dark offset should be ~1.0 V, got {offset}"
    );
    assert!(s.is_zeroed());
    assert_eq!(s.zero_offset(), offset);

    // With the same level, the corrected reading is ~0 %.
    let zeroed = read_value(&mut s);
    assert!(
        zeroed.abs() < 0.02,
        "post-zero reading should be ~0%, got {zeroed}"
    );

    // clear_zero restores the persisted electrical baseline, not uncorrected raw.
    s.clear_zero();
    assert!(s.is_zeroed());
    assert!((s.zero_offset() - DEFAULT_DARK_OFFSET_V).abs() < 1e-6);
    let corrected = read_value(&mut s);
    let expected = (1.0 - DEFAULT_DARK_OFFSET_V) / GAIN_VOLTAGES[DEFAULT_GAIN] * 100.0;
    assert!(
        (corrected - expected).abs() < 0.02,
        "after clear_zero got {corrected}, expected ~{expected}"
    );
}

#[test]
fn device_dark_offset_persists_and_session_zero_overrides_it() {
    let (mut s, sim) = sensor_with(0.1, 0.0);
    assert!((s.device_dark_offset() - DEFAULT_DARK_OFFSET_V).abs() < 1e-6);
    assert_eq!(sim.dark_offset_writes(), 0);
    assert!(s.set_device_dark_offset(DEFAULT_DARK_OFFSET_V).unwrap());
    assert!(
        s.set_device_dark_offset(DEFAULT_DARK_OFFSET_V + DARK_OFFSET_WRITE_TOLERANCE_V / 2.0)
            .unwrap()
    );
    assert_eq!(
        sim.dark_offset_writes(),
        0,
        "matching dark values must not wear flash"
    );
    assert!(s.set_device_dark_offset(0.08).unwrap());
    assert_eq!(sim.dark_offset_writes(), 1);
    assert!((s.device_dark_offset() - 0.08).abs() < 1e-12);

    let session = s.zero(5).unwrap();
    assert!((session - 0.1).abs() < 1e-3);
    assert_eq!(s.session_zero_offset(), Some(session));
    s.clear_zero();
    assert_eq!(s.session_zero_offset(), None);
    assert!((s.zero_offset() - 0.08).abs() < 1e-12);

    sim.set_level(0.08);
    assert!(read_value(&mut s).abs() < 0.02);
    assert!(s.reset_device_dark_offset().unwrap());
    assert!((s.device_dark_offset() - DEFAULT_DARK_OFFSET_V).abs() < 1e-12);
    assert!(!s.set_device_dark_offset(0.3).unwrap());
}

/// The dark offset is stored in volts, so it stays physically correct
/// across gain changes: at the new gain the correction is offset/fs*100.
#[test]
fn zero_offset_survives_gain_change() {
    let (mut s, sim) = sensor_with(1.0, 0.0); // zero at default gain 1
    let offset = s.zero(5).unwrap();
    assert!((offset - 1.0).abs() < 1e-3);

    sim.set_level(1.5);
    assert!(s.set_gain(2).unwrap());
    let got = read_value(&mut s);
    let expected = (1.5 - offset) / GAIN_VOLTAGES[2] * 100.0;
    assert!(
        (got - expected).abs() < 0.05,
        "at gain 2 got {got}, expected ~{expected}"
    );
}

//
// firmware-side autoexposure via read() (set_autogain(true))
//
// The band and gain ladder are the FIRMWARE's, emulated by `SimTransport`.
// The driver never steps gain itself: it flips autogain on the device, reads,
// and records the settled gain the device reports in the 4th field.
//

/// Firmware autoexposure band, in % of full scale (mirrors `sim.rs` / the
/// firmware). The consts used to live in the driver; autoexposure moved to the
/// device, so the band is no longer a driver concern — the tests keep a local
/// copy only to assert where a settled sample lands.
const BAND_LOW_PCT: f64 = 40.0;
const BAND_HIGH_PCT: f64 = 90.0;

fn read_full(s: &mut LightSensor<SharedSim>) -> Reading {
    s.read().expect("link ok").expect("sim always answers")
}

/// A dim scene wastes dynamic range at the default gain. The device steps
/// toward MORE sensitive gains (higher index) until in-band or railed at 5;
/// the driver only reports the settled gain, and it agrees with the sim.
#[test]
fn autoexpose_dim_steps_to_more_sensitive_gain() {
    let (mut dim, sim) = sensor_with(0.1, 0.0);
    let start = dim.gain;
    assert_eq!(start, DEFAULT_GAIN);
    dim.set_autogain(true).unwrap();

    let r = read_full(&mut dim);
    assert!(
        dim.gain > start,
        "dim scene must settle at a more sensitive gain (up from {start}), got {}",
        dim.gain
    );
    assert_eq!(
        sim.gain(),
        dim.gain,
        "device did the stepping; driver reports its gain"
    );
    // In-band, or railed at the most sensitive gain (0.1 V lands just under
    // the LOW band at gain 5, so the rail is the terminating condition).
    assert!(
        r.value >= BAND_LOW_PCT || dim.gain == GAIN_VOLTAGES.len() - 1,
        "returned value {} must be in-band or the walk railed at gain 5",
        r.value
    );
}

/// A near-rail bright scene at a very sensitive gain is saturated. The device
/// steps toward WIDER ranges (lower index) until the sample clears saturation
/// and drops below HIGH — unless it rails at gain 0.
#[test]
fn autoexpose_bright_steps_to_wider_range() {
    let (mut bright, sim) = sensor_with(2.5, 0.0);
    assert!(
        bright.set_gain(5).unwrap(),
        "seed a sensitive (saturating) gain"
    );
    bright.set_autogain(true).unwrap();

    let r = read_full(&mut bright);
    assert!(
        bright.gain < 5,
        "bright scene must widen the range (down from 5), got {}",
        bright.gain
    );
    assert_eq!(
        sim.gain(),
        bright.gain,
        "device did the stepping; driver reports its gain"
    );
    if bright.gain > 0 {
        assert!(
            !r.adc_sat,
            "off the rail the sample must not be ADC-saturated, got {r:?}"
        );
        assert!(
            r.value < BAND_HIGH_PCT,
            "off the rail the sample must drop below HIGH, got {}",
            r.value
        );
    }
}

/// With autogain OFF at a known gain and a clean level, read() takes a single
/// sample: the 4 fields parse, `value` is dark-corrected raw / full-scale, and
/// `self.gain` equals the gain the device echoed in the 4th field.
#[test]
fn read_parses_four_fields_at_manual_gain() {
    let (mut s, sim) = sensor_with(1.0, 0.0);
    assert!(s.set_gain(2).unwrap(), "manual gain 2 (full scale 2.048 V)");
    assert!(!s.autogain_enabled(), "manual gain turns autogain off");

    let r = read_full(&mut s);
    // raw = round(1.0 / 2.048 * 32767); correction is applied last in volts.
    let full_scale = GAIN_VOLTAGES[2];
    let expected = (1.0 - DEFAULT_DARK_OFFSET_V) / full_scale * 100.0;
    assert!(
        (r.value - expected).abs() < 0.05,
        "value {} must track corrected level/full-scale ({expected})",
        r.value
    );
    assert_eq!(s.gain, 2, "self.gain set from the 4th field");
    assert_eq!(sim.gain(), 2, "device gain unchanged (no autoexposure)");
    assert!(
        !r.sensor_sat && !r.adc_sat,
        "clean mid level is not saturated, got {r:?}"
    );
}

/// set_gain() disables firmware autoexposure: after enabling autogain and then
/// setting a manual gain, both the driver mirror and the device report autogain
/// OFF, and a subsequent read() at a mid level does not move off that gain.
#[test]
fn set_gain_disables_autogain() {
    let (mut s, sim) = sensor_with(0.7, 0.0); // gain 3: 68.4% — mid-band, no step
    s.set_autogain(true).unwrap();
    assert!(s.autogain_enabled() && sim.autogain(), "autogain enabled");

    assert!(s.set_gain(3).unwrap(), "manual gain 3");
    assert!(!s.autogain_enabled(), "driver mirror cleared");
    assert!(!sim.autogain(), "device autogain cleared");

    let _ = read_full(&mut s);
    assert_eq!(
        s.gain, 3,
        "autogain off: read() must not step off the manual gain"
    );
    assert_eq!(sim.gain(), 3, "device gain unchanged");
}

/// get_autogain() reflects the device's `A` reply: enabled + current gain after
/// set_autogain(true), and disabled + the new gain after a manual set_gain().
#[test]
fn get_autogain_reports_state_and_gain() {
    let (mut s, _sim) = sensor_with(1.0, 0.0);
    s.set_autogain(true).unwrap();
    assert_eq!(
        s.get_autogain().unwrap(),
        Some((true, DEFAULT_GAIN)),
        "autogain on, still at the default gain (no read() yet)"
    );

    assert!(s.set_gain(2).unwrap());
    assert_eq!(
        s.get_autogain().unwrap(),
        Some((false, 2)),
        "manual gain 2, autogain off"
    );
}

/// One step lands a mid-dim level in the [LOW, HIGH) band, and a second read()
/// at the settled gain does not move again — the band is wider than the
/// adjacent-gain ratio, so autoexposure never oscillates.
#[test]
fn autoexpose_converges_in_one_step_and_is_stable() {
    let (mut s, sim) = sensor_with(1.0, 0.0); // gain 1: 24.4% (under); gain 2: 48.8% (in-band)
    s.set_autogain(true).unwrap();

    let first = read_full(&mut s);
    assert_eq!(
        s.gain,
        DEFAULT_GAIN + 1,
        "exactly one step up from the default"
    );
    assert!(
        first.value >= BAND_LOW_PCT && first.value < BAND_HIGH_PCT,
        "settled value {} must be inside the band",
        first.value
    );

    let settled_gain = s.gain;
    let second = read_full(&mut s);
    assert_eq!(
        s.gain, settled_gain,
        "already in-band: read() must not change gain"
    );
    assert_eq!(sim.gain(), settled_gain, "device gain also unchanged");
    assert!(
        (second.value - first.value).abs() < 1e-9,
        "stable value across reads"
    );
}

/// The walk stops at a rail even when still out of band: an extremely dim
/// scene drives to the most sensitive gain (5) and stops; an extremely bright
/// scene drives to the widest range (0) and stops.
#[test]
fn autoexpose_stops_at_gain_rails() {
    let (mut faint, faint_sim) = sensor_with(0.001, 0.0);
    faint.set_autogain(true).unwrap();
    let _ = read_full(&mut faint);
    assert_eq!(
        faint.gain,
        GAIN_VOLTAGES.len() - 1,
        "faint scene rails at the most sensitive gain"
    );
    assert_eq!(faint_sim.gain(), GAIN_VOLTAGES.len() - 1);

    let (mut blazing, blazing_sim) = sensor_with(6.0, 0.0);
    blazing.set_autogain(true).unwrap();
    let _ = read_full(&mut blazing);
    assert_eq!(blazing.gain, 0, "blazing scene rails at the widest range");
    assert_eq!(blazing_sim.gain(), 0);
}

/// With autogain OFF, read() takes a single sample at the manual gain and
/// never re-exposes, no matter how far out of band the level is.
#[test]
fn autoexpose_off_leaves_gain_untouched() {
    // Extremely dim: with autogain on this would rail to gain 5.
    let (mut s, sim) = sensor_with(0.001, 0.0);
    assert!(s.set_gain(3).unwrap());
    assert!(!s.autogain_enabled(), "autogain defaults off");

    for _ in 0..3 {
        let _ = read_full(&mut s);
        assert_eq!(s.gain, 3, "autogain off: gain must stay put");
        assert_eq!(sim.gain(), 3);
    }
}

//
// mute (resync path)
//

/// A dead device must not fail construction: the handshake is best-effort.
/// It waits out the device command timeout (~5.5 s) at most twice, so the
/// whole thing must complete well under 15 s with `info == None`.
#[test]
fn mute_device_still_constructs_best_effort() {
    let mut sim = SimTransport::default();
    sim.mute = true;
    let start = Instant::now();
    let s = LightSensor::new(sim).expect("handshake is best-effort");
    let elapsed = start.elapsed();
    assert!(s.info.is_none(), "mute device cannot report identity");
    assert!(
        elapsed.as_secs_f64() < 15.0,
        "resync must bound its silence, took {elapsed:?}"
    );
}

//
// averaging
//

fn variance(samples: &[f64]) -> f64 {
    let mean = samples.iter().sum::<f64>() / samples.len() as f64;
    samples.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / samples.len() as f64
}

/// Firmware-side averaging (`r<n>`) reduces noise variance ~1/n. With a
/// generous statistical margin: average=100 must beat average=1 by >=4x.
#[test]
fn averaging_reduces_variance() {
    let collect = |average: u32| -> Vec<f64> {
        let (mut s, _) = sensor_with(1.0, 0.3);
        s.average = average;
        (0..30).map(|_| read_value(&mut s)).collect()
    };
    let noisy = variance(&collect(1));
    let smooth = variance(&collect(100));
    assert!(noisy > 0.0, "noise must actually show up at average=1");
    assert!(
        smooth < noisy / 4.0,
        "average=100 variance {smooth} not well below average=1 variance {noisy}"
    );
}

/// The crate's semver MAJOR is pinned to the wire protocol version it
/// speaks: `lightmeter = "2"` is a promise of proto-2 compatibility. A
/// `PROTO_VERSION` bump without a matching `Cargo.toml` major bump (or vice
/// versa) breaks that promise silently — catch it here instead.
#[test]
fn package_major_version_matches_protocol() {
    let major: u32 = env!("CARGO_PKG_VERSION_MAJOR").parse().unwrap();
    assert_eq!(
        major, PROTO_VERSION,
        "Cargo.toml major version ({major}) must equal PROTO_VERSION ({PROTO_VERSION}) — bump both together"
    );
}
