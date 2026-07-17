//! Firmware emulator implementing protocol v2 line-by-line — the same test
//! double role `SimBus` plays for the turret. Drives the REAL parse paths in
//! the driver, and doubles as the hardware-free backend for GUIs.
//!
//! Autoexposure lives on the (simulated) device, mirroring `lightsensor.ino`:
//! `a1`/`a0` toggle it, `A` queries it, and `r` reports the settled gain as a
//! 4th field.

use std::collections::VecDeque;

use crate::sensor::{DEFAULT_DARK_OFFSET_V, GAIN_VOLTAGES, MAX_DARK_OFFSET_V, SATURATION_VOLTAGE};
use crate::transport::{Result, Transport};

/// Autoexposure band on % of full scale (matches the firmware).
const AUTOGAIN_LOW_PCT: f64 = 40.0;
const AUTOGAIN_HIGH_PCT: f64 = 90.0;

/// Simulated device state; light level is set by the test/GUI in volts.
pub struct SimTransport {
    /// Op-amp output voltage the "photodiode" currently produces.
    pub level_volts: f64,
    /// Deterministic pseudo-noise amplitude in volts (0 = clean).
    pub noise_volts: f64,
    gain: usize,
    autogain: bool,
    replies: VecDeque<String>,
    dark_offset_v: f64,
    rng: u64,
    /// When set, the device plays dead (tests the resync path).
    pub mute: bool,
}

impl Default for SimTransport {
    fn default() -> Self {
        Self {
            level_volts: 1.0,
            noise_volts: 0.0,
            gain: crate::sensor::DEFAULT_GAIN,
            autogain: false,
            replies: VecDeque::new(),
            rng: 0x1234_5678_9ABC_DEF0,
            dark_offset_v: DEFAULT_DARK_OFFSET_V,
            mute: false,
        }
    }
}

impl SimTransport {
    pub fn gain(&self) -> usize {
        self.gain
    }

    pub fn autogain(&self) -> bool {
        self.autogain
    }

    fn noise(&mut self) -> f64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        let unit = (self.rng >> 40) as f64 / (1u64 << 24) as f64; // 0..1
        (unit * 2.0 - 1.0) * self.noise_volts
    }

    /// One noisy sample → (raw, sensor_sat, adc_sat) at the current gain.
    /// Saturation per docs/reference.md: gains 0–1 saturate at the op-amp
    /// rail (3.2 V), gains 2–5 saturate the ADC counter.
    fn one_sample(&mut self) -> (i32, bool, bool) {
        let volts = (self.level_volts + self.noise()).max(0.0);
        let full_scale = GAIN_VOLTAGES[self.gain];
        let sensor_sat = full_scale > SATURATION_VOLTAGE && volts >= SATURATION_VOLTAGE;
        let raw = ((volts / full_scale) * 32767.0).round().min(32767.0) as i32;
        let adc_sat = full_scale < SATURATION_VOLTAGE && raw >= 32767;
        (raw, sensor_sat, adc_sat)
    }

    /// Step gain (single samples) until in-band or railed — the firmware's
    /// `autoExpose()`.
    fn autoexpose(&mut self) {
        for _ in 0..GAIN_VOLTAGES.len() {
            let (raw, sensor_sat, adc_sat) = self.one_sample();
            let pct = raw as f64 / 32767.0 * 100.0;
            let over = sensor_sat || adc_sat || pct >= AUTOGAIN_HIGH_PCT;
            let under = pct < AUTOGAIN_LOW_PCT;
            if over && self.gain > 0 {
                self.gain -= 1;
            } else if under && self.gain < GAIN_VOLTAGES.len() - 1 {
                self.gain += 1;
            } else {
                break;
            }
        }
    }

    /// `r<n>`: autoexpose (if enabled), average n, reply with 4 fields.
    fn sample(&mut self, n: u32) -> String {
        if self.autogain {
            self.autoexpose();
        }
        let count = n.clamp(1, 1000);
        let mut acc = 0.0;
        for _ in 0..count {
            acc += (self.level_volts + self.noise()).max(0.0);
        }
        let volts = acc / count as f64;
        let full_scale = GAIN_VOLTAGES[self.gain];
        let sensor_sat = full_scale > SATURATION_VOLTAGE && volts >= SATURATION_VOLTAGE;
        let raw = ((volts / full_scale) * 32767.0).round().min(32767.0) as i32;
        let adc_sat = full_scale < SATURATION_VOLTAGE && raw >= 32767;
        format!(
            "{},{},{},{}",
            raw.min(32767),
            sensor_sat as u8,
            adc_sat as u8,
            self.gain
        )
    }

    fn handle(&mut self, cmd: &str) {
        let reply = match cmd {
            "p" => "pong".to_string(),
            "I" => format!(
                "lightsensor proto=2 fw=sim-2.1.0 id=00:11:22:33:44:55 sps=860 vsat=3.20 dark={:.6} gains={}",
                self.dark_offset_v,
                GAIN_VOLTAGES.map(|v| v.to_string()).join(",")
            ),
            "G" => self.gain.to_string(),
            "D" => format!("{:.9}", self.dark_offset_v),
            "A" => format!("{} {}", self.autogain as u8, self.gain),
            "a1" => {
                self.autogain = true;
                "ok".to_string()
            }
            "a0" => {
                self.autogain = false;
                "ok".to_string()
            }
            _ if cmd.starts_with('r') => {
                let n = cmd[1..].trim().parse().unwrap_or(1);
                self.sample(n)
            }
            _ if cmd.starts_with('g') => match cmd[1..].parse::<usize>() {
                Ok(i) if i < GAIN_VOLTAGES.len() => {
                    self.gain = i;
                    self.autogain = false; // manual gain turns autoexposure off
                    "ok".to_string()
                }
                _ => "err 1".to_string(),
            },
            _ if cmd.starts_with('d') => match cmd[1..].trim().parse::<f64>() {
                Ok(offset) if offset.is_finite() && offset.abs() <= MAX_DARK_OFFSET_V => {
                    self.dark_offset_v = offset;
                    "ok".to_string()
                }
                _ => "err 1".to_string(),
            },
            _ => "err 1".to_string(),
        };
        self.replies.push_back(reply);
    }
}

impl Transport for SimTransport {
    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        if self.mute {
            return Ok(()); // device plays dead: consumes bytes, answers nothing
        }
        let text = String::from_utf8_lossy(bytes);
        self.handle(text.trim_end_matches('\n'));
        Ok(())
    }

    fn read_line(&mut self) -> Result<Option<String>> {
        Ok(self.replies.pop_front())
    }

    fn drain(&mut self) {
        self.replies.clear();
    }
}
