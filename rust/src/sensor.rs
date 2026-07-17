//! The driver: gain model, identity handshake, readings, autogain, zeroing.
//!
//! A faithful port of `lightmeter/sensor.py` (protocol v2, see
//! `docs/reference.md`). Deliberately NOT ported yet: calibration transfer
//! (`W`/`C`/`H`/`X`) and the spectral/photometric conversion — raw values and
//! volts are what the point camera stores; physical units land once the
//! absolute calibration is real.
//!
//! Threading: methods take `&mut self`; wrap the sensor in a `Mutex` to share
//! it (the Python driver's internal `RLock` is not replicated).

use std::collections::HashMap;
use std::time::Duration;

use crate::transport::{Result, Transport};

/// Protocol version this driver speaks; the device reports its own in `I`.
pub const PROTO_VERSION: u32 = 2;

/// Gain index → ADC full-scale voltage. Index 1 (±4.096 V) is the default.
pub const GAIN_VOLTAGES: [f64; 6] = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256];
pub const GAIN_LABELS: [&str; 6] = [
    "±6.144V", "±4.096V", "±2.048V", "±1.024V", "±0.512V", "±0.256V",
];
pub const DEFAULT_GAIN: usize = 1;

/// OPA323 output saturates ~34 mV below the 3.3 V rail (measured).
pub const SATURATION_VOLTAGE: f64 = 3.2;

/// Nominal R1/R3 electrical dark baseline: 3.3 V × 270 Ω / (13 kΩ + 270 Ω).
pub const DEFAULT_DARK_OFFSET_V: f64 = 3.3 * 270.0 / (13_000.0 + 270.0);
pub const MAX_DARK_OFFSET_V: f64 = 0.25;
/// Skip flash writes when a measured dark level changed by no more than 100 µV.
pub const DARK_OFFSET_WRITE_TOLERANCE_V: f64 = 0.0001;

/// Highest gain index that keeps `max_voltage` below saturation with the
/// given headroom (default 0.85). Falls back to 0 (widest range).
pub fn best_gain(max_voltage: f64, headroom: f64) -> usize {
    for (i, &fs) in GAIN_VOLTAGES.iter().enumerate().rev() {
        let ceiling = fs.min(SATURATION_VOLTAGE) * headroom;
        if max_voltage < ceiling {
            return i;
        }
    }
    0
}

/// One sample. `value` is % of ADC full-scale (0–100), gain-relative.
/// The saturation flags reflect the TRUE level (never dark-corrected);
/// they are mutually exclusive on this hardware.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Reading {
    pub value: f64,
    /// Op-amp near supply rail (low-gain indices 0–1).
    pub sensor_sat: bool,
    /// ADC raw hit 32767 (high-gain indices 2–5).
    pub adc_sat: bool,
}

/// Identity reported by the device's `I` command.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceInfo {
    pub name: String,
    pub proto: u32,
    pub firmware: String,
    pub id: String,
    /// All key=value pairs from the identity line (sps, vsat, gains, …).
    pub fields: HashMap<String, String>,
}

/// Parse an identity line (`lightsensor proto=2 fw=… id=… …`), or `None`
/// if the line doesn't look like one.
pub fn parse_identity(line: &str) -> Option<DeviceInfo> {
    let mut parts = line.split_whitespace();
    let name = parts.next()?.to_string();
    let mut fields = HashMap::new();
    for part in parts {
        if let Some((k, v)) = part.split_once('=') {
            fields.insert(k.to_string(), v.to_string());
        }
    }
    let proto: u32 = fields.get("proto")?.parse().ok()?;
    Some(DeviceInfo {
        name,
        proto,
        firmware: fields.get("fw").cloned().unwrap_or_else(|| "?".into()),
        id: fields.get("id").cloned().unwrap_or_else(|| "?".into()),
        fields,
    })
}

/// Device error codes returned as `err <code>`. Keep in sync with firmware.
pub fn err_text(resp: &str) -> String {
    const MESSAGES: [(u32, &str); 7] = [
        (1, "bad argument"),
        (2, "bad length"),
        (3, "out of memory"),
        (4, "transfer timeout / short read"),
        (5, "filesystem open failed"),
        (6, "write size mismatch"),
        (7, "erase failed"),
    ];
    if let Some(code) = resp
        .strip_prefix("err ")
        .and_then(|c| c.trim().parse::<u32>().ok())
        && let Some((_, text)) = MESSAGES.iter().find(|(c, _)| *c == code)
    {
        return format!("err {code} ({text})");
    }
    if resp.is_empty() {
        "no response".into()
    } else {
        resp.into()
    }
}

pub struct LightSensor<T: Transport> {
    transport: T,
    /// Applied gain index, tracked locally (updated by `set_gain`).
    pub gain: usize,
    /// Firmware-side samples averaged per read (~√n noise, ×n time).
    pub average: u32,
    /// Identity from the connect handshake (`None` = device never answered).
    pub info: Option<DeviceInfo>,
    /// When set, `read` autoexposes: re-reads, stepping gain, until the
    /// sample lands in the band or a gain rail is hit.
    autogain: bool,
    /// Persisted per-device electrical dark correction in volts.
    device_dark_offset_v: f64,
    /// Temporary session dark/background correction; overrides the device value.
    session_zero_offset_v: Option<f64>,
}

/// Longest the device can block waiting for a command payload (the `W`
/// receive loop); resync must stay silent at least this long.
const DEVICE_CMD_TIMEOUT: Duration = Duration::from_millis(5500);

impl<T: Transport> LightSensor<T> {
    /// Wrap an open transport and run the connect handshake.
    pub fn new(transport: T) -> Result<Self> {
        let mut sensor = Self {
            transport,
            gain: DEFAULT_GAIN,
            average: 1,
            info: None,
            autogain: false,
            device_dark_offset_v: DEFAULT_DARK_OFFSET_V,
            session_zero_offset_v: None,
        };
        sensor.handshake()?;
        Ok(sensor)
    }

    /// Drain stale input and probe with a ping.
    fn try_sync(&mut self) -> Result<bool> {
        self.transport.drain();
        self.ping()
    }

    /// Re-establish a clean command stream (see `docs/reference.md`).
    ///
    /// If a quick ping fails, the device may be stuck mid-command (e.g. an
    /// interrupted `W` waiting for payload). Go SILENT past its receive
    /// timeout so the command self-aborts — pinging would feed the pending
    /// read and reset its timeout forever. Best-effort: logs instead of
    /// failing so older firmware without `I`/`p` still connects.
    fn handshake(&mut self) -> Result<()> {
        if !self.try_sync()? {
            log::warn!("no pong; resyncing (waiting out device timeout)");
            std::thread::sleep(DEVICE_CMD_TIMEOUT);
            if !self.try_sync()? {
                log::warn!("device still unresponsive after resync");
            }
        }
        self.info = self.identify()?;
        self.load_device_dark_offset();
        let Some(info) = &self.info else {
            log::warn!("device did not report identity (old firmware?)");
            return Ok(());
        };
        if info.proto != PROTO_VERSION {
            log::warn!(
                "protocol mismatch: device proto={}, driver expects {PROTO_VERSION}",
                info.proto
            );
        }
        verify_constants(info);
        Ok(())
    }

    /// `true` if the device answers `pong`. Pure link check, no I2C.
    pub fn ping(&mut self) -> Result<bool> {
        self.transport.send(b"p")?;
        Ok(self.transport.read_line()?.as_deref() == Some("pong"))
    }

    /// Query device identity (`I`).
    pub fn identify(&mut self) -> Result<Option<DeviceInfo>> {
        self.transport.send(b"I")?;
        Ok(self
            .transport
            .read_line()?
            .as_deref()
            .and_then(parse_identity))
    }

    fn load_device_dark_offset(&mut self) {
        let Some(value) = self.info.as_ref().and_then(|info| info.fields.get("dark")) else {
            return;
        };
        match value.parse::<f64>() {
            Ok(offset) if offset.is_finite() && offset.abs() <= MAX_DARK_OFFSET_V => {
                self.device_dark_offset_v = offset;
            }
            _ => log::warn!("invalid device dark offset: {value:?}"),
        }
    }

    /// One `r<n>` transaction → `(raw, sensor_sat, adc_sat, gain)`. The gain
    /// is the 4th field (proto 2): in autogain mode it is the settled gain,
    /// in manual mode it echoes the current gain. `Ok(None)` on timeout or a
    /// malformed line (logged, not silent).
    fn read_raw(&mut self) -> Result<Option<(i32, bool, bool, usize)>> {
        let n = self.average.max(1);
        self.transport.send(format!("r{n}\n").as_bytes())?;
        let Some(line) = self.transport.read_line()? else {
            log::debug!("read timeout (no line)");
            return Ok(None);
        };
        let fields: Vec<&str> = line.split(',').collect();
        let parsed = match fields.as_slice() {
            [raw, s_sat, a_sat, gain] => (|| {
                Some((
                    raw.trim().parse::<i32>().ok()?,
                    s_sat.trim().parse::<u8>().ok()? != 0,
                    a_sat.trim().parse::<u8>().ok()? != 0,
                    gain.trim().parse::<usize>().ok()?,
                ))
            })(),
            _ => None,
        };
        if parsed.is_none() {
            log::debug!("read parse error: {line:?}");
        }
        Ok(parsed)
    }

    /// One reading, or `Ok(None)` on a timeout/parse failure. Link errors
    /// propagate as `Err` — the caller owns reconnect policy.
    ///
    /// Autoexposure lives in the firmware (enabled via [`set_autogain`]); the
    /// device reports the gain it used, which this records locally so
    /// [`reading_voltage`] converts correctly.
    pub fn read(&mut self) -> Result<Option<Reading>> {
        let Some((raw, sensor_sat, adc_sat, gain)) = self.read_raw()? else {
            return Ok(None);
        };
        self.gain = gain.min(GAIN_VOLTAGES.len() - 1);
        let mut reading = Reading {
            value: raw as f64 / 32767.0 * 100.0,
            sensor_sat,
            adc_sat,
        };
        // Dark offset subtracted last (display only); saturation flags keep
        // reflecting the TRUE level.
        let offset = self.effective_dark_offset();
        if offset != 0.0 {
            reading.value -= offset / GAIN_VOLTAGES[self.gain] * 100.0;
        }
        Ok(Some(reading))
    }

    /// Dark-corrected sensor voltage for a reading at the current gain.
    pub fn reading_voltage(&self, reading: Reading) -> f64 {
        reading.value / 100.0 * GAIN_VOLTAGES[self.gain]
    }

    /// Set ADC gain (index 0–5); this also turns autogain off on the device.
    /// `Ok(true)` on device ack.
    pub fn set_gain(&mut self, gain_index: usize) -> Result<bool> {
        self.transport.send(format!("g{gain_index}").as_bytes())?;
        let resp = self.transport.read_line()?.unwrap_or_default();
        if resp == "ok" {
            self.gain = gain_index;
            self.autogain = false; // firmware disables autoexposure on manual gain
            return Ok(true);
        }
        log::warn!("set_gain({gain_index}) failed: {}", err_text(&resp));
        Ok(false)
    }

    /// Query the device's current gain index.
    pub fn get_gain(&mut self) -> Result<Option<usize>> {
        self.transport.send(b"G")?;
        Ok(self
            .transport
            .read_line()?
            .and_then(|l| l.trim().parse().ok()))
    }

    /// Persisted per-device electrical dark correction in volts.
    pub fn device_dark_offset(&self) -> f64 {
        self.device_dark_offset_v
    }

    /// Temporary session dark/background correction, if active.
    pub fn session_zero_offset(&self) -> Option<f64> {
        self.session_zero_offset_v
    }

    /// Active correction: session zero overrides the persisted device baseline.
    pub fn effective_dark_offset(&self) -> f64 {
        self.session_zero_offset_v
            .unwrap_or(self.device_dark_offset_v)
    }

    /// Persist a per-device electrical dark correction in volts.
    ///
    /// Returns `Ok(true)` after a firmware-acknowledged write, or without
    /// writing when the saved value is already within 100 µV.
    pub fn set_device_dark_offset(&mut self, offset: f64) -> Result<bool> {
        if !offset.is_finite() || offset.abs() > MAX_DARK_OFFSET_V {
            return Ok(false);
        }
        if (offset - self.device_dark_offset_v).abs() <= DARK_OFFSET_WRITE_TOLERANCE_V {
            return Ok(true);
        }
        self.transport.send(format!("d{offset:.9}\n").as_bytes())?;
        let response = self.transport.read_line()?.unwrap_or_default();
        if response != "ok" {
            log::warn!(
                "set_device_dark_offset({offset}) failed: {}",
                err_text(&response)
            );
            return Ok(false);
        }
        self.device_dark_offset_v = offset;
        if let Some(info) = &mut self.info {
            info.fields.insert("dark".into(), format!("{offset:.9}"));
        }
        Ok(true)
    }

    /// Restore the calculated R1/R3 divider baseline on the device.
    pub fn reset_device_dark_offset(&mut self) -> Result<bool> {
        self.set_device_dark_offset(DEFAULT_DARK_OFFSET_V)
    }

    /// Persist the existing session dark/background correction.
    ///
    /// Returns `Ok(false)` when no session zero is active.
    pub fn save_session_dark_offset(&mut self) -> Result<bool> {
        let Some(offset) = self.session_zero_offset_v else {
            return Ok(false);
        };
        self.set_device_dark_offset(offset)
    }

    /// Measure covered-sensor dark voltage and persist it on the device.
    ///
    /// Autogain and session zeroing are suspended while sampling the true
    /// electrical level. Returns `Ok(None)` if no valid samples arrive or the
    /// device rejects the flash write.
    pub fn calibrate_device_dark_offset(&mut self, n: usize) -> Result<Option<f64>> {
        let was_autogain = self.autogain;
        let previous_session = self.session_zero_offset_v;
        if was_autogain {
            self.set_autogain(false)?;
        }
        self.session_zero_offset_v = None;
        let measured = self.mean_uncorrected_voltage(n);
        self.session_zero_offset_v = previous_session;
        if was_autogain {
            self.set_autogain(true)?;
        }
        let Some(offset) = measured? else {
            return Ok(None);
        };
        Ok(self.set_device_dark_offset(offset)?.then_some(offset))
    }

    /// Temporarily zero the current dark/background level over `n` samples.
    ///
    /// A session zero overrides, but never overwrites, the persisted device
    /// dark correction. Returns the active offset in volts.
    pub fn zero(&mut self, n: usize) -> Result<f64> {
        let was_autogain = self.autogain;
        let previous_session = self.session_zero_offset_v;
        if was_autogain {
            self.set_autogain(false)?;
        }
        self.session_zero_offset_v = None;
        let measured = self.mean_uncorrected_voltage(n);
        self.session_zero_offset_v = previous_session;
        if was_autogain {
            self.set_autogain(true)?;
        }
        self.session_zero_offset_v = measured?.or(previous_session);
        Ok(self.effective_dark_offset())
    }

    /// Clear the session zero and resume the persisted device correction.
    pub fn clear_zero(&mut self) {
        self.session_zero_offset_v = None;
    }

    pub fn is_zeroed(&self) -> bool {
        self.effective_dark_offset() != 0.0
    }

    /// Current effective dark correction in volts.
    pub fn zero_offset(&self) -> f64 {
        self.effective_dark_offset()
    }

    /// Enable/disable firmware autoexposure (`a1`/`a0`). `read` then reports
    /// the settled gain. `Ok(())` on device ack.
    pub fn set_autogain(&mut self, enabled: bool) -> Result<()> {
        self.transport.send(if enabled { b"a1" } else { b"a0" })?;
        let resp = self.transport.read_line()?.unwrap_or_default();
        if resp == "ok" {
            self.autogain = enabled;
            Ok(())
        } else {
            Err(std::io::Error::other(format!(
                "set_autogain failed: {}",
                err_text(&resp)
            )))
        }
    }

    /// Query the device's autogain state and current gain (`A`).
    pub fn get_autogain(&mut self) -> Result<Option<(bool, usize)>> {
        self.transport.send(b"A")?;
        let Some(line) = self.transport.read_line()? else {
            return Ok(None);
        };
        let mut it = line.split_whitespace();
        let auto = it.next().and_then(|s| s.parse::<u8>().ok());
        let gain = it.next().and_then(|s| s.parse::<usize>().ok());
        Ok(auto.zip(gain).map(|(a, g)| (a != 0, g)))
    }

    pub fn autogain_enabled(&self) -> bool {
        self.autogain
    }

    /// Consume the sensor, closing the transport.
    pub fn close(self) {}

    //
    // helpers
    //

    fn sample_uncorrected_voltage(&mut self) -> Result<Option<f64>> {
        let Some((raw, _, _, gain)) = self.read_raw()? else {
            return Ok(None);
        };
        self.gain = gain.min(GAIN_VOLTAGES.len() - 1);
        Ok(Some(raw as f64 / 32767.0 * GAIN_VOLTAGES[self.gain]))
    }

    fn mean_uncorrected_voltage(&mut self, n: usize) -> Result<Option<f64>> {
        let mut voltages = Vec::with_capacity(n.max(1));
        for _ in 0..n.max(1) {
            if let Some(voltage) = self.sample_uncorrected_voltage()? {
                voltages.push(voltage);
            }
        }
        Ok((!voltages.is_empty()).then(|| voltages.iter().sum::<f64>() / voltages.len() as f64))
    }
}

/// Warn if the driver's mirrored constants drift from the device's — the
/// firmware is the source of truth and reports them in the identity line.
fn verify_constants(info: &DeviceInfo) {
    if let Some(gains) = info.fields.get("gains") {
        let device: Option<Vec<f64>> = gains.split(',').map(|x| x.parse().ok()).collect();
        match device {
            Some(device) if device != GAIN_VOLTAGES => {
                log::warn!("gain table mismatch: device={device:?} driver={GAIN_VOLTAGES:?}");
            }
            _ => {}
        }
    }
    if let Some(vsat) = info.fields.get("vsat")
        && let Ok(v) = vsat.parse::<f64>()
        && (v - SATURATION_VOLTAGE).abs() > 0.01
    {
        log::warn!("saturation voltage mismatch: device={v} driver={SATURATION_VOLTAGE}");
    }
}
