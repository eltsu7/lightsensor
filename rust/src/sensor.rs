//! The driver: gain model, identity handshake, readings, autogain, zeroing.
//!
//! A faithful port of `lightmeter/sensor.py` (protocol v1, see
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
pub const GAIN_LABELS: [&str; 6] =
    ["±6.144V", "±4.096V", "±2.048V", "±1.024V", "±0.512V", "±0.256V"];
pub const DEFAULT_GAIN: usize = 1;

/// OPA323 output saturates ~34 mV below the 3.3 V rail (measured).
pub const SATURATION_VOLTAGE: f64 = 3.2;

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

/// Parse an identity line (`lightsensor proto=1 fw=… id=… …`), or `None`
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
    if let Some(code) = resp.strip_prefix("err ").and_then(|c| c.trim().parse::<u32>().ok())
        && let Some((_, text)) = MESSAGES.iter().find(|(c, _)| *c == code)
    {
        return format!("err {code} ({text})");
    }
    if resp.is_empty() { "no response".into() } else { resp.into() }
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
    /// Dark offset in volts — kept in volts so it survives gain changes.
    zero_offset_v: f64,
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
            zero_offset_v: 0.0,
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
        Ok(self.transport.read_line()?.as_deref().and_then(parse_identity))
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
        let mut reading = Reading { value: raw as f64 / 32767.0 * 100.0, sensor_sat, adc_sat };
        // Dark offset subtracted last (display only); saturation flags keep
        // reflecting the TRUE level.
        if self.zero_offset_v != 0.0 {
            reading.value -= self.zero_offset_v / GAIN_VOLTAGES[self.gain] * 100.0;
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
        Ok(self.transport.read_line()?.and_then(|l| l.trim().parse().ok()))
    }

    /// Measure the dark level over `n` samples and subtract it from all
    /// future reads. Returns the offset in volts. Autogain and any existing
    /// offset are suspended so the true level is captured.
    pub fn zero(&mut self, n: usize) -> Result<f64> {
        let was_autogain = self.autogain;
        let prev_offset = self.zero_offset_v;
        if was_autogain {
            self.set_autogain(false)?;
        }
        self.zero_offset_v = 0.0;

        let result = self.mean_voltage(n);

        if was_autogain {
            self.set_autogain(true)?;
        }
        self.zero_offset_v = result?.unwrap_or(prev_offset);
        Ok(self.zero_offset_v)
    }

    pub fn clear_zero(&mut self) {
        self.zero_offset_v = 0.0;
    }

    pub fn is_zeroed(&self) -> bool {
        self.zero_offset_v != 0.0
    }

    /// Current dark offset in volts (0.0 if not zeroed).
    pub fn zero_offset(&self) -> f64 {
        self.zero_offset_v
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
            Err(std::io::Error::other(format!("set_autogain failed: {}", err_text(&resp))))
        }
    }

    /// Query the device's autogain state and current gain (`A`).
    pub fn get_autogain(&mut self) -> Result<Option<(bool, usize)>> {
        self.transport.send(b"A")?;
        let Some(line) = self.transport.read_line()? else { return Ok(None) };
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

    fn sample_voltage(&mut self) -> Result<Option<f64>> {
        Ok(self.read()?.map(|r| r.value * GAIN_VOLTAGES[self.gain] / 100.0))
    }

    fn mean_voltage(&mut self, n: usize) -> Result<Option<f64>> {
        let mut voltages = Vec::with_capacity(n);
        for _ in 0..n {
            if let Some(v) = self.sample_voltage()? {
                voltages.push(v);
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
