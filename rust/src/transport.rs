//! Byte transport to the device: real serial port or a test double.

use std::io::Read;
use std::time::{Duration, Instant};

pub type Result<T, E = std::io::Error> = std::result::Result<T, E>;

/// Line-oriented command link to the sensor. One implementation talks to the
/// real serial port; [`crate::sim::SimTransport`] emulates the firmware.
pub trait Transport {
    /// Write raw command bytes (no implicit newline).
    fn send(&mut self, bytes: &[u8]) -> Result<()>;
    /// Read one `\n`-terminated line (trimmed); `None` on timeout.
    fn read_line(&mut self) -> Result<Option<String>>;
    /// Discard any pending input.
    fn drain(&mut self);
}

impl<T: Transport + ?Sized> Transport for Box<T> {
    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        (**self).send(bytes)
    }

    fn read_line(&mut self) -> Result<Option<String>> {
        (**self).read_line()
    }

    fn drain(&mut self) {
        (**self).drain()
    }
}

/// USB VID/PID pairs of known devices (ESP32-C3 native USB Serial/JTAG).
const KNOWN_HWIDS: [(u16, u16); 1] = [(0x303A, 0x1001)];
/// Fallback substrings matched against port descriptions.
const DESCRIPTION_HINTS: [&str; 4] = ["espressif", "esp32", "usb jtag", "usb serial/jtag"];

/// Find the port the sensor is most likely on (mirrors `port_detect.py`):
/// VID/PID match → description match → the only port present.
pub fn autodetect_port() -> Result<String> {
    let ports = serialport::available_ports()
        .map_err(|e| std::io::Error::other(format!("port enumeration failed: {e}")))?;
    if ports.is_empty() {
        return Err(std::io::Error::other("no serial ports found — is the device plugged in?"));
    }

    let usb_info = |p: &serialport::SerialPortInfo| match &p.port_type {
        serialport::SerialPortType::UsbPort(u) => Some(u.clone()),
        _ => None,
    };

    for p in &ports {
        if let Some(usb) = usb_info(p)
            && KNOWN_HWIDS.contains(&(usb.vid, usb.pid))
        {
            return Ok(p.port_name.clone());
        }
    }
    for p in &ports {
        if let Some(usb) = usb_info(p) {
            let text = format!(
                "{} {}",
                usb.product.as_deref().unwrap_or(""),
                usb.manufacturer.as_deref().unwrap_or("")
            )
            .to_lowercase();
            if DESCRIPTION_HINTS.iter().any(|h| text.contains(h)) {
                return Ok(p.port_name.clone());
            }
        }
    }
    if let [only] = ports.as_slice() {
        return Ok(only.port_name.clone());
    }

    let available: Vec<_> = ports.iter().map(|p| p.port_name.clone()).collect();
    Err(std::io::Error::other(format!(
        "could not auto-detect the device port; available: {}",
        available.join(", ")
    )))
}

/// Real serial link. The ESP32-C3 uses native USB CDC — no DTR/RTS reset
/// dance needed, a plain open works on Linux and Windows.
pub struct SerialTransport {
    port: Box<dyn serialport::SerialPort>,
    /// Reassembly buffer for partial lines between reads.
    pending: Vec<u8>,
    timeout: Duration,
}

impl SerialTransport {
    pub const DEFAULT_BAUD: u32 = 115_200;
    pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);

    /// Open `path`, or autodetect when `None`.
    pub fn open(path: Option<&str>) -> Result<Self> {
        let path = match path {
            Some(p) => p.to_string(),
            None => autodetect_port()?,
        };
        let port = serialport::new(&path, Self::DEFAULT_BAUD)
            .timeout(Self::DEFAULT_TIMEOUT)
            .open()
            .map_err(|e| std::io::Error::other(format!("open {path} failed: {e}")))?;
        Ok(Self { port, pending: Vec::new(), timeout: Self::DEFAULT_TIMEOUT })
    }

    pub fn path(&self) -> Option<String> {
        self.port.name()
    }
}

impl Transport for SerialTransport {
    fn send(&mut self, bytes: &[u8]) -> Result<()> {
        use std::io::Write;
        self.port.write_all(bytes)?;
        self.port.flush()
    }

    fn read_line(&mut self) -> Result<Option<String>> {
        let deadline = Instant::now() + self.timeout;
        loop {
            if let Some(nl) = self.pending.iter().position(|&b| b == b'\n') {
                let line: Vec<u8> = self.pending.drain(..=nl).collect();
                let text = String::from_utf8_lossy(&line).trim().to_string();
                return Ok(Some(text));
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            let mut buf = [0u8; 256];
            match self.port.read(&mut buf) {
                Ok(0) => return Ok(None),
                Ok(n) => self.pending.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => return Ok(None),
                Err(e) => return Err(e),
            }
        }
    }

    fn drain(&mut self) {
        self.pending.clear();
        let _ = self.port.clear(serialport::ClearBuffer::Input);
    }
}
