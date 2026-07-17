//! Rust driver for the ESP32-C3 + OPA323 + ADS1115 light sensor.
//!
//! Speaks serial protocol v2 (see `../docs/reference.md`, the contract shared
//! with `firmware/lightsensor` and the Python driver). Calibration transfer
//! and spectral/photometric conversion are intentionally not ported yet —
//! they live behind placeholder numbers on the Python side too.
//!
//! ```no_run
//! use lightmeter::{LightSensor, SerialTransport};
//!
//! let transport = SerialTransport::open(None)?; // port autodetected
//! let mut sensor = LightSensor::new(transport)?;
//! sensor.set_gain(2)?;
//! if let Some(reading) = sensor.read()? {
//!     println!("{:.3} V", sensor.reading_voltage(reading));
//! }
//! # Ok::<(), std::io::Error>(())
//! ```
//!
//! Hardware-free (tests, GUIs):
//!
//! ```
//! use lightmeter::{LightSensor, sim::SimTransport};
//!
//! let mut sensor = LightSensor::new(SimTransport::default())?;
//! let reading = sensor.read()?.unwrap();
//! # Ok::<(), std::io::Error>(())
//! ```

pub mod sensor;
pub mod sim;
pub mod transport;

pub use sensor::{
    DEFAULT_DARK_OFFSET_V, DEFAULT_GAIN, DeviceInfo, GAIN_LABELS, GAIN_VOLTAGES, LightSensor,
    MAX_DARK_OFFSET_V, PROTO_VERSION, Reading, SATURATION_VOLTAGE, best_gain, parse_identity,
};
pub use transport::{SerialTransport, Transport, autodetect_port};
