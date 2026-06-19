from dataclasses import dataclass
import binascii
import serial
import time

from port_detect import autodetect_port

# Gain index maps to: 0=±6.144V, 1=±4.096V, 2=±2.048V, 3=±1.024V, 4=±0.512V, 5=±0.256V
GAIN_LABELS = ["±6.144V", "±4.096V", "±2.048V", "±1.024V", "±0.512V", "±0.256V"]
GAIN_VOLTAGES = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]
DEFAULT_GAIN = 1  # ±4.096V

# OPA323 output saturates ~34 mV below the 3.3 V supply rail (measured).
# In absolute scale (value * gain_voltage) this equals ~326.6.
SATURATION_VOLTAGE = 3.2  # V


def best_gain(max_voltage, headroom=0.85):
    """Return the highest gain index that won't saturate for the given peak voltage.

    Iterates from highest gain (±0.256 V) to lowest (±6.144 V) and returns
    the first index where max_voltage fits below the saturation threshold
    with the given headroom factor.
    """
    for g in range(len(GAIN_VOLTAGES) - 1, -1, -1):
        threshold = min(SATURATION_VOLTAGE, GAIN_VOLTAGES[g]) * headroom
        if max_voltage < threshold:
            return g
    return 0  # fall back to lowest gain (widest range)


@dataclass
class Calibration:
    """Parsed calibration: metadata header + spectral responsivity curve."""

    metadata: dict  # header key/value pairs (device_id, cal_date, scale_factor, …)
    wavelengths: list  # nm
    responsivity: list  # one value per wavelength
    raw_text: str  # the original CSV as stored on the device

    @property
    def scale_factor(self):
        v = self.metadata.get("scale_factor")
        return float(v) if v is not None else None


def parse_calibration(text):
    """Parse calibration CSV text into a Calibration.

    Header lines start with '#' as 'key: value'. The data section is a
    'wavelength_nm,responsivity' table.
    """
    metadata = {}
    wavelengths = []
    responsivity = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("#"):
            body = line[1:].strip()
            if ":" in body:
                key, val = body.split(":", 1)
                metadata[key.strip()] = val.strip()
            continue
        if line.lower().startswith("wavelength"):
            continue  # column header
        parts = line.split(",")
        if len(parts) >= 2:
            try:
                wavelengths.append(float(parts[0]))
                responsivity.append(float(parts[1]))
            except ValueError:
                pass
    return Calibration(metadata, wavelengths, responsivity, text)


@dataclass
class Reading:
    value: float  # light level, % of ADC full-scale (0–100)
    sensor_sat: bool  # op-amp near supply rail (gain full-scale > VDD)
    adc_sat: bool  # ADC raw hit 32767 (gain full-scale < VDD)


class LightSensor:
    def __init__(self, port=None, baud=115200):
        self.port = port or autodetect_port()
        self.baud = baud
        self.ser = None
        self.gain = DEFAULT_GAIN  # locally tracked; updated by set_gain()
        # Continuous autogain: when True, read() manages gain automatically.
        self.autogain = False
        self.autogain_interval = 0.25  # seconds between gain evaluations
        self.autogain_window = 0.5  # seconds of history to consider
        self._autogain_history: list = []  # (timestamp, voltage_V) pairs
        self._autogain_last_check = 0.0
        # Dark-offset ("zero"): voltage subtracted from every reading. Stored
        # as volts so it stays correct across gain changes.
        self._zero_offset_v = 0.0
        # Firmware-side averaging: each read() averages this many ADC samples
        # on the device and returns one Reading.
        self.average = 1
        self.open()

    def open(self):
        """(Re)open the serial port.

        The ESP32-C3 uses native USB CDC, which has no DTR/RTS auto-reset
        circuit to work around, so a plain open is fine on both Linux and
        Windows.
        """
        self.close()
        self.ser = serial.Serial(self.port, self.baud, timeout=1)

    def read(self):
        """Return a Reading(value, sensor_sat, adc_sat) or None on parse failure.

        value      -- light level as % of ADC full-scale (0–100)
        sensor_sat -- op-amp output near supply rail (low-gain settings)
        adc_sat    -- ADC raw reading hit 32767 (high-gain settings)
        """
        if self.ser is None or not self.ser.is_open:
            self.open()
        n = self.average if self.average and self.average > 1 else 1
        self.ser.write(f"r{n}\n".encode())
        line = self.ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) != 3:
            return None
        try:
            raw, sensor_sat, adc_sat = (
                int(parts[0]),
                bool(int(parts[1])),
                bool(int(parts[2])),
            )
        except ValueError:
            return None
        gain_at_read = self.gain
        reading = Reading(raw / 32767 * 100, sensor_sat, adc_sat)
        # Autogain and saturation must see the TRUE level, so run autogain on
        # the un-zeroed reading first.
        if self.autogain:
            self._autogain_update(reading)
        # Subtract the dark offset as the very last step (display only) so it
        # never affects gain selection or saturation. Use the gain the sample
        # was actually taken at, in case autogain just changed it.
        if self._zero_offset_v:
            reading.value -= self._zero_offset_v / GAIN_VOLTAGES[gain_at_read] * 100
        return reading

    def zero(self, n=50):
        """Measure the dark/background level over n samples and subtract it from
        all future reads. Returns the measured offset in volts.

        Continuous autogain and any existing offset are suspended during the
        measurement so the true level is captured at the current gain.
        """
        was_autogain = self.autogain
        prev_offset = self._zero_offset_v
        self.autogain = False
        self._zero_offset_v = 0.0
        try:
            voltages = []
            for _ in range(n):
                r = self.read()
                if r is not None:
                    voltages.append(r.value * GAIN_VOLTAGES[self.gain] / 100)
            self._zero_offset_v = sum(voltages) / len(voltages) if voltages else prev_offset
        finally:
            self.autogain = was_autogain
        return self._zero_offset_v

    def clear_zero(self):
        """Remove the dark offset so reads return the raw level again."""
        self._zero_offset_v = 0.0

    @property
    def is_zeroed(self):
        return self._zero_offset_v != 0.0

    @property
    def zero_offset(self):
        """Current dark offset in volts (0.0 if not zeroed)."""
        return self._zero_offset_v

    def _autogain_update(self, reading):
        now = time.monotonic()
        voltage = reading.value * GAIN_VOLTAGES[self.gain] / 100
        self._autogain_history.append((now, voltage))
        cutoff = now - self.autogain_window
        self._autogain_history = [(t, v) for t, v in self._autogain_history if t > cutoff]
        if now - self._autogain_last_check >= self.autogain_interval:
            self._autogain_last_check = now
            if self._autogain_history:
                new_gain = best_gain(max(v for _, v in self._autogain_history))
                if new_gain != self.gain:
                    self._autogain_history.clear()
                    self.set_gain(new_gain)

    def autogain_oneshot(self, n=100):
        """Collect n samples, find the best gain, apply it, and return the gain index.

        Temporarily disables continuous autogain during the measurement so the
        gain stays fixed for the full sample set.
        """
        was_autogain = self.autogain
        prev_offset = self._zero_offset_v
        self.autogain = False
        self._zero_offset_v = 0.0  # select gain on the true level, not zeroed
        try:
            voltages = []
            for _ in range(n):
                r = self.read()
                if r is not None:
                    voltages.append(r.value * GAIN_VOLTAGES[self.gain] / 100)
            if voltages:
                self.set_gain(best_gain(max(voltages)))
        finally:
            self.autogain = was_autogain
            self._zero_offset_v = prev_offset
        return self.gain

    def set_gain(self, gain_index):
        """Set ADC gain. gain_index 0–5 maps to ±6.144V … ±0.256V. Returns True on success."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(f"g{gain_index}".encode())
        resp = self.ser.readline().decode(errors="ignore").strip()
        if resp == "ok":
            self.gain = gain_index
            return True
        return False

    def get_gain(self):
        """Return current gain index (0–5), or None on failure."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(b"G")
        resp = self.ser.readline().decode(errors="ignore").strip()
        return int(resp) if resp.isdigit() else None

    # --- Calibration storage (on-device LittleFS) ---------------------------

    def has_calibration(self):
        """Return the stored calibration size in bytes (0 if none)."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(b"H")
        resp = self.ser.readline().decode(errors="ignore").strip()
        return int(resp) if resp.isdigit() else 0

    def write_calibration(self, text):
        """Store calibration CSV text on the device. Returns True if the
        device-computed CRC32 matches the host's (transfer verified)."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        data = text.encode() if isinstance(text, str) else text
        expected = binascii.crc32(data) & 0xFFFFFFFF
        self.ser.write(f"W{len(data)}\n".encode())
        self.ser.flush()
        # Throttle the payload: a single fast burst overruns the device's
        # USB-CDC RX buffer (it can't drain while writing flash), dropping
        # bytes. Small flushed chunks with a brief pause keep it reliable.
        for i in range(0, len(data), 128):
            self.ser.write(data[i : i + 128])
            self.ser.flush()
            time.sleep(0.002)
        resp = self.ser.readline().decode(errors="ignore").strip()
        parts = resp.split()
        if len(parts) == 2 and parts[0] == "ok":
            return int(parts[1]) == expected
        return False

    def write_calibration_file(self, path):
        """Store a calibration CSV file from disk. Returns True on verified write."""
        with open(path, "r") as f:
            return self.write_calibration(f.read())

    def read_calibration(self):
        """Read the stored calibration. Returns a Calibration, or None if the
        device has no calibration or the CRC32 check fails."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(b"C")
        header = self.ser.readline().decode(errors="ignore").strip()
        parts = header.split()
        if len(parts) != 2:
            return None
        size, crc = int(parts[0]), int(parts[1])
        if size == 0:
            return None
        data = self.ser.read(size)  # blocks up to the serial timeout
        if len(data) != size or (binascii.crc32(data) & 0xFFFFFFFF) != crc:
            return None
        return parse_calibration(data.decode(errors="ignore"))

    def clear_calibration(self):
        """Erase the stored calibration. Returns True on success."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(b"X")
        return self.ser.readline().decode(errors="ignore").strip() == "ok"

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
