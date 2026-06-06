import sys
from dataclasses import dataclass
import serial
import serial.tools.list_ports
import time

# USB identifiers and description hints for the sensor's USB-to-UART bridge
# (Silicon Labs CP210x).
_KNOWN_HWIDS = ((0x10C4, 0xEA60),)  # (VID, PID) for CP210x
_DESCRIPTION_HINTS = ("cp210", "silicon labs", "uart")

# Gain index maps to: 0=±6.144V, 1=±4.096V, 2=±2.048V, 3=±1.024V, 4=±0.512V, 5=±0.256V
GAIN_LABELS = ["±6.144V", "±4.096V", "±2.048V", "±1.024V", "±0.512V", "±0.256V"]
GAIN_VOLTAGES = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]
DEFAULT_GAIN = 1  # ±4.096V

# OPA323 output saturates ~34 mV below the 3.3 V supply rail (measured).
# In absolute scale (value * gain_voltage) this equals ~326.6.
SATURATION_VOLTAGE = 3.2  # V


@dataclass
class Reading:
    value: float      # light level, % of ADC full-scale (0–100)
    sensor_sat: bool  # op-amp near supply rail (gain full-scale > VDD)
    adc_sat: bool     # ADC raw hit 32767 (gain full-scale < VDD)


def autodetect_port():
    """Return the serial port the sensor is most likely connected to.

    Prefers a known CP210x bridge (by USB VID/PID, then by description). If
    nothing matches but exactly one port exists, that port is used. Raises
    RuntimeError if no suitable port can be determined.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Is the sensor plugged in?")

    # 1) Match by USB VID/PID.
    for p in ports:
        if p.vid is not None and (p.vid, p.pid) in _KNOWN_HWIDS:
            return p.device

    # 2) Match by description / hardware-id text.
    for p in ports:
        text = f"{p.description} {p.hwid}".lower()
        if any(hint in text for hint in _DESCRIPTION_HINTS):
            return p.device

    # 3) Fall back to the only available port.
    if len(ports) == 1:
        return ports[0].device

    available = ", ".join(f"{p.device} ({p.description})" for p in ports)
    raise RuntimeError(
        "Could not auto-detect the sensor port. Specify one with --port. "
        f"Available ports: {available}"
    )


class LightSensor:
    def __init__(self, port=None, baud=9600):
        self.port = port or autodetect_port()
        self.baud = baud
        self.ser = None
        self.open()

    def open(self):
        """(Re)open the serial port and give the device time to reset."""
        self.close()
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baud
        ser.timeout = 1
        if sys.platform == "win32":
            # On Windows the CP210x bridge asserts RTS/DTR by default, which
            # triggers the ESP8266 auto-reset circuit and causes WriteFile
            # error 22. Deassert both before opening to prevent this.
            ser.rts = False
            ser.dtr = False
            ser.open()
            self.ser = ser
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        else:
            # On Linux, changing DTR/RTS causes a transition that trips the
            # NodeMCU auto-reset circuit, making the CP210x do a full USB
            # disconnect/reconnect. Leave the lines untouched so the device
            # never resets when we open the port.
            ser.open()
            self.ser = ser

    def read(self):
        """Return a Reading(value, sensor_sat, adc_sat) or None on parse failure.

        value      -- light level as % of ADC full-scale (0–100)
        sensor_sat -- op-amp output near supply rail (low-gain settings)
        adc_sat    -- ADC raw reading hit 32767 (high-gain settings)
        """
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(b"r")
        line = self.ser.readline().decode(errors="ignore").strip()
        parts = line.split(",")
        if len(parts) != 3:
            return None
        try:
            raw, sensor_sat, adc_sat = int(parts[0]), bool(int(parts[1])), bool(int(parts[2]))
        except ValueError:
            return None
        return Reading(raw / 32767 * 100, sensor_sat, adc_sat)

    def set_gain(self, gain_index):
        """Set ADC gain. gain_index 0–5 maps to ±6.144V … ±0.256V. Returns True on success."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(f"g{gain_index}".encode())
        resp = self.ser.readline().decode(errors="ignore").strip()
        return resp == "ok"

    def get_gain(self):
        """Return current gain index (0–5), or None on failure."""
        if self.ser is None or not self.ser.is_open:
            self.open()
        self.ser.write(b"G")
        resp = self.ser.readline().decode(errors="ignore").strip()
        return int(resp) if resp.isdigit() else None

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
