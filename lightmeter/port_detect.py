"""Serial port auto-detection for the LightSensor device (ESP32-C3 SuperMini).

Works on Linux and Windows. Importable (`autodetect_port()`) and runnable as a
script that prints the detected port — used by the justfile when flashing:

    arduino-cli upload -p "$(uv run python port_detect.py)" ...
"""

import sys

import serial.tools.list_ports

# The ESP32-C3 SuperMini exposes the chip's native USB Serial/JTAG.
# (VID, PID) pairs for known devices.
_KNOWN_HWIDS = (
    (0x303A, 0x1001),  # Espressif native USB (ESP32-C3 / S3 USB Serial/JTAG)
)
# Fallback substring hints matched against the port description / hardware id.
_DESCRIPTION_HINTS = ("espressif", "esp32", "usb jtag", "usb serial/jtag")


def autodetect_port():
    """Return the serial port the sensor is most likely connected to.

    Resolution order:
      1. USB VID/PID match against known devices.
      2. Description / hardware-id substring match.
      3. If exactly one serial port exists, use it.
    Raises RuntimeError if no suitable port can be determined.
    """
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Is the device plugged in?")

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
        "Could not auto-detect the device port. Specify one explicitly. "
        f"Available ports: {available}"
    )


if __name__ == "__main__":
    try:
        print(autodetect_port())
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
