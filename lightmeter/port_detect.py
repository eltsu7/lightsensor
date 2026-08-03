"""USB serial discovery for the LightSensor v3 RP2040 board."""

import sys

import serial.tools.list_ports

USB_VID_PID = (0x2E8A, 0xF00A)
USB_PRODUCT = "lightsensor v3"


def _is_lightsensor(port):
    product = (port.product or "").strip().lower()
    manufacturer = (port.manufacturer or "").strip().lower()
    description = (port.description or "").lower()
    return product == USB_PRODUCT or (
        (port.vid, port.pid) == USB_VID_PID
        and (manufacturer == "lightsensor" or USB_PRODUCT in description)
    )


def autodetect_port(device_id=None):
    """Return a unique LightSensor v3 CDC port.

    ``device_id`` optionally selects the uppercase W25Q32 UID exposed as the USB
    serial number. Protocol identity is still verified by :class:`LightSensor`.
    """
    requested = device_id.upper() if device_id else None
    ports = list(serial.tools.list_ports.comports())
    candidates = [port for port in ports if _is_lightsensor(port)]
    if requested:
        candidates = [
            port for port in candidates if (port.serial_number or "").upper() == requested
        ]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        target = f" with device ID {requested}" if requested else ""
        raise RuntimeError(f"No LightSensor v3 USB device found{target}.")
    available = ", ".join(
        f"{port.device} ({port.serial_number or 'no serial'})" for port in candidates
    )
    raise RuntimeError(
        "Multiple LightSensor v3 devices found; specify a port or device ID. "
        f"Candidates: {available}"
    )


if __name__ == "__main__":
    try:
        print(autodetect_port(sys.argv[1] if len(sys.argv) > 1 else None))
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        sys.exit(1)
