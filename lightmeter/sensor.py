from dataclasses import dataclass
import binascii
import logging
import math
import threading
import serial
import time

from lightmeter.port_detect import autodetect_port

log = logging.getLogger(__name__)

# Protocol version this driver speaks. Must match the device's reported `proto`;
# a mismatch means the command set / response formats may have diverged.
PROTO_VERSION = 2

# Device error codes returned as "err <code>". Keep in sync with firmware ERR_*.
ERR_MESSAGES = {
    1: "bad argument",
    2: "bad length",
    3: "out of memory",
    4: "transfer timeout / short read",
    5: "filesystem open failed",
    6: "write size mismatch",
    7: "erase failed",
}


def _err_text(resp):
    """Human-readable text for an 'err [code]' response line."""
    parts = resp.split()
    if len(parts) >= 2 and parts[1].isdigit():
        code = int(parts[1])
        return f"err {code} ({ERR_MESSAGES.get(code, 'unknown')})"
    return resp or "no response"

# Gain index maps to: 0=±6.144V, 1=±4.096V, 2=±2.048V, 3=±1.024V, 4=±0.512V, 5=±0.256V
GAIN_LABELS = ["±6.144V", "±4.096V", "±2.048V", "±1.024V", "±0.512V", "±0.256V"]
GAIN_VOLTAGES = [6.144, 4.096, 2.048, 1.024, 0.512, 0.256]
DEFAULT_GAIN = 1  # ±4.096V

# OPA323 output saturates ~34 mV below the 3.3 V supply rail (measured).
# In absolute scale (value * gain_voltage) this equals ~326.6.
SATURATION_VOLTAGE = 3.2  # V

# Electrical dark baseline from the schematic's R1/R3 divider:
# 3.3 V × 270 Ω / (13 kΩ + 270 Ω). Firmware persists a per-device override.
DEFAULT_DARK_OFFSET_V = 3.3 * 270 / (13_000 + 270)
MAX_DARK_OFFSET_V = 0.25


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
class DeviceInfo:
    """Identity reported by the device's `I` command."""

    product: str  # fixed product token, e.g. "lightsensor"
    proto: int  # protocol version the device speaks
    fw: str  # firmware version string
    id: str  # unique device id (eFuse MAC hex) — usable as a serial number
    fields: dict  # all parsed key=value pairs (sps, ngains, …)


def parse_identity(line):
    """Parse an identity line into a DeviceInfo, or None if it doesn't look like one.

    Format: '<product> key=value key=value ...' e.g.
    'lightsensor proto=2 fw=2.0.0 id=AABBCCDDEEFF sps=860 ngains=6'
    """
    parts = line.split()
    if not parts or "=" in parts[0]:
        return None
    fields = {}
    for tok in parts[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            fields[k] = v
    if "proto" not in fields:
        return None
    try:
        proto = int(fields["proto"])
    except ValueError:
        return None
    return DeviceInfo(parts[0], proto, fields.get("fw", "?"), fields.get("id", "?"), fields)


@dataclass
class Calibration:
    """Parsed calibration: metadata header + spectral responsivity curve."""

    metadata: dict  # header key/value pairs (device_id, cal_date, scale_factor, …)
    wavelengths: list  # nm
    responsivity: list  # one value per wavelength
    raw_text: str  # the original CSV as stored on the device

    @property
    def scale_factor(self):
        """Absolute scale (physical_unit per volt), or None if uncalibrated.

        Set by one reference measurement against a known source/meter. Until
        that's taken it may be a placeholder (e.g. 1.0); a value of None or 1.0
        means readings are not yet absolutely calibrated.
        """
        v = self.metadata.get("scale_factor")
        return float(v) if v is not None else None

    @property
    def scale_units(self):
        """Unit string the scale_factor maps to, e.g. 'W/m^2' (None if unset)."""
        return self.metadata.get("scale_units")

    @property
    def provenance(self):
        """Where this calibration came from: 'measured' (a real reference/
        monochromator run) or 'datasheet-typical' (the bundled nominal fallback,
        spectral shape only). Defaults to 'measured' when unspecified, since a
        cal stored on the device is assumed real."""
        return self.metadata.get("provenance", "measured")

    @property
    def is_nominal(self):
        """True for the datasheet-typical fallback — readings are order-of-
        magnitude only and carry no part-specific absolute scale."""
        return self.provenance == "datasheet-typical"

    def responsivity_at(self, wl):
        """Relative responsivity R(λ) at one wavelength by linear interpolation.

        Returns 0.0 outside the measured band (the sensor has no calibrated
        response there). Assumes wavelengths are ascending, as written by the
        monochromator sweep.
        """
        wls, rs = self.wavelengths, self.responsivity
        if not wls or wl < wls[0] or wl > wls[-1]:
            return 0.0
        # Binary search for the bracketing pair.
        lo, hi = 0, len(wls) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if wls[mid] <= wl:
                lo = mid
            else:
                hi = mid
        span = wls[hi] - wls[lo]
        if span == 0:
            return rs[lo]
        frac = (wl - wls[lo]) / span
        return rs[lo] + frac * (rs[hi] - rs[lo])

    def source_weighted_responsivity(self, source_wl, source_intensity):
        """Source-weighted mean responsivity R̄ = ∫ s(λ)R(λ) dλ / ∫ s(λ) dλ.

        s is the (relative) source spectrum sampled at `source_wl`; R is this
        calibration's responsivity, interpolated onto the same wavelengths.
        Trapezoidal integration over the source grid. This is the factor that
        makes the same voltage mean different physical levels for different
        source spectra. Returns None if the integral can't be formed.
        """
        if len(source_wl) != len(source_intensity) or len(source_wl) < 2:
            return None
        num = den = 0.0
        for i in range(len(source_wl) - 1):
            w0, w1 = source_wl[i], source_wl[i + 1]
            dw = w1 - w0
            if dw <= 0:
                continue
            s0, s1 = source_intensity[i], source_intensity[i + 1]
            r0 = self.responsivity_at(w0)
            r1 = self.responsivity_at(w1)
            num += 0.5 * (s0 * r0 + s1 * r1) * dw
            den += 0.5 * (s0 + s1) * dw
        if den == 0:
            return None
        return num / den

    def voltage_to_value(self, voltage, source=None):
        """Convert a (dark-corrected) sensor voltage to a physical value.

        Model: physical = scale_factor · V / R̄_source

        source -- (wavelengths, intensities) of the light being measured, or
            None. The sensor integrates the source over its spectral response,
            so the same voltage means different physical levels for different
            spectra; supplying the source applies that spectral correction.
            With source=None, R̄_source defaults to 1.0 — i.e. you're measuring
            the same spectrum the absolute scale was calibrated against.

        Returns the value in `scale_units`, or None if no scale_factor is set
        or the source spectrum doesn't overlap the calibrated band.
        """
        if self.scale_factor is None:
            return None
        r_bar = 1.0
        if source is not None:
            r_bar = self.source_weighted_responsivity(source[0], source[1])
            if not r_bar:  # None or 0 — no usable overlap
                return None
        return self.scale_factor * voltage / r_bar


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


_default_calibration_cache = None


def default_calibration():
    """The bundled BPW34 datasheet-typical calibration (spectral shape only).

    A nominal fallback used when the device has no stored calibration: it carries
    the real R(λ) shape from the Vishay BPW34 datasheet but no absolute
    scale_factor (that needs the PCB transimpedance resistor and a reference
    measurement). provenance == 'datasheet-typical'; see is_nominal. Cached after
    first load. Returns None if the bundled file is missing.
    """
    global _default_calibration_cache
    if _default_calibration_cache is None:
        try:
            from importlib.resources import files

            text = (files("lightmeter") / "data" / "calibration_bpw34_typical.csv").read_text()
            _default_calibration_cache = parse_calibration(text)
        except (FileNotFoundError, ModuleNotFoundError, OSError):
            return None
    return _default_calibration_cache


def daylight_spectrum(temp_k=6500, lo_nm=380, hi_nm=1100, step_nm=5):
    """A nominal daylight spectrum as (wavelengths_nm, relative_intensities).

    A Planck blackbody at temp_k (default 6500 K ≈ CIE D65 correlated colour
    temperature) sampled across the band. Only the *shape* matters for source
    weighting (R̄ = ∫sR/∫s normalises out absolute scale), so this is returned
    unnormalised. A blackbody is used rather than the tabulated D65 illuminant
    because D65 stops at 830 nm, whereas this sensor (BPW34) responds out to
    ~1100 nm — truncating there would drop the near-IR the silicon sees. This is
    an approximation for an out-of-the-box estimate, not a metrological source.
    """
    c2 = 1.438776877e-2  # second radiation constant hc/k_B, m·K
    wls, intensities = [], []
    wl = lo_nm
    while wl <= hi_nm:
        lam = wl * 1e-9  # m
        # Planck spectral radiance per wavelength, relative (constants dropped).
        b = 1.0 / (lam**5 * (math.exp(c2 / (lam * temp_k)) - 1.0))
        wls.append(wl)
        intensities.append(b)
        wl += step_nm
    return wls, intensities


# CIE 1924 photopic luminous efficiency V(λ), 380–780 nm at 10 nm steps
# (peak 1.0 near 555 nm). Used to weight a spectrum for photometric units.
LUMINOUS_EFFICACY_PEAK = 683.0  # lm/W at 555 nm (definition of the candela)
PHOTOPIC_V = [
    (380, 0.0000), (390, 0.0001), (400, 0.0004), (410, 0.0012), (420, 0.0040),
    (430, 0.0116), (440, 0.0230), (450, 0.0380), (460, 0.0600), (470, 0.0910),
    (480, 0.1390), (490, 0.2080), (500, 0.3230), (510, 0.5030), (520, 0.7100),
    (530, 0.8620), (540, 0.9540), (550, 0.9950), (560, 0.9950), (570, 0.9520),
    (580, 0.8700), (590, 0.7570), (600, 0.6310), (610, 0.5030), (620, 0.3810),
    (630, 0.2650), (640, 0.1750), (650, 0.1070), (660, 0.0610), (670, 0.0320),
    (680, 0.0170), (690, 0.0082), (700, 0.0041), (710, 0.0021), (720, 0.0010),
    (730, 0.0005), (740, 0.0003), (750, 0.0001), (760, 0.0001), (770, 0.0000),
    (780, 0.0000),
]
_photopic_cache = None


def _photopic_response():
    """The photopic V(λ) curve wrapped as a Calibration so its trapezoidal
    source-weighting can be reused. Cached."""
    global _photopic_cache
    if _photopic_cache is None:
        wls = [w for w, _ in PHOTOPIC_V]
        vs = [v for _, v in PHOTOPIC_V]
        _photopic_cache = Calibration({}, wls, vs, "")
    return _photopic_cache


def luminous_efficacy(source_wl, source_intensity):
    """Luminous efficacy K of a source spectrum, in lm/W.

    K = 683 · V̄, where V̄ is the source-weighted mean photopic efficiency
    (∫ s V dλ / ∫ s dλ). Multiply a radiometric irradiance (W/m²) by K to get
    illuminance (lux). Returns None if the weighting can't be formed.
    """
    v_bar = _photopic_response().source_weighted_responsivity(source_wl, source_intensity)
    if v_bar is None:
        return None
    return LUMINOUS_EFFICACY_PEAK * v_bar


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
        # Autogain (autoexposure) is a firmware mode; this mirrors its state.
        # When True, read() sends `r` and the device steps gain before
        # replying with the settled gain.
        self.autogain = False
        # Device dark offset is persisted by firmware; a session zero may
        # temporarily override it. Both are volts, so gain changes are safe.
        self._device_dark_offset_v = DEFAULT_DARK_OFFSET_V
        self._session_zero_offset_v = None
        # Firmware-side averaging: each read() averages this many ADC samples
        # on the device and returns one Reading.
        self.average = 1
        # Identity reported by the device at connect (None until handshake runs).
        self.info = None
        # Cached calibration (loaded on demand via load_calibration()).
        self.calibration = None
        # Re-entrant lock guarding every serial transaction so the driver is
        # safe to share across threads (e.g. the GUI sampler). Re-entrant
        # because higher-level calls nest lower-level ones (zero() -> read()).
        self._lock = threading.RLock()
        # Opt-in: when True, read() transparently reconnects on a link error
        # instead of raising. Off by default so callers that manage their own
        # reconnect (e.g. main.py) keep seeing the exception.
        self.auto_reconnect = False
        self.open()

    def open(self):
        """(Re)open the serial port.

        The ESP32-C3 uses native USB CDC, which has no DTR/RTS auto-reset
        circuit to work around, so a plain open is fine on both Linux and
        Windows.
        """
        with self._lock:
            self.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self._handshake()

    @property
    def connected(self):
        """True if the serial port is currently open."""
        return self.ser is not None and self.ser.is_open

    def reconnect(self, attempts=5, backoff=0.5):
        """Re-establish the link after a disconnect (e.g. unplug/replug).

        Re-detects the port if the original path is gone (the device may
        re-enumerate to a different name), reopens, and handshakes. Returns True
        once connected, False if all attempts fail. Raises nothing.
        """
        with self._lock:
            for i in range(attempts):
                try:
                    self.close()
                    # The device can re-enumerate under a new path; re-detect if
                    # the configured one isn't present, but keep an explicit port.
                    try:
                        self.port = autodetect_port()
                    except Exception:
                        pass  # fall back to the existing self.port
                    self.ser = serial.Serial(self.port, self.baud, timeout=1)
                    self._handshake()
                    if self.connected:
                        log.info("reconnected to %s", self.port)
                        return True
                except (serial.SerialException, OSError) as exc:
                    log.warning("reconnect attempt %d/%d failed: %s", i + 1, attempts, exc)
                time.sleep(backoff)
            return False

    def ping(self):
        """Return True if the device answers `pong`. No I2C — pure link check."""
        with self._lock:
            if not self.connected:
                return False
            self.ser.write(b"p")
            return self.ser.readline().decode(errors="ignore").strip() == "pong"

    def _try_sync(self):
        """Drain stale input and probe once with a ping. True if `pong` returns."""
        self.ser.reset_input_buffer()
        return self.ping()

    def identify(self):
        """Query device identity (`I`). Returns DeviceInfo or None on failure."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(b"I")
            line = self.ser.readline().decode(errors="ignore").strip()
        return parse_identity(line)

    # Longest the device can block waiting for a command's payload (the `W`
    # receive loop). Recovery must wait at least this long, in silence, for a
    # stuck command to self-abort.
    DEVICE_CMD_TIMEOUT = 5.0

    def _handshake(self):
        """Re-establish a clean command stream after (re)opening the port.

        A killed script or interrupted transfer can leave the device mid-command
        with unread bytes, desyncing the stream. If a quick ping doesn't get a
        clean `pong`, the device is likely mid-transfer (e.g. a `W` awaiting its
        payload) — so go SILENT for longer than its receive timeout, letting the
        stuck command self-abort. Do not keep pinging: each byte we send feeds
        the pending read and resets its timeout, so it would never recover.

        After resync, read identity and check the protocol version. Best-effort:
        logs warnings rather than raising so a device with older firmware
        (no `I`/`p`) still connects. The `W` abort path discards only the partial
        upload, never the stored calibration.
        """
        if not self._try_sync():
            log.warning("no pong from %s; resyncing (waiting out device timeout)", self.port)
            time.sleep(self.DEVICE_CMD_TIMEOUT + 0.5)
            if not self._try_sync():
                log.warning("%s still unresponsive after resync", self.port)
        self.info = self.identify()
        if self.info is None:
            log.warning("device on %s did not report identity (old firmware?)", self.port)
            return
        if self.info.proto != PROTO_VERSION:
            log.warning(
                "protocol mismatch on %s: device proto=%d, driver expects %d",
                self.port, self.info.proto, PROTO_VERSION,
            )
        self._verify_constants()
        self._load_device_dark_offset()

    def _load_device_dark_offset(self):
        """Adopt the firmware-reported dark offset when it is valid.

        Older firmware omits the optional identity field; retain the calculated
        schematic default in that case.
        """
        value = self.info.fields.get("dark")
        if value is None:
            return
        try:
            offset = float(value)
        except ValueError:
            offset = None
        if offset is None or not math.isfinite(offset) or abs(offset) > MAX_DARK_OFFSET_V:
            log.warning("invalid device dark offset: %r", value)
            return
        self._device_dark_offset_v = offset

    def _verify_constants(self):
        """Warn if the driver's mirrored constants drift from the device's.

        The firmware is the source of truth for the gain table and saturation
        voltage and reports them in the identity line; this catches a driver/
        firmware mismatch (e.g. after editing one but not the other) at connect.
        """
        fields = self.info.fields
        if "gains" in fields:
            try:
                dev = [float(x) for x in fields["gains"].split(",")]
            except ValueError:
                dev = None
            if dev and dev != GAIN_VOLTAGES:
                log.warning("gain table mismatch: device=%s driver=%s", dev, GAIN_VOLTAGES)
        if "vsat" in fields:
            try:
                if abs(float(fields["vsat"]) - SATURATION_VOLTAGE) > 0.01:
                    log.warning(
                        "saturation voltage mismatch: device=%s driver=%s",
                        fields["vsat"], SATURATION_VOLTAGE,
                    )
            except ValueError:
                pass

    def _read_raw(self):
        """One locked serial transaction: send 'r<n>' and parse the reply line.

        Returns (raw, sensor_sat, adc_sat, gain) or None on a timeout /
        malformed line. The 4th field (proto 2) is the gain the device used —
        the settled gain when autogain is on. Raises serial.SerialException /
        OSError if the link is gone. Logs parse failures.
        """
        n = self.average if self.average and self.average > 1 else 1
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(f"r{n}\n".encode())
            line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            log.debug("read timeout (no line)")
            return None
        parts = line.split(",")
        if len(parts) != 4:
            log.debug("read parse error: %r", line)
            return None
        try:
            return int(parts[0]), bool(int(parts[1])), bool(int(parts[2])), int(parts[3])
        except ValueError:
            log.debug("read parse error: %r", line)
            return None

    def read(self):
        """Return a Reading(value, sensor_sat, adc_sat) or None on parse failure.

        value      -- light level as % of ADC full-scale (0–100)
        sensor_sat -- op-amp output near supply rail (low-gain settings)
        adc_sat    -- ADC raw reading hit 32767 (high-gain settings)

        Autoexposure lives in the firmware (see set_autogain); the device
        reports the gain it used, which we record so reading_voltage converts
        correctly. On a link error: if auto_reconnect is set, reconnects and
        returns None; otherwise the exception propagates.
        """
        try:
            raw_parts = self._read_raw()
        except (serial.SerialException, OSError) as exc:
            if not self.auto_reconnect:
                raise
            log.warning("read link error: %s — reconnecting", exc)
            self.reconnect()
            return None
        if raw_parts is None:
            return None
        raw, sensor_sat, adc_sat, gain = raw_parts
        self.gain = gain  # device is the source of truth (autogain may have stepped)
        reading = Reading(raw / 32767 * 100, sensor_sat, adc_sat)
        # Subtract the effective dark offset last (display only) so it never
        # affects saturation flags. Use the gain the sample was actually taken at.
        offset = self.effective_dark_offset_v
        if offset:
            reading.value -= offset / GAIN_VOLTAGES[gain] * 100
        return reading

    def reading_voltage(self, reading):
        """Sensor voltage (V) for a Reading at the current gain, dark-corrected.

        Reading.value is % of full-scale, which is gain-relative; this recovers
        the absolute voltage the physical conversion needs.
        """
        return reading.value / 100 * GAIN_VOLTAGES[self.gain]

    def read_physical(self, source=None):
        """Read once and convert to a physical value via the cached calibration.

        Loads calibration on first use (see load_calibration). Returns the value
        in `calibration.scale_units`, or None if there's no reading, no
        calibration/scale_factor, or the source spectrum doesn't overlap the
        calibrated band. `source` is forwarded to Calibration.voltage_to_value.
        """
        if self.calibration is None:
            self.load_calibration()
        if self.calibration is None:
            return None
        reading = self.read()
        if reading is None:
            return None
        return self.calibration.voltage_to_value(self.reading_voltage(reading), source)

    def _measure_uncorrected_offset(self, n):
        """Average n uncorrected samples in volts, or return None on failure."""
        voltages = []
        for _ in range(max(1, int(n))):
            raw_parts = self._read_raw()
            if raw_parts is None:
                continue
            raw, _, _, gain = raw_parts
            self.gain = gain
            voltages.append(raw / 32767 * GAIN_VOLTAGES[gain])
        return sum(voltages) / len(voltages) if voltages else None

    @property
    def device_dark_offset_v(self):
        """Persisted per-device electrical dark correction in volts."""
        return self._device_dark_offset_v

    @property
    def session_zero_offset_v(self):
        """Temporary session dark/background correction, or None if inactive."""
        return self._session_zero_offset_v

    @property
    def effective_dark_offset_v(self):
        """Active correction in volts: session zero overrides device calibration."""
        return (
            self._session_zero_offset_v
            if self._session_zero_offset_v is not None
            else self._device_dark_offset_v
        )

    def set_device_dark_offset(self, offset_v):
        """Persist a per-device electrical dark correction in volts.

        Valid offsets are finite values within ±0.25 V. Returns True only after
        firmware acknowledges the flash write.
        """
        try:
            offset = float(offset_v)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(offset) or abs(offset) > MAX_DARK_OFFSET_V:
            return False
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(f"d{offset:.9g}\n".encode())
            resp = self.ser.readline().decode(errors="ignore").strip()
        if resp != "ok":
            log.warning("set_device_dark_offset(%s) failed: %s", offset, _err_text(resp))
            return False
        self._device_dark_offset_v = offset
        if self.info is not None:
            self.info.fields["dark"] = f"{offset:.9g}"
        return True

    def reset_device_dark_offset(self):
        """Restore the calculated R1/R3 divider baseline on the device."""
        return self.set_device_dark_offset(DEFAULT_DARK_OFFSET_V)

    def calibrate_device_dark_offset(self, n=200):
        """Measure covered-sensor dark voltage and persist it on the device.

        Autogain and session zeroing are suspended while sampling the true
        electrical level. Returns the persisted volts, or None on failure.
        """
        was_autogain = self.autogain
        previous_session = self._session_zero_offset_v
        if was_autogain:
            self.set_autogain(False)
        try:
            self._session_zero_offset_v = None
            offset = self._measure_uncorrected_offset(n)
        finally:
            self._session_zero_offset_v = previous_session
            if was_autogain:
                self.set_autogain(True)
        return offset if offset is not None and self.set_device_dark_offset(offset) else None

    def zero(self, n=50):
        """Temporarily zero the current dark/background level over n samples.

        A session zero overrides, but never overwrites, the persisted device
        dark correction. Returns the effective offset in volts.
        """
        was_autogain = self.autogain
        previous_session = self._session_zero_offset_v
        if was_autogain:
            self.set_autogain(False)
        try:
            self._session_zero_offset_v = None
            offset = self._measure_uncorrected_offset(n)
            self._session_zero_offset_v = offset if offset is not None else previous_session
        finally:
            if was_autogain:
                self.set_autogain(True)
        return self.effective_dark_offset_v

    def clear_zero(self):
        """Clear the session zero and resume the persisted device correction."""
        self._session_zero_offset_v = None

    @property
    def is_zeroed(self):
        """True when either a session or persisted dark correction is active."""
        return self.effective_dark_offset_v != 0.0

    @property
    def zero_offset(self):
        """Current effective dark correction in volts."""
        return self.effective_dark_offset_v

    def set_autogain(self, enabled):
        """Enable/disable firmware autoexposure (a1/a0). read() then reports
        the settled gain. Returns True on device ack."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(b"a1" if enabled else b"a0")
            resp = self.ser.readline().decode(errors="ignore").strip()
        if resp == "ok":
            self.autogain = enabled
            return True
        log.warning("set_autogain(%s) failed: %s", enabled, _err_text(resp))
        return False

    def get_autogain(self):
        """Query device autogain state and current gain (A). Returns
        (enabled, gain) or None on failure."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(b"A")
            resp = self.ser.readline().decode(errors="ignore").strip()
        parts = resp.split()
        if len(parts) == 2 and parts[0] in ("0", "1") and parts[1].isdigit():
            return parts[0] == "1", int(parts[1])
        return None

    def set_gain(self, gain_index):
        """Set ADC gain. gain_index 0–5 maps to ±6.144V … ±0.256V. Returns True on success."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(f"g{gain_index}".encode())
            resp = self.ser.readline().decode(errors="ignore").strip()
        if resp == "ok":
            self.gain = gain_index
            self.autogain = False  # manual gain turns firmware autoexposure off
            return True
        log.warning("set_gain(%s) failed: %s", gain_index, _err_text(resp))
        return False

    def get_gain(self):
        """Return current gain index (0–5), or None on failure."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(b"G")
            resp = self.ser.readline().decode(errors="ignore").strip()
        return int(resp) if resp.isdigit() else None

    # --- Calibration storage (on-device LittleFS) ---------------------------

    def has_calibration(self):
        """Return the stored calibration size in bytes (0 if none)."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(b"H")
            resp = self.ser.readline().decode(errors="ignore").strip()
        return int(resp) if resp.isdigit() else 0

    def write_calibration(self, text):
        """Store calibration CSV text on the device. Returns True if the
        device-computed CRC32 matches the host's (transfer verified)."""
        data = text.encode() if isinstance(text, str) else text
        expected = binascii.crc32(data) & 0xFFFFFFFF
        with self._lock:
            if not self.connected:
                self.open()
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
            ok = int(parts[1]) == expected
            if not ok:
                log.warning("write_calibration CRC mismatch: device=%s host=%s", parts[1], expected)
            elif self.calibration is not None:
                self.calibration = None  # invalidate cache; reloaded on next use
            return ok
        log.warning("write_calibration failed: %s", _err_text(resp))
        return False

    def write_calibration_file(self, path):
        """Store a calibration CSV file from disk. Returns True on verified write."""
        with open(path, "r") as f:
            return self.write_calibration(f.read())

    def read_calibration(self):
        """Read the stored calibration. Returns a Calibration, or None if the
        device has no calibration or the CRC32 check fails."""
        with self._lock:
            if not self.connected:
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

    def load_calibration(self, use_default=True):
        """Read the device calibration and cache it in self.calibration.

        Returns the Calibration. read_physical() calls this on first use; call it
        explicitly to refresh after a write. When the device has no stored cal and
        use_default is True, falls back to the bundled BPW34 datasheet-typical
        calibration (provenance 'datasheet-typical', see Calibration.is_nominal) so
        readings have a real spectral shape out of the box — note it has no absolute
        scale_factor, so read_physical() still returns None until one is set. Pass
        use_default=False to get None when the device is uncalibrated.
        """
        cal = self.read_calibration()
        if cal is None and use_default:
            cal = default_calibration()
        self.calibration = cal
        return self.calibration

    def clear_calibration(self):
        """Erase the stored calibration. Returns True on success."""
        with self._lock:
            if not self.connected:
                self.open()
            self.ser.write(b"X")
            ok = self.ser.readline().decode(errors="ignore").strip() == "ok"
        if ok:
            self.calibration = None  # invalidate cache
        return ok

    def close(self):
        with self._lock:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
