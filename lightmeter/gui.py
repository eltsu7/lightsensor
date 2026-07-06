import argparse
import csv
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import ttk

import serial
import numpy as np
from matplotlib.patches import Patch
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)
from matplotlib.figure import Figure

from lightmeter.sensor import (
    LightSensor,
    autodetect_port,
    best_gain,
    daylight_spectrum,
    default_calibration,
    luminous_efficacy,
    GAIN_LABELS,
    GAIN_VOLTAGES,
    DEFAULT_GAIN,
    SATURATION_VOLTAGE,
)

# Defaults
DEFAULT_INTERVAL_MS = 0  # 0 = scan as fast as the device allows
WINDOW_SECONDS = 10  # how much history to keep on screen
REFRESH_MS = 30  # GUI redraw interval (~33 fps); decoupled from sampling
MAX_POINTS = 20000  # cap on stored points

# Recordings are saved here, one CSV per session, named by start time so a
# future "previous measurements" selector can list and sort them chronologically.
RECORDINGS_DIR = Path(__file__).parent / "recordings"
CSV_COLUMNS = ["time_s", "voltage_v", "sensor_sat", "adc_sat"]


class SensorSampler:
    """Reads the sensor in a background thread so serial I/O never blocks or
    crashes the GUI. The scan interval can be changed at runtime."""

    def __init__(self, port, baud, interval_s):
        self.port = port
        self.baud = baud
        self.interval_s = interval_s  # plain float; updated from GUI thread
        self._lock = threading.Lock()
        self._desired_gain = DEFAULT_GAIN  # updated from GUI thread
        self._applied_gain = None  # forces (re)apply on connect / change
        self._skip_next = False  # discard one sample after a gain change
        self._sensor_sat = False
        self._adc_sat = False
        self._autogain_continuous = True   # desired autoexposure state
        self._autogain_applied = None      # device state; None forces (re)apply
        self._zero_request = 0  # n samples to zero over (0 = no request)
        self._clear_zero_req = False
        self._zeroing = False
        self._zeroed = False
        self._zero_offset_v = 0.0
        self._rate_window = deque()  # recent sample times for sps calc
        self._average = 1  # firmware-side samples averaged per read
        # Physical-units conversion (volts -> W/m²) under a nominal daylight
        # spectrum. Seeded from the bundled default cal so it works before a
        # device connects; refreshed from the device's own cal on connect.
        self._daylight = daylight_spectrum()
        _dc = default_calibration()
        self._physical_factor = (
            _dc.voltage_to_value(1.0, source=self._daylight) if _dc else None
        )
        self._physical_units = _dc.scale_units if _dc else None
        # Luminous efficacy of the daylight spectrum (lm/W), constant. Multiplying
        # the radiometric W/m²-per-volt factor by it gives lux per volt.
        self._daylight_efficacy = luminous_efficacy(*self._daylight)
        self._times = deque(maxlen=MAX_POINTS)
        self._values = deque(maxlen=MAX_POINTS)
        # Recording: unbounded capture independent of the display buffer.
        self._recording = False
        self._rec_start = None
        self._rec_times = []
        self._rec_values = []
        self._rec_sensor_sat = []
        self._rec_adc_sat = []
        self._running = threading.Event()
        self._acquiring = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self.status = "starting"

    def start(self):
        self._running.set()
        self._acquiring.set()
        self._start = time.perf_counter()
        self._thread.start()

    def pause(self):
        self._acquiring.clear()
        self.status = "stopped"

    def resume(self):
        self._acquiring.set()

    @property
    def acquiring(self):
        return self._acquiring.is_set()

    def set_gain(self, gain_index):
        """Request a manual gain change; also stops continuous autogain."""
        self._autogain_continuous = False
        self._desired_gain = gain_index
        self._skip_next = True

    def enable_autogain(self):
        self._autogain_continuous = True

    def disable_autogain(self):
        self._autogain_continuous = False

    def start_zero(self, n=50):
        """Request a dark-offset measurement over n samples (sampler thread)."""
        self._zero_request = n

    def request_clear_zero(self):
        self._clear_zero_req = True

    def set_average(self, n):
        """Set the number of ADC samples the firmware averages per read."""
        self._average = max(1, int(n))

    @property
    def average(self):
        return self._average

    @property
    def physical_factor(self):
        """W/m² per volt under the nominal daylight spectrum (None if unknown)."""
        return self._physical_factor

    @property
    def physical_units(self):
        return self._physical_units

    @property
    def lux_factor(self):
        """Lux per volt under the nominal daylight spectrum (None if unknown)."""
        if self._physical_factor is None or self._daylight_efficacy is None:
            return None
        return self._physical_factor * self._daylight_efficacy

    def _note_sample(self, t):
        """Record a sample timestamp and trim to the last 0.25 s (call under lock)."""
        self._rate_window.append(t)
        cutoff = t - 0.25
        while self._rate_window and self._rate_window[0] < cutoff:
            self._rate_window.popleft()

    @property
    def sample_rate(self):
        """Samples per second over the last ~0.25 s (0 if idle/insufficient)."""
        with self._lock:
            w = self._rate_window
            if len(w) < 2 or time.perf_counter() - w[-1] > 0.5:
                return 0.0  # idle / paused
            span = w[-1] - w[0]
            return (len(w) - 1) / span if span > 0 else 0.0

    @property
    def zeroing(self):
        return self._zeroing

    @property
    def zeroed(self):
        return self._zeroed

    @property
    def zero_offset(self):
        return self._zero_offset_v

    @property
    def autogain_continuous(self):
        return self._autogain_continuous

    def clear(self):
        with self._lock:
            self._times.clear()
            self._values.clear()
            self._start = time.perf_counter()

    def start_recording(self):
        with self._lock:
            self._rec_times = []
            self._rec_values = []
            self._rec_sensor_sat = []
            self._rec_adc_sat = []
            self._rec_start = time.perf_counter()
            self._recording = True

    def stop_recording(self):
        """Stop recording and return (times, values, sensor_sat, adc_sat) lists."""
        with self._lock:
            self._recording = False
            return (
                list(self._rec_times),
                list(self._rec_values),
                list(self._rec_sensor_sat),
                list(self._rec_adc_sat),
            )

    @property
    def is_recording(self):
        return self._recording

    @property
    def recording_count(self):
        return len(self._rec_times)

    def shutdown(self):
        self._running.clear()

    def _run(self):
        last_value = 0.0
        sensor = None
        while self._running.is_set():
            if not self._acquiring.is_set():
                time.sleep(0.05)
                continue
            loop_start = time.perf_counter()
            try:
                if sensor is None:
                    self.status = "connecting..."
                    sensor = LightSensor(self.port, self.baud)
                    self._applied_gain = None  # reapply gain on fresh connection
                    # Prefer the device's own calibration for unit conversion;
                    # falls back to the bundled default when it has none.
                    cal = sensor.load_calibration()
                    if cal is not None:
                        f = cal.voltage_to_value(1.0, source=self._daylight)
                        if f is not None:
                            self._physical_factor = f
                            self._physical_units = cal.scale_units
                    self.status = "connected"

                sensor.average = self._average
                if self._clear_zero_req:
                    self._clear_zero_req = False
                    sensor.clear_zero()
                    self._zeroed = False
                    self._zero_offset_v = 0.0
                if self._zero_request > 0:
                    n = self._zero_request
                    self._zero_request = 0
                    self._zeroing = True
                    try:
                        sensor.zero(n)
                        self._zeroed = sensor.is_zeroed
                        self._zero_offset_v = sensor.zero_offset
                    finally:
                        self._zeroing = False
                    self._skip_next = True
                    continue
                # Manual gain (only when autogain is off; a manual gain also
                # turns autoexposure off on the device).
                if not self._autogain_continuous and self._desired_gain != self._applied_gain:
                    if sensor.set_gain(self._desired_gain):
                        self._applied_gain = self._desired_gain
                        self._autogain_applied = False
                        self._skip_next = True
                    continue
                # Autogain mode change (firmware-side autoexposure).
                if self._autogain_continuous != self._autogain_applied:
                    if sensor.set_autogain(self._autogain_continuous):
                        self._autogain_applied = self._autogain_continuous
                        self._skip_next = True
                    continue
                reading = sensor.read()
                if self._skip_next:
                    self._skip_next = False
                    continue
                # The device may have stepped gain (autoexposure); sensor.gain
                # reflects it after read().
                if sensor.gain != self._applied_gain:
                    self._applied_gain = sensor.gain
                    self._desired_gain = sensor.gain
                    self._skip_next = True
                    continue
                if reading is None:
                    value = last_value
                else:
                    # Store as actual voltage (V) so data is gain-independent.
                    value = reading.value * GAIN_VOLTAGES[self._applied_gain] / 100
                    last_value = value
                    self._sensor_sat = reading.sensor_sat
                    self._adc_sat = reading.adc_sat
            except (serial.SerialException, OSError) as exc:
                # Transient link error: drop the connection and retry.
                self.status = f"reconnecting ({exc.__class__.__name__})"
                if sensor is not None:
                    sensor.close()
                    sensor = None
                self._applied_gain = None
                time.sleep(0.5)
                continue

            t = time.perf_counter()
            now = t - self._start
            with self._lock:
                self._times.append(now)
                self._values.append(value)
                self._note_sample(t)
                if self._recording:
                    self._rec_times.append(t - self._rec_start)
                    self._rec_values.append(value)
                    self._rec_sensor_sat.append(int(self._sensor_sat))
                    self._rec_adc_sat.append(int(self._adc_sat))

            # Pace the loop to the (possibly updated) target interval.
            remaining = self.interval_s - (time.perf_counter() - loop_start)
            if remaining > 0:
                time.sleep(remaining)

        if sensor is not None:
            sensor.close()

    @property
    def sensor_saturated(self):
        return self._sensor_sat

    @property
    def adc_saturated(self):
        return self._adc_sat

    @property
    def current_gain(self):
        return self._applied_gain if self._applied_gain is not None else DEFAULT_GAIN

    def snapshot(self):
        with self._lock:
            return list(self._times), list(self._values)


def save_recording(times, values, sensor_sat, adc_sat, started_at=None):
    """Write a recording to RECORDINGS_DIR as CSV. Returns the file Path.

    Filename is the recording start time so a future selector can list and
    sort recordings chronologically.
    """
    RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
    started_at = started_at or datetime.now()
    path = RECORDINGS_DIR / f"rec_{started_at:%Y-%m-%d_%H-%M-%S}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for row in zip(times, values, sensor_sat, adc_sat):
            writer.writerow(row)
    return path


def open_recording_plot(parent, path):
    """Open a recorded CSV in a standalone, zoom/pan-able plot window.

    Reusable by a future "previous measurements" selector — it only needs a
    path to a CSV written by save_recording().
    """
    path = Path(path)
    times, values, sensor_sat, adc_sat = [], [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            times.append(float(r["time_s"]))
            values.append(float(r["voltage_v"]))
            sensor_sat.append(int(r["sensor_sat"]))
            adc_sat.append(int(r["adc_sat"]))

    win = tk.Toplevel(parent)
    win.title(f"Recording — {path.name}")
    fig = Figure(figsize=(9, 5), dpi=100)
    ax = fig.add_subplot(111)
    ax.plot(times, values, lw=1.0, color="tab:orange")
    # Mark saturated samples, if any.
    sat_t = [t for t, s, a in zip(times, sensor_sat, adc_sat) if s or a]
    sat_v = [v for v, s, a in zip(values, sensor_sat, adc_sat) if s or a]
    if sat_t:
        ax.plot(sat_t, sat_v, ".", color="red", ms=3, label="saturated")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Light (V)")
    ax.set_title(path.stem)
    ax.grid(True, alpha=0.3)

    canvas = FigureCanvasTkAgg(fig, master=win)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    NavigationToolbar2Tk(canvas, win)
    canvas.draw()
    return win


class SensorApp:
    """Tkinter window embedding the real-time matplotlib plot plus controls."""

    def __init__(self, root, sampler, port):
        self.root = root
        self.sampler = sampler
        root.title(f"Light Sensor ({port})")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        # --- sidebar (right) ----------------------------------------------
        sidebar = ttk.Frame(root, padding=8)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)

        def section(title):
            ttk.Label(sidebar, text=title, font=("TkDefaultFont", 9, "bold")).pack(
                side=tk.TOP, anchor=tk.W, pady=(10, 2)
            )
            frame = ttk.Frame(sidebar)
            frame.pack(side=tk.TOP, fill=tk.X)
            return frame

        # View section
        view = section("View")
        self.follow_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            view,
            text="Follow latest",
            variable=self.follow_var,
        ).pack(side=tk.TOP, anchor=tk.W)
        self.autoscale_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            view,
            text="Auto Y-scale",
            variable=self.autoscale_var,
        ).pack(side=tk.TOP, anchor=tk.W)
        unit_row = ttk.Frame(view)
        unit_row.pack(side=tk.TOP, fill=tk.X, anchor=tk.W)
        ttk.Label(unit_row, text="Units:").pack(side=tk.LEFT)
        # W/m² is a nominal irradiance under an assumed daylight spectrum (see
        # the sampler's physical_factor); default to it when available.
        self.unit_var = tk.StringVar(
            value="W/m²" if sampler.physical_factor else "V"
        )
        ttk.Combobox(
            unit_row,
            width=6,
            state="readonly",
            values=["%", "V", "W/m²", "lux"],
            textvariable=self.unit_var,
        ).pack(side=tk.LEFT, padx=(4, 0))
        self.rawpoints_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            view,
            text="Show raw data points",
            variable=self.rawpoints_var,
        ).pack(side=tk.TOP, anchor=tk.W)
        rollavg_row = ttk.Frame(view)
        rollavg_row.pack(side=tk.TOP, fill=tk.X, anchor=tk.W)
        self.rollavg_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            rollavg_row,
            text="Rolling average",
            variable=self.rollavg_var,
        ).pack(side=tk.LEFT)
        self.rollavg_sec_var = tk.StringVar(value="0.05")
        ttk.Entry(rollavg_row, width=4, textvariable=self.rollavg_sec_var).pack(
            side=tk.LEFT, padx=(4, 2)
        )
        ttk.Label(rollavg_row, text="s").pack(side=tk.LEFT)

        # Overlays section
        overlays = section("Overlays")
        self.avg_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            overlays,
            text="Window average",
            variable=self.avg_var,
        ).pack(side=tk.TOP, anchor=tk.W)
        self.fit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            overlays,
            text="Line fit",
            variable=self.fit_var,
        ).pack(side=tk.TOP, anchor=tk.W)
        self.noise_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            overlays,
            text="Noise band",
            variable=self.noise_var,
        ).pack(side=tk.TOP, anchor=tk.W)

        # Gain section
        gain = section("Gain")
        gain_row = ttk.Frame(gain)
        gain_row.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(gain_row, text="−", width=2, command=self._gain_down).pack(side=tk.LEFT)
        self.gain_var = tk.StringVar(value=GAIN_LABELS[DEFAULT_GAIN])
        gain_combo = ttk.Combobox(
            gain_row,
            width=8,
            state="readonly",
            values=GAIN_LABELS,
            textvariable=self.gain_var,
        )
        gain_combo.pack(side=tk.LEFT, padx=2)
        gain_combo.bind("<<ComboboxSelected>>", lambda _e: self._apply_gain())
        ttk.Button(gain_row, text="+", width=2, command=self._gain_up).pack(side=tk.LEFT)
        autogain_label = "Auto gain ●" if sampler.autogain_continuous else "Auto gain"
        self._autogain_btn = ttk.Button(gain, text=autogain_label, command=self._toggle_autogain)
        self._autogain_btn.pack(side=tk.TOP, fill=tk.X, pady=(4, 0))

        # Zero (dark-offset) section
        zero = section("Zero")
        self._zero_btn = ttk.Button(zero, text="Zero (dark)", command=self._zero)
        self._zero_btn.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(zero, text="Clear zero", command=self._clear_zero).pack(
            side=tk.TOP, fill=tk.X, pady=(4, 0)
        )

        # Acquisition section
        acq = section("Acquisition")
        interval_row = ttk.Frame(acq)
        interval_row.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(interval_row, text="Scan interval (ms):").pack(side=tk.LEFT)
        self.interval_var = tk.StringVar(value=str(int(sampler.interval_s * 1000)))
        interval_entry = ttk.Entry(interval_row, width=6, textvariable=self.interval_var)
        interval_entry.pack(side=tk.LEFT, padx=(4, 4))
        interval_entry.bind("<Return>", lambda _e: self._apply_interval())
        ttk.Button(acq, text="Apply interval", command=self._apply_interval).pack(
            side=tk.TOP, fill=tk.X, pady=(4, 0)
        )
        avg_row = ttk.Frame(acq)
        avg_row.pack(side=tk.TOP, fill=tk.X, pady=(4, 0))
        ttk.Label(avg_row, text="Avg samples:").pack(side=tk.LEFT)
        self.average_var = tk.StringVar(value=str(sampler.average))
        avg_entry = ttk.Entry(avg_row, width=6, textvariable=self.average_var)
        avg_entry.pack(side=tk.LEFT, padx=(4, 4))
        avg_entry.bind("<Return>", lambda _e: self._apply_average())
        ttk.Button(acq, text="Apply averaging", command=self._apply_average).pack(
            side=tk.TOP, fill=tk.X, pady=(4, 0)
        )
        self.startstop_btn = ttk.Button(acq, text="Stop", command=self._toggle_run)
        self.startstop_btn.pack(side=tk.TOP, fill=tk.X, pady=(4, 0))
        ttk.Button(acq, text="Clear", command=self.sampler.clear).pack(
            side=tk.TOP, fill=tk.X, pady=(4, 0)
        )
        self._record_btn = ttk.Button(acq, text="● Record", command=self._toggle_record)
        self._record_btn.pack(side=tk.TOP, fill=tk.X, pady=(4, 0))
        self._record_started_at = None

        self.status_var = tk.StringVar(value="")
        ttk.Label(sidebar, textvariable=self.status_var, wraplength=160).pack(
            side=tk.BOTTOM, anchor=tk.W, pady=(10, 0)
        )

        # --- plot ----------------------------------------------------------
        plot_frame = ttk.Frame(root)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(8, 4.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        (self.line,) = self.ax.plot([], [], lw=1.5, color="tab:orange", zorder=3)
        (self.avg_line,) = self.ax.plot(
            [], [], lw=1.5, ls="--", color="tab:blue", label="average", zorder=1
        )
        (self.fit_line,) = self.ax.plot(
            [], [], lw=1.5, ls="--", color="tab:green", label="fit", zorder=1
        )
        (self.rollavg_line,) = self.ax.plot(
            [], [], lw=1.5, color="tab:purple", zorder=4, label="rolling avg"
        )
        self._noise_patch = None
        self.sat_line = self.ax.axhline(y=0, color="red", ls="--", lw=0.8, alpha=0.5, zorder=2)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Light (%)")
        self.ax.set_ylim(0, 100)
        self.ax.grid(True, alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, plot_frame)  # pan/zoom/save toolbar

        # Detect user-driven axis changes (toolbar pan/zoom) so we can drop the
        # automatic following/scaling. Our own programmatic limit updates are
        # guarded by _suppress_lim_cb so they don't trip these callbacks.
        self._suppress_lim_cb = False
        self.ax.callbacks.connect("xlim_changed", self._on_xlim_changed)
        self.ax.callbacks.connect("ylim_changed", self._on_ylim_changed)

        self._schedule_redraw()

    def _set_xlim(self, lo, hi):
        self._suppress_lim_cb = True
        self.ax.set_xlim(lo, hi)
        self._suppress_lim_cb = False

    def _set_ylim(self, lo, hi):
        self._suppress_lim_cb = True
        self.ax.set_ylim(lo, hi)
        self._suppress_lim_cb = False

    def _on_xlim_changed(self, _ax):
        # User panned/zoomed in time -> stop following the newest points.
        if not self._suppress_lim_cb:
            self.follow_var.set(False)

    def _on_ylim_changed(self, _ax):
        # User zoomed the Y-axis -> drop automatic Y-scaling.
        if not self._suppress_lim_cb:
            self.autoscale_var.set(False)

    def _toggle_run(self):
        if self.sampler.acquiring:
            self.sampler.pause()
            self.startstop_btn.config(text="Start")
        else:
            self.sampler.resume()
            self.startstop_btn.config(text="Stop")

    def _toggle_record(self):
        if self.sampler.is_recording:
            data = self.sampler.stop_recording()
            self._record_btn.config(text="● Record")
            times, values, sensor_sat, adc_sat = data
            if times:
                path = save_recording(
                    times,
                    values,
                    sensor_sat,
                    adc_sat,
                    started_at=self._record_started_at,
                )
                open_recording_plot(self.root, path)
        else:
            self._record_started_at = datetime.now()
            self.sampler.start_recording()
            self._record_btn.config(text="■ Stop recording")

    def _zero(self):
        self.sampler.start_zero(50)

    def _clear_zero(self):
        self.sampler.request_clear_zero()

    def _apply_gain(self):
        gain_index = GAIN_LABELS.index(self.gain_var.get())
        self.sampler.set_gain(gain_index)  # also disables continuous autogain
        self._autogain_btn.config(text="Auto gain")

    def _gain_up(self):
        idx = GAIN_LABELS.index(self.gain_var.get())
        if idx < len(GAIN_LABELS) - 1:
            self.gain_var.set(GAIN_LABELS[idx + 1])
            self._apply_gain()  # _apply_gain already resets autogain button

    def _gain_down(self):
        idx = GAIN_LABELS.index(self.gain_var.get())
        if idx > 0:
            self.gain_var.set(GAIN_LABELS[idx - 1])
            self._apply_gain()  # scale changed; old samples no longer comparable

    def _toggle_autogain(self):
        if self.sampler.autogain_continuous:
            self.sampler.disable_autogain()
            self._autogain_btn.config(text="Auto gain")
        else:
            self.sampler.enable_autogain()
            self._autogain_btn.config(text="Auto gain ●")

    def _apply_interval(self):
        try:
            ms = float(self.interval_var.get())
            if ms < 0:
                raise ValueError
        except ValueError:
            # Reset the field to the current value on bad input.
            self.interval_var.set(str(int(self.sampler.interval_s * 1000)))
            return
        self.sampler.interval_s = ms / 1000.0

    def _apply_average(self):
        try:
            n = int(float(self.average_var.get()))
            if n < 1:
                raise ValueError
        except ValueError:
            self.average_var.set(str(self.sampler.average))
            return
        self.sampler.set_average(n)
        self.average_var.set(str(n))

    @staticmethod
    def _rolling_average(times, values, window_s):
        """Trailing moving average: each point is the mean of all samples
        within the preceding window_s seconds. Vectorised via prefix sums."""
        prefix = np.concatenate([[0.0], np.cumsum(values)])
        lefts = np.searchsorted(times, times - window_s, side="left")
        idx = np.arange(len(values))
        counts = idx - lefts + 1
        sums = prefix[idx + 1] - prefix[lefts]
        return sums / counts

    def _redraw(self):
        times, values = self.sampler.snapshot()

        gain_v = GAIN_VOLTAGES[self.sampler.current_gain]
        # Saturation is a true-voltage limit; shift it down by the dark offset
        # so it lines up with the (offset-subtracted) displayed values.
        sat_v = SATURATION_VOLTAGE - self.sampler.zero_offset
        # Values are stored as true volts (gain-independent); convert per the
        # selected display unit. W/m² requires the physical factor (falls back
        # to V if unavailable).
        mode = self.unit_var.get()
        factor = self.sampler.physical_factor
        lux_factor = self.sampler.lux_factor
        if mode == "W/m²" and factor:
            values = [v * factor for v in values]
            unit, vfmt, rfmt = "W/m²", ".4f", ".6f"
            sat_threshold = sat_v * factor
            self.ax.set_ylabel(f"Irradiance ({self.sampler.physical_units}, daylight)")
        elif mode == "lux" and lux_factor:
            values = [v * lux_factor for v in values]
            unit, vfmt, rfmt = "lux", ".2f", ".3f"
            sat_threshold = sat_v * lux_factor
            self.ax.set_ylabel("Illuminance (lux, daylight)")
        elif mode == "%":
            values = [v / gain_v * 100 for v in values]
            unit, vfmt, rfmt = "%", ".2f", ".4f"
            sat_threshold = sat_v / gain_v * 100
            self.ax.set_ylabel("Light (%)")
        else:
            # Values already stored as V — use directly.
            unit, vfmt, rfmt = "V", ".4f", ".6f"
            sat_threshold = sat_v
            self.ax.set_ylabel("Light (V)")

        self.line.set_data(times, values)
        self.line.set_visible(self.rawpoints_var.get())

        # Rolling average (trailing window in seconds).
        if self.rollavg_var.get() and len(values) >= 2:
            try:
                win_s = float(self.rollavg_sec_var.get())
            except ValueError:
                win_s = 0.0
            if win_s > 0:
                ra = self._rolling_average(np.array(times), np.array(values), win_s)
                self.rollavg_line.set_data(times, ra)
                self.rollavg_line.set_visible(True)
            else:
                self.rollavg_line.set_visible(False)
        else:
            self.rollavg_line.set_visible(False)

        self.sat_line.set_ydata([sat_threshold, sat_threshold])
        # Sync gain combobox with whatever gain is currently active
        # (may have been changed by autogain).
        current_label = GAIN_LABELS[self.sampler.current_gain]
        if self.gain_var.get() != current_label:
            self.gain_var.set(current_label)

        # Show the current dark level on the button (0 when cleared).
        self._zero_btn.config(text=f"Zero (dark): {self.sampler.zero_offset:.4f} V")

        # Sample-rate / activity indicator.
        rate = self.sampler.sample_rate
        if self.sampler.acquiring and rate > 0:
            rate_str = f"▶ {rate:.0f} sps"
        elif self.sampler.acquiring:
            rate_str = "▶ …"  # acquiring but no samples yet / stalled
        else:
            rate_str = "❙❙ paused"

        sensor_sat = self.sampler.sensor_saturated
        adc_sat = self.sampler.adc_saturated
        status = f"{rate_str}  {self.sampler.status}"
        if sensor_sat:
            status = f"⚠ SENSOR SAT  {status}"
        elif adc_sat:
            status = f"⚠ ADC SAT  {status}"
        if self.sampler.zeroing:
            status = f"zeroing…  {status}"
        if self.sampler.is_recording:
            status = f"● REC {self.sampler.recording_count}  {status}"
        self.status_var.set(status)

        if times:
            # Follow the live window only when requested; user pan/zoom turns
            # this off automatically (via _on_xlim_changed).
            if self.follow_var.get():
                xmax = times[-1]
                xmin = max(0.0, xmax - WINDOW_SECONDS)
                self._set_xlim(xmin, xmax if xmax > xmin else xmin + 1)

            # Stats/overlays operate over whatever x-range is actually visible.
            xmin, xmax = self.ax.get_xlim()
            win = [(t, v) for t, v in zip(times, values) if xmin <= t <= xmax]
            wt = np.array([t for t, _ in win])
            wv = np.array([v for _, v in win])

            # Remove previous noise patch; will be recreated below if needed.
            if self._noise_patch is not None:
                self._noise_patch.remove()
                self._noise_patch = None

            legend_handles = []
            if self.avg_var.get() and wv.size:
                mean = wv.mean()
                self.avg_line.set_data([xmin, xmax], [mean, mean])
                self.avg_line.set_label(f"average = {mean:{vfmt}} {unit}")
                legend_handles.append(self.avg_line)
            else:
                self.avg_line.set_data([], [])

            if self.noise_var.get() and wv.size >= 2:
                mean = wv.mean()
                std = wv.std()
                ptp = np.ptp(wv)
                rel = (std / mean * 100) if mean else 0
                self._noise_patch = self.ax.fill_between(
                    [xmin, xmax],
                    [mean - std, mean - std],
                    [mean + std, mean + std],
                    alpha=0.2,
                    color="tab:red",
                    zorder=0,
                )
                legend_handles.append(
                    Patch(
                        facecolor="tab:red",
                        alpha=0.4,
                        label=f"σ = {std:{rfmt}} {unit}  ({rel:.2f} %)  p-p = {ptp:{rfmt}} {unit}",
                    )
                )

            if self.fit_var.get() and wv.size >= 2 and np.ptp(wt) > 0:
                slope, intercept = np.polyfit(wt, wv, 1)
                self.fit_line.set_data(
                    [xmin, xmax], [slope * xmin + intercept, slope * xmax + intercept]
                )
                self.fit_line.set_label(
                    f"fit: {slope:+{rfmt}} {unit}/s, intercept {intercept:{vfmt}} {unit}"
                )
                legend_handles.append(self.fit_line)
            else:
                self.fit_line.set_data([], [])

            if legend_handles:
                self.ax.legend(handles=legend_handles, loc="upper right", fontsize=8)
            elif self.ax.get_legend() is not None:
                self.ax.get_legend().remove()

            # Auto Y-scale fits the visible data; user Y-zoom turns it off
            # automatically (via _on_ylim_changed).
            if self.autoscale_var.get():
                window = list(wv) if wv.size else values
                lo, hi = min(window), max(window)
                # Pad by 10% of the visible range, but at least 0.1% of the
                # signal magnitude so a stable signal still has breathing room.
                pad = max((hi - lo) * 0.1, abs((hi + lo) / 2) * 0.001, 1e-6)
                self._set_ylim(lo - pad, hi + pad)

        self.canvas.draw_idle()

    def _schedule_redraw(self):
        self._redraw()
        self._redraw_job = self.root.after(REFRESH_MS, self._schedule_redraw)

    def _on_close(self):
        self.root.after_cancel(self._redraw_job)
        self.sampler.shutdown()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Real-time light sensor GUI")
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port of the sensor (e.g. COM5). Auto-detected if omitted.",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL_MS,
        help="Initial scan interval in milliseconds (0 = as fast as possible)",
    )
    args = parser.parse_args()

    try:
        port = args.port or autodetect_port()
    except RuntimeError as exc:
        parser.error(str(exc))
    if not args.port:
        print(f"Auto-detected sensor on {port}")

    sampler = SensorSampler(port, args.baud, args.interval / 1000.0)
    sampler.start()

    root = tk.Tk()
    SensorApp(root, sampler, port)
    try:
        root.mainloop()
    finally:
        sampler.shutdown()


if __name__ == "__main__":
    main()
