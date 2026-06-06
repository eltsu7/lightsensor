import argparse
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import serial
import numpy as np
from matplotlib.patches import Patch
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)
from matplotlib.figure import Figure

from lightsensor import (
    LightSensor, Reading, autodetect_port,
    GAIN_LABELS, GAIN_VOLTAGES, DEFAULT_GAIN, SATURATION_VOLTAGE,
)

# Defaults
DEFAULT_INTERVAL_MS = 0  # 0 = scan as fast as the device allows
WINDOW_SECONDS = 10  # how much history to keep on screen
REFRESH_MS = 30  # GUI redraw interval (~33 fps); decoupled from sampling
MAX_POINTS = 20000  # cap on stored points


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
        self._times = deque(maxlen=MAX_POINTS)
        self._values = deque(maxlen=MAX_POINTS)
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
        """Request a gain change; applied by the sampler thread on its next loop."""
        self._desired_gain = gain_index
        self._skip_next = True

    def clear(self):
        with self._lock:
            self._times.clear()
            self._values.clear()
            self._start = time.perf_counter()

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
                    self.status = "connected"
                if self._desired_gain != self._applied_gain:
                    if sensor.set_gain(self._desired_gain):
                        self._applied_gain = self._desired_gain
                reading = sensor.read()
                if self._skip_next:
                    self._skip_next = False
                    continue
                if reading is None:
                    value = last_value
                else:
                    value = reading.value
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

            now = time.perf_counter() - self._start
            with self._lock:
                self._times.append(now)
                self._values.append(value)

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


class SensorApp:
    """Tkinter window embedding the real-time matplotlib plot plus controls."""

    def __init__(self, root, sampler, port):
        self.root = root
        self.sampler = sampler
        root.title(f"Light Sensor ({port})")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        # --- control bar ---------------------------------------------------
        controls = ttk.Frame(root, padding=8)
        controls.pack(side=tk.TOP, fill=tk.X)

        self.autoscale_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            controls,
            text="Auto Y-scale",
            variable=self.autoscale_var,
        ).pack(side=tk.LEFT, padx=(0, 16))

        self.avg_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Window average",
            variable=self.avg_var,
        ).pack(side=tk.LEFT, padx=(0, 16))

        self.fit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Line fit",
            variable=self.fit_var,
        ).pack(side=tk.LEFT, padx=(0, 16))

        self.noise_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Noise band",
            variable=self.noise_var,
        ).pack(side=tk.LEFT, padx=(0, 16))

        self.absscale_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            controls,
            text="Absolute scale",
            variable=self.absscale_var,
        ).pack(side=tk.LEFT, padx=(0, 16))

        ttk.Label(controls, text="Gain:").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(controls, text="−", width=2, command=self._gain_down).pack(side=tk.LEFT)
        self.gain_var = tk.StringVar(value=GAIN_LABELS[DEFAULT_GAIN])
        gain_combo = ttk.Combobox(
            controls,
            width=8,
            state="readonly",
            values=GAIN_LABELS,
            textvariable=self.gain_var,
        )
        gain_combo.pack(side=tk.LEFT, padx=2)
        gain_combo.bind("<<ComboboxSelected>>", lambda _e: self._apply_gain())
        ttk.Button(controls, text="+", width=2, command=self._gain_up).pack(side=tk.LEFT, padx=(0, 16))

        ttk.Label(controls, text="Scan interval (ms):").pack(side=tk.LEFT)
        self.interval_var = tk.StringVar(value=str(int(sampler.interval_s * 1000)))
        interval_entry = ttk.Entry(controls, width=7, textvariable=self.interval_var)
        interval_entry.pack(side=tk.LEFT, padx=(4, 4))
        interval_entry.bind("<Return>", lambda _e: self._apply_interval())
        ttk.Button(controls, text="Apply", command=self._apply_interval).pack(
            side=tk.LEFT
        )

        self.startstop_btn = ttk.Button(
            controls, text="Stop", width=6, command=self._toggle_run
        )
        self.startstop_btn.pack(side=tk.LEFT, padx=(16, 4))
        ttk.Button(controls, text="Clear", command=self.sampler.clear).pack(
            side=tk.LEFT
        )

        self.status_var = tk.StringVar(value="")
        ttk.Label(controls, textvariable=self.status_var).pack(side=tk.RIGHT)

        # --- plot ----------------------------------------------------------
        self.fig = Figure(figsize=(8, 4.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        (self.line,) = self.ax.plot([], [], lw=1.5, color="tab:orange", zorder=3)
        (self.avg_line,) = self.ax.plot(
            [], [], lw=1.5, ls="--", color="tab:blue", label="average", zorder=1
        )
        (self.fit_line,) = self.ax.plot(
            [], [], lw=1.5, ls="--", color="tab:green", label="fit", zorder=1
        )
        self._noise_patch = None
        self.sat_line = self.ax.axhline(
            y=0, color="red", ls="--", lw=0.8, alpha=0.5, zorder=2
        )
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Light (%)")
        self.ax.set_ylim(0, 100)
        self.ax.grid(True, alpha=0.3)

        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, root)  # pan/zoom/save toolbar

        self._schedule_redraw()

    def _toggle_run(self):
        if self.sampler.acquiring:
            self.sampler.pause()
            self.startstop_btn.config(text="Start")
        else:
            self.sampler.resume()
            self.startstop_btn.config(text="Stop")

    def _apply_gain(self):
        gain_index = GAIN_LABELS.index(self.gain_var.get())
        self.sampler.set_gain(gain_index)
        self.sampler.clear()

    def _gain_up(self):
        idx = GAIN_LABELS.index(self.gain_var.get())
        if idx < len(GAIN_LABELS) - 1:
            self.gain_var.set(GAIN_LABELS[idx + 1])
            self._apply_gain()

    def _gain_down(self):
        idx = GAIN_LABELS.index(self.gain_var.get())
        if idx > 0:
            self.gain_var.set(GAIN_LABELS[idx - 1])
            self._apply_gain()  # scale changed; old samples no longer comparable

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

    def _redraw(self):
        times, values = self.sampler.snapshot()

        if self.absscale_var.get():
            scale = GAIN_VOLTAGES[self.sampler.current_gain]
            values = [v * scale for v in values]
            self.ax.set_ylabel("Light (abs.)")
        else:
            self.ax.set_ylabel("Light (%)")

        self.line.set_data(times, values)

        # Saturation threshold in the current scale.
        gain_v = GAIN_VOLTAGES[self.sampler.current_gain]
        if self.absscale_var.get():
            sat_threshold = SATURATION_VOLTAGE * 100
        else:
            sat_threshold = SATURATION_VOLTAGE / gain_v * 100
        self.sat_line.set_ydata([sat_threshold, sat_threshold])

        sensor_sat = self.sampler.sensor_saturated
        adc_sat = self.sampler.adc_saturated
        status = self.sampler.status
        if sensor_sat:
            self.status_var.set(f"⚠ SENSOR SAT  {status}")
        elif adc_sat:
            self.status_var.set(f"⚠ ADC SAT  {status}")
        else:
            self.status_var.set(status)

        if times:
            xmax = times[-1]
            xmin = max(0.0, xmax - WINDOW_SECONDS)
            self.ax.set_xlim(xmin, xmax if xmax > xmin else xmin + 1)

            # Data within the visible window, used for stats overlays.
            win = [(t, v) for t, v in zip(times, values) if t >= xmin]
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
                self.avg_line.set_label(f"average = {mean:.2f} %")
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
                    alpha=0.2, color="tab:red", zorder=0,
                )
                legend_handles.append(Patch(
                    facecolor="tab:red", alpha=0.4,
                    label=f"σ = {std:.4f} %  ({rel:.2f} %)  p-p = {ptp:.4f} %",
                ))

            if self.fit_var.get() and wv.size >= 2 and np.ptp(wt) > 0:
                slope, intercept = np.polyfit(wt, wv, 1)
                self.fit_line.set_data(
                    [xmin, xmax], [slope * xmin + intercept, slope * xmax + intercept]
                )
                self.fit_line.set_label(
                    f"fit: {slope:+.3f} %/s, intercept {intercept:.2f} %"
                )
                legend_handles.append(self.fit_line)
            else:
                self.fit_line.set_data([], [])

            if legend_handles:
                self.ax.legend(handles=legend_handles, loc="upper right", fontsize=8)
            elif self.ax.get_legend() is not None:
                self.ax.get_legend().remove()

            if self.autoscale_var.get():
                window = list(wv) if wv.size else values
                lo, hi = min(window), max(window)
                pad = max(1.0, (hi - lo) * 0.1)
                self.ax.set_ylim(lo - pad, hi + pad)
            else:
                self.ax.set_ylim(0, 100)

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
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
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
