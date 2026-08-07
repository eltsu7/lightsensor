import argparse
import csv
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from lightmeter.sensor import (
    DeviceError,
    ErrorEvent,
    LightSensor,
    RawSample,
    SampleStatus,
    StopReason,
    StreamConfig,
    StreamFormat,
    StreamHeader,
    StreamMode,
    StreamStopped,
    VoltageSample,
)

REFRESH_MS = 50
WINDOW_SECONDS = 10
MAX_POINTS = 20_000
RECORDINGS_DIR = Path(__file__).parent.parent / "recordings"
CSV_COLUMNS = [
    "row_type",
    "host_utc_us",
    "device_timestamp_us",
    "sequence",
    "volts",
    "raw_code",
    "gain_index",
    "gain",
    "status",
    "temperature_c",
    "stream_start_device_us",
    "stream_start_utc_us",
    "format",
    "mode",
    "profile_id",
    "profile_name",
    "measured_sps",
    "autogain",
    "window",
    "output_count",
    "registers_hex",
    "dark_source",
    "active_dark_volts",
]


@dataclass(frozen=True)
class SamplerSnapshot:
    status: str
    connected: bool
    acquiring: bool
    info: object | None
    profiles: tuple
    header: StreamHeader | None
    desired_config: StreamConfig
    times: tuple
    values: tuple
    statuses: tuple
    latest: VoltageSample | RawSample | None
    sample_rate: float
    gap_count: int
    recording: bool
    recording_count: int
    recording_path: Path | None
    zero_offset_v: float | None


class SensorSampler:
    """Own the driver and all serial I/O on one daemon thread."""

    def __init__(self, port=None, *, timeout=2.0):
        self.port = port
        self.timeout = timeout
        self._lock = threading.Lock()
        self._commands = queue.Queue()
        self._running = threading.Event()
        self._thread = threading.Thread(target=self._run, name="SensorSampler", daemon=True)
        self._desired_config = StreamConfig()
        self._status = "Starting"
        self._connected = False
        self._acquiring = False
        self._info = None
        self._device_id = None
        self._profiles = ()
        self._header = None
        self._times = deque(maxlen=MAX_POINTS)
        self._values = deque(maxlen=MAX_POINTS)
        self._statuses = deque(maxlen=MAX_POINTS)
        self._latest = None
        self._rate_times = deque()
        self._expected_sequence = None
        self._gap_count = 0
        self._recording = False
        self._recording_count = 0
        self._recording_path = None
        self._last_recording_path = None
        self._record_file = None
        self._record_writer = None
        self._record_last_flush = 0.0
        self._zero_offset_v = None

    def start(self):
        self._running.set()
        self._thread.start()

    def shutdown(self):
        self._running.clear()
        self._commands.put(("shutdown", None))
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=3)

    def apply(self, config):
        with self._lock:
            self._desired_config = config
            self._status = "Applying stream configuration"
        self._commands.put(("apply", config))

    def pause(self):
        self._commands.put(("pause", None))

    def resume(self):
        self._commands.put(("resume", None))

    def clear_plot(self):
        self._commands.put(("clear_plot", None))

    def zero(self, sample_count):
        self._commands.put(("zero", sample_count))

    def clear_zero(self):
        self._commands.put(("clear_zero", None))

    def save_baseline(self):
        self._commands.put(("save_baseline", None))

    def start_recording(self):
        self._commands.put(("record_on", None))

    def stop_recording(self):
        self._commands.put(("record_off", None))

    @property
    def last_recording_path(self):
        with self._lock:
            return self._last_recording_path

    def snapshot(self):
        with self._lock:
            now = time.perf_counter()
            recent = [value for value in self._rate_times if value >= now - 0.5]
            if len(recent) >= 2:
                sample_rate = (len(recent) - 1) / (recent[-1] - recent[0])
            else:
                sample_rate = 0.0
            return SamplerSnapshot(
                self._status,
                self._connected,
                self._acquiring,
                self._info,
                self._profiles,
                self._header,
                self._desired_config,
                tuple(self._times),
                tuple(self._values),
                tuple(self._statuses),
                self._latest,
                sample_rate,
                self._gap_count,
                self._recording,
                self._recording_count,
                self._recording_path,
                self._zero_offset_v,
            )

    def _set_status(self, status, *, acquiring=None):
        with self._lock:
            self._status = status
            if acquiring is not None:
                self._acquiring = acquiring

    def _clear_plot_state(self):
        with self._lock:
            self._times.clear()
            self._values.clear()
            self._statuses.clear()
            self._rate_times.clear()
            self._latest = None
            self._expected_sequence = None
            self._gap_count = 0

    def _stream_started(self, header):
        close_error = self._close_recording_file()
        self._clear_plot_state()
        with self._lock:
            self._header = header
            self._acquiring = True
            self._zero_offset_v = header.active_dark_volts if header.dark_source == 1 else None
            self._status = (
                f"Streaming {header.measured_sps:.3f} SPS — "
                f"{header.config.window}-conversion mean"
            )
            if close_error is not None:
                self._status += f"; recording disabled: {close_error}"
            recording = self._recording
        if recording:
            self._open_recording_file(header)

    def _stream_ended(self, status):
        close_error = self._close_recording_file()
        with self._lock:
            self._header = None
            self._acquiring = False
            self._status = status
            if close_error is not None:
                self._status += f"; recording disabled: {close_error}"
            self._expected_sequence = None

    def _profile_name(self, profile_id):
        for profile in self._profiles:
            if profile.id == profile_id:
                return profile.name
        return str(profile_id)

    def _recording_failed(self, error, *, partial_path=None):
        file = self._record_file
        self._record_file = None
        self._record_writer = None
        if file is not None:
            try:
                file.close()
            except (OSError, ValueError):
                pass
        cleanup_error = None
        if partial_path is not None:
            try:
                partial_path.unlink(missing_ok=True)
            except OSError as exc:
                cleanup_error = exc
        with self._lock:
            self._recording = False
            self._recording_path = None
            self._status = f"Recording failed: {error}; stream continues"
            if cleanup_error is not None:
                self._status += f"; incomplete file could not be removed: {cleanup_error}"

    def _open_recording_file(self, header):
        path = None
        try:
            RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
            started = datetime.now(timezone.utc)
            path = RECORDINGS_DIR / (
                f"stream_{started:%Y-%m-%dT%H-%M-%S.%fZ}_"
                f"{header.stream_start_device_us}.csv"
            )
            self._record_file = open(path, "w", newline="")
            self._record_writer = csv.DictWriter(self._record_file, fieldnames=CSV_COLUMNS)
            self._record_writer.writeheader()
            registers = "".join(f"{value:02X}" for value in header.registers)
            self._record_writer.writerow(
                {
                    "row_type": "stream_start",
                    "host_utc_us": header.stream_start_utc_us,
                    "stream_start_device_us": header.stream_start_device_us,
                    "stream_start_utc_us": header.stream_start_utc_us,
                    "format": header.config.format.name.lower(),
                    "mode": header.config.mode.name.lower(),
                    "profile_id": header.config.profile_id,
                    "profile_name": self._profile_name(header.config.profile_id),
                    "measured_sps": header.measured_sps,
                    "gain_index": header.config.gain_index,
                    "gain": header.config.gain,
                    "autogain": int(header.config.autogain),
                    "window": header.config.window,
                    "output_count": header.config.output_count,
                    "registers_hex": registers,
                    "dark_source": header.dark_source,
                    "active_dark_volts": header.active_dark_volts,
                }
            )
            self._record_file.flush()
        except (OSError, csv.Error, ValueError) as exc:
            self._recording_failed(exc, partial_path=path)
            return False
        self._record_last_flush = time.monotonic()
        with self._lock:
            self._recording_path = path
            self._last_recording_path = path
            self._recording_count = 0
        return True

    def _close_recording_file(self):
        file = self._record_file
        self._record_file = None
        self._record_writer = None
        with self._lock:
            self._recording_path = None
        if file is None:
            return None
        try:
            file.flush()
            file.close()
        except (OSError, ValueError) as exc:
            try:
                file.close()
            except (OSError, ValueError):
                pass
            with self._lock:
                self._recording = False
                self._status = f"Recording close failed: {exc}; recording disabled"
            return exc
        return None

    def _record_sample(self, sample):
        if self._record_writer is None or self._header is None:
            return
        header = self._header
        utc_us = header.stream_start_utc_us + (
            sample.device_timestamp_us - header.stream_start_device_us
        )
        is_volts = isinstance(sample, VoltageSample)
        try:
            self._record_writer.writerow(
                {
                    "row_type": "sample",
                    "host_utc_us": utc_us,
                    "device_timestamp_us": sample.device_timestamp_us,
                    "sequence": sample.sequence,
                    "volts": sample.value if is_volts else "",
                    "raw_code": "" if is_volts else sample.value,
                    "gain_index": sample.gain_index,
                    "gain": sample.gain,
                    "status": int(sample.status),
                    "temperature_c": sample.temperature_c,
                    "stream_start_device_us": header.stream_start_device_us,
                    "stream_start_utc_us": header.stream_start_utc_us,
                    "format": header.config.format.name.lower(),
                    "profile_id": header.config.profile_id,
                    "profile_name": self._profile_name(header.config.profile_id),
                }
            )
            with self._lock:
                self._recording_count += 1
            now = time.monotonic()
            if now - self._record_last_flush >= 1.0:
                self._record_file.flush()
                self._record_last_flush = now
        except (OSError, csv.Error, ValueError) as exc:
            self._recording_failed(exc)

    def _handle_sample(self, sample):
        now = time.perf_counter()
        with self._lock:
            if self._header is None:
                return
            if self._expected_sequence is not None and sample.sequence != self._expected_sequence:
                delta = (sample.sequence - self._expected_sequence) & 0xFFFFFFFF
                self._gap_count += delta
            self._expected_sequence = (sample.sequence + 1) & 0xFFFFFFFF
            elapsed = (
                sample.device_timestamp_us - self._header.stream_start_device_us
            ) / 1_000_000.0
            self._times.append(elapsed)
            self._values.append(float(sample.value))
            self._statuses.append(int(sample.status))
            self._rate_times.append(now)
            cutoff = now - 0.5
            while self._rate_times and self._rate_times[0] < cutoff:
                self._rate_times.popleft()
            self._latest = sample
        self._record_sample(sample)

    def _handle_command(self, sensor, action, value):
        if action == "shutdown":
            return
        if action == "clear_plot":
            self._clear_plot_state()
            return
        if action == "record_on":
            with self._lock:
                if self._recording:
                    return
                self._recording = True
            if self._header is not None:
                if self._open_recording_file(self._header):
                    self._set_status("Recording")
            else:
                self._set_status("Recording armed")
            return
        if action == "record_off":
            with self._lock:
                self._recording = False
            close_error = self._close_recording_file()
            if close_error is None:
                self._set_status("Streaming" if self._header else "Stopped")
            return
        if action == "pause":
            if self._header is not None:
                sensor.stop_stream()
            self._stream_ended("Paused")
            return
        if action in ("apply", "resume"):
            config = value if action == "apply" else self._desired_config
            header = sensor.start_stream(config)
            with self._lock:
                self._desired_config = config
            self._stream_started(header)
            return
        if action == "zero":
            close_error = self._close_recording_file()
            status = f"Measuring dark baseline over {value} samples"
            if close_error is not None:
                status += f"; recording disabled: {close_error}"
            self._set_status(status, acquiring=False)
            baseline = sensor.zero(
                value,
                profile_id=self._desired_config.profile_id,
                gain_index=7,
            )
            with self._lock:
                self._zero_offset_v = baseline
            self._stream_started(sensor.start_stream(self._desired_config))
            return
        if action == "clear_zero":
            header = sensor.clear_zero()
            with self._lock:
                self._zero_offset_v = None
            if header is not None:
                self._stream_started(header)
            else:
                self._set_status("Session zero cleared")
            return
        if action == "save_baseline":
            header = sensor.save_baseline()
            with self._lock:
                self._info = sensor.info
            if header is not None:
                self._stream_started(header)
            else:
                self._set_status("Session baseline saved to flash")
            return
        raise RuntimeError(f"unknown sampler command {action}")

    def _run(self):
        sensor = None
        first_connection = True
        try:
            while self._running.is_set():
                if sensor is None:
                    self._set_status("Connecting", acquiring=False)
                    try:
                        sensor = LightSensor(
                            self.port,
                            timeout=self.timeout,
                            device_id=self._device_id,
                        )
                        info = sensor.connect()
                        self._device_id = info.device_id
                        with self._lock:
                            self._connected = True
                            self._info = info
                            self._profiles = sensor.profiles
                        if first_connection:
                            self._stream_started(sensor.start_stream(self._desired_config))
                            first_connection = False
                        else:
                            self._set_status("Reconnected — press Resume", acquiring=False)
                    except Exception as exc:
                        if sensor is not None:
                            sensor.close()
                        sensor = None
                        with self._lock:
                            self._connected = False
                            self._status = f"Connect failed: {exc}"
                        time.sleep(1.0)
                        continue

                handled_command = False
                while True:
                    try:
                        action, value = self._commands.get_nowait()
                    except queue.Empty:
                        break
                    handled_command = True
                    if action == "shutdown":
                        return
                    try:
                        self._handle_command(sensor, action, value)
                    except DeviceError as exc:
                        self._stream_ended(f"Command failed: {exc}")
                    except ValueError as exc:
                        self._set_status(f"Command failed: {exc}")
                    except Exception as exc:
                        sensor.close()
                        sensor = None
                        self._close_recording_file()
                        with self._lock:
                            self._connected = False
                            self._acquiring = False
                            self._header = None
                            self._status = f"Link lost: {exc}"
                        break

                if sensor is None:
                    continue

                if self._header is None:
                    if not handled_command:
                        time.sleep(0.02)
                    continue
                try:
                    event = sensor.read_event(0.05)
                except TimeoutError:
                    continue
                except Exception as exc:
                    sensor.close()
                    sensor = None
                    self._close_recording_file()
                    with self._lock:
                        self._connected = False
                        self._acquiring = False
                        self._header = None
                        self._status = f"Link lost: {exc}"
                    continue
                if isinstance(event, VoltageSample | RawSample):
                    self._handle_sample(event)
                elif isinstance(event, StreamStopped):
                    reason = {
                        StopReason.REQUESTED: "Stopped",
                        StopReason.FINITE_COMPLETE: "Finite stream complete",
                        StopReason.REPLACED: "Stream replaced",
                    }[event.reason]
                    self._stream_ended(reason)
                elif isinstance(event, ErrorEvent):
                    self._stream_ended(
                        f"Device error: {event.message} (code {event.code}, detail {event.detail})"
                    )
        finally:
            self._close_recording_file()
            if sensor is not None:
                if self._header is not None:
                    try:
                        sensor.stop_stream()
                    except Exception:
                        pass
                sensor.close()
            with self._lock:
                self._connected = False
                self._acquiring = False
                self._header = None


def open_recording_plot(parent, path):
    path = Path(path)
    times = []
    values = []
    statuses = []
    first_timestamp = None
    with open(path, newline="") as file:
        for row in csv.DictReader(file):
            if row["row_type"] != "sample" or not row["volts"]:
                continue
            timestamp = int(row["device_timestamp_us"])
            if first_timestamp is None:
                first_timestamp = timestamp
            times.append((timestamp - first_timestamp) / 1_000_000.0)
            values.append(float(row["volts"]))
            statuses.append(int(row["status"]))
    if not values:
        raise ValueError(f"{path.name} has no voltage samples")

    window = tk.Toplevel(parent)
    window.title(f"Recording — {path.name}")
    figure = Figure(figsize=(9, 5), dpi=100)
    axes = figure.add_subplot(111)
    axes.plot(times, values, lw=0.8, color="tab:orange")
    clipped_times = [time_ for time_, status in zip(times, statuses) if status & 0x07]
    clipped_values = [value for value, status in zip(values, statuses) if status & 0x07]
    if clipped_times:
        axes.plot(clipped_times, clipped_values, ".", color="red", ms=3, label="clipped")
        axes.legend(loc="upper right")
    axes.set_xlabel("Time (s)")
    axes.set_ylabel("Differential signal (V)")
    axes.grid(True, alpha=0.3)
    canvas = FigureCanvasTkAgg(figure, master=window)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    NavigationToolbar2Tk(canvas, window)
    canvas.draw()
    return window


class SensorApp:
    def __init__(self, root, sampler, port):
        self.root = root
        self.sampler = sampler
        self._profile_ids = {
            "normal_20_50_60": 0,
            "normal_330": 1,
            "turbo_2000": 2,
        }
        self._last_profiles = None
        root.title(f"LightSensor v3 — {port or 'auto'}")
        root.geometry("1100x720")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        controls = ttk.Frame(root, padding=8)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="Profile").grid(row=0, column=0, sticky=tk.W)
        self.profile_var = tk.StringVar(value="normal_330")
        self.profile_box = ttk.Combobox(
            controls,
            textvariable=self.profile_var,
            values=tuple(self._profile_ids),
            state="readonly",
            width=20,
        )
        self.profile_box.grid(row=1, column=0, padx=(0, 8))

        ttk.Label(controls, text="Gain").grid(row=0, column=1, sticky=tk.W)
        self.gain_var = tk.StringVar(value="1×")
        self.gain_box = ttk.Combobox(
            controls,
            textvariable=self.gain_var,
            values=tuple(f"{1 << index}×" for index in range(8)),
            state="readonly",
            width=8,
        )
        self.gain_box.grid(row=1, column=1, padx=(0, 8))

        self.autogain_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text="Autogain", variable=self.autogain_var).grid(
            row=1, column=2, padx=(0, 8)
        )

        ttk.Label(controls, text="Average (conversions)").grid(row=0, column=3, sticky=tk.W)
        self.window_var = tk.StringVar(value="1")
        ttk.Spinbox(controls, from_=1, to=1024, textvariable=self.window_var, width=8).grid(
            row=1, column=3, padx=(0, 8)
        )
        ttk.Button(controls, text="Apply", command=self._apply).grid(row=1, column=4, padx=3)
        ttk.Button(controls, text="Pause", command=sampler.pause).grid(row=1, column=5, padx=3)
        ttk.Button(controls, text="Resume", command=sampler.resume).grid(row=1, column=6, padx=3)
        ttk.Button(controls, text="Clear plot", command=sampler.clear_plot).grid(
            row=1, column=7, padx=3
        )

        zero_controls = ttk.Frame(root, padding=(8, 0, 8, 8))
        zero_controls.pack(fill=tk.X)
        ttk.Label(zero_controls, text="Zero samples").pack(side=tk.LEFT)
        self.zero_count_var = tk.StringVar(value="64")
        ttk.Spinbox(
            zero_controls,
            from_=1,
            to=10000,
            textvariable=self.zero_count_var,
            width=8,
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(zero_controls, text="Measure zero", command=self._zero).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(zero_controls, text="Clear zero", command=sampler.clear_zero).pack(
            side=tk.LEFT, padx=3
        )
        ttk.Button(zero_controls, text="Save as device baseline", command=self._save).pack(
            side=tk.LEFT, padx=3
        )
        self.record_button = ttk.Button(
            zero_controls, text="Start recording", command=self._toggle_recording
        )
        self.record_button.pack(side=tk.LEFT, padx=(20, 3))
        ttk.Button(zero_controls, text="Open last CSV", command=self._open_last).pack(
            side=tk.LEFT, padx=3
        )

        status_frame = ttk.Frame(root, padding=(8, 0, 8, 6))
        status_frame.pack(fill=tk.X)
        self.banner_var = tk.StringVar(value="Starting")
        self.banner = tk.Label(
            status_frame,
            textvariable=self.banner_var,
            anchor=tk.W,
            padx=6,
            pady=4,
            bg="#665500",
            fg="white",
        )
        self.banner.pack(fill=tk.X)
        self.metrics_var = tk.StringVar(value="Waiting for sensor")
        ttk.Label(status_frame, textvariable=self.metrics_var).pack(fill=tk.X, pady=(4, 0))
        self.identity_var = tk.StringVar(value="")
        ttk.Label(status_frame, textvariable=self.identity_var).pack(fill=tk.X)

        figure = Figure(figsize=(10, 5), dpi=100)
        self.axes = figure.add_subplot(111)
        self.axes.set_xlabel("Stream time (s)")
        self.axes.set_ylabel("Differential signal (V)")
        self.axes.grid(True, alpha=0.3)
        (self.line,) = self.axes.plot([], [], color="tab:orange", lw=0.9)
        (self.clipped_line,) = self.axes.plot([], [], ".", color="red", ms=3)
        self.canvas = FigureCanvasTkAgg(figure, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        NavigationToolbar2Tk(self.canvas, root)
        self._redraw_job = root.after(REFRESH_MS, self._redraw)

    def _apply(self):
        try:
            profile_id = self._profile_ids[self.profile_var.get()]
            gain_index = int(self.gain_var.get().removesuffix("×")).bit_length() - 1
            window = int(self.window_var.get())
            config = StreamConfig(
                format=StreamFormat.VOLTS,
                mode=StreamMode.CONTINUOUS,
                profile_id=profile_id,
                gain_index=gain_index,
                autogain=self.autogain_var.get(),
                window=window,
                output_count=0,
            )
        except (KeyError, ValueError) as exc:
            messagebox.showerror("Invalid stream configuration", str(exc), parent=self.root)
            return
        self.sampler.apply(config)

    def _zero(self):
        try:
            count = int(self.zero_count_var.get())
            if not 1 <= count <= 10000:
                raise ValueError("zero sample count must be 1..10000")
        except ValueError as exc:
            messagebox.showerror("Invalid zero sample count", str(exc), parent=self.root)
            return
        self.sampler.zero(count)

    def _save(self):
        if messagebox.askyesno(
            "Persist baseline",
            "Save the current session zero as this device's flash baseline?",
            parent=self.root,
        ):
            self.sampler.save_baseline()

    def _toggle_recording(self):
        snapshot = self.sampler.snapshot()
        if snapshot.recording:
            self.sampler.stop_recording()
        else:
            self.sampler.start_recording()

    def _open_last(self):
        path = self.sampler.last_recording_path
        if path is None:
            messagebox.showinfo("Recording", "No recording has been created yet.", parent=self.root)
            return
        try:
            open_recording_plot(self.root, path)
        except (OSError, ValueError, KeyError, csv.Error) as exc:
            messagebox.showerror("Recording", str(exc), parent=self.root)

    def _redraw(self):
        snapshot = self.sampler.snapshot()
        if snapshot.profiles != self._last_profiles and snapshot.profiles:
            self._last_profiles = snapshot.profiles
            self._profile_ids = {profile.name: profile.id for profile in snapshot.profiles}
            self.profile_box.configure(values=tuple(self._profile_ids))

        self.banner_var.set(snapshot.status)
        if not snapshot.connected or "failed" in snapshot.status.lower() or "error" in snapshot.status.lower():
            self.banner.configure(bg="#8B1A1A")
        elif snapshot.acquiring:
            self.banner.configure(bg="#176B2C")
        else:
            self.banner.configure(bg="#665500")

        latest = snapshot.latest
        if latest is None:
            latest_text = "No samples"
        else:
            unit = "V" if isinstance(latest, VoltageSample) else "code"
            flags = SampleStatus(latest.status)
            latest_text = (
                f"Signal {latest.value:.9g} {unit}  |  gain {latest.gain}×  |  "
                f"temperature {latest.temperature_c:.3f} °C  |  status {flags.name or 'NONE'}"
            )
        recording = (
            f"recording {snapshot.recording_count} samples"
            if snapshot.recording
            else "not recording"
        )
        zero = (
            f"session zero {snapshot.zero_offset_v:.9g} V"
            if snapshot.zero_offset_v is not None
            else "device baseline"
        )
        self.metrics_var.set(
            f"{latest_text}  |  delivered {snapshot.sample_rate:.1f}/s  |  "
            f"gaps {snapshot.gap_count}  |  {zero}  |  {recording}"
        )
        if snapshot.info is not None:
            version = ".".join(str(value) for value in snapshot.info.firmware_version)
            self.identity_var.set(
                f"Device {snapshot.info.device_id}  |  firmware {version}  |  "
                f"protocol 3  |  storage {snapshot.info.storage_state.name.lower()}"
            )
        self.record_button.configure(
            text="Stop recording" if snapshot.recording else "Start recording"
        )

        if snapshot.times:
            self.line.set_data(snapshot.times, snapshot.values)
            clip_pairs = [
                (time_, value)
                for time_, value, status in zip(
                    snapshot.times, snapshot.values, snapshot.statuses
                )
                if status & 0x07
            ]
            self.clipped_line.set_data(
                [pair[0] for pair in clip_pairs], [pair[1] for pair in clip_pairs]
            )
            right = snapshot.times[-1]
            self.axes.set_xlim(max(0.0, right - WINDOW_SECONDS), max(WINDOW_SECONDS, right))
            visible_values = [
                value
                for time_, value in zip(snapshot.times, snapshot.values)
                if time_ >= right - WINDOW_SECONDS
            ]
            low = min(visible_values)
            high = max(visible_values)
            margin = max((high - low) * 0.1, 1e-9)
            self.axes.set_ylim(low - margin, high + margin)
        else:
            self.line.set_data([], [])
            self.clipped_line.set_data([], [])
        self.canvas.draw_idle()
        self._redraw_job = self.root.after(REFRESH_MS, self._redraw)

    def _on_close(self):
        self.root.after_cancel(self._redraw_job)
        self.sampler.shutdown()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="LightSensor v3 streaming GUI")
    parser.add_argument("--port", default=None, help="CDC port; auto-detected if omitted")
    parser.add_argument("--timeout", type=float, default=2.0, help="protocol timeout in seconds")
    args = parser.parse_args()

    sampler = SensorSampler(args.port, timeout=args.timeout)
    sampler.start()
    root = tk.Tk()
    SensorApp(root, sampler, args.port)
    try:
        root.mainloop()
    finally:
        sampler.shutdown()


if __name__ == "__main__":
    main()
