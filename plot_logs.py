#!/usr/bin/env python3
"""
SubOptimal PID log viewer.
Fetches /logs from the sub over WiFi and plots the run data.
Check serial output after a run for the sub's IP address.

Dependencies: pip install requests matplotlib
"""

import io
import csv
import os
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime

import requests
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


def fetch_logs(ip: str) -> str:
    resp = requests.get(f"http://{ip}/logs", timeout=5)
    resp.raise_for_status()
    return resp.text


def parse_csv(text: str) -> dict:
    reader = csv.DictReader(io.StringIO(text))
    data = {k: [] for k in reader.fieldnames}
    for row in reader:
        for k, v in row.items():
            data[k].append(float(v))
    return data


STATE_ARMED = 0
STATE_RUNNING = 1
STATE_RECOVERY = 2


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("SubOptimal PID Logs")
        root.bind("<F5>", lambda _: self.refresh())

        # --- Controls bar ---
        bar = ttk.Frame(root, padding=(6, 4))
        bar.pack(fill=tk.X)

        ttk.Label(bar, text="Sub IP:").pack(side=tk.LEFT)
        self.ip_var = tk.StringVar(value="")
        ip_entry = ttk.Entry(bar, textvariable=self.ip_var, width=18)
        ip_entry.pack(side=tk.LEFT, padx=(4, 8))
        ip_entry.focus()

        refresh_btn = ttk.Button(bar, text="Refresh  [F5]", command=self.refresh)
        refresh_btn.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="Enter the IP printed on serial after the sub reconnects.")
        ttk.Label(bar, textvariable=self.status_var, foreground="gray").pack(side=tk.LEFT, padx=10)

        # --- Plot area ---
        self.fig = Figure(figsize=(13, 12), tight_layout=True)
        canvas = FigureCanvasTkAgg(self.fig, master=root)
        NavigationToolbar2Tk(canvas, root)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas = canvas

        self._draw_empty()

    @staticmethod
    def _autosave(text: str, data: dict) -> str:
        """Save the raw CSV to logs/{run_number}-{HHMM}.csv next to this script."""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        logs_dir = os.path.join(script_dir, "logs")
        os.makedirs(logs_dir, exist_ok=True)

        run_number = int(data["run_number"][0]) if "run_number" in data else 0
        timestamp = datetime.now().strftime("%H%M")
        filename = f"{run_number}-{timestamp}.csv"
        path = os.path.join(logs_dir, filename)
        with open(path, "w", newline="") as f:
            f.write(text)
        return os.path.relpath(path, script_dir)

    def _draw_empty(self):
        self.fig.clear()
        self.fig.text(0.5, 0.5, "No data yet — enter IP and press Refresh",
                      ha="center", va="center", color="gray", fontsize=13)
        self.canvas.draw()

    def refresh(self):
        ip = self.ip_var.get().strip()
        if not ip:
            messagebox.showwarning("No IP", "Enter the sub's IP address first.")
            return

        self.status_var.set("Fetching...")
        self.root.update_idletasks()

        try:
            text = fetch_logs(ip)
            data = parse_csv(text)
            n = len(data["timestamp_ms"])
            if n == 0:
                self.status_var.set("Got 0 entries — did the sub run?")
                self._draw_empty()
                return
            saved_path = self._autosave(text, data)
            self._plot(data)
            self.status_var.set(f"{n} entries loaded. Saved → {saved_path}")
        except requests.exceptions.ConnectionError:
            self.status_var.set("Connection failed.")
            messagebox.showerror("Connection failed", f"Could not reach http://{ip}/logs\nIs the sub on WiFi?")
        except Exception as exc:
            self.status_var.set(f"Error: {exc}")
            messagebox.showerror("Error", str(exc))

    @staticmethod
    def _center_zero(ax, *series):
        """Set y-limits so that 0 is exactly centred on the axis."""
        vals = [v for s in series for v in s]
        if not vals:
            return
        half = max(abs(min(vals)), abs(max(vals)))
        ax.set_ylim(-(half or 1.0), (half or 1.0))

    @staticmethod
    def _find_transition_time(states, times):
        """Find the time of the Armed->Running transition."""
        for i in range(1, len(states)):
            if states[i - 1] == STATE_ARMED and states[i] == STATE_RUNNING:
                return times[i]
        return None

    @staticmethod
    def _compute_run_duration(states, times, pitches, pickup_thresh=45.0):
        """Compute duration from first Running sample to last flat-pitch sample.

        Backtracks from the end of the log to find the last sample where pitch
        was below half the pickup threshold — matching the firmware's own logic
        for determining when the sub was still in the water.
        """
        run_start = None
        for i, s in enumerate(states):
            if s in (STATE_RUNNING, STATE_RECOVERY) and run_start is None:
                run_start = times[i]

        if run_start is None:
            return None

        flat_thresh = pickup_thresh / 2.0
        run_end = None
        for i in range(len(times) - 1, -1, -1):
            if abs(pitches[i]) < flat_thresh:
                run_end = times[i]
                break

        if run_end is not None and run_end > run_start:
            return run_end - run_start
        return None

    def _add_transition_line(self, ax, transition_t):
        """Add a vertical line marking Armed->Running transition."""
        if transition_t is not None:
            ax.axvline(transition_t, color="limegreen", linewidth=1.5,
                       linestyle="-", alpha=0.7, zorder=0)

    def _plot(self, d: dict):
        t = [ms / 1000.0 for ms in d["timestamp_ms"]]
        states = [int(s) for s in d["state"]]
        self.fig.clear()

        transition_t = self._find_transition_time(states, t)
        run_duration = self._compute_run_duration(states, t, d["pitch"])

        # Build title with run duration
        title = "SubOptimal Run"
        if run_duration is not None:
            title += f"  |  Run: {run_duration:.2f}s"
        if transition_t is not None:
            title += f"  |  Launch @ {transition_t:.2f}s"
        self.fig.suptitle(title, fontsize=11, fontweight="bold")

        # --- Pitch ---
        ax1 = self.fig.add_subplot(5, 1, 1)
        ax1.plot(t, d["pitch"], label="pitch (actual)", color="steelblue")
        ax1.plot(t, d["pitchSetpt"], color="tomato", linestyle="--", label="setpoint")
        ax1_r = ax1.twinx()
        ax1_r.plot(t, d["pitchOut"], color="orange", alpha=0.6, linewidth=1, label="PID output")
        ax1_r.set_ylabel("Output", color="orange")
        ax1_r.tick_params(axis="y", labelcolor="orange")
        ax1.set_ylabel("Pitch (\u00b0)")
        ax1.set_title("Pitch")
        self._center_zero(ax1,  d["pitch"], d["pitchSetpt"])
        self._center_zero(ax1_r, d["pitchOut"])
        lines1 = ax1.get_lines() + ax1_r.get_lines()
        ax1.legend(lines1, [l.get_label() for l in lines1], loc="upper right", fontsize=8)
        ax1.grid(True, alpha=0.4)
        self._add_transition_line(ax1, transition_t)

        # --- Roll ---
        ax2 = self.fig.add_subplot(5, 1, 2)
        ax2.plot(t, d["roll"], label="roll (actual)", color="steelblue")
        ax2.axhline(d["rollSetpt"][0], color="tomato", linestyle="--",
                    label=f"setpoint ({d['rollSetpt'][0]:.1f}\u00b0)")
        ax2_r = ax2.twinx()
        ax2_r.plot(t, d["rollOut"], color="orange", alpha=0.6, linewidth=1, label="PID output")
        ax2_r.set_ylabel("Output", color="orange")
        ax2_r.tick_params(axis="y", labelcolor="orange")
        ax2.set_ylabel("Roll (\u00b0)")
        ax2.set_title("Roll")
        self._center_zero(ax2,  d["roll"], [d["rollSetpt"][0]])
        self._center_zero(ax2_r, d["rollOut"])
        lines2 = ax2.get_lines() + ax2_r.get_lines()
        ax2.legend(lines2, [l.get_label() for l in lines2], loc="upper right", fontsize=8)
        ax2.grid(True, alpha=0.4)
        self._add_transition_line(ax2, transition_t)

        # --- Yaw ---
        heading_setpt = [((y + yi) % 360) for y, yi in zip(d["yaw"], d["yawIn"])]
        ax3 = self.fig.add_subplot(5, 1, 3)
        ax3.plot(t, d["yaw"], label="yaw (actual)", color="steelblue")
        ax3.plot(t, heading_setpt, color="tomato", linestyle="--", label="heading setpoint")
        ax3_r = ax3.twinx()
        ax3_r.plot(t, d["yawOut"], color="orange", alpha=0.6, linewidth=1, label="PID output")
        ax3_r.set_ylabel("Output", color="orange")
        ax3_r.tick_params(axis="y", labelcolor="orange")
        ax3.set_ylabel("Yaw (\u00b0)")
        ax3.set_title("Yaw")
        ax3_r.set_ylim(-0.5, 0.5)
        lines3 = ax3.get_lines() + ax3_r.get_lines()
        ax3.legend(lines3, [l.get_label() for l in lines3], loc="upper right", fontsize=8)
        ax3.grid(True, alpha=0.4)
        self._add_transition_line(ax3, transition_t)

        # --- Motor outputs ---
        ax4 = self.fig.add_subplot(5, 1, 4)
        ax4.plot(t, d["motorTL"], label="TL", color="steelblue")
        ax4.plot(t, d["motorTR"], label="TR", color="tomato")
        ax4.plot(t, d["motorBL"], label="BL", color="mediumseagreen")
        ax4.plot(t, d["motorBR"], label="BR", color="mediumpurple")
        ax4.set_ylabel("Throttle (0\u20131)")
        ax4.set_title("Motor Outputs")
        ax4.set_ylim(0, 1)
        ax4.legend(loc="upper right", fontsize=8)
        ax4.grid(True, alpha=0.4)
        self._add_transition_line(ax4, transition_t)

        # --- Battery voltage ---
        ax5 = self.fig.add_subplot(5, 1, 5)
        ax5.plot(t, d["voltage"], label="battery", color="goldenrod", linewidth=1.5)
        ax5.set_ylabel("Voltage (V)")
        ax5.set_title("Battery Voltage")
        ax5.set_xlabel("Time (s)")
        ax5.legend(loc="upper right", fontsize=8)
        ax5.grid(True, alpha=0.4)
        self._add_transition_line(ax5, transition_t)

        self.fig.tight_layout(rect=[0, 0, 1, 0.96])
        self.canvas.draw()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
