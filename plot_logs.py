#!/usr/bin/env python3
"""
SubOptimal PID log viewer.
Fetches /logs from the sub over WiFi and plots the run data.
Check serial output after a run for the sub's IP address.

Dependencies: pip install requests matplotlib
"""

import io
import csv
import tkinter as tk
from tkinter import ttk, messagebox

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
        self.fig = Figure(figsize=(13, 10), tight_layout=True)
        canvas = FigureCanvasTkAgg(self.fig, master=root)
        NavigationToolbar2Tk(canvas, root)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas = canvas

        self._draw_empty()

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
            self._plot(data)
            self.status_var.set(f"{n} entries loaded.")
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

    def _plot(self, d: dict):
        t = [ms / 1000.0 for ms in d["timestamp_ms"]]
        self.fig.clear()

        # --- Pitch ---
        ax1 = self.fig.add_subplot(4, 1, 1)
        ax1.plot(t, d["pitch"], label="pitch (actual)", color="steelblue")
        ax1.axhline(d["pitchSetpt"][0], color="tomato", linestyle="--",
                    label=f"setpoint ({d['pitchSetpt'][0]:.1f}°)")
        ax1_r = ax1.twinx()
        ax1_r.plot(t, d["pitchOut"], color="orange", alpha=0.6, linewidth=1, label="PID output")
        ax1_r.set_ylabel("Output", color="orange")
        ax1_r.tick_params(axis="y", labelcolor="orange")
        ax1.set_ylabel("Pitch (°)")
        ax1.set_title("Pitch")
        self._center_zero(ax1,  d["pitch"], [d["pitchSetpt"][0]])
        self._center_zero(ax1_r, d["pitchOut"])
        lines1 = ax1.get_lines() + ax1_r.get_lines()
        ax1.legend(lines1, [l.get_label() for l in lines1], loc="upper right", fontsize=8)
        ax1.grid(True, alpha=0.4)

        # --- Roll ---
        ax2 = self.fig.add_subplot(4, 1, 2)
        ax2.plot(t, d["roll"], label="roll (actual)", color="steelblue")
        ax2.axhline(d["rollSetpt"][0], color="tomato", linestyle="--",
                    label=f"setpoint ({d['rollSetpt'][0]:.1f}°)")
        ax2_r = ax2.twinx()
        ax2_r.plot(t, d["rollOut"], color="orange", alpha=0.6, linewidth=1, label="PID output")
        ax2_r.set_ylabel("Output", color="orange")
        ax2_r.tick_params(axis="y", labelcolor="orange")
        ax2.set_ylabel("Roll (°)")
        ax2.set_title("Roll")
        self._center_zero(ax2,  d["roll"], [d["rollSetpt"][0]])
        self._center_zero(ax2_r, d["rollOut"])
        lines2 = ax2.get_lines() + ax2_r.get_lines()
        ax2.legend(lines2, [l.get_label() for l in lines2], loc="upper right", fontsize=8)
        ax2.grid(True, alpha=0.4)

        # --- Yaw ---
        ax3 = self.fig.add_subplot(4, 1, 3)
        ax3.plot(t, d["yawIn"], label="yaw error (actual)", color="steelblue")
        ax3.axhline(d["yawSetpt"][0], color="tomato", linestyle="--",
                    label=f"setpoint ({d['yawSetpt'][0]:.1f}°)")
        ax3_r = ax3.twinx()
        ax3_r.plot(t, d["yawOut"], color="orange", alpha=0.6, linewidth=1, label="PID output")
        ax3_r.set_ylabel("Output", color="orange")
        ax3_r.tick_params(axis="y", labelcolor="orange")
        ax3.set_ylabel("Yaw (°)")
        ax3.set_title("Yaw")
        self._center_zero(ax3,  d["yawIn"], [d["yawSetpt"][0]])
        self._center_zero(ax3_r, d["yawOut"])
        lines3 = ax3.get_lines() + ax3_r.get_lines()
        ax3.legend(lines3, [l.get_label() for l in lines3], loc="upper right", fontsize=8)
        ax3.grid(True, alpha=0.4)

        # --- Velocity ---
        ax4 = self.fig.add_subplot(4, 1, 4)
        ax4.plot(t, d["velX"], label="velX", color="mediumseagreen")
        ax4.plot(t, d["velY"], label="velY", color="mediumpurple")
        ax4.plot(t, d["velZ"], label="velZ", color="coral")
        ax4.axhline(0, color="gray", linestyle="--", linewidth=0.8)
        self._center_zero(ax4, d["velX"], d["velY"], d["velZ"])
        ax4.set_ylabel("Velocity (m/s)")
        ax4.set_title("Velocity")
        ax4.set_xlabel("Time (s)")
        ax4.legend(loc="upper right", fontsize=8)
        ax4.grid(True, alpha=0.4)

        self.fig.tight_layout()
        self.canvas.draw()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
