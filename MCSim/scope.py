#!/usr/bin/env python3
"""
scope.py — Real-time oscilloscope for PMSM FOC simulation
Usage:  ./pmsm_foc | python3 scope.py
        python3 scope.py --file sim_log.csv
"""

import sys
import argparse
import collections
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import numpy as np

# ── Fixed subplot layout ─────────────────────────────────────────────────────
SUBPLOTS = [
    {
        "title":  "Speed [RPM]",
        "ylabel": "RPM",
        "vars":   ["omega_m_rpm", "omega_ref_rpm"],
        "labels": ["ω_m (meas)", "ω_ref"],
        "colors": ["#2196F3", "#FF5722"],
        "styles": ["-", "--"],
    },
    {
        "title":  "dq Currents [A]",
        "ylabel": "A",
        "vars":   ["id", "iq", "id_ref", "iq_ref"],
        "labels": ["Id", "Iq", "Id*", "Iq*"],
        "colors": ["#2196F3", "#FF5722", "#90CAF9", "#FFAB91"],
        "styles": ["-", "-", "--", "--"],
    },
    {
        "title":  "dq Voltages [V]",
        "ylabel": "V",
        "vars":   ["vd", "vq"],
        "labels": ["Vd", "Vq"],
        "colors": ["#4CAF50", "#9C27B0"],
        "styles": ["-", "-"],
    },
    {
        "title":  "Torque [N·m]",
        "ylabel": "N·m",
        "vars":   ["Te", "TL"],
        "labels": ["Te (elec)", "TL (load)"],
        "colors": ["#FF9800", "#E91E63"],
        "styles": ["-", "--"],
    },
    {
        "title":  "Electrical Angle [rad]",
        "ylabel": "rad",
        "vars":   ["theta_e"],
        "labels": ["θe"],
        "colors": ["#00BCD4"],
        "styles": ["-"],
    },
]

WINDOW_S = 3.0   # visible time window [s]
MAX_PTS  = 15000  # max points per channel


def parse_args():
    p = argparse.ArgumentParser(description="PMSM FOC real-time scope")
    p.add_argument("--file", default=None, help="Read from CSV file instead of stdin")
    p.add_argument("--window", type=float, default=WINDOW_S, help=f"Time window [s] (default {WINDOW_S})")
    return p.parse_args()


def run_scope(source, window_s):
    all_vars = list(dict.fromkeys(v for sp in SUBPLOTS for v in sp["vars"]))

    buf_t = collections.deque(maxlen=MAX_PTS)
    bufs  = {v: collections.deque(maxlen=MAX_PTS) for v in all_vars}
    header = None

    # ── Layout ───────────────────────────────────────────────────────────────
    plt.style.use("dark_background")
    n = len(SUBPLOTS)
    fig = plt.figure(figsize=(13, 2.4 * n), facecolor="#0d1117")
    fig.suptitle("PMSM FOC — Real-time Scope", color="#e6edf3", fontsize=12, y=0.99)
    gs  = gridspec.GridSpec(n, 1, hspace=0.55,
                            left=0.07, right=0.97, top=0.96, bottom=0.04)
    axes = [fig.add_subplot(gs[i]) for i in range(n)]

    line_map = {}  # var → (ax, Line2D)

    for ax, sp in zip(axes, SUBPLOTS):
        ax.set_facecolor("#161b22")
        ax.tick_params(colors="#8b949e", labelsize=8)
        ax.set_ylabel(sp["ylabel"], color="#8b949e", fontsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#30363d")
        ax.set_title(sp["title"], color="#e6edf3", fontsize=9, pad=3)
        ax.set_xlim(0, window_s)
        ax.grid(True, color="#21262d", linewidth=0.6)

        for v, label, color, style in zip(sp["vars"], sp["labels"], sp["colors"], sp["styles"]):
            (line,) = ax.plot([], [], lw=1.3, color=color, label=label, linestyle=style)
            line_map[v] = (ax, line)

        ax.legend(loc="upper right", fontsize=7.5,
                  facecolor="#161b22", edgecolor="#30363d",
                  labelcolor="#c9d1d9", ncol=len(sp["vars"]))

    axes[-1].set_xlabel("t [s]", color="#8b949e", fontsize=8)

    # ── Animation ────────────────────────────────────────────────────────────
    def read_chunk():
        lines = []
        try:
            for line in source:
                lines.append(line.strip())
                if len(lines) >= 300:
                    break
        except Exception:
            pass
        return lines

    def update(_frame):
        nonlocal header
        for raw in read_chunk():
            if not raw:
                continue
            if header is None:
                header = raw.split(",")
                continue
            parts = raw.split(",")
            if len(parts) != len(header):
                continue
            row = dict(zip(header, parts))
            try:
                t_val = float(row["t"])
            except (KeyError, ValueError):
                continue
            buf_t.append(t_val)
            for v in all_vars:
                try:
                    bufs[v].append(float(row.get(v, 0.0)))
                except ValueError:
                    bufs[v].append(0.0)

        if not buf_t:
            return []

        t_arr = np.array(buf_t)
        t_now = t_arr[-1]
        t_lo  = max(0.0, t_now - window_s)
        mask  = t_arr >= t_lo
        t_win = t_arr[mask]

        changed = []
        for ax, sp in zip(axes, SUBPLOTS):
            y_all = []
            for v in sp["vars"]:
                _, line = line_map[v]
                y_arr = np.array(bufs[v])[mask]
                line.set_data(t_win, y_arr)
                if len(y_arr):
                    y_all.extend([y_arr.min(), y_arr.max()])
                changed.append(line)
            ax.set_xlim(t_lo, t_lo + window_s)
            if y_all:
                mn, mx = min(y_all), max(y_all)
                pad = max(abs(mx - mn) * 0.12, 0.05)
                ax.set_ylim(mn - pad, mx + pad)

        return changed

    ani = FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass


def main():
    args = parse_args()
    if args.file:
        with open(args.file, "r") as f:
            source = iter(f.read().splitlines())
    else:
        source = iter(sys.stdin.readline, "")
    run_scope(source, args.window)


if __name__ == "__main__":
    main()