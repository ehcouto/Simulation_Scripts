"Author: Eduardo H. Couto"
"Version: 1.2"
"Date: January/2023"


"""
pmsm_sim.py — PMSM FOC Simulation with real-time GUI
Optimized: simulation runs in batches, UI stays responsive.
Features: ramped speed reference and load torque transitions.

Run:  python3 pmsm_sim.py
"""

import threading
import collections
import time
import math
import tkinter as tk
from tkinter import ttk

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
matplotlib.rcParams["toolbar"] = "None"
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

# ═══════════════════════════════════════════════════════════════════════════════
#  MOTOR CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════
RS     = 25.4
LD     = 79.0e-3
LQ     = 79.0e-3
LAMBDA = 0.0753
P      = 3
J      = 0.0001
B_FRIC = 0.0002
VDC    = 310.0
VMAX   = VDC / 2.0
IQ_MAX = 0.85
MAX_SPEED = 3600.0
MAX_LOAD  = 0.15

T_CTRL      = 1.0/8000.0
TS          = T_CTRL/8.0
BATCH_S     = 0.02
MAX_PTS     = 60000
CTRL_EVERY  = int(round(T_CTRL / TS))

# ═══════════════════════════════════════════════════════════════════════════════
#  INITIAL PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════
INIT_SPEED = 3000.0
INIT_LOAD  = 0.0
INIT_RAMP_RPMS = 2000.0
INIT_LOAD_NMS = 0.1

#SPEED CONTROLLER
INIT_KP = 0.0006
INIT_KI = 0.002

#CURRENT CONTROLLERS (iq & id)
INIT_KP_DQ = 180.0 
INIT_KI_DQ = 10000.0

# ═══════════════════════════════════════════════════════════════════════════════
#  SHARED STATE
# ═══════════════════════════════════════════════════════════════════════════════
class SimParams:
    def __init__(self):
        self.lock              = threading.Lock()
        # Targets set by UI
        self.speed_target_rpm  = INIT_SPEED        # UI slider value
        self.load_target       = INIT_LOAD         # UI slider value
        # Ramp rates
        self.speed_ramp_rpm_s  = INIT_RAMP_RPMS    # [RPM/s]
        self.load_ramp_nm_s    = INIT_LOAD_NMS     # [N·m/s]
        # PI gains
        self.kp_spd            = INIT_KP
        self.ki_spd            = INIT_KI
        self.iq_max            = IQ_MAX
        self.kp_cur            = INIT_KP_DQ
        self.ki_cur            = INIT_KI_DQ
        # Control flags
        self.running           = False
        self.reset             = False

params = SimParams()

buf_lock      = threading.Lock()
buf_t         = collections.deque(maxlen=MAX_PTS)
buf_omega_rpm = collections.deque(maxlen=MAX_PTS)
buf_omega_ref = collections.deque(maxlen=MAX_PTS)
buf_id        = collections.deque(maxlen=MAX_PTS)
buf_iq        = collections.deque(maxlen=MAX_PTS)
buf_id_ref    = collections.deque(maxlen=MAX_PTS)
buf_iq_ref    = collections.deque(maxlen=MAX_PTS)
buf_vd        = collections.deque(maxlen=MAX_PTS)
buf_vq        = collections.deque(maxlen=MAX_PTS)
buf_Te        = collections.deque(maxlen=MAX_PTS)
buf_TL        = collections.deque(maxlen=MAX_PTS)
buf_theta     = collections.deque(maxlen=MAX_PTS)

BUF_MAP = {
    "omega_rpm": buf_omega_rpm, "omega_ref": buf_omega_ref,
    "id": buf_id,   "iq": buf_iq,
    "id_ref": buf_id_ref, "iq_ref": buf_iq_ref,
    "vd": buf_vd,   "vq": buf_vq,
    "Te": buf_Te,   "TL": buf_TL,
    "theta": buf_theta,
}

# ═══════════════════════════════════════════════════════════════════════════════
#  SIMULATION THREAD
# ═══════════════════════════════════════════════════════════════════════════════
def sim_thread():
    id_ = iq_ = 0.0
    theta_e = omega_e = omega_m = 0.0
    integ_spd = integ_id = integ_iq = 0.0
    va = vb = vd_cmd = vq_cmd = 0.0
    iq_ref = id_ref = 0.0
    t = 0.0
    ctrl_counter = 0

    # Internal ramped references (actual values fed to controller)
    speed_ref_rpm = 0.0
    load_ref      = 0.0

    SQ3          = math.sqrt(3.0)
    BATCH_STEPS  = int(round(BATCH_S / TS))
    DECIMATE     = max(1, BATCH_STEPS // 50)

    def clamp(v, lo, hi):
        return lo if v < lo else (hi if v > hi else v)

    def pi_step(integ, kp, ki, err, dt, out_max):
        integ = clamp(integ + ki * err * dt, -out_max, out_max)
        return integ, clamp(kp * err + integ, -out_max, out_max)

    def ramp_toward(current, target, rate, dt):
        """Move current toward target at max rate [units/s]."""
        step = rate * dt
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + math.copysign(step, diff)

    while True:
        while not params.running:
            time.sleep(0.05)

        # Handle reset
        with params.lock:
            do_reset = params.reset
            if do_reset:
                params.reset = False
        if do_reset:
            id_ = iq_ = theta_e = omega_e = omega_m = 0.0
            integ_spd = integ_id = integ_iq = 0.0
            va = vb = vd_cmd = vq_cmd = 0.0
            t = 0.0
            ctrl_counter = 0
            speed_ref_rpm = 0.0
            load_ref      = 0.0
            with buf_lock:
                for b in BUF_MAP.values(): b.clear()
                buf_t.clear()

        # Snapshot params for this batch
        with params.lock:
            if not params.running:
                continue
            speed_target  = params.speed_target_rpm
            load_target   = params.load_target
            speed_ramp    = params.speed_ramp_rpm_s
            load_ramp     = params.load_ramp_nm_s
            kp_s          = params.kp_spd
            ki_s          = params.ki_spd
            iq_max        = params.iq_max
            kp_c          = params.kp_cur
            ki_c          = params.ki_cur

        out_t     = []
        out_rpm   = []; out_rpmref = []
        out_id    = []; out_iq     = []
        out_idref = []; out_iqref  = []
        out_vd    = []; out_vq     = []
        out_Te    = []; out_TL     = []
        out_th    = []

        for step in range(BATCH_STEPS):

            # ── Ramp references ──────────────────────────────────────────
            speed_ref_rpm = ramp_toward(speed_ref_rpm, speed_target,
                                        speed_ramp, TS)
            load_ref      = ramp_toward(load_ref, load_target,
                                        load_ramp, TS)

            w_ref_e = speed_ref_rpm * (2.0 * math.pi / 60.0) * P

            # ── FOC control (every CTRL_EVERY steps) ────────────────────
            if ctrl_counter == 0:
                integ_spd, iq_ref = pi_step(integ_spd, kp_s, ki_s,
                                             w_ref_e - omega_e, T_CTRL, iq_max)
                id_ref = 0.0
                vd_ff  = -omega_e * LQ * iq_
                vq_ff  =  omega_e * (LD * id_ + LAMBDA)
                integ_id, vd_pi = pi_step(integ_id, kp_c, ki_c,
                                           id_ref - id_, T_CTRL, VMAX)
                integ_iq, vq_pi = pi_step(integ_iq, kp_c, ki_c,
                                           iq_ref - iq_, T_CTRL, VMAX)
                vd_cmd = clamp(vd_pi + vd_ff, -VMAX, VMAX)
                vq_cmd = clamp(vq_pi + vq_ff, -VMAX, VMAX)
                c = math.cos(theta_e); s = math.sin(theta_e)
                valpha = c * vd_cmd - s * vq_cmd
                vbeta  = s * vd_cmd + c * vq_cmd
                va =  valpha
                vb = (-valpha + SQ3 * vbeta) / 2.0
                va = clamp(va, -VMAX, VMAX)
                vb = clamp(vb, -VMAX, VMAX)
            ctrl_counter = (ctrl_counter + 1) % CTRL_EVERY

            # ── Motor Euler step ─────────────────────────────────────────
            c = math.cos(theta_e); s = math.sin(theta_e)
            vd_in =  c * va + s * (va + 2.0 * vb) / SQ3
            vq_in = -s * va + c * (va + 2.0 * vb) / SQ3

            id_ += (vd_in - RS * id_ + omega_e * LQ * iq_) / LD * TS
            iq_ += (vq_in - RS * iq_ - omega_e * (LD * id_ + LAMBDA)) / LQ * TS

            Te       = 1.5 * P * (LAMBDA + (LD - LQ) * id_) * iq_
            omega_m += (Te - load_ref - B_FRIC * omega_m) / J * TS
            omega_e  = omega_m * P
            theta_e += omega_e * TS
            if   theta_e >  2.0 * math.pi: theta_e -= 2.0 * math.pi
            elif theta_e <  0.0:           theta_e += 2.0 * math.pi
            t += TS

            # ── Decimate output ──────────────────────────────────────────
            if step % DECIMATE == 0:
                out_t.append(t)
                out_rpm.append(omega_m * 60.0 / (2.0 * math.pi))
                out_rpmref.append(speed_ref_rpm)
                out_id.append(id_);     out_iq.append(iq_)
                out_idref.append(id_ref); out_iqref.append(iq_ref)
                out_vd.append(vd_cmd);  out_vq.append(vq_cmd)
                out_Te.append(Te);      out_TL.append(load_ref)
                out_th.append(theta_e)

        with buf_lock:
            buf_t.extend(out_t)
            buf_omega_rpm.extend(out_rpm);  buf_omega_ref.extend(out_rpmref)
            buf_id.extend(out_id);          buf_iq.extend(out_iq)
            buf_id_ref.extend(out_idref);   buf_iq_ref.extend(out_iqref)
            buf_vd.extend(out_vd);          buf_vq.extend(out_vq)
            buf_Te.extend(out_Te);          buf_TL.extend(out_TL)
            buf_theta.extend(out_th)

        time.sleep(0.005)

# ═══════════════════════════════════════════════════════════════════════════════
#  GUI
# ═══════════════════════════════════════════════════════════════════════════════
DARK_BG  = "#0d1117"
PANEL_BG = "#161b22"
TEXT_FG  = "#e6edf3"
MUTED_FG = "#8b949e"
ACCENT   = "#2196F3"
BTN_GO   = "#2ea043"
BTN_STP  = "#da3633"
BTN_RST  = "#6e7681"
WINDOW_S = 10.0

SUBPLOT_DEFS = [
    dict(title="Speed [RPM]",             ylabel="RPM",
         bufs=["omega_rpm","omega_ref"],
         labels=["ω_m","ω_ref"],
         colors=["#2196F3","#FF5722"],    styles=["-","--"]),
    dict(title="dq Currents [A]",          ylabel="A",
         bufs=["id","iq","id_ref","iq_ref"],
         labels=["Id","Iq","Id*","Iq*"],
         colors=["#2196F3","#FF5722","#90CAF9","#FFAB91"], styles=["-","-","--","--"]),
    dict(title="dq Voltages [V]",          ylabel="V",
         bufs=["vd","vq"],
         labels=["Vd","Vq"],
         colors=["#4CAF50","#9C27B0"],    styles=["-","-"]),
    dict(title="Torque [N·m]",             ylabel="N·m",
         bufs=["Te","TL"],
         labels=["Te","TL (ramped)"],
         colors=["#FF9800","#E91E63"],    styles=["-","--"]),
    dict(title="Electrical Angle [rad]",   ylabel="rad",
         bufs=["theta"],
         labels=["θe"],
         colors=["#00BCD4"],              styles=["-"]),
]


class App:
    def __init__(self, root):
        self.root       = root
        self.status_lbl = None
        root.title("PMSM FOC Simulator - Eduardo H. Couto (eduardo.h.couto@beko.com) - Beko Europe 2026")
        root.configure(bg=DARK_BG)
        root.geometry("1400x900")
        root.state("zoomed")   
        self._build_layout()
        self._build_scope()
        self._build_panel()
        self._build_metrics()
        threading.Thread(target=sim_thread, daemon=True).start()
        self._start_animation()

    def _build_layout(self):
        self.frame_scope = tk.Frame(self.root, bg=DARK_BG)
        self.frame_scope.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        # Right column: controls on top, metrics at bottom
        self.frame_right = tk.Frame(self.root, bg=PANEL_BG, width=300)
        self.frame_right.pack(side=tk.RIGHT, fill=tk.Y)
        self.frame_right.pack_propagate(False)
        # Metrics frame (bottom, fixed height)
        self.frame_metrics = tk.Frame(self.frame_right, bg="#0d1f2d", width=300, height=260)
        self.frame_metrics.pack(side=tk.BOTTOM, fill=tk.X)
        self.frame_metrics.pack_propagate(False)
        # Controls frame (fills remaining space)
        self.frame_panel = tk.Frame(self.frame_right, bg=PANEL_BG, width=300)
        self.frame_panel.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.frame_panel.pack_propagate(False)

    def _build_scope(self):
        plt.style.use("dark_background")
        n = len(SUBPLOT_DEFS)
        self.fig = plt.Figure(figsize=(10, 9), facecolor=DARK_BG)
        self.fig.suptitle("PMSM FOC — Real-time Scope", color=TEXT_FG, fontsize=11)
        gs = gridspec.GridSpec(n, 1, hspace=0.55,
                               left=0.07, right=0.97, top=0.95, bottom=0.04,
                               figure=self.fig)
        self.axes  = []
        self.lines = {}
        for i, sp in enumerate(SUBPLOT_DEFS):
            ax = self.fig.add_subplot(gs[i])
            ax.set_facecolor("#161b22")
            ax.tick_params(colors=MUTED_FG, labelsize=7)
            ax.set_ylabel(sp["ylabel"], color=MUTED_FG, fontsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor("#30363d")
            ax.set_title(sp["title"], color=TEXT_FG, fontsize=8.5, pad=2)
            ax.set_xlim(0, WINDOW_S)
            ax.grid(True, color="#21262d", linewidth=0.5)
            for bname, label, color, style in zip(sp["bufs"], sp["labels"],
                                                   sp["colors"], sp["styles"]):
                (ln,) = ax.plot([], [], lw=1.2, color=color,
                                label=label, linestyle=style)
                self.lines[bname] = (ax, ln)
            ax.legend(loc="upper right", fontsize=7, facecolor="#161b22",
                      edgecolor="#30363d", labelcolor="#c9d1d9",
                      ncol=len(sp["bufs"]))
            self.axes.append(ax)
        self.axes[-1].set_xlabel("t [s]", color=MUTED_FG, fontsize=8)
        canvas = FigureCanvasTkAgg(self.fig, master=self.frame_scope)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas = canvas

    def _build_panel(self):
        p = self.frame_panel

        def section(txt):
            tk.Label(p, text=txt, bg=PANEL_BG, fg=ACCENT,
                     font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=12, pady=(8,2))
            ttk.Separator(p, orient="horizontal").pack(fill="x", padx=12)

        def spinbox(label, from_, to, init, step, cmd, fmt="{:.1f}"):
            frm = tk.Frame(p, bg=PANEL_BG)
            frm.pack(fill="x", padx=12, pady=3)
            tk.Label(frm, text=label, bg=PANEL_BG, fg=TEXT_FG,
                     font=("Segoe UI", 8), width=19, anchor="w").pack(side=tk.LEFT)

            var = tk.StringVar(value=fmt.format(init))

            def _apply(*_):
                try:
                    val = float(var.get().strip())
                    val = max(from_, min(to, val))
                    var.set(fmt.format(val))
                    cmd(val)
                except ValueError:
                    var.set(fmt.format(init))

            def _step(direction):
                try:
                    cur = float(var.get())
                except ValueError:
                    cur = init
                nv = max(from_, min(to, cur + direction * step))
                var.set(fmt.format(nv))
                cmd(nv)

            btn_down = tk.Button(frm, text="▼", width=2, relief="flat",
                                 bg="#21262d", fg=ACCENT,
                                 font=("Segoe UI", 7), cursor="hand2",
                                 command=lambda: _step(-1))
            btn_down.pack(side=tk.RIGHT, padx=(0, 2))

            btn_up = tk.Button(frm, text="▲", width=2, relief="flat",
                               bg="#21262d", fg=ACCENT,
                               font=("Segoe UI", 7), cursor="hand2",
                               command=lambda: _step(+1))
            btn_up.pack(side=tk.RIGHT, padx=(0, 1))

            sb = ttk.Spinbox(frm, from_=from_, to=to, increment=step,
                             textvariable=var, width=10,
                             font=("Courier New", 8))
            sb.pack(side=tk.RIGHT, padx=(4, 4))

            sb.bind("<Return>",   _apply)
            sb.bind("<FocusOut>", _apply)
            sb.bind("<KP_Enter>", _apply)
            return sb

        tk.Label(p, text="PMSM FOC Parameters", bg=PANEL_BG, fg=TEXT_FG,
                 font=("Segoe UI", 10, "bold")).pack(pady=(14,4))
        ttk.Separator(p, orient="horizontal").pack(fill="x", padx=8)

        # ── Setpoints ──────────────────────────────────────────────────────
        section("  Setpoints")
        spinbox("Speed target [RPM]",  0, MAX_SPEED, INIT_SPEED, 100,
               lambda v: self._set("speed_target_rpm", v), "{:.0f}")
        spinbox("Speed ramp [RPM/s]",  100, 2000, INIT_RAMP_RPMS, 100,
               lambda v: self._set("speed_ramp_rpm_s", v), "{:.0f}")
        spinbox("Load target [N·m]",   0, MAX_LOAD, INIT_LOAD, 0.01,
               lambda v: self._set("load_target", v), "{:.2f}")
        spinbox("Load ramp [N·m/s]",   0.1, MAX_LOAD, INIT_LOAD_NMS, 0.01,
               lambda v: self._set("load_ramp_nm_s", v), "{:.1f}")

        # ── Speed PI ───────────────────────────────────────────────────────
        section("  Speed Controller (PI)")
        spinbox("Kp speed",  INIT_KP/20.0, INIT_KP*20,   INIT_KP,  INIT_KP/20.0,
               lambda v: self._set("kp_spd", v), "{:.5f}")
        spinbox("Ki speed",  INIT_KI/20.0,  INIT_KI*20.0, INIT_KI, INIT_KI/20.0,
               lambda v: self._set("ki_spd", v), "{:.5f}")
        spinbox("Iq max [A]",0.01,  IQ_MAX,  IQ_MAX, 0.01,
               lambda v: self._set("iq_max", v), "{:.2f}")

        # ── Current PI ─────────────────────────────────────────────────────
        section("  Current Controllers (PI)")
        spinbox("Kp current",INIT_KP_DQ/20.0,  INIT_KP_DQ*20.0,  INIT_KP_DQ, INIT_KP_DQ/20.0,
               lambda v: self._set("kp_cur", v), "{:.1f}")
        spinbox("Ki current",INIT_KP_DQ/20.0,  INIT_KP_DQ*20.0,  INIT_KI_DQ, INIT_KI_DQ/20.0,
               lambda v: self._set("ki_cur", v), "{:.0f}")

        # ── Buttons ────────────────────────────────────────────────────────
        tk.Label(p, text="", bg=PANEL_BG).pack()
        bf = tk.Frame(p, bg=PANEL_BG)
        bf.pack(fill="x", padx=12, pady=4)

        self.btn_start = tk.Button(bf, text="▶  START", bg=BTN_GO, fg="white",
                                   font=("Segoe UI", 9, "bold"), relief="flat",
                                   cursor="hand2", command=self._on_start)
        self.btn_start.pack(fill="x", pady=2)

        self.btn_stop = tk.Button(bf, text="■  STOP", bg=BTN_STP, fg="white",
                                  font=("Segoe UI", 9, "bold"), relief="flat",
                                  cursor="hand2", state="disabled",
                                  command=self._on_stop)
        self.btn_stop.pack(fill="x", pady=2)

        tk.Button(bf, text="↺  RESET", bg=BTN_RST, fg="white",
                  font=("Segoe UI", 9, "bold"), relief="flat",
                  cursor="hand2", command=self._on_reset).pack(fill="x", pady=2)

        tk.Label(p, text="", bg=PANEL_BG).pack()
        self.status_var = tk.StringVar(value="● Stopped")
        self.status_lbl = tk.Label(p, textvariable=self.status_var,
                                   bg=PANEL_BG, fg=BTN_STP,
                                   font=("Segoe UI", 9))
        self.status_lbl.pack()

    def _build_metrics(self):
        m = self.frame_metrics
        MBKG  = "#0d1f2d"
        MHEAD = "#58a6ff"
        MVAL  = "#e6edf3"
        MLBL  = "#8b949e"

        tk.Label(m, text="━" * 32, bg=MBKG, fg="#1f3a4d").pack(pady=(6,0))
        tk.Label(m, text="FOC Metrics - Average Signals",
                 bg=MBKG, fg=MHEAD,
                 font=("Segoe UI", 8, "bold")).pack(pady=(2,4))

        # Signal definitions: (display label, buf key, unit, decimal places)
        self._metric_defs = [
            ("Speed",    "omega_rpm", "RPM", 1),
            ("Spd ref",  "omega_ref", "RPM", 1),
            ("Id",       "id",        "A",   4),
            ("Iq",       "iq",        "A",   4),
            ("Vd",       "vd",        "V",   2),
            ("Vq",       "vq",        "V",   2),
            ("Torque",   "Te",        "N·m", 4),
        ]

        self._metric_vars = {}
        for label, key, unit, _ in self._metric_defs:
            row = tk.Frame(m, bg=MBKG)
            row.pack(fill="x", padx=14, pady=1)
            tk.Label(row, text=label, bg=MBKG, fg=MLBL,
                     font=("Segoe UI", 8), width=8, anchor="w").pack(side=tk.LEFT)
            var = tk.StringVar(value="—")
            tk.Label(row, textvariable=var, bg=MBKG, fg=MVAL,
                     font=("Courier New", 8, "bold"), width=10,
                     anchor="e").pack(side=tk.RIGHT)
            tk.Label(row, text=unit, bg=MBKG, fg=MLBL,
                     font=("Segoe UI", 7), width=4,
                     anchor="w").pack(side=tk.RIGHT)
            self._metric_vars[key] = (var, _)   # (StringVar, decimals)

        tk.Label(m, text="", bg=MBKG, height=1).pack()

    # ── Debounce: coalesce rapid slider events into one write ──────────────
    _debounce_jobs: dict = {}

    def _set(self, key, value):
        """Apply param update immediately (called after debounce delay)."""
        with params.lock:
            setattr(params, key, value)

    def _set_debounced(self, key, value, delay_ms=40):
        """Debounce slider writes: cancel pending job and reschedule."""
        if key in self._debounce_jobs:
            self.root.after_cancel(self._debounce_jobs[key])
        self._debounce_jobs[key] = self.root.after(
            delay_ms, lambda: self._set(key, value)
        )

    def _on_start(self):
        with params.lock:
            params.running = True
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.status_var.set("● Running...")
        self.status_lbl.config(fg=BTN_GO)

    def _on_stop(self):
        with params.lock:
            params.running = False
        self.btn_start.config(state="normal")
        self.btn_stop.config(state="disabled")
        self.status_var.set("● Stopped!")
        self.status_lbl.config(fg=BTN_STP)

    def _on_reset(self):
        with params.lock:
            params.reset   = True
            params.running = True
        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")
        self.status_var.set("● Running...")
        self.status_lbl.config(fg=BTN_GO)

    def _start_animation(self):
        self.ani = FuncAnimation(self.fig, self._update_plot,
                                 interval=80, blit=True, cache_frame_data=False)
        self.canvas.draw()

    def _update_plot(self, _frame):
        with buf_lock:
            if not buf_t:
                return []
            t_arr = np.array(buf_t)
            data  = {k: np.array(v) for k, v in BUF_MAP.items()}

        t_now = t_arr[-1]
        t_lo  = max(0.0, t_now - WINDOW_S)
        mask  = t_arr >= t_lo
        t_win = t_arr[mask]

        changed = []
        for sp in SUBPLOT_DEFS:
            y_all = []
            for bname in sp["bufs"]:
                ax, ln = self.lines[bname]
                y = data[bname][mask]
                ln.set_data(t_win, y)
                if len(y):
                    y_all.extend([float(y.min()), float(y.max())])
                changed.append(ln)
            ax, _ = self.lines[sp["bufs"][0]]
            ax.set_xlim(t_lo, t_lo + WINDOW_S)
            if y_all:
                mn, mx = min(y_all), max(y_all)
                pad = max(abs(mx - mn) * 0.12, 0.1)
                ax.set_ylim(mn - pad, mx + pad)

        self.canvas.draw_idle()

        # ── Update metrics (1 s average) ────────────────────────────────
        if len(t_arr) > 1:
            avg_mask = t_arr >= (t_now - 1.0)
            for _, key, _, decimals in self._metric_defs:
                arr = data[key][avg_mask]
                if len(arr):
                    avg = float(arr.mean())
                    fmt = f"{{:.{decimals}f}}"
                    self._metric_vars[key][0].set(fmt.format(avg))

        return changed


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()