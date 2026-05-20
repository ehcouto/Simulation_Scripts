# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.integrate import solve_ivp

# ============================================================
# SYSTEM PARAMETERS
# ============================================================
Vdc    = 325.0        # DC bus voltage [V]  (rectified 220Vac ~ 310V)
L      = 100e-6       # Resonant inductance [H]
Cr     = 270e-9       # Resonant capacitor [F]
Rload  = 20.0         # Equivalent load resistance (cookware) [Ohm]
f_sw   = 30e3         # Switching frequency [Hz]
T_sw   = 1 / f_sw     # Switching period [s]
D      = 0.8         # Duty cycle

# Derived resonance parameters
f_res  = 1 / (2 * np.pi * np.sqrt(L * Cr))   # Resonant frequency [Hz]
T_res  = 1 / f_res                             # Resonant period [s]
Z0     = np.sqrt(L / Cr)                       # Characteristic impedance [Ohm]
omega0 = 2 * np.pi * f_res

# ZCS condition: half resonant period must fit inside the ON-time
t_half = np.pi / omega0   # half resonant period [s]
zcs_ok = t_half < D * T_sw

print("Single-Switch Quasi-Resonant Inverter  -  ZCS Topology")
print("=" * 70)
print("Resonant frequency:        {:.1f} kHz".format(f_res / 1e3))
print("Characteristic impedance:  {:.1f} Ohm".format(Z0))
print("Switching frequency:       {:.1f} kHz".format(f_sw / 1e3))
print("f_sw / f_res ratio:        {:.3f}".format(f_sw / f_res))
print("Half resonant period:      {:.2f} us".format(t_half * 1e6))
print("ON-time (D x T_sw):        {:.2f} us".format(D * T_sw * 1e6))
print("ZCS condition met:         {}".format("YES" if zcs_ok else "NO  -  increase D or reduce f_sw"))

# ============================================================
# ODE-BASED SIMULATION
# ============================================================
# Correct topology: Cr is in PARALLEL with Rload (not in series).
# The inductor L is in series with the switch S1 and blocking diode Db.
# Three operating modes per cycle:
#
#   Mode 1  S1 ON  (0 < t_mod < D*T_sw):
#           Vdc feeds through Db and L into the Cr||Rload tank.
#           diL  = (Vdc - vCr) / L
#           dvCr = (iL - vCr/Rload) / Cr
#
#   Mode 2  S1 OFF, iL > 0  (freewheeling diode Df conducts):
#           L freewheels its stored energy into Cr||Rload.
#           diL  = -vCr / L
#           dvCr = (iL - vCr/Rload) / Cr
#           ZCS occurs when iL naturally returns to zero.
#
#   Mode 3  S1 OFF, iL <= 0  (Df blocks  -  idle):
#           L is disconnected; Cr simply discharges into Rload.
#           This vCr reset is essential: it allows the next S1 ON
#           to restart the resonant swing from a low initial voltage.
#           diL  = 0
#           dvCr = -vCr / (Rload * Cr)

N_cycles = 25             # simulate enough cycles to reach steady state
t_end    = N_cycles * T_sw

# Use adaptive solver without t_eval  -  the resonant pulse is narrow
# max_step ensures the solver captures the fast transient.
max_step = T_res / 300    # Enough steps to resolve 

def single_switch_qr(t, y, Vdc, L, Cr, Rload, T_sw, D):
    """
    Single-switch ZCS quasi-resonant inverter  -  corrected state-space model.
    Cr is in parallel with Rload (not series).
    """
    iL, vCr = y
    t_mod = t % T_sw

    if t_mod < D * T_sw:
        # Mode 1: S1 ON, blocking diode Db conducts
        diL  = (Vdc - vCr) / L
        dvCr = (iL - vCr / Rload) / Cr
    else:
        if iL > 1e-3:
            # Mode 2: S1 OFF, freewheeling diode Df conducts
            diL  = -vCr / L
            dvCr = (iL - vCr / Rload) / Cr
        else:
            # Mode 3: idle  -  Cr discharges through Rload only
            diL  = 0.0
            dvCr = -vCr / (Rload * Cr)

    return [diL, dvCr]

# Initial conditions: zero current, capacitor fully discharged
y0 = [0.0, 0.0]

# Lambda closure for scipy < 1.1 compatibility (no args= parameter)
sol = solve_ivp(
    lambda t, y: single_switch_qr(t, y, Vdc, L, Cr, Rload, T_sw, D),
    [0, t_end],
    y0,
    max_step=max_step,
    rtol=1e-9,
    atol=1e-12
)

# Unpack  -  clamp negative current (ideal diode Df model)
t   = sol.t
iL  = np.maximum(sol.y[0], 0.0)
vCr = sol.y[1]

# Derived signals
gate_S1 = np.where((t % T_sw) < D * T_sw, 1.0, 0.0)
Vmid    = gate_S1 * Vdc          # switching node (simplified)
Vds     = (1 - gate_S1) * vCr   # S1 drain-source voltage when OFF
Vload   = iL * Rload
P_inst  = iL**2 * Rload

# Steady-state: last 6 cycles
mask  = t >= t_end - 6 * T_sw
P_avg = np.mean(P_inst[mask])
V_rms = np.sqrt(np.mean(Vload[mask]**2))
I_pk  = np.max(iL[mask])
V_pk  = np.max(vCr[mask])

# ============================================================
# PLOTTING
# ============================================================
plt.style.use('dark_background')

fig = plt.figure(figsize=(16, 22), facecolor='#0a0e1a')
gs  = gridspec.GridSpec(7, 2, figure=fig, hspace=0.58, wspace=0.35,
                        left=0.08, right=0.97, top=0.94, bottom=0.04)

accent1 = '#00e5ff'   # cyan      -> inductor current
accent2 = '#ff6b35'   # orange    -> gate
accent3 = '#a8ff3e'   # lime      -> power / load
accent4 = '#ff3ea8'   # magenta   -> capacitor voltage
accent5 = '#ffe066'   # yellow    -> Vmid / averages
accent6 = '#bf5fff'   # violet    -> Vds switch stress
grid_c  = '#1e2a3a'

fig.text(0.5, 0.966,
         'Single-Switch Quasi-Resonant Inverter (ZCS)  -  Induction Heating',
         ha='center', va='center', fontsize=17, fontweight='bold',
         color='white', fontname='Courier New')
fig.text(0.5, 0.952,
         'f_sw={:.0f}kHz | f_res={:.1f}kHz | L={:.0f}uH | Cr={:.0f}nF | Rload={:.0f}Ohm | Vdc={:.0f}V | D={:.2f}'.format(
             f_sw/1e3, f_res/1e3, L*1e6, Cr*1e9, Rload, Vdc, D),
         ha='center', va='center', fontsize=9.5, color='#8ab4cc', fontname='Courier New')

t_us = t * 1e6

def style_ax(ax, title, ylabel, color):
    ax.set_facecolor('#0d1526')
    ax.set_title(title, color=color, fontsize=10, fontname='Courier New', pad=5)
    ax.set_ylabel(ylabel, color='#8ab4cc', fontsize=8.5, fontname='Courier New')
    ax.set_xlabel('Time [us]', color='#8ab4cc', fontsize=8, fontname='Courier New')
    ax.tick_params(colors='#5a7a9a', labelsize=8)
    ax.grid(True, color=grid_c, linewidth=0.5, alpha=0.8)
    for spine in ax.spines.values():
        spine.set_color('#1e2a3a')

def style_legend(leg):
    """White legend text  -  replaces labelcolor= (matplotlib < 3.2)."""
    leg.get_frame().set_facecolor('#0d1526')
    leg.get_frame().set_alpha(0.2)
    for text in leg.get_texts():
        text.set_color('white')

# --- Plot 1: Gate signal ---
ax1 = fig.add_subplot(gs[0, :])
ax1.fill_between(t_us[mask], gate_S1[mask] * Vdc * 0.04,
                 alpha=0.35, color=accent2, label='Gate S1')
ax1.plot(t_us[mask], gate_S1[mask] * Vdc * 0.04, color=accent2, lw=1.4, label='_')
ax1.set_ylim(-1, Vdc * 0.04 * 1.5)
ax1.set_yticks([0, Vdc * 0.04])
ax1.set_yticklabels(['OFF', 'ON'], color='#8ab4cc', fontsize=8)
style_legend(ax1.legend(loc='upper right', fontsize=8))
style_ax(ax1, 'Gate Signal  -  S1  (single MOSFET, ZCS turn-off)', '', accent2)

# --- Plot 2: Switching node voltage ---
ax2 = fig.add_subplot(gs[1, :])
ax2.plot(t_us[mask], Vmid[mask], color=accent5, lw=1.3, label='V_mid')
ax2.axhline(Vdc, color='#ffffff33', lw=0.8, ls='--',
            label='Vdc = {} V'.format(int(Vdc)))
ax2.fill_between(t_us[mask], Vmid[mask], alpha=0.12, color=accent5)
style_legend(ax2.legend(loc='upper right', fontsize=8))
style_ax(ax2, 'Switching Node Voltage  -  V_mid  (Vdc when ON, 0 when OFF)', 'V_mid [V]', accent5)

# --- Plot 3: Resonant inductor current ---
ax3 = fig.add_subplot(gs[2, :])
ax3.plot(t_us[mask], iL[mask], color=accent1, lw=1.4, label='i_L')
ax3.fill_between(t_us[mask], iL[mask], alpha=0.12, color=accent1)
ax3.axhline(0, color='#ffffff33', lw=0.7)
# Mark ZCS turn-off instants (where S1 opens and current is near zero)
t_ss = t_end - 6 * T_sw
for n in range(7):
    t_off = t_ss + n * T_sw + D * T_sw
    if t_off <= t[-1]:
        ax3.axvline(t_off * 1e6, color=accent2, lw=0.8, ls=':', alpha=0.7)
style_legend(ax3.legend(loc='upper right', fontsize=8))
style_ax(ax3, 'Resonant Inductor Current  -  i_L  (dotted = ZCS turn-off instants)', 'i_L [A]', accent1)

# --- Plot 4: Resonant capacitor voltage ---
ax4 = fig.add_subplot(gs[3, 0])
ax4.plot(t_us[mask], vCr[mask], color=accent4, lw=1.4, label='v_Cr')
ax4.fill_between(t_us[mask], vCr[mask], alpha=0.12, color=accent4)
ax4.axhline(Vdc, color='#ffffff44', lw=0.8, ls='--',
            label='Vdc = {} V'.format(int(Vdc)))
ax4.axhline(V_pk, color=accent2, lw=0.9, ls='--',
            label='v_Cr peak = {:.0f} V'.format(V_pk))
style_legend(ax4.legend(loc='upper right', fontsize=8))
style_ax(ax4, 'Resonant Capacitor Voltage  -  v_Cr', 'v_Cr [V]', accent4)

# --- Plot 5: Switch S1 voltage stress (Vds when S1 is OFF) ---
ax5 = fig.add_subplot(gs[3, 1])
ax5.plot(t_us[mask], Vds[mask], color=accent6, lw=1.3, label='V_ds S1 (OFF state)')
ax5.fill_between(t_us[mask], Vds[mask], alpha=0.12, color=accent6)
ax5.axhline(Vdc, color='#ffffff44', lw=0.8, ls='--',
            label='Vdc = {} V'.format(int(Vdc)))
style_legend(ax5.legend(loc='upper right', fontsize=8))
style_ax(ax5, 'Switch Voltage Stress  -  V_ds  (MOSFET must withstand v_Cr peak)', 'V_ds [V]', accent6)

# --- Plot 6: Instantaneous load power ---
ax6 = fig.add_subplot(gs[4, 0])
ax6.plot(t_us[mask], P_inst[mask], color=accent3, lw=1.2, label='P(t)')
ax6.fill_between(t_us[mask], P_inst[mask], alpha=0.15, color=accent3)
ax6.axhline(P_avg, color=accent5, lw=1.2, ls='--',
            label='P_avg = {:.0f} W'.format(P_avg))
style_legend(ax6.legend(loc='upper right', fontsize=8))
style_ax(ax6, 'Instantaneous Load Power  -  P(t)', 'P [W]', accent3)

# --- Plot 7: Phase-plane portrait (iL vs vCr) ---
# ZCS single-switch: half-ellipse orbits clipped at iL = 0 (Df blocks reverse)
ax7 = fig.add_subplot(gs[4, 1])
sc = ax7.scatter(vCr[mask], iL[mask],
                 c=t[mask], cmap='plasma', s=1.0, alpha=0.6)
ax7.set_facecolor('#0d1526')
ax7.set_title('Phase Plane  -  i_L x v_Cr  (half-ellipse ZCS orbits)',
              color=accent4, fontsize=10, fontname='Courier New', pad=5)
ax7.set_xlabel('v_Cr [V]', color='#8ab4cc', fontsize=8.5, fontname='Courier New')
ax7.set_ylabel('i_L [A]', color='#8ab4cc', fontsize=8.5, fontname='Courier New')
ax7.axhline(0, color='#ffffff22', lw=0.7)
ax7.tick_params(colors='#5a7a9a', labelsize=8)
ax7.grid(True, color=grid_c, linewidth=0.5, alpha=0.8)
for sp in ax7.spines.values():
    sp.set_color('#1e2a3a')
cb = plt.colorbar(sc, ax=ax7)
cb.set_label('Time', color='#8ab4cc', fontsize=8)
cb.ax.yaxis.set_tick_params(color='#5a7a9a', labelcolor='#8ab4cc', labelsize=7)

# --- Plot 8: FFT spectrum of inductor current ---
ax8 = fig.add_subplot(gs[5, :])
N_fft  = len(iL[mask])
dt_fft = np.mean(np.diff(t[mask]))   # average step in the adaptive grid
freq   = np.fft.rfftfreq(N_fft, d=dt_fft)
fft_i  = np.abs(np.fft.rfft(iL[mask])) / N_fft
ax8.semilogy(freq / 1e3, fft_i + 1e-9, color=accent2, lw=0.8)
ax8.axvline(f_sw / 1e3,  color=accent5, lw=1.2, ls='--',
            label='f_sw  = {:.0f} kHz'.format(f_sw / 1e3))
ax8.axvline(f_res / 1e3, color=accent1, lw=1.2, ls='--',
            label='f_res = {:.1f} kHz'.format(f_res / 1e3))
ax8.set_xlim(0, 500)
style_legend(ax8.legend(loc='upper right', fontsize=8))
style_ax(ax8, 'FFT Spectrum  -  Inductor Current i_L', '|I_L| [A]', accent2)

# --- Plot 9: Load voltage ---
ax9 = fig.add_subplot(gs[6, :])
ax9.plot(t_us[mask], Vload[mask], color=accent3, lw=1.2,
         label='V_load = i_L x R')
ax9.fill_between(t_us[mask], Vload[mask], alpha=0.12, color=accent3)
ax9.axhline(V_rms, color=accent5, lw=1.0, ls='--',
            label='V_rms = {:.1f} V'.format(V_rms))
style_legend(ax9.legend(loc='upper right', fontsize=8))
style_ax(ax9, 'Load Voltage  -  Inductive Element (Cookware)', 'V_load [V]', accent3)

# ============================================================
# SUMMARY RESULTS
# ============================================================
print("\n=== Simulation Results (steady state) ===")
print("Average load power:              {:.0f} W".format(P_avg))
print("RMS load voltage:                {:.1f} V".format(V_rms))
print("Peak inductor current:           {:.2f} A".format(I_pk))
print("Peak capacitor / switch voltage: {:.0f} V".format(V_pk))
print("Voltage stress ratio Vpk/Vdc:    {:.2f}x  (MOSFET must withstand this)".format(V_pk / Vdc))

# ============================================================
# DISPLAY
# ============================================================
plt.tight_layout()
plt.show()