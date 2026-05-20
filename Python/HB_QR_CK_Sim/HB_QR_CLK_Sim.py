import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.integrate import solve_ivp

# ============================================================
# SYSTEM PARAMETERS
# ============================================================
Vdc    = 325.0        # DC bus voltage [V]
L      = 70e-6        # Resonant inductance [H]
Cr     = 140e-9       # Resonant capacitor [F]
Rload  = 2.0          # Load resistance (equivalent cookware) [Ohm]
f_sw   = 25e3         # Switching frequency [Hz]
T_sw   = 1/f_sw       # Switching period [s]
D      = 0.47         # Approximate duty cycle

# Derived resonance parameters
f_res  = 1/(2*np.pi*np.sqrt(L*Cr))   # Resonant frequency [Hz]
Z0     = np.sqrt(L/Cr)                # Characteristic impedance [Ohm]
omega0 = 2*np.pi*f_res                # Angular resonant frequency [rad/s]

print("Resonant frequency:       {:.2f} kHz".format(f_res/1e3))
print("Characteristic impedance: {:.2f} Ohm".format(Z0))
print("Switching frequency:      {:.2f} kHz".format(f_sw/1e3))
print("f_sw / f_res ratio:       {:.3f}".format(f_sw/f_res))

# ============================================================
# ODE-BASED SIMULATION
# ============================================================
N_cycles = 12             # Number of switching cycles to simulate
t_end    = N_cycles * T_sw
dt       = T_sw / 2000    # Time step (2000 points per period)
t_eval   = np.arange(0, t_end, dt)

def half_bridge_qr(t, y, Vdc, L, Cr, Rload, T_sw, D):
    """
    State-space model of the quasi-resonant half-bridge inverter.

    State vector: y = [iL, vCr]
        iL  - inductor current [A]
        vCr - resonant capacitor voltage [V]

    Switching node voltage (Vs) is determined by gate timing:
        S1 (high-side) ON  ->  Vs = Vdc
        Dead-time interval ->  Vs = vCr  (body diode freewheels)
        S2 (low-side)  ON  ->  Vs = 0
    """
    iL, vCr = y

    t_mod = t % T_sw
    dead  = 0.02 * T_sw   # Dead-time = 2% of switching period

    if t_mod < D * T_sw - dead / 2:
        Vs = Vdc       # S1 ON: switching node tied to DC bus
    elif t_mod < D * T_sw + dead / 2:
        Vs = vCr       # Dead-time: node voltage follows resonant capacitor
    else:
        Vs = 0.0       # S2 ON: switching node tied to ground

    # Differential equations (KVL on LC tank + load)
    diL  = (Vs - vCr - Rload * iL) / L   # Inductor voltage equation
    dvCr = iL / Cr                        # Capacitor current equation

    return [diL, dvCr]

# Initial conditions: zero current, capacitor pre-charged to half the bus voltage
y0 = [0.0, Vdc / 2]

# Use lambda closure instead of args= (args= not supported in scipy < 1.1)
sol = solve_ivp(
    lambda t, y: half_bridge_qr(t, y, Vdc, L, Cr, Rload, T_sw, D),
    [0, t_end],
    y0,
    t_eval=t_eval,
    method='RK45',
    rtol=1e-8,
    atol=1e-10
)

# Unpack ODE solution
t   = sol.t
iL  = sol.y[0]   # Inductor current  [A]
vCr = sol.y[1]   # Capacitor voltage [V]

# Reconstruct switching node voltage from time stamps
Vmid = np.array([
    Vdc if (t_ % T_sw) < (D * T_sw - 0.01 * T_sw)
    else (0.0 if (t_ % T_sw) > (D * T_sw + 0.01 * T_sw) else vCr[i])
    for i, t_ in enumerate(t)
])

# Load voltage (resistive drop)
Vload = iL * Rload

# Instantaneous load power
P_inst = iL**2 * Rload

# Steady-state mask: last 5 cycles
mask  = t >= t_end - 5 * T_sw

# Average and RMS values (computed before plotting so labels are ready)
P_avg = np.mean(P_inst[mask])
V_rms = np.sqrt(np.mean(Vload[mask]**2))

# Gate drive signals (normalized amplitude for display)
gate_S1 = np.array([1.0 if (t_ % T_sw) < D * T_sw else 0.0 for t_ in t])
gate_S2 = 1.0 - gate_S1

# ============================================================
# PLOTTING
# ============================================================
plt.style.use('dark_background')

fig = plt.figure(figsize=(16, 20), facecolor='#0a0e1a')
gs  = gridspec.GridSpec(6, 2, figure=fig, hspace=0.55, wspace=0.35,
                        left=0.08, right=0.97, top=0.93, bottom=0.05)

# Color palette
accent1 = '#00e5ff'   # cyan      -> inductor current
accent2 = '#ff6b35'   # orange    -> S2 gate / FFT
accent3 = '#a8ff3e'   # lime      -> power / load voltage
accent4 = '#ff3ea8'   # magenta   -> capacitor voltage
accent5 = '#ffe066'   # yellow    -> switching node / averages
grid_c  = '#1e2a3a'   # grid line color

# Figure title and parameter subtitle
fig.text(0.5, 0.965,
         'Quasi-Resonant Half-Bridge Inverter - Induction Heating Simulation',
         ha='center', va='center', fontsize=17, fontweight='bold',
         color='white', fontname='Courier New')
fig.text(0.5, 0.950,
         'f_sw={:.0f}kHz | f_res={:.1f}kHz | L={:.0f}uH | Cr={:.0f}nF | Rload={:.0f}Ohm | Vdc={:.0f}V'.format(
             f_sw/1e3, f_res/1e3, L*1e6, Cr*1e9, Rload, Vdc),
         ha='center', va='center', fontsize=9.5, color='#8ab4cc', fontname='Courier New')

t_us = t * 1e6   # Time axis in microseconds

# ------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------
def style_ax(ax, title, ylabel, color):
    """Apply consistent dark-theme styling to a matplotlib axis."""
    ax.set_facecolor('#0d1526')
    ax.set_title(title, color=color, fontsize=10, fontname='Courier New', pad=5)
    ax.set_ylabel(ylabel, color='#8ab4cc', fontsize=8.5, fontname='Courier New')
    ax.set_xlabel('Time [us]', color='#8ab4cc', fontsize=8, fontname='Courier New')
    ax.tick_params(colors='#5a7a9a', labelsize=8)
    ax.grid(True, color=grid_c, linewidth=0.5, alpha=0.8)
    for spine in ax.spines.values():
        spine.set_color('#1e2a3a')

def style_legend(leg):
    """
    Style legend text color to white.
    Replaces the labelcolor= kwarg which was only added in matplotlib 3.2.
    """
    leg.get_frame().set_facecolor('#0d1526')
    leg.get_frame().set_alpha(0.2)
    for text in leg.get_texts():
        text.set_color('white')

# --- Plot 1: Gate drive signals for S1 and S2 ---
ax1 = fig.add_subplot(gs[0, :])
ax1.fill_between(t_us[mask], gate_S1[mask]*Vdc*0.04,
                 alpha=0.3, color=accent1, label='Gate S1 (high-side)')
ax1.fill_between(t_us[mask], gate_S2[mask]*Vdc*0.04,
                 alpha=0.3, color=accent2, label='Gate S2 (low-side)')
ax1.plot(t_us[mask], gate_S1[mask]*Vdc*0.04, color=accent1, lw=1.4, label='_')
ax1.plot(t_us[mask], gate_S2[mask]*Vdc*0.04, color=accent2, lw=1.4, label='_')
ax1.set_ylim(-1, Vdc*0.04*1.4)
ax1.set_yticks([0, Vdc*0.04])
ax1.set_yticklabels(['LOW', 'HIGH'], color='#8ab4cc', fontsize=8)
style_legend(ax1.legend(loc='upper right', fontsize=8))
style_ax(ax1, 'Gate Signals - S1 and S2 (complementary switching)', '', accent1)

# --- Plot 2: Switching node voltage (half-bridge midpoint) ---
ax2 = fig.add_subplot(gs[1, :])
ax2.plot(t_us[mask], Vmid[mask], color=accent5, lw=1.3, label='V_mid')
ax2.axhline(Vdc/2, color='#ffffff44', lw=0.8, ls='--',
            label='Vdc/2 = {} V'.format(int(Vdc/2)))
ax2.fill_between(t_us[mask], Vmid[mask], alpha=0.12, color=accent5)
style_legend(ax2.legend(loc='upper right', fontsize=8))
style_ax(ax2, 'Switching Node Voltage - Half-Bridge Midpoint (V_mid)', 'V_mid [V]', accent5)

# --- Plot 3: Resonant inductor current ---
ax3 = fig.add_subplot(gs[2, :])
ax3.plot(t_us[mask], iL[mask]*1000, color=accent1, lw=1.4, label='i_L')
ax3.fill_between(t_us[mask], iL[mask]*1000, alpha=0.12, color=accent1)
ax3.axhline(0, color='#ffffff33', lw=0.7)
style_legend(ax3.legend(loc='upper right', fontsize=8))
style_ax(ax3, 'Resonant Inductor Current - i_L', 'i_L [mA]', accent1)

# --- Plot 4: Resonant capacitor voltage ---
ax4 = fig.add_subplot(gs[3, 0])
ax4.plot(t_us[mask], vCr[mask], color=accent4, lw=1.4)
ax4.fill_between(t_us[mask], vCr[mask], alpha=0.12, color=accent4)
ax4.axhline(Vdc/2, color='#ffffff44', lw=0.8, ls='--', label='Vdc/2')
style_legend(ax4.legend(loc='upper right', fontsize=8))
style_ax(ax4, 'Resonant Capacitor Voltage - v_Cr', 'v_Cr [V]', accent4)

# --- Plot 5: Instantaneous power delivered to the load ---
ax5 = fig.add_subplot(gs[3, 1])
ax5.plot(t_us[mask], P_inst[mask], color=accent3, lw=1.2)
ax5.fill_between(t_us[mask], P_inst[mask], alpha=0.15, color=accent3)
ax5.axhline(P_avg, color=accent5, lw=1.2, ls='--',
            label='P_avg = {:.2f} W'.format(P_avg))
style_legend(ax5.legend(loc='upper right', fontsize=8))
style_ax(ax5, 'Instantaneous Load Power - P(t)', 'P [W]', accent3)

# --- Plot 6: Phase-plane portrait (iL vs vCr) ---
# Elliptical orbits reveal the quasi-resonant switching behaviour
ax6 = fig.add_subplot(gs[4, 0])
sc = ax6.scatter(vCr[mask], iL[mask]*1000,
                 c=t[mask], cmap='plasma', s=0.8, alpha=0.7)
ax6.set_facecolor('#0d1526')
ax6.set_title('Phase Plane - i_L x v_Cr (quasi-resonant orbits)',
              color=accent4, fontsize=10, fontname='Courier New', pad=5)
ax6.set_xlabel('v_Cr [V]', color='#8ab4cc', fontsize=8.5, fontname='Courier New')
ax6.set_ylabel('i_L [mA]', color='#8ab4cc', fontsize=8.5, fontname='Courier New')
ax6.tick_params(colors='#5a7a9a', labelsize=8)
ax6.grid(True, color=grid_c, linewidth=0.5, alpha=0.8)
for sp in ax6.spines.values():
    sp.set_color('#1e2a3a')
cb = plt.colorbar(sc, ax=ax6)
cb.set_label('Time', color='#8ab4cc', fontsize=8)
cb.ax.yaxis.set_tick_params(color='#5a7a9a', labelcolor='#8ab4cc', labelsize=7)

# --- Plot 7: FFT spectrum of the inductor current ---
ax7 = fig.add_subplot(gs[4, 1])
N_fft = len(iL[mask])
freq  = np.fft.rfftfreq(N_fft, d=dt)             # Frequency axis [Hz]
fft_i = np.abs(np.fft.rfft(iL[mask])) / N_fft    # Single-sided amplitude spectrum
ax7.semilogy(freq/1e3, fft_i*1000 + 1e-6, color=accent2, lw=0.9)
ax7.axvline(f_sw/1e3,  color=accent5, lw=1.2, ls='--',
            label='f_sw  = {:.0f} kHz'.format(f_sw/1e3))
ax7.axvline(f_res/1e3, color=accent1, lw=1.2, ls='--',
            label='f_res = {:.1f} kHz'.format(f_res/1e3))
ax7.set_xlim(0, 300)
style_legend(ax7.legend(loc='upper right', fontsize=8))
style_ax(ax7, 'FFT Spectrum - Inductor Current i_L', '|I_L| [mA]', accent2)

# --- Plot 8: Load voltage ---
ax8 = fig.add_subplot(gs[5, :])
ax8.plot(t_us[mask], Vload[mask], color=accent3, lw=1.2,
         label='V_load = i_L x R')
ax8.fill_between(t_us[mask], Vload[mask], alpha=0.12, color=accent3)
ax8.axhline( V_rms, color=accent5, lw=1.0, ls='--',
             label='V_rms = {:.2f} V'.format(V_rms))
ax8.axhline(-V_rms, color=accent5, lw=1.0, ls='--')
style_legend(ax8.legend(loc='upper right', fontsize=8))
style_ax(ax8, 'Load Voltage - Inductive Element (Cookware)', 'V_load [V]', accent3)

# ============================================================
# DISPLAY FIGURE
# ============================================================
plt.tight_layout()
plt.show()

# ============================================================
# SUMMARY RESULTS
# ============================================================
print("\n=== Simulation Results ===")
print("Average load power:     {:.2f} W".format(P_avg))
print("RMS load voltage:       {:.2f} V".format(V_rms))
print("Peak inductor current:  {:.1f} mA".format(np.max(np.abs(iL[mask]))*1000))
print("Peak capacitor voltage: {:.1f} V".format(np.max(vCr[mask])))
