import numpy as np
import matplotlib.pyplot as plt

# Constants and Parameters
f_ac = 50  # AC mains frequency in Hz (50 Hz)
V_ac_peak = 325  # Peak AC voltage (in volts)
R_load = 10  # Load resistance (ohms)
L = 1e-3  # Inductor value (1 mH)
C = 1e-6  # Output capacitor value (1 μF)
f_switch = 100e3  # Switching frequency of the boost converter (100 kHz)

# Time parameters
T = 1 / f_ac  # Period of AC voltage
time_end = 0.1  # Duration of simulation (seconds)
dt = 1e-6  # Time step for simulation
time = np.arange(0, time_end, dt)  # Time vector

# AC input voltage (sinusoidal)
V_ac = V_ac_peak * np.sin(2 * np.pi * f_ac * time)

# Rectified DC voltage (ideal diode bridge)
V_dc = np.abs(V_ac)

# Simulation of Active PFC with a Boost Converter
# The goal is to make the input current as sinusoidal as the input voltage

# Assume a simple Proportional-Integral (PI) controller for the boost converter duty cycle
Kp = 0.002  # Proportional gain
Ki = 0.01  # Integral gain

# Control variables
I_in = np.zeros_like(time)  # Input current (to be shaped)
duty_cycle = np.zeros_like(time)  # Duty cycle for boost converter
error_integral = 0  # Integral error for PI controller
I_target = V_ac / R_load  # Target current waveform (for simplicity, assume it's the voltage divided by load resistance)

# Boost converter current calculation (simplified model)
for i in range(1, len(time)):
    # Error in current
    error = I_target[i] - I_in[i-1]
    error_integral += error * dt  # Integral of error for PI controller
    
    # PI controller to adjust the duty cycle
    duty_cycle[i] = Kp * error + Ki * error_integral
    
    # Ensure duty cycle is between 0 and 1
    duty_cycle[i] = np.clip(duty_cycle[i], 0, 1)
    
    # Simulate the inductor behavior in the boost converter (simplified)
    # We use a basic voltage equation for the boost converter: V_in * duty = L * di/dt
    # Here we assume a small enough step so that we can calculate the current change directly
    dI = (V_ac[i] * duty_cycle[i] - V_dc[i]) * dt / L
    I_in[i] = I_in[i-1] + dI

# Plotting the results
plt.figure(figsize=(10, 6))

# Plot the input AC voltage
plt.subplot(3, 1, 1)
plt.plot(time, V_ac, label='AC Voltage (V)', color='blue')
plt.title('AC Input Voltage')
plt.xlabel('Time [s]')
plt.ylabel('Voltage [V]')
plt.grid(True)

# Plot the input current
plt.subplot(3, 1, 2)
plt.plot(time, I_in, label='Input Current (A)', color='orange')
plt.title('Input Current with Active PFC Control')
plt.xlabel('Time [s]')
plt.ylabel('Current [A]')
plt.grid(True)

# Plot the duty cycle of the boost converter
plt.subplot(3, 1, 3)
plt.plot(time, duty_cycle, label='Duty Cycle', color='green')
plt.title('Boost Converter Duty Cycle')
plt.xlabel('Time [s]')
plt.ylabel('Duty Cycle')
plt.grid(True)

plt.tight_layout()
plt.show()
