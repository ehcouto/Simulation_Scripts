import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
Fs = 7550.0    # Switching Frequency
dt = 1.0/Fs    # Time step (s)
t_end = 0.01  # Simulation end time (s)
time = np.arange(0, t_end, dt)

# Motor parameters
Pp   = 1      # Pole Pairs
Ld   = 0.1324   # d-axis inductance (H)
Lq   = 0.1324   # q-axis inductance (H)
Rs   = 46.0   # Stator resistance (Ohm)
B    = 3.0    # Controller Bandwidth x motor constant time (adm)
Sp   = 900    # Motor Speed (RPM) 
Phi  = 0.072 # Magnet Flux (Wb)
J = 0.000002349
b = 0.000016


print(" ****** dq Observer Coefficients ******")
# Motor Control Time Constant
tau = Ld/Rs
print("Motor Time Constant (ms): ", tau*1000.0)

print("\n ****** dq Observer Coefficients ******")
# dq Observer Parameters
I_Coeff  = (Ld-dt*Rs)/Ld # Current Coefficient
U_Coeff  = dt/Ld         # Voltage & BEMF Coefficient
Wi_Coeff = U_Coeff*Lq    # Omega*current coefficient

print("I_Coeff: ", I_Coeff)
print("U_Coeff: ", U_Coeff)
print("Wi_Coeff: ", Wi_Coeff)

print("\n ****** dq Observer PI Gains ******")
# dq Controller Observer Parameters
wn      = 200.0   # 200Hz Bandwidth
epsilon = 0.85    # Epsilon for stability response
Kp_dqObs = 2.0*wn*2.0*np.pi*epsilon*Ld - Rs
Ki_dqObs = (wn*2.0*np.pi)**2 * Ld

print("Kp_dqObs: ", Kp_dqObs) 
print("Ki_dqObs: ", Ki_dqObs)


print("\n ****** Angle Tracking PI Gains ******")
# dq Controller Observer Parameters
wn      = 11   # 200Hz Bandwidth
epsilon = 2.3    # Epsilon for stability response
Kp_ato = 2.0*wn*2.0*np.pi*epsilon
Ki_ato = (wn*2.0*np.pi)**2

print("Kp_ato: ", Kp_ato) 
print("Ki_ato: ", Ki_ato)


print("\n ****** Current Controller Gains ******")
#Calculing PI parameters
w_c = (B / tau) # Controller Bandwidth

# PI controller parameters
Kp = w_c * Ld
Ki = w_c * Rs

print("Kp Gain: ", Kp)
print("Ki Gain: ", Ki)



print("\n ****** Speed Controller Gains ******")
#Calculing PI parameters
w_c = w_c / (5 * 7.55) # Controller Bandwidth
Kt = 0.106

# PI controller parameters
Kp_sp = J * w_c / Kt
Ki_sp = b * w_c / Kt

print("Kp_sp Gain: ", Kp_sp)
print("Ki_sp Gain: ", Ki_sp)

# Reference currents
id_ref = 0.0  # d-axis current reference
iq_ref = 0.6  # q-axis current reference

# Initialize variables
isd = 0.0
isq = 0.0
vd = 0.0
vq = 0.0
id_error_int = 0.0
iq_error_int = 0.0

# Store results
id_list = []
iq_list = []
id_ref_list = []
iq_ref_list = []
vd_list = []
vq_list = []

#Mechanical Speed in rad/s
Wm = Sp*2.0*np.pi*Pp/60.0

# Simulation loop
for t in time:
    # PI controllers
    id_error = id_ref - isd
    iq_error = iq_ref - isq
    id_error_int += id_error * Ki * dt
    iq_error_int += iq_error * Ki * dt

    vd = Kp * id_error + id_error_int - (Wm*Pp*Lq*isq)
    vq = Kp * iq_error + iq_error_int + (Wm*Pp*(isd*Ld + Phi))
    
    vd_dec = vd #- (Wm*Pp*Lq*isq)
    vq_dec = vq #+ (Wm*Pp*(isd*Ld + Phi))
    
    
    # Motor model update (simplified)
    did = (vd_dec - Rs * isd + (Wm*Pp*Lq*isq))          / Ld 
    diq = (vq_dec - Rs * isq - (Wm*Pp*(isd*Ld + Phi)))  / Lq 
    isd += did*dt
    isq  += diq*dt

    # Store results
    id_ref_list.append(id_ref)
    iq_ref_list.append(iq_ref)
    id_list.append(isd)
    iq_list.append(isq)
    vd_list.append(vd_dec)
    vq_list.append(vq_dec)

# Plot results
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(time, id_ref_list, label='id_ref')
plt.plot(time, id_list, label='iq')
plt.title('Id Current Response')
plt.xlabel('Time (s)')
plt.ylabel('Id Current (A)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(time, iq_ref_list, label='iq_ref')
plt.plot(time, iq_list, label='iq')
plt.title('Iq Current Response')
plt.xlabel('Time (s)')
plt.ylabel('Iq Current (A)')
plt.legend()
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(time, vd_list, label='vd')
plt.plot(time, vq_list, label='vq')
plt.title('Voltage Commands')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
