import numpy as np
import matplotlib.pyplot as plt
from PI import PI_Control

# Simulation parameters
Fs = 1000.0    # Switching Frequency
dt = 1.0/Fs    # Time step (s)
t_end = 1.0  # Simulation end time (s)
time = np.arange(0, t_end, dt)

# Parameters
Kp = 0.004
Ki = 0.001
Kt = 0.30
Speed_Ref = 100.0

print(" ****** PI Controller Classic ******")
# Motor Control Time Constant
Kp_c = Kp
Ki_c = Ki
print("Kp_c: ", Kp_c)
print("Ki_c: ", Ki_c)

print("\n ****** PI Controller Beko ******")
Kp_b = Kp / Kt
Ki_b = Ki / Kt
print("Kp_b: ", Kp_b)
print("Ki_b: ", Ki_b)

# Store results
Speed_Log = []
Speed_Ref_Log = []
Iq_Ref_Clas_Log = []
Iq_Ref_Beko_Log = []

Speed = 0.0

PI_Clas = PI_Control(Kp_c, Ki_c, -256.0, 256.0)
PI_Beko = PI_Control(Kp_b, Ki_b, -256.0, 256.0)

# Simulation loop
for t in time:
    # PI controllers
    
    error = Speed_Ref - Speed;
    
    temp_c = PI_Clas.calc(error)
    temp_b = PI_Beko.calc(error)
    
    Iq_Class = temp_c / Kt
    Iq_Beko = temp_b
    
    Speed = Speed + 1.0
    if Speed > Speed_Ref:
        Speed = Speed_Ref
    
    # Store results
    Speed_Log.append(Speed_Log)
    Speed_Ref_Log.append(Speed_Ref)
    Iq_Ref_Clas_Log.append(Iq_Class)
    Iq_Ref_Beko_Log.append(Iq_Beko)

# Plot results
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(time, Speed_Log, label='Speed')
plt.plot(time, Speed_Ref_Log, label='Speed Ref')
plt.title('Speed Response')
plt.xlabel('Time (s)')
plt.ylabel('Speed (RPM)')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(time, Iq_Ref_Clas_Log, label='iq_C')
plt.plot(time, Iq_Ref_Beko_Log, label='iq_B')
plt.title('Iq Current Response')
plt.xlabel('Time (s)')
plt.ylabel('Iq Current (A)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
