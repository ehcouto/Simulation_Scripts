import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
Fs = 1000.0    # Switching Frequency
dt = 1.0/Fs    # Time step (s)
t_end = 1.5  # Simulation end time (s)
time = np.arange(0, t_end, dt)

prm_minsp_o2c = 210.0
prm_k1min = 500.0
prm_k1max = 4700.0
prm_merge_step = dt
prm_spref_max = 3500.0
prm_spref_com = 3500.0


v_spref = 0.0
v_k1_coeff = (prm_k1max - prm_k1min) / (prm_spref_max - prm_minsp_o2c);
v_k1_merge_coeff = 1.0
v_k1_unmerged = 0.0
v_k1 = 0.0

def Merge(k1_unmerged, k1, p_merge_step, k1_merge_coeff):
    if k1_merge_coeff > 1.0:
        k1_merge_coeff = 1.0
    
    output = k1_merge_coeff * k1_unmerged + (1.0 - k1_merge_coeff) * k1
    k1_merge_coeff += p_merge_step
    
    if output == k1:
        k1_merge_coeff = 0.0
    
    return output


# Store results
v_k1_log = []
spRef_log = []

#Mechanical Speed in rad/s
#Wm = Sp*2.0*np.pi*Pp/60.0

# Simulation loop
for t in time:
    # PI controllers
    if v_spref > prm_minsp_o2c:
        v_k1_unmerged = prm_k1min + v_k1_coeff * (v_spref - prm_minsp_o2c)
    else:
        v_k1_unmerged = prm_k1min
        v_k1_merge_coeff = 1.0
        
    v_k1 = Merge(v_k1_unmerged, v_k1, prm_merge_step, v_k1_merge_coeff)
    
    v_spref += 4.0
    
    if v_spref > prm_spref_com:
        v_spref = prm_spref_com
    
    print("v_k1: ", v_k1)
    print("v_spref: ", v_spref)
    print("\n")

    v_k1_log.append(v_k1)
    spRef_log.append(v_spref)

# Plot results
plt.figure(figsize=(12, 6))
plt.plot(time, v_k1_log, label='k1')
plt.plot(time, spRef_log, label='speed ref')
plt.title('Coefficient Response')
plt.xlabel('Time (s)')
#plt.ylabel('Id Current (A)')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()
