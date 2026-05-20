'''
Created on 3 dic 2025

@author: 93600004
'''
import matplotlib.pyplot as plt
import numpy as np

#set_iq = 0.53  #3500rpm w/ Water
#set_iq = 0.53  #3500rpm w/ Water
#set_iq = 0.53  #3500rpm w/ Water
#set_iq = 0.53  #3500rpm w/ Water
#set_iq = 0.2  #No Water
set_iq = 0.32

simulation_time_sec = 3000 # 1h --> 3600 s * 1000 = 3600000 ms
sampling_time = 1.0/1000.0
time = np.arange(0.0, simulation_time_sec, sampling_time)

Iq_Current = set_iq  # Apk
p_mincurrent = 0.25**2
p_overload_intgMax = 100000.0

time_log = []
iamp2_log = []
overload_intg_log = []
over_load_flag_log = []

overload_intg = 0.0
halt_duration = 0.0
over_load_flag = 0.0

dc_counter = 0
dc_state = 1

Iq_Current_Meas = Iq_Current
time_on_count = 0
for i in range(1,time.shape[0]+1):
    
    t = (i-1)*sampling_time
    
    iamp2 = Iq_Current_Meas**2 
    
    overload_intg += iamp2 - p_mincurrent
    if(overload_intg < 0.0):
        overload_intg = 0.0

    if(overload_intg > p_overload_intgMax):
        over_load_flag = p_overload_intgMax
    
    time_log.append(t)
    iamp2_log.append(iamp2)
    overload_intg_log.append(overload_intg)
    over_load_flag_log.append(over_load_flag)  

    if over_load_flag == 0:
        dc_counter += 1
        if dc_state == 1:
            time_on_count += 1 
            if dc_counter > 45000: 
                dc_counter = 0
                Iq_Current_Meas = 0.0
                dc_state = 2
        elif dc_state == 2:
            if dc_counter > 30000:
                dc_counter = 0
                Iq_Current_Meas = Iq_Current
                dc_state = 1
    else:
        Iq_Current_Meas = 0.0
#         if overload_intg == 0:
#             over_load_flag = 0
#             dc_state = 1
#             Iq_Current_Meas = Iq_Current

           
print("Current iamps2 =", iamp2)
print("Total Time ON =", (time_on_count - 1)*sampling_time)
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time_log, iamp2_log, label ='Current Iamps2', color ='red')
plt.legend(loc='upper right', fontsize = 'small')

plt.subplot(2, 1, 2)
plt.plot(time_log, overload_intg_log, label ='overload_integral', color ='red')
plt.plot(time_log, over_load_flag_log, label ='overload_Fault', color ='green')
plt.legend(loc='upper right', fontsize = 'small')

plt.grid
plt.show()

print('tutto bene')
