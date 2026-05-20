import matplotlib.pyplot as plt
import numpy as np
from ctypes import *



########
# RIF - VWI Simulation
########


sampling_frequency = 1.0     # 500ms
simulation_time_sec = 60*60 #1Hr
tracking_time = 5*60

  
print("Starting the Simulation...")
    
time = np.arange(0.0, simulation_time_sec, 1.0/sampling_frequency)
Temp_Out = np.zeros(time.shape[0])
Tracking = np.zeros(time.shape[0])
Elapsed_Time = np.zeros(time.shape[0])
Power_Correction = np.zeros(time.shape[0])
Active_Power_Comp = np.zeros(time.shape[0])

elapsed_time = 0

time_lock = 0

for k in np.arange(time.shape[0]):
    
    if k == 0:
        Temp_Out[0] = 40
        tracking = Temp_Out[0]+1;
    else:
        Temp_Out[k] = 0.9989*Temp_Out[k-1] + 0.001*70
        
        elapsed_time += 1
        
        if Temp_Out[k] > tracking:
            elapsed_time = 0
            tracking = Temp_Out[k] + 1
        
        if elapsed_time > tracking_time:
            if time_lock == 0:
                time_lock = 1
                print("Tempo Esgotado!")
                print(time[k])
     
    Tracking[k] = tracking
    Elapsed_Time[k] = elapsed_time

        
plt.figure()

plt.plot(time, Temp_Out, label='Temperature Rise [C]', color='red')
plt.plot(time, Tracking, label='Tracking', color='blue')
plt.plot(time, Elapsed_Time, label='Elapsed Time', color='Black')

print("Finished!")
plt.legend(loc='upper right', fontsize = 'small')
plt.grid()
plt.show()    

