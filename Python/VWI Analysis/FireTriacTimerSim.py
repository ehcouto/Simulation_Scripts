import matplotlib.pyplot as plt
import numpy as np
from ctypes import *



########
# RIF - VWI Simulation
########


#plt.switch_backend('Qt4Agg')

Timer_Clock = 8000000.0
sampling_frequency = 50000.0     # 200 us - sampling frequency in Hertz
simulation_time_ms = (2.0 / 60.0) * 1000.0
V_Adcounts = 311.0
freq_mains_Hz = 60.0
Volt_Angle = 0.0
Zero_Crossing = 0

print "Creating voltage."
    
time = np.arange(0.0, simulation_time_ms/1000.0, 1.0/sampling_frequency)
Voltage  = V_Adcounts * np.sin(2 * np.pi * freq_mains_Hz * time + (np.pi * Volt_Angle / 180.0)) + 0
Timer = np.zeros(time.shape[0])


print "Preparing current signal" 
for k in np.arange(time.shape[0]):
    
 
    if(k==0):
        Timer[0] = 0
    else:
        Timer[k] = Timer[k-1] + Timer_Clock/sampling_frequency
        
    if (Zero_Crossing == 0):
        if(Voltage[k] > 0):
            Zero_Crossing = 1
            Timer[k] = 0
    else:
        if(Voltage[k] <= 0):
            Zero_Crossing = 0
            Timer[k] = 0
        
plt.figure()
plt.plot(time, Voltage*100, label='Voltage signal [Adcount]', color='red')
plt.plot(time, Timer, label='Timer [Count]', color='blue')

print "Plotting Graphic.."
plt.legend(loc='lower left', fontsize = 'small')
plt.grid()
plt.show()    
print "Finished!"

