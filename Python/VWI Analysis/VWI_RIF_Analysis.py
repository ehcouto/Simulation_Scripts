import matplotlib.pyplot as plt
import numpy as np
from ctypes import *



########
# RIF - VWI Simulation
########


sampling_frequency = 5000.0     # 200 us - sampling frequency in Hertz   
simulation_time_ms = (3.0 / 60.0)
V_Adcounts = 2952.0 # 135Vrms (190Vpk)
I_Adcounts = 868.0 #3.0L Water
#I_Adcounts = 450.0 #0.0L Water
freq_mains_Hz = 60.0
Volt_Angle = 0.0

Curr_Angle = -16.0 #Water
#Curr_Angle = -30.0 #No water

Curr_Angle = (14.0/400.0)*I_Adcounts - 45.75

print(Curr_Angle)

if(sampling_frequency == 4000.0):
    P2P_Volt = 1800.0  #Yoda 2
if(sampling_frequency == 5000.0):
    P2P_Volt = 1900.0  #FINN
    
ACCUM_THRESHOLD = 12873557.0


ACTIVE_POWER_FACTOR = (3852.0 * 83.0)

print("Creating voltage and current.")
    
time = np.arange(0.0, simulation_time_ms, 1.0/sampling_frequency)
Voltage  = V_Adcounts * 0.5 * np.sin(2 * np.pi * freq_mains_Hz * time + (np.pi * Volt_Angle / 180.0)) + 0
Current  = I_Adcounts * 0.5 * np.sin(2 * np.pi * freq_mains_Hz * time + (np.pi * Curr_Angle / 180.0)) + 0
Power_Accumulator = np.zeros(time.shape[0])
Zero_Crossing_Event = np.zeros(time.shape[0])
Active_Power = np.zeros(time.shape[0])
Power_Correction = np.zeros(time.shape[0])
Active_Power_Comp = np.zeros(time.shape[0])

Zero_Crossing = 0

print("Preparing current signal") 
for k in np.arange(time.shape[0]):
    
    Zero_Crossing_Event[k] = 0;
    
    if (Zero_Crossing == 0):
        if(Voltage[k] > 0):
            Zero_Crossing = 1
            Zero_Crossing_Event[k] = 500.0;
    else:
        if(Voltage[k] <= 0):
            Zero_Crossing = 0;
            
    Active_Power[k] = Active_Power[k-1]
    Power_Correction[k] = Power_Correction[k-1]
    Active_Power_Comp[k] = Active_Power_Comp[k-1]
   
    if(k==0):
        Power_Accumulator[0] = 0 #Current[k] * Voltage[k]
        Active_Power[0]      = 0
        Power_Correction[0]  = 0
        Active_Power_Comp[0] = 0
    else:
        Power_Accumulator[k] = Power_Accumulator[k-1] + (Current[k] * Voltage[k])
        
        if(Zero_Crossing_Event[k] > 0):
            
            #print "Accumulator = ", Power_Accumulator[k]
            #if(Power_Accumulator[k] < 12873557.0):
            if(Power_Accumulator[k] < ACCUM_THRESHOLD):
                #print "Power zeroed!"
                Active_Power[k] = 0
            else:
                Active_Power[k] = (Power_Accumulator[k] - ACCUM_THRESHOLD) / (ACTIVE_POWER_FACTOR)
                #print "Active Power = ", Active_Power[k]
            
            #temp_volt = P2P_Volt - 1626.0
            temp_volt = P2P_Volt/10.0 - 120.0
        
            #Power_Correction[k] = ((((temp_volt * temp_volt) + (873.0 * temp_volt)) / 5682.0) + 4.0)
            Power_Correction[k] = ((((115 * temp_volt) - (temp_volt * temp_volt * 2.0)) / 218.0) + 6.0)
            
            print("Power Correction = ", Power_Correction[k])
            print("Active Power = ", Active_Power[k])
            if(Power_Correction[k] > Active_Power[k]):
                #print "Compensation zeroed!"
                Active_Power_Comp[k] = 0
            else:
                Active_Power_Comp[k] = Active_Power[k] - Power_Correction[k]
                #print Active_Power_Comp[k]
            
            print("Active_Power_Comp value = ", Active_Power_Comp[k])
            print("Accumulated value = ", Power_Accumulator[k])
            Power_Accumulator[k] = 0;
        
plt.figure()
Plot_Flag = 3

if(Plot_Flag == 1):
    plt.plot(time, Voltage, label='Voltage signal [Adcount]', color='red')
    plt.plot(time, Current, label='Current signal [Adcount]', color='blue')
if(Plot_Flag == 2):    
    plt.plot(time, Zero_Crossing_Event, label='ZC_Event', color='black')
    plt.plot(time, Active_Power * 10, label='Active Power', color='Pink')
    plt.plot(time, Power_Correction, label='Power Correction', color='Orange')
    plt.plot(time, Active_Power_Comp * 1000, label='Active Power compensated', color='green')
if(Plot_Flag == 3): 
    plt.plot(time, Voltage, label='Voltage signal', color='red')
    plt.plot(time, Current, label='Current signal', color='blue')   
    #plt.plot(time, Zero_Crossing_Event, label='ZC_Event', color='black')
    plt.plot(time, Active_Power, label='Active Power', color='pink')
    plt.plot(time, Power_Correction, label='Power Correction', color='Orange')
    plt.plot(time, Active_Power_Comp, label='Active Power compensated', color='green')
if(Plot_Flag == 4):    
    plt.plot(time, Power_Accumulator, label='Accumulator', color='blue')
    plt.plot(time, Voltage*10000, label='Voltage signal [Adcount]', color='red')
    plt.plot(time, Current*10000, label='Current signal [Adcount]', color='black')
    plt.plot(time, Active_Power*100000, label='Active Power', color='green')
if(Plot_Flag == 5): 
    plt.plot(time, Active_Power_Comp, label='Active Power compensated', color='green')

print("Finished!")
plt.legend(loc='upper right', fontsize = 'small')
plt.grid()
plt.show()    

