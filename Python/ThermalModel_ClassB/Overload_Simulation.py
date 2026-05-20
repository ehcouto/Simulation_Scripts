import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
from math import pi

NO = 0
YES = 1

PolePairs = 4.0 #Motor pole pairs
simulation_time_sec = 10*60*60 # 1h --> 3600 s * 1000 = 3600000 ms
time = np.arange(0.0, simulation_time_sec, 1.0)

#### Motor SR Table Parameters ####

# Rx_Thermal = Iron Losses + Motor Temperature + Joule Losses + Motor Thermal Conductance
# Iron Losses = {a1,a2}
# Motor Temperature = {Room Temperature, Motor Thermal Capacitance}
# Joule Losses = {R Winding, Alfa Winding}
# Motor Thermal Conductiance = {Spin Speed, G Tumbling, G Tumbling 1, G Spinning}

Rc_Thermal = {'OH_Period': 1, 
              'Tamb': 20, 
              'Motor_Thermal_Capacitance': 1.0/900.0, #from 300 to 200
              'a1': 0.0045*15, 
              'a2': 0.0000000763*225, 
              'Rwind': 2.80, 
              'Alfa_Winding': 0.004308, 
              'SSpeed': 5000.0*PolePairs/60.0, 
              'Gspin': 1.50, 
              'Gtumb': 0.04,
              'Gtumb1': 0.005}

Ph_Current_Rms = 0.92
Motor_Speed_Rpm = 50.0 * 11.64 
SR_Overload_Temp = np.zeros(time.shape[0])
SR_Overload_Temp[0] = 40.0 #Temp. Init.
SR_Overload_Temp_High_Temp = 130.0 #High Temp

Temp_Limit = 40.0
Elapsed_Time = 0
Time_Limit = 20*60

squared_current_f = Ph_Current_Rms**2 #Squared Current Rms
motor_speed_f = Motor_Speed_Rpm * PolePairs/60.0; # Motor speed in electrical Hz

print("Thermal Capacitance: ", 1.0/Rc_Thermal['Motor_Thermal_Capacitance'])

Fault = NO

for i in range(1,time.shape[0]):
    
    # delta temperature calculation
    temp_delta = SR_Overload_Temp[i-1] - Rc_Thermal['Tamb']
    #### Losses generated in the motor - steel + joule ####
    # steel losses
    psteel = Rc_Thermal['a1']*motor_speed_f+Rc_Thermal['a2']*motor_speed_f*motor_speed_f
    
    # joule losses
    qjoule = 3*Rc_Thermal['Rwind']*(float(1.0)+Rc_Thermal['Alfa_Winding']*temp_delta)*squared_current_f

    #### Heat dissipated by the motor to the air ####
    if (motor_speed_f > Rc_Thermal['SSpeed']):
        gt = Rc_Thermal['Gspin']
    else:
        gt = (Rc_Thermal['Gtumb1']*temp_delta) + Rc_Thermal['Gtumb']

    # Dissipated Heat
    qdiss = gt*temp_delta # [W/C] * [C] = [W]
    
    # Net heat stored inside the motor
    qnet = qjoule + psteel - qdiss

    # Temperature derivative with time
    dTdt = qnet*Rc_Thermal['Motor_Thermal_Capacitance']

    #Update motor temperature
    SR_Overload_Temp[i] = SR_Overload_Temp[i-1] + (dTdt*Rc_Thermal['OH_Period'])
    
    
    
    if SR_Overload_Temp[i] > Temp_Limit:
        Temp_Limit = SR_Overload_Temp[i] + 2.0
        Elapsed_Time = 0
        
        
    if Elapsed_Time >= Time_Limit: # temperature stabilized...
        squared_current_f = squared_current_f * (1.10**2) #+10% current increase
        Elapsed_Time = 0
        Temp_Limit = SR_Overload_Temp[i] + 2.0
        Time_Limit = 20*60
        if Fault == NO:
            Keep_Transition_Time = i
            print("Time Current Change: ", i)
        
    Elapsed_Time = Elapsed_Time + 1
    
    if (SR_Overload_Temp[i] > (SR_Overload_Temp_High_Temp+1.0)) and (Fault == NO):
        print("Elapsed Time to Fault [Sec]: ", i)
        Time_To_Fail = i - Keep_Transition_Time
        print("Time to Fault [Sec]: ", Time_To_Fail)
        print("Squared Current [Arms^2]: ", squared_current_f)
        squared_current_f = 0.0
        motor_speed_f = 0.0
        Fault = YES

print("Final Temp: ", SR_Overload_Temp[time.shape[0]-1])
plt.figure()
plt.plot(time, SR_Overload_Temp, label ='Overload Temperature', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid
plt.show()

print('tutto bene')
