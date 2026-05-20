import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
from math import pi

NO = 0
YES = 1

sampling_frequency = 1
simulation_time_sec = 1800.0 # 1h --> 3600 s * 1000 = 3600000 ms
time = np.arange(0.0, simulation_time_sec, 1.0/sampling_frequency)

#### Motor SR Table Parameters ####

# Rx_Thermal = Iron Losses + Motor Temperature + Joule Losses + Motor Thermal Conductance
# Iron Losses = {a1,a2}
# Motor Temperature = {Room Temperature, Motor Thermal Capacitance}
# Joule Losses = {R Winding, Alfa Winding}
# Motor Thermal Conductiance = {Spin Speed, G Tumbling, G Tumbling 1, G Spinning}

PolePairs = 4.0 #Motor pole pairs
Rc_Thermal = {'OH_Period': 1, 
              'Tamb': 40, 
              'Motor_Thermal_Capacitance': 1.0/100.0, 
              'a1': 0.0234, 
              'a2': 0.0000162, 
              'Rwind': 5.54, 
              'Alfa_Winding': 0.00402, 
              'SSpeed': 5000.0*PolePairs/60.0, 
              'Gspin': 1.49, 
              'Gtumb': 0.07683*1.3, 
              'Gtumb1': 0.007771*1.3}

Fault = NO

plt.figure()
while Rc_Thermal['Motor_Thermal_Capacitance'] >= (0.001*0.999):
    print("Thermal Capacitance: ", 1.0/Rc_Thermal['Motor_Thermal_Capacitance'])
    
    Ph_Current_Rms = 2.0
    Motor_Speed_Rpm = 18000.0
    SR_Overload_Temp = np.zeros(time.shape[0])
    SR_Overload_Temp[0] = 40 #Temp. Init.
    SR_Overload_Temp_High_Temp = 130.0 #High Temp
    
    squared_current_f = Ph_Current_Rms**2 #Squared Current Rms
    motor_speed_f = Motor_Speed_Rpm * PolePairs/60; # Motor speed in electrical Hz

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
        
        if (SR_Overload_Temp[i] > SR_Overload_Temp_High_Temp) and (Fault == NO):
            print("Elapsed Time to Fault [Sec]: ", i) 
            squared_current_f = 0.0
            motor_speed_f = 0.0
            Fault = YES

    cmot_name = 'Temperature for Cmot ' + str(1.0/Rc_Thermal['Motor_Thermal_Capacitance'])
    plt.plot(time, SR_Overload_Temp, label = cmot_name)
    plt.legend(loc='upper right', fontsize = 'small')
    plt.grid
    
    Rc_Thermal['Motor_Thermal_Capacitance'] = Rc_Thermal['Motor_Thermal_Capacitance'] - 0.001
    Fault = NO
    i = 0

plt.show()
print('tutto bene')
