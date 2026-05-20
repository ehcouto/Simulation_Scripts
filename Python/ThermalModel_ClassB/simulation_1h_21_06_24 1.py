import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
from math import pi
from pyqtgraph.tests.test_stability import raiseException

NO = 0
YES = 1

PolePairs = 4.0 #Motor pole pairs
simulation_time_sec = 3*60*60 # 1h --> 3600 s * 1000 = 3600000 ms
time = np.arange(0.0, simulation_time_sec, 1.0)

#### Motor SR Table Parameters ####

# Rx_Thermal = Iron Losses + Motor Temperature + Joule Losses + Motor Thermal Conductance
# Iron Losses = {a1,a2}
# Motor Temperature = {Room Temperature, Motor Thermal Capacitance}
# Joule Losses = {R Winding, Alfa Winding}
# Motor Thermal Conductiance = {Spin Speed, G Tumbling, G Tumbling 1, G Spinning}

ORCAH24 = 1
ORCAH34 = 2
HEFEIH24 = 3

MOTOR_SELECTED = HEFEIH24

if MOTOR_SELECTED == ORCAH24:
    Rc_Thermal = {'OH_Period': 1, 
                  'Tamb': 40, 
                  'Motor_Thermal_Capacitance': 1.0/120.0, #from 200 to 130
                  'a1': 0.0195, 
                  'a2': 0.0000136, 
                  'Rwind': 5.54, 
                  'Alfa_Winding': 0.00402, 
                  'SSpeed': 5000.0*PolePairs/60.0, 
                  'Gspin': 1.26,  #from 1.20 to 1.36
                  'Gtumb': 0.07683,    
                  'Gtumb1': 0.00771}
    
#     Rc_Thermal = {'OH_Period': 1, 
#                   'Tamb': 40, 
#                   'Motor_Thermal_Capacitance': 1.0/200.0, 
#                   'a1': 0.0195, 
#                   'a2': 0.0000136, 
#                   'Rwind': 5.54, 
#                   'Alfa_Winding': 0.00402, 
#                   'SSpeed': 5000.0*PolePairs/60.0, 
#                   'Gspin': 1.20, 
#                   'Gtumb': 0.07683, 
#                   'Gtumb1': 0.00771}

elif MOTOR_SELECTED == ORCAH34:
    Rc_Thermal = {'OH_Period': 1, 
                  'Tamb': 40, 
                  'Motor_Thermal_Capacitance': 1.0/200.0, #from 300 to 200
                  'a1': -0.00023707*15.0, 
                  'a2': 0.00000020203*225.0, 
                  'Rwind': 5.836, 
                  'Alfa_Winding': 0.003839, 
                  'SSpeed': 5000.0*PolePairs/60.0, 
                  'Gspin': 1.90, 
                  'Gtumb': 0.101415,
                  'Gtumb1': 0.00771}
    
#     Rc_Thermal = {'OH_Period': 1, 
#                   'Tamb': 40, 
#                   'Motor_Thermal_Capacitance': 1.0/300.0, 
#                   'a1': -0.00023707*15.0, 
#                   'a2': 0.00000020203*225.0, 
#                   'Rwind': 5.836, 
#                   'Alfa_Winding': 0.003839, 
#                   'SSpeed': 5000.0*PolePairs/60.0, 
#                   'Gspin': 1.90,
#                   'Gtumb': 0.115245, 
#                   'Gtumb1': 0.00771}

elif MOTOR_SELECTED == HEFEIH24:
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
    
#     Rc_Thermal = {'OH_Period': 1, 
#                   'Tamb': 40, 
#                   'Motor_Thermal_Capacitance': 1.0/1284.0,
#                   'a1': 0.0037096599*15, 
#                   'a2': 0.00000011843200*225, 
#                   'Rwind': 2.87, 
#                   'Alfa_Winding': 0.004308, 
#                   'SSpeed': 8000.0*PolePairs/60.0, 
#                   'Gspin': 1.70, 
#                   'Gtumb': 0.52,
#                   'Gtumb1': 0.0012}
else:
    raiseException();

Ph_Current_Rms = 2.17*1.1 #1.78 #1.7*1.105
Motor_Speed_Rpm = 582.0 #1375.0 * 13.09 
SR_Overload_Temp = np.zeros(time.shape[0])
SR_Overload_Temp[0] = 125 #Temp. Init.
SR_Overload_Temp_High_Temp = 130.0 #High Temp

squared_current_f = Ph_Current_Rms**2 #Squared Current Rms
motor_speed_f = Motor_Speed_Rpm * PolePairs/60.0; # Motor speed in electrical Hz

print("Thermal Capacitance: ", 1.0/Rc_Thermal['Motor_Thermal_Capacitance'])

Fault = NO

# My_Max_Temp = 0.0
# My_Current_rms = 0.0

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
    
    if (SR_Overload_Temp[i] > (SR_Overload_Temp_High_Temp)) and (Fault == NO):
        print("Elapsed Time to Fault [Sec]: ", i)
        print("Current [Arms]: ", math.sqrt(squared_current_f)) 
        squared_current_f = 0.0
        motor_speed_f = 0.0
        Fault = YES
    
#     if squared_current_f > 0:     
#         squared_current_f = squared_current_f - 0.00148
        
#     if My_Max_Temp < SR_Overload_Temp[i]:
#         My_Max_Temp = SR_Overload_Temp[i]
#         My_Current_rms = math.sqrt(squared_current_f)
        
#         
#     print("Elapsed Time to Fault [Sec]: ", i)
#     if squared_current_f <= 0:
#         print("Current [Arms]: ", 0.0)
#     else:
#         print("Current [Arms]: ", math.sqrt(squared_current_f))

print("Final Temp: ", SR_Overload_Temp[time.shape[0]-1])
# print("Max Temp: ", My_Max_Temp)
# print("Current (rms)", My_Current_rms)
plt.figure()
plt.plot(time, SR_Overload_Temp, label ='Overload Temperature', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid
plt.show()

print('tutto bene')
