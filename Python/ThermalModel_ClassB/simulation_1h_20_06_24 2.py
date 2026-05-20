import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math

sampling_frequency = 1.0     # 200 us - sampling frequency in Hertz   
simulation_time_ms = 3600 # 1h

#### Motor SR Table Parameters ####

# Rx_Thermal = Iron Losses + Motor Temperature + Joule Losses + Motor Thermal Conductance
# Iron Losses = {a1,a2}
# Motor Temperature = {High Temp Threshold, Room Temperature, Motor Thermal Capacitance}
# Joule Losses = {R Winding, Alfa Winding}
# Motor Thermal Conductiance = {Spin Speed, G Tumbling, G Tumbling 1, G Spinning}

Rc_Thermal = {'OH_Period': 1, 'High_Temp_Threshold': 130, 'Tamb': 25, 'Motor_Thermal_Capacitance': 1284, 'a1': 3.7096599*pow(10,-3), 'a2': 1.1843200*pow(10,-7), 'Rwind': 2.87, 'Alfa_Winding': 0.004308, 'SSpeed': 8000, 'Gspin': 1.7, 'Gtumb': 0.52, 'Gtumb1': 0.0012}

squared_current_f = 1.5
motor_speed_f = 1000

# delta temperature calculation
temp_delta = Rc_Thermal['High_Temp_Threshold'] - Rc_Thermal['Tamb']

#### Losses generated in the motor - steel + joule ####

# steel losses
psteel = ((Rc_Thermal['a1'])*motor_speed_f)+(Rc_Thermal['a2']*(motor_speed_f*motor_speed_f))

# joule losses
qjoule = 3*Rc_Thermal['Rwind']*(float(1.0)+(Rc_Thermal['Alfa_Winding']*temp_delta))*squared_current_f

#### Heat dissipated by the motor to the air ####
if (motor_speed_f > Rc_Thermal['SSpeed']):
    gt = Rc_Thermal['Gspin']
else:
    gt = (Rc_Thermal['Gtumb1']*temp_delta) + Rc_Thermal['Gtumb']


# Dissipated Heat
qdiss = gt*temp_delta

# Net heat stored inside the motor
qnet = qjoule + psteel - qdiss

# Temperature derivative with time
dTdt = qnet*Rc_Thermal['Motor_Thermal_Capacitance']

#Update motor temperature
Rc_Thermal['High_Temp_Threshold'] = Rc_Thermal['High_Temp_Threshold'] + (dTdt*Rc_Thermal['OH_Period'])

time = np.arange(0.0, simulation_time_ms, 1.0/sampling_frequency)

plt.figure()
plt.plot(time, Rc_Thermal['High_Temp_Threshold'], label = 'Overload Temperature', color = 'red')
plt.grid
plt.show()

print('tutto bene')
