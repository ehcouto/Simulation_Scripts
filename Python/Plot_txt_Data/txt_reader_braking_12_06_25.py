import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
import array

file = open('Test2_acc50.txt','r')
content=file.readlines()
#print(content)
file.close()

content[0:3] = []

time = np.zeros(len(content))
speed_rot_ref = np.zeros(len(content))
speed_rot_mech = np.zeros(len(content))
torque = np.zeros(len(content))
torque_ref = np.zeros(len(content))
stator_flux_mag = np.zeros(len(content))
Vdc = np.zeros(len(content))
torque_brake_max = np.zeros(len(content))
rotor_curr_iq = np.zeros(len(content))
rotor_curr_id = np.zeros(len(content))
id_braking = np.zeros(len(content))
is_abs = np.zeros(len(content))
stator_resistance = np.zeros(len(content))


list_of_strings = np.zeros(12)
list_of_numbers = np.zeros(12)


for i in range(0,len(content)):
    list_of_strings = content[i].split('\t')
    list_of_numbers = [float(j) for j in list_of_strings]
    time[i] = list_of_numbers[0]
    speed_rot_mech[i] = list_of_numbers[1]
    speed_rot_ref[i] = list_of_numbers[2]
    torque_ref[i] = list_of_numbers[3]
    torque[i] = list_of_numbers[4]
    torque_brake_max[i] = list_of_numbers[5]
    stator_flux_mag[i] = list_of_numbers[6]
    Vdc[i] = list_of_numbers[7]
    rotor_curr_iq[i] = list_of_numbers[8]
    rotor_curr_id[i] = list_of_numbers[9]
    id_braking[i] = list_of_numbers[10]
    is_abs[i] = list_of_numbers[11]
    stator_resistance[i] = list_of_numbers[12]


plt.figure(1)
plt.plot(time, speed_rot_ref, linewidth=1, linestyle='dashed', label ='Speed Ref', color ='red')
plt.plot(time, speed_rot_mech, linewidth=1, label ='Speed Mech', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(2)
plt.plot(time, torque, linewidth=1, label ='Torque', color ='red')
plt.plot(time, torque_ref, linewidth=1, linestyle='dashed', label ='Torque Ref', color ='blue')
plt.plot(time, torque_brake_max, linewidth=1, label ='Torque Brake Max', color ='green')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(3)
plt.plot(time, rotor_curr_iq, linewidth=1, label ='Current Iq', color ='red')
plt.plot(time, rotor_curr_id, linewidth=1, label ='Current Id', color ='blue')
plt.plot(time, id_braking, linewidth=1, label ='Id Braking', color ='black')
plt.plot(time, is_abs, linewidth=1, label ='Current Is', color ='green')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid

plt.figure(4)
plt.plot(time, stator_flux_mag, linewidth=1, label ='stator_flux_mag', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid

plt.figure(5)
plt.plot(time, Vdc, linewidth=1, label ='Vdc', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid

plt.figure(6)
plt.plot(time, stator_resistance, linewidth=1, label ='Stator Resistance', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid
plt.show()