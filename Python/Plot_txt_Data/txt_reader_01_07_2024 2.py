import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
import array

file = open('C:\\Users\\HenriE2\\Downloads\\Log_04_07_2024_load_15L.txt','r')
content=file.readlines()
print(content)
file.close()

content[0:2] = []

time = np.zeros(len(content))
speed_rot_ref = np.zeros(len(content))
speed_rot_mech = np.zeros(len(content))
torque = np.zeros(len(content))
torque_ref = np.zeros(len(content))
is_rms = np.zeros(len(content))
rotor_curr_id = np.zeros(len(content))
rotor_curr_iq = np.zeros(len(content))
flux_ref = np.zeros(len(content))

list_of_strings = np.zeros(10)
list_of_numbers = np.zeros(10)


for i in range(0,len(content)):
    list_of_strings = content[i].split('\t')
    list_of_numbers = [float(j) for j in list_of_strings]
    time[i] = list_of_numbers[0]
    speed_rot_ref[i] = list_of_numbers[1]
    speed_rot_mech[i] = list_of_numbers[2]
    torque[i] = list_of_numbers[3]
    torque_ref[i] = list_of_numbers[4]
    is_rms[i] = list_of_numbers[5]
    rotor_curr_id[i] = list_of_numbers[6]
    rotor_curr_iq[i] = list_of_numbers[7]
    flux_ref[i] = list_of_numbers[8]


plt.figure(1)
plt.plot(time, speed_rot_ref, linewidth=1, linestyle='dashed', label ='Speed Ref', color ='red')
plt.plot(time, speed_rot_mech, linewidth=1, label ='Speed Mech', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(2)
plt.plot(time, torque, linewidth=1, label ='Torque', color ='red')
plt.plot(time, torque_ref, linewidth=1, linestyle='dashed', label ='Torque Ref', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(3)
plt.plot(time, is_rms, linewidth=1, label ='Current Is', color ='red')
plt.plot(time, rotor_curr_id, linewidth=1, label ='Current Id', color ='blue')
plt.plot(time, rotor_curr_iq, linewidth=1, label ='Current Iq', color ='green')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(4)
plt.plot(time, flux_ref, linewidth=1, label ='Flux', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid
plt.show()