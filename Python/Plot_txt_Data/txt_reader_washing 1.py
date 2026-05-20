import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
import array

file = open('osc00014.txt','r')
content=file.readlines()
#print(content)
file.close()

content[0:3] = []

time = np.zeros(len(content))
speed_rot_ref = np.zeros(len(content))
speed_rot_mech = np.zeros(len(content))
torque = np.zeros(len(content))
torque_ref = np.zeros(len(content))
is_rms = np.zeros(len(content))
stator_flux_mag = np.zeros(len(content))
Vdc = np.zeros(len(content))
overheating_temp= np.zeros(len(content))
winding_temp = np.zeros(len(content))

list_of_strings = np.zeros(9)
list_of_numbers = np.zeros(9)


for i in range(0,len(content)):
    list_of_strings = content[i].split('\t')
    list_of_numbers = [float(j) for j in list_of_strings]
    time[i] = list_of_numbers[0]
    speed_rot_mech[i] = list_of_numbers[1]
    speed_rot_ref[i] = list_of_numbers[2]
    torque_ref[i] = list_of_numbers[3]
    torque[i] = list_of_numbers[4]
    is_rms[i] = list_of_numbers[5]
    stator_flux_mag[i] = list_of_numbers[6]
    Vdc[i] = list_of_numbers[7]
    overheating_temp[i] = list_of_numbers[8]
    winding_temp[i] = list_of_numbers[9]


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
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(4)
plt.plot(time, stator_flux_mag, linewidth=1, label ='Stator Flux Mag', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(5)
plt.plot(time, Vdc, linewidth=1, label ='Vdc', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(6)
plt.plot(time, overheating_temp, linewidth=1, label ='Overheating temperature', color ='red')
plt.plot(time, winding_temp, linewidth=1, label ='Winding temperature', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid
plt.show()