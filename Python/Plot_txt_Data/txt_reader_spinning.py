import matplotlib.pyplot as plt
import numpy as np
from ctypes import *
import math
import array

file = open('Test_.txt','r')
content=file.readlines()
#print(content)
file.close()

content[0:3] = []

time = np.zeros(len(content))
speed_rot_mech = np.zeros(len(content))
speed_rot_ref = np.zeros(len(content))
torque_controller_upper_limit = np.zeros(len(content))
torque_controller_out = np.zeros(len(content))
torque_controller_lower_limit = np.zeros(len(content))
electrical_power = np.zeros(len(content))
flux_controller_upper_limit = np.zeros(len(content))
flux_controller_out = np.zeros(len(content))
flux_controller_lower_limit = np.zeros(len(content))
rotor_current_iq = np.zeros(len(content))
rotor_current_id = np.zeros(len(content))
Vdc = np.zeros(len(content))



list_of_strings = np.zeros(9)
list_of_numbers = np.zeros(9)


for i in range(0,len(content)):
    list_of_strings = content[i].split('\t')
    list_of_numbers = [float(j) for j in list_of_strings]
    time[i] = list_of_numbers[0]
    speed_rot_mech[i] = list_of_numbers[1]
    speed_rot_ref[i] = list_of_numbers[2]
    electrical_power[i] = list_of_numbers[3]
    flux_controller_upper_limit[i] = list_of_numbers[4]
    flux_controller_out[i] = list_of_numbers[5]
    flux_controller_lower_limit[i] = list_of_numbers[6]
    torque_controller_upper_limit[i] = list_of_numbers[7]
    torque_controller_out[i] = list_of_numbers[8]
    torque_controller_lower_limit[i] = list_of_numbers[9]
    Vdc[i] = list_of_numbers[10]
    rotor_current_id[i] = list_of_numbers[11]
    rotor_current_iq[i] = list_of_numbers[12]


plt.figure(1)
plt.plot(time, speed_rot_ref, linewidth=1, linestyle='dashed', label ='Speed Ref', color ='red')
plt.plot(time, speed_rot_mech, linewidth=1, label ='Speed Mech', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(2)
plt.plot(time, electrical_power, linewidth=1, label ='Electrical Power', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid

plt.figure(3)
plt.plot(time, flux_controller_upper_limit, linewidth=1, label ='Flux Controller Upper Limit', color ='red')
plt.plot(time, flux_controller_out, linewidth=1, linestyle='dashed', label ='Flux Controller Out', color ='blue')
plt.plot(time, flux_controller_lower_limit, linewidth=1, label ='Flux Controller Lower Limit', color ='yellow')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(4)
plt.plot(time, torque_controller_upper_limit, linewidth=1, label ='Torque Controller Upper Limit', color ='red')
plt.plot(time, torque_controller_out, linewidth=1, linestyle='dashed', label ='Torque Controller Out', color ='blue')
plt.plot(time, torque_controller_lower_limit, linewidth=1, label ='Torque Controller Lower Limit', color ='yellow')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid

plt.figure(5)
plt.plot(time, Vdc, linewidth=1, label ='Vdc', color ='red')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid


plt.figure(6)
plt.plot(time, rotor_current_id, linewidth=1, label ='Current id', color ='red')
plt.plot(time, rotor_current_iq, linewidth=1, label ='Current iq', color ='blue')
plt.legend(loc='upper right', fontsize = 'small')
plt.grid
plt.show()