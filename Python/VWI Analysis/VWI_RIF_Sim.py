import matplotlib.pyplot as plt
import numpy as np
from ctypes import *


ACCUMULATOR_THR = 12873557.0
ACTIVE_POWER_FACTOR = (3852.0 * 67.0)

#ACCUMULATOR_THR = 12873557.0
#ACTIVE_POWER_FACTOR = (3852.0 * 83.0)

########
# RIF - VWI Simulation
########

# 1- Input is the accumulator to calculate the active power
# 2- Jon's Gephart tuning.
# 3- Eduardo's Tuning.

Simulation = 2


if Simulation == 1:
    
    print("VWI Simulation #1 Running...")
    
    n_accumulator = np.arange(6000000, 25000000, 1000000)
    n_P2P_signal = np.arange(1500, 2250, 50)
    Active_Power = np.zeros((n_accumulator.shape[0], n_P2P_signal.shape[0]), dtype=int)
    
    Low_Limit = 20.0*np.ones(n_P2P_signal.shape[0], dtype=int)
    High_Limit = 27.0*np.ones(n_P2P_signal.shape[0], dtype=int)
    
    print(n_P2P_signal)
 
    plt.figure()
    plt.plot(n_P2P_signal, Low_Limit, color = 'black')  
    plt.plot(n_P2P_signal, High_Limit, color = 'black')  
    
    for k in np.arange(n_accumulator.shape[0]): # for any active power input...
        
        if(n_accumulator[k] <= ACCUMULATOR_THR):
            Mci_Active_Power = 0.0
        else:
            Mci_Active_Power = (n_accumulator[k] - ACCUMULATOR_THR) / (ACTIVE_POWER_FACTOR)
            
        #print(Mci_Active_Power)
            
        for i in np.arange(n_P2P_signal.shape[0]):# for any P2P data input...
          
            temp_volt = (n_P2P_signal[i]/10.0) - 120.0
          
            power_correction = (((115.0*temp_volt) - (2.0*temp_volt*temp_volt))/218.0) + 6.0
          
            if (-power_correction) >= Mci_Active_Power:
                Active_Power[k,i] = 0.0
            else:
                #Active_Power[k,i] = Mci_Active_Power + power_correction 
                #Active_Power[k,i] = Mci_Active_Power 
                Active_Power[k,i] = power_correction 
        
                #print(Active_Power[k,i])    
        #plt.plot(n_P2P_signal, Active_Power[k,:], label='Acc = ' + str(n_accumulator[k]) + ' P_in = ' + str(Mci_Active_Power)) 
        plt.plot(n_P2P_signal, Active_Power[k,:], label='Acc = ' + str(n_accumulator[k])) 
        
        
        
        
        
        
        
    print(Active_Power[2,:])        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
ACCUMULATOR_THR_2 = 12873557.0;        
ACCUMULATOR_THR_2 = 1070000.0;
#ACCUMULATOR_THR_2 = 9400000.0; 
        
        
if Simulation == 2:
    
    print("VWI Simulation #2 Running...")
    
    n_accumulator = np.arange(10000000, 34000000, 7000000)
    n_P2P_signal = np.arange(1500, 2300, 100)
    Active_Power = np.zeros((n_accumulator.shape[0], n_P2P_signal.shape[0]), dtype=int)
 
    plt.figure()
    for k in np.arange(n_accumulator.shape[0]): # for any active power input...                        
        for i in np.arange(n_P2P_signal.shape[0]):# for any P2P data input...

            temp_volt = (1919 - n_P2P_signal[i])*(6880)
            
            #print(temp_volt)
            Mci_Active_Power = (n_accumulator[k] + temp_volt)
            
            #print(Mci_Active_Power)
          
            if(Mci_Active_Power <= ACCUMULATOR_THR_2):
                Mci_Active_Power = 0.0
            else:
                Active_Power[k,i] = (Mci_Active_Power - ACCUMULATOR_THR_2)/(3852.0 * 83.0)
            print(Active_Power[k,i])
            
        plt.plot(n_P2P_signal, Active_Power[k,:], label='Acc = ' + str(n_accumulator[k]) + ' P_comp = ' + str(Mci_Active_Power))   
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
if Simulation == 3:
    
    print("VWI Simulation #1 Running...")
    
    n_accumulator = np.arange(6000000, 7000000, 1000000)
    n_P2P_signal = np.arange(1500, 2250, 50)
    Active_Power = np.zeros((n_accumulator.shape[0], n_P2P_signal.shape[0]), dtype=int)
    
    Low_Limit = 20.0*np.ones(n_P2P_signal.shape[0], dtype=int)
    High_Limit = 27.0*np.ones(n_P2P_signal.shape[0], dtype=int)
    
    print(n_P2P_signal)
 
    plt.figure()
    #plt.plot(n_P2P_signal, Low_Limit, color = 'black')  
    #plt.plot(n_P2P_signal, High_Limit, color = 'black')  
    
    for k in np.arange(n_accumulator.shape[0]): # for any active power input...
        
        if(n_accumulator[k] <= ACCUMULATOR_THR):
            Mci_Active_Power = 0.0
        else:
            Mci_Active_Power = (n_accumulator[k] - ACCUMULATOR_THR) / (3852.0 * 83.0)
            
        #print(Mci_Active_Power)
            
        for i in np.arange(n_P2P_signal.shape[0]):# for any P2P data input...
          
            temp_volt = n_P2P_signal[i] - 1529.0
          
            power_correction = (((-378.0*temp_volt) - (3.0*temp_volt*temp_volt))/65535.0) + 10.0
          
            if (-power_correction) >= Mci_Active_Power:
                Active_Power[k,i] = 0.0
            else:
                #Active_Power[k,i] = Mci_Active_Power + power_correction 
                #Active_Power[k,i] = Mci_Active_Power 
                Active_Power[k,i] = power_correction 
        
                #print(Active_Power[k,i])    
        #plt.plot(n_P2P_signal, Active_Power[k,:], label='Acc = ' + str(n_accumulator[k]) + ' P_in = ' + str(Mci_Active_Power)) 
        plt.plot(n_P2P_signal, Active_Power[k,:], label='Acc = ' + str(n_accumulator[k])) 
        
        
        
        
        
        
        
    #print(Active_Power[2,:])        
        
        
        
        
        
print("Finished!")
plt.legend(loc='upper right', fontsize = 'small')
plt.grid()
plt.show()    