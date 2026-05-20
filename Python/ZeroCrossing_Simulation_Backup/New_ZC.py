'''
Created on 23 de jan de 2017

@author: SILVAMC
'''

import pylab
import scipy
import scipy.signal
import matplotlib.pyplot as plt
import numpy as np
import math as mt
import sys
import csv
import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt
from ctypes import *

#plt.switch_backend('Qt4Agg')


# const sint32 Coeff_Butterworth_Num[BUTTER_FILTER_NUM] =
# {
#     { FILTER_COEFF_TO_IQ(0.0000132937289) }, // 0
#     { FILTER_COEFF_TO_IQ(0.0000531749156) }, // 1
#     { FILTER_COEFF_TO_IQ(0.0000797623734) }, // 2
#     { FILTER_COEFF_TO_IQ(0.0000531749156) }, // 3
#     { FILTER_COEFF_TO_IQ(0.0000132937289) }, // 4
# };
# 
# const sint32 Coeff_Butterworth_Den[BUTTER_FILTER_DEN] =
# {
#     { FILTER_COEFF_TO_IQ(-3.67172909) }, // 0
#     { FILTER_COEFF_TO_IQ(5.067998390) }, // 1
#     { FILTER_COEFF_TO_IQ(-3.11596693) }, // 2
#     { FILTER_COEFF_TO_IQ(0.719910330) }, // 3
# };


Select_Float = 0  ## 1- Float / 0- Fixed Point

if (Select_Float == 1):
    mydll=cdll.LoadLibrary("C:\Data\Projects\PythonProjects\ZeroCrossing_Simulation\DLL\ZeroCrossSim\Debug\ZeroCrossSim_float.dll")
    mydll.New_ZC_Filter.argtypes = [c_float]
    mydll.New_ZC_Filter.restype = c_float
    print("\nDLL float loaded!")
else:
    mydll=cdll.LoadLibrary("C:\Data\Projects\PythonProjects\ZeroCrossing_Simulation\DLL\MciZeroCross\Debug\MciZeroCross.dll")
    mydll.New_ZC_Filter.argtypes = [c_int]
    mydll.New_ZC_Filter.restype = c_int
#     mydll.New_ZC_GetOutput.argtypes = [c_int]
#     mydll.New_ZC_GetOutput.restype = c_int
    print("\nDLL fixed point loaded!")



def Design_Filter(sampling_frequency, plot_data = False):

    print("\nCalculating filter...")


#     selec_filter = "ellipitc low pass"
#    selec_filter = "butterworth band pass"
#     selec_filter = "kaiser low pass"
    selec_filter = "butterworth low pass"
    
    print("\nSelected Filter is: " + str(selec_filter))

    if selec_filter == "ellipitc low pass":
    
        ###############################################################################################
        ###############################################################################################
        ###############################################################################################
        ################### elliptic filter
        N_order = 3                     # int: The order of the filter.
        ripple_pass_band = 0.01         # float: The maximum ripple allowed below unity gain in the passband. 
                                        # Specified in decibels, as a positive number.
        ripple_stop_band = 120.0        # float: The minimum attenuation required in the stop band. Specified 
                                        # in decibels, as a positive number.
        pass_band_edge = 50.0           # pass band edge in Hertz
    
    
        temp = pass_band_edge/sampling_frequency
        num, den = scipy.signal.ellip(N_order, 
                                      ripple_pass_band,
                                      ripple_stop_band, 
                                      temp, 
                                      output='ba');
    


    if selec_filter == "butterworth low pass":
        ###############################################################################################
        ###############################################################################################
        ###############################################################################################
        ################### Butterworth filter - LOW PASS FILTER
        # Filter requirements.
        cutoff = 80.0  # desired cutoff frequency of the filter, Hz
        N_order = 4
#         sampling_frequency = 4000.0
    
        # Get the filter coefficients so we can check its frequency response.
        # low pass butterworth filter
    
        normal_cutoff = cutoff /  (0.5 * sampling_frequency)
        num, den = butter(N_order, normal_cutoff, btype='low', analog=False)



    if selec_filter == "butterworth band pass":
        ##############################################################################################
        ##############################################################################################
        ##############################################################################################
        ################## Butterworth filter - BAND PASS FILTER
        ## Filter requirements.
        N_order = 3
        
        low_cut_freq = 15.0
        high_cut_freq = 50.0
        
        low = low_cut_freq / (0.5 * sampling_frequency)
        high = high_cut_freq / (0.5 * sampling_frequency)
        num, den = butter(N_order, [low, high], btype='band')


    if selec_filter == "kaiser low pass":
        ##############################################################################################
        ##############################################################################################
        ##############################################################################################
        ################## Kaiser filter
        N_order = 240
        cutoff = 110.0
        num = scipy.signal.firwin(N_order, cutoff, window=('kaiser', 8), nyq = (sampling_frequency/2.0) )
        den = 1
        w, h = scipy.signal.freqz(num)



    print("Filter numerator: " + str(num))
    print("Filter denominatorr: " + str(den))


    if plot_data == True:
        fig = plt.figure()
       
#         plt.title('Zero Crossing Ellipse filter response\nOrder:' + str(N_order) + "- ripple band:" + str(ripple_pass_band) + "dB - ripple stop band: " + str(ripple_stop_band) + "dB - Sample freq: " + str(sampling_frequency) + "Hz") 
        plt.title('Zero Crossing Ellipse filter response\nOrder:' + str(N_order) + " - Sample freq: " + str(sampling_frequency) + "Hz") 
       
        ax1 = fig.add_subplot(111)
        h,w = scipy.signal.freqz(num, den)
           
        plt.axvline(50.0, color='k')
#         plt.axvline(100.0, color='k')
#         plt.axvline(150.0, color='k')
#         plt.axvline(200.0, color='k')
#         plt.axvline(250.0, color='k')
#         plt.axvline(300.0, color='k')
#         plt.axvline(350.0, color='k')
#         plt.axvline(400.0, color='k')
#         plt.axvline(450.0, color='k')
#         
        plt.axvline(60.0, color='k')
#         plt.axvline(180.0, color='k')
#         plt.axvline(240.0, color='k')
#         plt.axvline(300.0, color='k')
#         plt.axvline(360.0, color='k')
#         plt.axvline(420.0, color='k')
#         plt.axvline(480.0, color='k')
        
        plt.plot( (h*(sampling_frequency/np.pi)), np.abs(w), 'b')
        # plt.semilogy( (h*(sampling_frequency/np.pi)), np.abs(w), 'b')
        
        plt.ylabel('Amplitude (dB)', color='b')
        plt.xlabel('Frequency (Hz)')
        plt.grid()
        plt.legend()
        ax2 = ax1.twinx()
           
        ax1.set_xlim([0, 500])
        ax2.set_xlim([0, 500])
           
        angles = np.angle(w)
        angles = np.unwrap(np.angle(w))
        plt.plot((h*(sampling_frequency/np.pi)), angles*30/np.pi, 'g')
        plt.ylabel('Angle (degrees)', color='g')

#         plt.show()
        
    return num, den











## Simulate the filter
def Convert_Input_Voltage_in_Digital_Input_Signal(time, signal, signal_assymetry):
    
    print("\nConverting input voltage into a digital input signal...")
    
    zero_average = 0.5
    
    digital_input_ZC_signal = np.zeros(time.shape[0])
    for index in np.arange(0, time.shape[0]):
        
        if signal[index] > signal_assymetry:
            digital_input_ZC_signal[index] = 1.0
    
    digital_input_ZC_signal = digital_input_ZC_signal - zero_average # remove the offset
    
    
    print("\n\nAveraged voltage - 0 means 50% duty cycle:", np.average(digital_input_ZC_signal))
    print("\n")
    
    return digital_input_ZC_signal 
def Read_Input_File_Voltage(file_path, sampling_frequency, period_multiplier, plot_data = False):
    
    print("\nReading data from file...")
    
    with open(file_path,'r') as dest_f:
        data_iter = csv.reader(dest_f, 
                           delimiter = ',', 
                           quotechar = '"')

        header_line_1 = np.asarray(data_iter.next(), dtype = None)

        data = [data for data in data_iter]
        data_array = np.asarray(data, dtype = None)
        data_array = data_array.astype(np.float)

        period_multiplier = period_multiplier + 1

        file_time = data_array[20:,0]
        file_volt = data_array[20:,1]
        
        file_temp_volt = file_volt
        file_temp_time = file_time
        time_offset = file_time[file_time.shape[0] - 1]
        for period_counter in np.arange(2, period_multiplier):        

            file_temp_volt = np.hstack( (file_temp_volt, file_volt) )

            temp = file_time + time_offset
            file_temp_time = np.hstack( (file_temp_time, temp ) )
            time_offset = period_counter * file_time[file_time.shape[0] - 1]

        file_time = file_temp_time
        file_volt = file_temp_volt

        file_sampling_frequency = 1.0/(file_time[2] - file_time[1])

        skip_points = int(file_sampling_frequency/sampling_frequency)
#         print "skip_points " + str(skip_points)
         
        nr_of_rows = mt.ceil(float(file_volt.shape[0])/skip_points)
#         print "file volt size " + str(file_volt.shape[0])
#         print "file volt resize " + str(nr_of_rows * skip_points)

        zero_pad = int((nr_of_rows * skip_points) - file_volt.shape[0]) 
        temp_time = np.pad(file_time, pad_width=zero_pad, mode='constant', constant_values=0)[zero_pad:]
        temp_volt = np.pad(file_volt, pad_width=zero_pad, mode='constant', constant_values=0)[zero_pad:]

        down_sampled_time = np.reshape(temp_time, (nr_of_rows, skip_points))
        down_sampled_volt = np.reshape(temp_volt, (nr_of_rows, skip_points))

#         print "file_time " + str(file_time)
#         print "d_time " + str(down_sampled_time.shape[0])


        print('\n')
        print("Sampling time found [s]: " + str(1.0/file_sampling_frequency))
        print("Sampling frequency found [Hz]: " + str(file_sampling_frequency))

        if plot_data == True:
            plt.switch_backend('qt4agg')
            plt.figure()
            plt.title("Input voltage read from file: " + str(file_path))
            plt.plot(file_time, file_volt, label='from file - 60 kHz', color='blue')
#             plt.plot(down_sampled_time[:,0], down_sampled_volt[:,0], label='down sampled - 4kHz', marker='o', color='red')
            
            plt.ylabel('Voltage [V]')
            plt.xlabel('Time [s]')
            plt.legend(loc='upper right', fontsize = 'small')
            plt.grid()

    fundamental = np.zeros(down_sampled_time.shape[0])

    return down_sampled_time[:,0], fundamental, down_sampled_volt[:,0]











## Create or read the file with input voltage 
def Create_Input_Voltage(Vmains_rms, freq_mains_Hz, harmonic_gains, harmonic_phases, sampling_frequency, plot_data = False, simulation_time_sec = (1.0/60.0) * 3.0):
    
    print("Creating input voltage.")
    
    time = np.arange(0.0, simulation_time_sec, 1.0/sampling_frequency)
    fundamental  = Vmains_rms * np.sin(2 * np.pi * freq_mains_Hz * time)
    
    harmonic = np.zeros((Gain_Harmonic.shape[0], fundamental.shape[0]))
    total_input_voltage = fundamental
    
    if plot_data == True:
        plt.figure()
        plt.plot(time, fundamental, label='Fundamental voltage signal [V]', color='red', linestyle='--')
        

    for index in np.arange(0, harmonic_gains.shape[0]):
        harmonic[index] = harmonic_gains[index] * Vmains_rms/(index+2.0) * np.sin(  ((index+2.0)  * 2 * np.pi * freq_mains_Hz * time) + harmonic_phases[index] * (np.pi/180.0)  )
        total_input_voltage = total_input_voltage + harmonic[index]
        
        if np.any(harmonic[index] > 0):
            if plot_data == True:
                plt.plot(time, harmonic[index], label='Harmonic: ' + str(index + 2), color='red', linestyle='--')
                  
   
            
    if plot_data == True:
        plt.title("Input voltage: fundamental + harmonics\nVotage mains: " + str(Vmains_rms) + "V - frequency mains: " + str(freq_mains_Hz) + "Hz")
        plt.plot(time, total_input_voltage, label='Input voltage [V]', color='blue')
        plt.legend(loc='upper right', fontsize = 'small')
        plt.grid()
    
    
    return time, fundamental, total_input_voltage
def Filter_Input_Signal(filter_z_num, filter_z_den, input_signal, plot_data = False):
    filtered_signal = scipy.signal.filtfilt(filter_z_num, filter_z_den, input_signal)
    return filtered_signal
    
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
#####################################################################################################################

























# sampling_frequency = 200000.0     # sampling frequency in Hertz
# sampling_frequency = 10000.0     # sampling frequency in Hertz
# sampling_frequency = 40000.0     # sampling frequency in Hertz
# sampling_frequency = 4000.0     # 250 us - sampling frequency in Hertz



sampling_frequency = 4800.0     # 200 us - sampling frequency in Hertz
# sampling_frequency = 10000.0     # 100 us - sampling frequency in Hertz
# sampling_frequency = 20000.0     # sampling frequency in Hertz
# sampling_frequency = 200000.0     # sampling frequency in Hertz


filter_num, filter_den = Design_Filter(sampling_frequency, plot_data = False)


Gain_Harmonic = np.array([
                            0.0,    ## Gain for 2nd harmonic 
                            0.0,    ## Gain for 3rd harmonic
                            0.0,    ## Gain for 4th harmonic
                            0.0,    ## Gain for 5th harmonic
                            0.0,    ## Gain for 6th harmonic
                            0.0,    ## Gain for 7th harmonic
                            0.0,    ## Gain for 8th harmonic
                            0.0,    ## Gain for 9th harmonic
                            
                            0.0,    ## Gain for 10th harmonic
                            0.0,    ## Gain for 11th harmonic
                            0.0,    ## Gain for 12th harmonic
                            0.0,    ## Gain for 13th harmonic
                            0.0,    ## Gain for 14th harmonic
                            0.0,    ## Gain for 15th harmonic
                            0.0,    ## Gain for 16th harmonic
                            0.0,    ## Gain for 17th harmonic
                            0.0,    ## Gain for 18th harmonic
                            0.0,    ## Gain for 19th harmonic
                            
                            0.0,    ## Gain for 20th harmonic
                            0.0,    ## Gain for 21st harmonic
                            0.0,    ## Gain for 22nd harmonic
                            0.0,    ## Gain for 23rd harmonic
                            0.0,    ## Gain for 24th harmonic
                            0.0,    ## Gain for 25th harmonic
                            0.0,    ## Gain for 26th harmonic
                            0.0,    ## Gain for 27th harmonic
                            0.0,    ## Gain for 28th harmonic
                            0.0,    ## Gain for 29th harmonic

                            5.0,    ## Gain for 30th harmonic
                            0.0,    ## Gain for 31th harmonic
                            0.0,    ## Gain for 32th harmonic
                            0.0,    ## Gain for 33th harmonic
                            0.0,    ## Gain for 34th harmonic
                            0.0,    ## Gain for 35th harmonic
                            0.0,    ## Gain for 36th harmonic
                            0.0,    ## Gain for 37th harmonic
                            0.0,    ## Gain for 38th harmonic
                            0.0,    ## Gain for 39th harmonic
                            
                            0.0,    ## Gain for 40th harmonic
                            0.0,    ## Gain for 41st harmonic
                            0.0,    ## Gain for 42nd harmonic
                            0.0,    ## Gain for 43rd harmonic
                            0.0,    ## Gain for 44th harmonic
                            0.0,    ## Gain for 45th harmonic
                            0.0,    ## Gain for 46th harmonic
                            0.0,    ## Gain for 47th harmonic
                            0.0,    ## Gain for 48th harmonic
                            0.0,    ## Gain for 49th harmonic

                            0.0,    ## Gain for 50th harmonic
                            0.0,    ## Gain for 51th harmonic
                            0.0,    ## Gain for 52th harmonic
                            0.0,    ## Gain for 53th harmonic
                            0.0,    ## Gain for 54th harmonic
                            0.0,    ## Gain for 55th harmonic
                            0.0,    ## Gain for 56th harmonic
                            0.0,    ## Gain for 57th harmonic
                            0.0,    ## Gain for 58th harmonic
                            0.0,    ## Gain for 59th harmonic
                            
                            0.0,    ## Gain for 60th harmonic
                            0.0,    ## Gain for 61st harmonic
                            0.0,    ## Gain for 62nd harmonic
                            0.0,    ## Gain for 63rd harmonic
                            0.0,    ## Gain for 64th harmonic
                            0.0,    ## Gain for 65th harmonic
                            0.0,    ## Gain for 66th harmonic
                            0.0,    ## Gain for 67th harmonic
                            0.0,    ## Gain for 68th harmonic
                            0.0,    ## Gain for 69th harmonic
                            
                            0.0     ## Gain for 70th harmonic
                          ])


Phase_Harmonic = np.array([
                            0.0,    ## Phase in degree for 2nd harmonic 
                            0.0,    ## Phase in degree for 3rd harmonic
                            0.0,    ## Phase in degree for 4th harmonic
                            0.0,    ## Phase in degree for 5th harmonic
                            0.0,    ## Phase in degree for 6th harmonic
                            0.0,    ## Phase in degree for 7th harmonic
                            0.0,    ## Phase in degree for 8th harmonic
                            180.0,    ## Phase in degree for 9th harmonic
                            
                            0.0,    ## Phase in degree for 10th harmonic
                            0.0,    ## Phase in degree for 11th harmonic
                            0.0,    ## Phase in degree for 12th harmonic
                            0.0,    ## Phase in degree for 13th harmonic
                            0.0,    ## Phase in degree for 14th harmonic
                            0.0,    ## Phase in degree for 15th harmonic
                            0.0,    ## Phase in degree for 16th harmonic
                            0.0,    ## Phase in degree for 17th harmonic
                            0.0,    ## Phase in degree for 18th harmonic
                            0.0,    ## Phase in degree for 19th harmonic
                            
                            0.0,    ## Phase in degree for 20th harmonic
                            0.0,    ## Phase in degree for 21st harmonic
                            0.0,    ## Phase in degree for 22nd harmonic
                            0.0,    ## Phase in degree for 23rd harmonic
                            0.0,    ## Phase in degree for 24th harmonic
                            0.0,    ## Phase in degree for 25th harmonic
                            0.0,    ## Phase in degree for 26th harmonic
                            0.0,    ## Phase in degree for 27th harmonic
                            0.0,    ## Phase in degree for 28th harmonic
                            0.0,    ## Phase in degree for 29th harmonic
                            
                            0.0,    ## Phase in degree for 30th harmonic
                            0.0,    ## Phase in degree for 31th harmonic
                            0.0,    ## Phase in degree for 32th harmonic
                            0.0,    ## Phase in degree for 33th harmonic
                            0.0,    ## Phase in degree for 34th harmonic
                            0.0,    ## Phase in degree for 35th harmonic
                            0.0,    ## Phase in degree for 36th harmonic
                            0.0,    ## Phase in degree for 37th harmonic
                            0.0,    ## Phase in degree for 38th harmonic
                            0.0,    ## Phase in degree for 39th harmonic
                            
                            0.0,    ## Phase in degree for 40th harmonic
                            0.0,    ## Phase in degree for 41st harmonic
                            0.0,    ## Phase in degree for 42nd harmonic
                            0.0,    ## Phase in degree for 43rd harmonic
                            0.0,    ## Phase in degree for 44th harmonic
                            0.0,    ## Phase in degree for 45th harmonic
                            0.0,    ## Phase in degree for 46th harmonic
                            0.0,    ## Phase in degree for 47th harmonic
                            0.0,    ## Phase in degree for 48th harmonic
                            0.0,    ## Phase in degree for 49th harmonic
                            
                            0.0,    ## Phase in degree for 50th harmonic
                            0.0,    ## Phase in degree for 51th harmonic
                            0.0,    ## Phase in degree for 52th harmonic
                            0.0,    ## Phase in degree for 53th harmonic
                            0.0,    ## Phase in degree for 54th harmonic
                            0.0,    ## Phase in degree for 55th harmonic
                            0.0,    ## Phase in degree for 56th harmonic
                            0.0,    ## Phase in degree for 57th harmonic
                            0.0,    ## Phase in degree for 58th harmonic
                            0.0,    ## Phase in degree for 59th harmonic
                            
                            0.0,    ## Phase in degree for 60th harmonic
                            0.0,    ## Phase in degree for 61st harmonic
                            0.0,    ## Phase in degree for 62nd harmonic
                            0.0,    ## Phase in degree for 63rd harmonic
                            0.0,    ## Phase in degree for 64th harmonic
                            0.0,    ## Phase in degree for 65th harmonic
                            0.0,    ## Phase in degree for 66th harmonic
                            0.0,    ## Phase in degree for 67th harmonic
                            0.0,    ## Phase in degree for 68th harmonic
                            0.0,    ## Phase in degree for 69th harmonic
                            
                            0.0     ## Phase in degree for 70th harmonic
                          ])





mains_frequency = 57.0
mains_frequency = 63.0
mains_frequency = 60.0

nr_of_input_cycles = 6.0



time, fundamental, intput_voltage_wave_form = Create_Input_Voltage(120, mains_frequency, Gain_Harmonic, Phase_Harmonic, sampling_frequency, plot_data = False, simulation_time_sec=((1.0/mains_frequency) * nr_of_input_cycles))
# time, fundamental, intput_voltage_wave_form = Read_Input_File_Voltage('Biguacu_Voltage.csv', sampling_frequency, period_multiplier = 4, plot_data = True)


# signal_assymetry = 0.0      # no assimetry
# signal_assymetry = 65.0     
# signal_assymetry = -65.0     
signal_assymetry = -11.3 # close to 3% assimetry     
digital_input_ZC_signal = Convert_Input_Voltage_in_Digital_Input_Signal(time, intput_voltage_wave_form, signal_assymetry=signal_assymetry)




filtered_signal = Filter_Input_Signal(filter_num, filter_den, 256.0 * digital_input_ZC_signal)


# graph_y_lim = 150.0
#
# plt.figure()
# plt.title('Signals Input and Output')
# plt.plot(time, intput_voltage_wave_form, label='Total Voltage signal [V]', color='red')
# if np.any(fundamental > 0):
#     plt.plot(time, fundamental, label='Fundamental Voltage signal [V]', color='purple')
# plt.plot(time, digital_input_ZC_signal, label='Digital input signal', color='green')
# plt.plot(time, filtered_signal, marker='o', label='Filtered signal', color='blue')
# plt.ylim((-graph_y_lim, graph_y_lim))
# plt.legend(loc='upper right', fontsize = 'small')
# plt.grid()










b = filter_num
a = filter_den
a_coeff = a[1:a.shape[0]]
print("a_coeff\n", a_coeff)
print("b\n", b)
print("\n")



Q_INPUT = 10
Q_FILTER = 19 #(31 - Q_INPUT)
Q_A_FILTER_BASE = 1.0
Q_B_FILTER_BASE = Q_A_FILTER_BASE

a_fp = (a/Q_A_FILTER_BASE)*(2.0**Q_FILTER)
b_fp = (b/Q_B_FILTER_BASE)*(2.0**Q_FILTER)

a_fp = a_fp.astype(int)
b_fp = b_fp.astype(int)
a_fp_coeff = a_fp[1:a_fp.shape[0]]
zc_signal_out = np.zeros(time.shape[0])
zc_signal_event = np.zeros(time.shape[0])

print("a_fp_coeff\n", a_fp_coeff)
print("b_fp\n", b_fp)
print("")

y_f = np.zeros(a_coeff.shape[0])
x_s = np.zeros(b.shape[0])
y_f_data = np.zeros(time.shape[0])
y_f_data_dll_fp = np.zeros(time.shape[0])
zc_data = np.zeros(time.shape[0])
zc_step_hist = np.zeros(time.shape[0])

neg_threshold_lobe =  515.0
pos_threshold_lobe = -485.0
neg_threshold_lobe_fp = np.int(  501.0*((2**Q_FILTER)/Q_A_FILTER_BASE)) 
pos_threshold_lobe_fp = np.int( -467.0*((2**Q_FILTER)/Q_A_FILTER_BASE)) 

if signal_assymetry == -11.3: # close to 3% assimetry
    neg_threshold_lobe =  525.0
    pos_threshold_lobe = -468.0
    neg_threshold_lobe_fp = np.int(  500.0*((2**Q_FILTER)/Q_A_FILTER_BASE))
    pos_threshold_lobe_fp = np.int( -442.0*((2**Q_FILTER)/Q_A_FILTER_BASE))

if signal_assymetry == 65.0:
    neg_threshold_lobe =  260.0
    pos_threshold_lobe = -598.0
    neg_threshold_lobe_fp = np.int(  256.0*((2**Q_FILTER)/Q_A_FILTER_BASE))
    pos_threshold_lobe_fp = np.int( -579.0*((2**Q_FILTER)/Q_A_FILTER_BASE))

if signal_assymetry == -65.0:
    neg_threshold_lobe =  609.0
    pos_threshold_lobe = -253.0
    neg_threshold_lobe_fp = np.int(  610.0*((2**Q_FILTER)/Q_A_FILTER_BASE))
    pos_threshold_lobe_fp = np.int( -293.0*((2**Q_FILTER)/Q_A_FILTER_BASE))



zc_step = 1
zc_step_fp = 1

averaged_freq_hyst_fp = np.zeros(time.shape[0])
out_duty_fp_prev = False 
out_duty_fp = False
Cycle_Sample_Counter_fp = 0
Averaged_Freq_fp = 0

averaged_freq_hyst = np.zeros(time.shape[0])
out_duty_prev = False 
out_duty = False
Cycle_Sample_Counter = 0
Averaged_Freq = 0

y_f_fp = np.zeros(a_fp_coeff.shape[0])
x_s_fp = np.zeros(b_fp.shape[0])
x_s_fp_dll = np.zeros(b_fp.shape[0])

# print "\ny_f size ", y_f.shape[0]
# print "x_s size ", x_s.shape[0]

y_out = 0.0
y_out_fp = 0

sum_num_data = np.zeros(time.shape[0])
sum_num_data_fp = np.zeros(time.shape[0])

sum_den_data = np.zeros(time.shape[0])
sum_den_data_fp = np.zeros(time.shape[0])

y_f_data_fp = np.zeros(time.shape[0])
zc_data_fp = np.zeros(time.shape[0])
zc_step_hist_fp = np.zeros(time.shape[0]) 


digital_input_ZC_signal_dll = (digital_input_ZC_signal) + 0.5
digital_input_ZC_signal = digital_input_ZC_signal * (2.0**Q_INPUT)

if (Select_Float == 1):
    response = mydll.New_ZC_Init()
    if (  response == 1):
        response = 'Float Succesfull initialized!'
    print(response)
else:
    response = mydll.New_ZC_Init()
    if (  response == 1):
        response = 'Fixed Point Succesfull initialized!'
        print(response)

# This for loop shall be the ISR code running at sampling frequency
for isr_i in np.arange(digital_input_ZC_signal.shape[0]):

  
    #####################################################################################
    #####################################################################################
    #####################################################################################
    #####################################################################################
    #####################################################################################
    #####################################################################################
    ################# FIXED POINT point implementation

    
    x_s_fp[0] = digital_input_ZC_signal[isr_i] # read gpio
    x_s_fp_dll[0] = digital_input_ZC_signal_dll[isr_i]
    y_f_fp[0] = y_out_fp   # output the output filter

    sum_num_fp = 0
    for i in np.arange(b_fp.shape[0]):
        temp = np.int(b_fp[i] * x_s_fp[i])
        if(temp >= (2.0**31.0)):
            pass
            print("Oveflow Num!!", temp)

        sum_num_fp = sum_num_fp + temp
        
        if(sum_num_fp >= (2.0**31)):
            pass
            print("sum_num_fp", sum_num_fp)

    sum_den_fp = 0
    #print a_fp_coeff.shape[0]
    for i in np.arange(a_fp_coeff.shape[0]):
        temp = np.int(a_fp_coeff[i]*y_f_fp[i])
#         if(temp >= (2.0**31.0)):
#             pass
#             print "Overflow Den!!", (temp) 
        if(temp >= (2.0**63.0)):
            pass
            print("Oveflow 64_bits Num!!", temp)  
        temp = temp/(2**(Q_FILTER))
        sum_den_fp = sum_den_fp + temp

        if(sum_den_fp >= (2.0**31)):
            pass
            print("sum_den_fp", sum_den_fp)


    sum_num_data_fp[isr_i] = sum_num_fp
    sum_den_data_fp[isr_i] = sum_den_fp

    y_out_fp = sum_num_fp - sum_den_fp # Q27 = Q27 + Q27
    
    if(y_out_fp >= (2.0**31.0)):
        pass
        print("y_out_fp", y_out_fp)


    for i in np.arange((x_s_fp.shape[0]-1), 0, -1):
        x_s_fp[i] = x_s_fp[i-1]

    for i in np.arange((y_f_fp.shape[0]-1), 0, -1):
        y_f_fp[i] = y_f_fp[i-1]


    if (Select_Float == 0):
        filter_response = mydll.New_ZC_Filter(np.int(x_s_fp_dll[0]))
        #print output_response
        zc_signal_event[isr_i] = filter_response
#         print "DLL", filter_response
#         print "Python:",  sum_den_fp

    if zc_step_fp == 1:
        if y_out_fp <= neg_threshold_lobe_fp:
            zc_step_fp = 2
    elif zc_step_fp == 2:
        if y_out_fp >= neg_threshold_lobe_fp:
            ## ZC event!!!
            zc_data_fp[isr_i] = 100.0
            zc_step_fp = 3
    
    elif zc_step_fp == 3:
        if y_out_fp >= pos_threshold_lobe_fp:
            zc_step_fp = 4
    elif zc_step_fp == 4:
        if y_out_fp <= pos_threshold_lobe_fp:
            ## ZC event !!!
            zc_data_fp[isr_i] = 100.0
            zc_step_fp = 1
    else:
        zc_step_fp = 0
 



    if y_out_fp >= 200:
        out_duty_fp = True
    else:
        out_duty_fp = False


    Cycle_Sample_Counter_fp = Cycle_Sample_Counter_fp + 1
    
    if (out_duty_fp == True) and (out_duty_fp_prev == False):
        if Cycle_Sample_Counter_fp != 0:
            Averaged_Freq_fp = np.int((2**9)*sampling_frequency/Cycle_Sample_Counter_fp)

        Cycle_Sample_Counter_fp = 0
        
        out_duty_fp_prev = True
    elif (out_duty_fp == False) and (out_duty_fp_prev == True):
        
        out_duty_fp_prev = False



    y_f_data_fp[isr_i] = y_out_fp
    zc_step_hist_fp[isr_i] = zc_step_fp
    averaged_freq_hyst_fp[isr_i] = np.float(Averaged_Freq_fp)/(2**9)




    #####################################################################################
    #####################################################################################
    #####################################################################################
    #####################################################################################
    #####################################################################################
    #####################################################################################
    ################# Floating point implementation
    
    x_s[0] = digital_input_ZC_signal[isr_i] # read gpio
    y_f[0] = y_out   # output the output filter


    sum_num = 0.0
    for i in np.arange(b.shape[0]):
        sum_num = sum_num + (b[i] * x_s[i])

    sum_den = 0.0
    for i in np.arange(a_coeff.shape[0]):
        sum_den = sum_den + (a_coeff[i] * y_f[i])
    
    
    sum_num_data[isr_i] = sum_num
    sum_den_data[isr_i] = sum_den

    y_out = sum_num - sum_den
  
    for i in np.arange((x_s.shape[0]-1), 0, -1):
        x_s[i] = x_s[i-1]
  
    for i in np.arange((y_f.shape[0]-1), 0, -1):
        y_f[i] = y_f[i-1]

    if (Select_Float == 1):
        filter_response = mydll.New_ZC_Filter(x_s[0])
        #filter_response = filter_response / (32768.0)
        #print filter_response
        zc_signal_out[isr_i] = filter_response
 
    if zc_step == 1:
        if y_out <= neg_threshold_lobe:
            zc_step = 2
    elif zc_step == 2:
        if y_out >= neg_threshold_lobe:
            ## ZC event!!!
            zc_data[isr_i] = 100.0
            zc_step = 3
    
    elif zc_step == 3:
        if y_out >= pos_threshold_lobe:
            zc_step = 4
    elif zc_step == 4:
        if y_out <= pos_threshold_lobe:
            ## ZC event !!!
            zc_data[isr_i] = 100.0
            zc_step = 1
 
    if y_out >= 200.0:
        out_duty = True
    else:
        out_duty = False


    Cycle_Sample_Counter = Cycle_Sample_Counter + 1
    
    if (out_duty == True) and (out_duty_prev == False):
        if Cycle_Sample_Counter != 0:
            Averaged_Freq = sampling_frequency/Cycle_Sample_Counter

        Cycle_Sample_Counter = 0
        
        out_duty_prev = True
    elif (out_duty == False) and (out_duty_prev == True):
        
        out_duty_prev = False


    averaged_freq_hyst[isr_i] = Averaged_Freq








    y_f_data[isr_i] = y_out 
    zc_step_hist[isr_i] = zc_step
 


if(Select_Float == 1):
    ### FLOATING point graphic
    plt.figure()
    plt.title('Zero Crossing Algorithm - FLOATING point - Mains frequency: ' + str(mains_frequency) + "Hz - Sampling frequency: " + str(sampling_frequency) + " Hz")
    plt.plot(time, y_f_data, 'g-', marker = 'x', label = 'y_f_data')
    plt.plot(time, zc_signal_out, label='zc_signal_out', color='black')
    plt.plot(time, intput_voltage_wave_form, 'b-', label='Input voltage')
    plt.plot(time, digital_input_ZC_signal, marker = 'p', label='Digital input signal', color='green')
#     if np.any(fundamental > 0):
#         plt.plot(time, fundamental, marker = 'o', label='Fundamental Voltage signal [V]', color='purple')
    plt.axhline(pos_threshold_lobe, color='y', label='pos_threshold_lobe')
    plt.axhline(neg_threshold_lobe, color='r', label='neg_threshold_lobe')
#     # plt.plot(time, zc_step_hist_fp*100, label='zc_step_hist_fp', color='purple')
#     # plt.plot(time, averaged_freq_hyst, label='averaged_freq_hyst', marker = 'x',color='blue')
    plt.plot(time, zc_data, label='zc_data', marker = 'x', color='blue')
    plt.plot(time, zc_signal_event, label='zc_signal_event', marker = 'x', color='black')
#     #plt.plot(time, sum_num_data, label='sum_num_data', marker = 'x', color='purple')
    #plt.plot(time, sum_num_data, label='sum_num_data', marker = 'x', color='red')
    #plt.plot(time, sum_den_data, label='sum_den_data', marker = 'x', color='blue')
    #plt.plot(time, zc_data, label='zc_data', marker = 'x', color='green')
              
    plt.legend(loc='upper right', fontsize = 'small')
    plt.grid()

else:
    ### FIXED point graphic
    plt.figure()
    plt.title('Zero Crossing Algorithm - FIXED point - Mains frequency: ' + str(mains_frequency) + "Hz - Sampling frequency: " + str(sampling_frequency) + " Hz")
    plt.plot(time, y_f_data_fp / 524288.0, 'g-', label = 'y_f_data_fp')
    # plt.plot(time, y_f_data_dll_fp * (Q_A_FILTER_BASE/(2.0**Q_FILTER)), 'y-', label = 'y_f_data_dll_fp')
    plt.plot(time, intput_voltage_wave_form, 'b-', label='Input voltage')
    #if np.any(fundamental > 0):
    #     plt.plot(time, fundamental, marker = 'o', label='Fundamental Voltage signal [V]', color='purple')
#     plt.plot(time, digital_input_ZC_signal, label='Digital input signal',marker = 'x', color='black')
    #plt.plot(time, sum_num_data_fp * (Q_A_FILTER_BASE/(2.0**Q_FILTER)), label='sum_num_data_fp', color='red')
#     plt.plot(time, sum_den_data_fp * (Q_A_FILTER_BASE/(2.0**Q_FILTER)), label='sum_den_data_fp', color='Green')
    plt.plot(time, zc_signal_event / 524288.0, label='zc_signal_event', color='black')
#     plt.plot(time, zc_signal_event, label='zc_signal_event', color='black')
    plt.plot(time, zc_data, label='zc_data', color='red')
#     plt.axhline((2.0**31), color='y', label='Max_Fixed_Point')
    plt.axhline(pos_threshold_lobe_fp * (Q_A_FILTER_BASE/(2.0**Q_FILTER)), color='y', label='pos_threshold_lobe_fp')
    plt.axhline(neg_threshold_lobe_fp * (Q_A_FILTER_BASE/(2.0**Q_FILTER)), color='r', label='neg_threshold_lobe_fp')
    # plt.plot(time, zc_step_hist_fp*100, label='zc_step_hist_fp', color='purple')
    # plt.plot(time, averaged_freq_hyst_fp, label='averaged_freq_hyst_fp', marker = 'x',color='blue')
    # plt.plot(time, zc_data_fp, label='zc_data_fp', marker = 'x', color='purple')
      
 
plt.legend(loc='upper right', fontsize = 'small')
plt.grid()

plt.show()
print("\nFinished.")



