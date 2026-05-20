'''
Created on 23 de jan de 2021

@author: COUTOEH
'''

import scipy
import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt



sampling_frequency = 5000



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
        cutoff = 120  # desired cutoff frequency of the filter, Hz
        N_order = 1
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
        N_order = 2
        
        low_cut_freq = 35.0
        high_cut_freq = 85.0
        
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
        plt.title('Zero Crossing ' + str(selec_filter) + ' response\nOrder:' + str(N_order) + " - Sample freq: " + str(sampling_frequency) + "Hz") 
       
        ax1 = fig.add_subplot(111)
        h,w = scipy.signal.freqz(num, den)
           
        plt.axvline(35.0, color='k')
        plt.axvline(85.0, color='k')
        
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


filter_num, filter_den = Design_Filter(sampling_frequency, True)




b = filter_num
a = filter_den
a_coeff = a[1:a.shape[0]]
print("a_coeff\n", a_coeff)
print("b\n", b)
print("\n")



a_fp = (a)*(2.0**15)
a_fp_coeff = a_fp[1:a.shape[0]]
b_fp = (b)*(2.0**15)

print("a_fp_coeff\n", a_fp_coeff)
print("b_fp\n", b_fp)
print("")