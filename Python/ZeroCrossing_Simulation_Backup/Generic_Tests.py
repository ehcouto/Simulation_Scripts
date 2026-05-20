import numpy as np
from scipy.signal import butter, lfilter, freqz
import matplotlib.pyplot as plt


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


# Filter requirements.
order = 3
fs = 4000.0       # sample rate, Hz
cutoff = 110.0    # desired cutoff frequency of the filter, Hz

# Get the filter coefficients so we can check its frequency response.
b, a = butter_lowpass(cutoff, fs, order)
print "b\n", b, "\na\n", a

#  Plot the frequency response.
w, h = freqz(b, a, worN=8000)
plt.subplot(2, 1, 1)
plt.plot(0.5*fs*w/np.pi, np.abs(h), 'b')
plt.plot(0.5*fs*w/np.pi, np.angle(h), 'g')
plt.plot(cutoff, 0.5*np.sqrt(2), 'ko')
plt.axvline(60, color='k')
plt.axvline(50, color='k')
plt.axvline(cutoff, color='k')
plt.xlim(0, 500.0)
plt.title("Lowpass Filter Frequency Response")
plt.xlabel('Frequency [Hz]')
plt.grid()


# Demonstrate the use of the filter.
# First make some data to be filtered.
T = 5.0         # seconds
n = int(T * fs) # total number of samples
t = np.linspace(0, T, n, endpoint=False)
# "Noisy" data.  We want to recover the 1.2 Hz signal from this.
data = np.sin(1.2*2*np.pi*t) + 1.5*np.cos(9*2*np.pi*t) + 0.5*np.sin(12.0*2*np.pi*t)

# Filter the data, and plot both the original and filtered signals.
y = butter_lowpass_filter(data, cutoff, fs, order)

#plt.subplot(2, 1, 2)
#plt.plot(t, data, 'b-', label='data')
#plt.plot(t, y, 'g-', linewidth=2, label='filtered data')
#plt.xlabel('Time [sec]')
#plt.grid()
#plt.legend()
#
#plt.subplots_adjust(hspace=0.35)


a_coeff = a[1:4]
print "a_coeff\n", a_coeff

y_f = np.zeros(a.shape[0] - 1)
x_s = np.zeros(b.shape[0])
y_f_data = np.zeros(t.shape[0])

print "\ny_f size ", y_f.shape[0]
print "x_s size ", x_s.shape[0]

y_out = 0.0
for i in np.arange(t.shape[0]):
    
    x_s[0] = data[i] # read gpio
    y_f[0] = y_out   # output the output filter
    
    #y_out = np.dot(b, x_s) - np.dot(a_coeff, y_f)
    
    temp_b = b[0]*x_s[0] + b[1]*x_s[1] + b[2]*x_s[2] + b[3]*x_s[3]  
    temp_a = a_coeff[0]*y_f[0] + a_coeff[1]*y_f[1] + a_coeff[2]*y_f[2]
    y_out = temp_b - temp_a
    
    x_s = np.roll(x_s, 1)
    y_f = np.roll(y_f, 1)
    
    y_f_data[i] = y_out


plt.figure()
plt.plot(t, y_f_data+0.1, 'r-', label = 'meu filtro')
plt.plot(t, data, 'b-', label='data')
plt.plot(t, y, 'g-', linewidth=2, label='filtered data')
plt.legend()
plt.grid()
plt.ylim([-5,5])

plt.show()