import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter


def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs  
    normal_cutoff = cutoff / nyq  
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def lowpass_filter(data, cutoff, fs, order=2):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


fs = 10000  
cutoff = 100 
t = np.linspace(0, 1.0, fs, endpoint=False) 

data = 4.5 * np.sin(2 * np.pi * 50 * t) + 0.5 * np.random.randn(t.size)


filtered_data = lowpass_filter(data, cutoff, fs, order=4)

plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(t, data, label='Sinal Original')
plt.title('Sinal Original com Ruido')
plt.xlabel('Tempo [s]')
plt.ylabel('Amplitude')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(t, filtered_data, label='Sinal Filtrado', color='orange')
plt.title('Sinal Filtrado com Filtro Passa-Baixas')
plt.xlabel('Tempo [s]')
plt.ylabel('Amplitude')
plt.legend()

plt.tight_layout()
plt.show()
