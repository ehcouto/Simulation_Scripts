import pandas as pd
import matplotlib.pyplot as plt


print("\nReading data from file...")
df = pd.read_csv(r'Biguacu_Voltage.csv')



plt.figure()
plt.title('Biguacu Waveform')
plt.plot(df.Time, df.Voltage, 'g-', label = 'y_f_data')
             
plt.legend(loc='upper right', fontsize = 'small')
plt.grid()
plt.show()