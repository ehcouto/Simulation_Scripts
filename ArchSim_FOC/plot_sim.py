import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
plt.close('all')
import pandas as pd
import os
import numpy as np


csv_path = os.path.join("Debug", "sim_output.csv")

# Lê o CSV com header
df = pd.read_csv(csv_path, dtype=float)

print("*** Simulation Results *** \n")
print("Total Time Simulation = {:.0f} Secs".format(df["t"].iloc[-1]))    
max_speed = df["SpeedRef"].max()
print("Max Speed Reached = {:.1f} RPM".format(max_speed))   
print("Max Peak Current Reached = {:.3f} A peak".format(df["iq"].max()))  
print("Max Current Reached = {:.3f} A rms".format(df["iq"].max()/np.sqrt(2.0)))    
max_torque = df["torque"].max() 
print("Max Shaft Torque = {:.1f} mNm".format(max_torque*1000.0))
print("Max Shaft Power = {:.1f} W".format(max_torque*max_speed * 2.0 * np.pi / 60.0)) 
      
plt.figure(figsize=(14, 8))

#Speed Data
plt.subplot(3, 2, 1)
plt.plot(df["t"], df["SpeedRef"], label="SpeedRef")
plt.plot(df["t"], df["SpeedEst"], label="SpeedEst")
plt.plot(df["t"], df["SpeedReal"], label="SpeedReal")
plt.grid(True)
plt.legend()
plt.title("Motor Speed [RPM]", fontsize=10, fontweight="bold")

#3ph Currents
plt.subplot(3, 2, 2)
plt.plot(df["t"], df["iu"], label="iu")
plt.plot(df["t"], df["iv"], label="iv")
plt.plot(df["t"], df["iw"], label="iw")
plt.grid(True)
plt.legend()
plt.title("Motor Currents [A]", fontsize=10, fontweight="bold")

#idq Currents
plt.subplot(3, 2, 3)
plt.plot(df["t"], df["id"], label="id")
plt.plot(df["t"], df["iq"], label="iq")
plt.plot(df["t"], df["id_m"], label="id_m")
plt.plot(df["t"], df["iq_m"], label="iq_m")
plt.grid(True)
plt.legend()
plt.title("Motor Current dq Axis [A]", fontsize=10, fontweight="bold")

#vdq voltages
plt.subplot(3, 2, 4)
plt.plot(df["t"], df["vd"], label="vd")
plt.plot(df["t"], df["vq"], label="vq")
plt.grid(True)
plt.legend()
plt.title("Motor Voltage dq Axis [V]", fontsize=10, fontweight="bold")

#Torque
plt.subplot(3, 2, 5)
plt.plot(df["t"], df["torque"], label="Torque")
plt.plot(df["t"], df["tq_m"], label="Torque Motor")
plt.plot(df["t"], df["tq_load"], label="Load")
plt.grid(True)
plt.legend()
plt.title("Motor Torque [Nm]", fontsize=10, fontweight="bold")

#Duty Cycles
plt.subplot(3, 2, 6)
plt.plot(df["t"], df["dc_u"], label="Duty Cycle U")
plt.plot(df["t"], df["dc_v"], label="Duty Cycle V")
plt.plot(df["t"], df["dc_w"], label="Duty Cycle W")
plt.grid(True)
plt.legend()
plt.title("Duty Cycle", fontsize=10, fontweight="bold")

plt.tight_layout()
plt.show()
