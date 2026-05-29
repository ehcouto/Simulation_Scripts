import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = os.path.join("Debug", "sim_output.csv")

# Lê o CSV com header
df = pd.read_csv(csv_path, dtype=float)

plt.figure(figsize=(14, 8))

plt.subplot(3, 1, 1)
plt.plot(df["t"], df["SpeedRef"], label="SpeedRef")
plt.plot(df["t"], df["SpeedEst"], label="SpeedEst")
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(df["t"], df["iu"], label="iu")
plt.plot(df["t"], df["iv"], label="iv")
plt.plot(df["t"], df["iw"], label="iw")
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(df["t"], df["id"], label="id")
plt.plot(df["t"], df["iq"], label="iq")
plt.plot(df["t"], df["torque"], label="torque")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
