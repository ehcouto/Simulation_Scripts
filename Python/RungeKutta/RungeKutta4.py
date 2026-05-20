'''
Created on 2020

@author: HENRIE2
'''


import math
import numpy as np
import matplotlib.pyplot as plt

def SinudoidalSignal(Vrms,t,theta):
    return Vrms*math.sqrt(2)*math.sin(2*np.pi*60*t+theta)


t = 0.0
v = 0.0
i = 0.0
zc = 0.0
dt = 0.0001

vs, cs, ts, zcs = [], [], [], []

while t <= 0.05:
    v = SinudoidalSignal(220, t, 0)
    i = SinudoidalSignal(100, t, -np.pi*3/4)
    t += dt
   
    #ZC Detection
    zc = 0.0 
    if v>0:
        zc = 200.0  
    
    zcs.append(zc)
    vs.append(v)
    cs.append(i)
    ts.append(t)

plt.plot(ts, vs)
plt.plot(ts, cs)
plt.plot(ts, zcs)
plt.show()