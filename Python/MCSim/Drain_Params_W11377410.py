# Motor parameters
Rs = 46.0         # Stator resistance (Ohm)
Ld = 0.14 
Lq = 0.14   # Inductance (H)
Psi_PM = 0.07837    # Permanent magnet flux linkage (Wb)
p = 1            # Pole pairs
omega_e = 0.0   # Electrical angular speed (rad/s)
B = 0.0000291
J = 0.000002134
T_L = 0.05
KT = 3.0*p*Psi_PM/2.0
Max_Speed = 3600.0

# PI controller gains
Kp = 141.0
Ki = 63114.0 # Ki/FastHandlerFrequency

# PI Speed gains
Kp_spd = 0.000955 #0.010
Ki_spd = 0.00159 #0.057 # Ki/SlowHandlerFrequency
IqMax = 0.65

#Params dq Obs
wn = 200.0
epsilon = 0.85

#Params Anlge Tracking
wn_ato = 22.57
eps_ato = 1.41