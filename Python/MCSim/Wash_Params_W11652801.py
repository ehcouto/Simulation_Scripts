# Motor parameters
Rs = 25.4         # Stator resistance (Ohm)
Ld = 0.079 
Lq = 0.079  # Inductance (H)
Psi_PM = 0.0753    # Permanent magnet flux linkage (Wb)
p = 3            # Pole pairs
omega_e = 0.0   # Electrical angular speed (rad/s)
B = 0.0002
J = 0.0001
T_L = 0.14
KT = 3.0*p*Psi_PM/2.0
Max_Speed = 3600.0 #RPM
 
# PI controller gains
Kp = 184.9
Ki = 59505.5 
 
# PI Speed gains
Kp_spd = 0.0005744
Ki_spd = 0.002154
IqMax = 0.85
 
#Params dq Obs
wn = 200.0
epsilon = 0.85
 
#Params Anlge Tracking
wn_ato = 10.91
eps_ato = 2.35