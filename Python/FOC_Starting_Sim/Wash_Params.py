# Motor parameters
Rs = 25.0         # Stator resistance (Ohm)
Ld = 0.08 
Lq = 0.08  # Inductance (H)
Psi_PM = 0.07424    # Permanent magnet flux linkage (Wb)
p = 3            # Pole pairs
omega_e = 0.0   # Electrical angular speed (rad/s)
B = 0.000025
J = 0.00001
T_L = 0.0
KT = 3.0*p*Psi_PM/2.0
Max_Speed = 3600.0 #RPM
 
# PI controller gains
Kp = 50.0
Ki = 350.0 
 
# PI Speed gains
Kp_spd = 0.032
Ki_spd = 0.006
IqMax = 1.2
 
#Params dq Obs
wn = 200.0
epsilon = 0.85
 
#Params Anlge Tracking
wn_ato = 25.0
eps_ato = 1.41