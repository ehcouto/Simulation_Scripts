# Motor parameters
Rs = 65.0         # Stator resistance (Ohm)
Ld = 0.13 
Lq = 0.13  # Inductance (H)
Psi_PM = 0.0661    # Permanent magnet flux linkage (Wb)
p = 1            # Pole pairs
omega_e = 0.0   # Electrical angular speed (rad/s)
B = 0.000055
J = 0.00001
T_L = 0.0
KT = 3.0*p*Psi_PM/2.0
Max_Speed = 3600.0

    # PI controller gains
Kp = 100.0
Ki = 50000.0 # Ki/FastHandlerFrequency

# PI Speed gains
Kp_spd = 0.010 #0.010
Ki_spd = 0.057 #0.057 # Ki/SlowHandlerFrequency
IqMax = 0.6

#Params dq Obs
wn = 200.0
epsilon = 0.85

#Params Anlge Tracking
wn_ato = 22.57
eps_ato = 1.41