import numpy as np
import matplotlib.pyplot as plt
from IPMSM import IPMSMMotor
from PI import PI_Control
from ClrkPark import ClarkPark

import Drain_Params as motor_params
#import Wash_Params as motor_params


#----------------------------------------------
#------------ Simulation Settings -------------
#----------------------------------------------
T_sim = 5.0
Fs = 8000.0
Fslow = 1000.0
dt = 1.0 / Fs
steps = int(T_sim / dt)

# Motor parameters
Rs = motor_params.Rs         # Stator resistance (Ohm)
Ld = motor_params.Ld         # Inductance Ld(H)
Lq = motor_params.Lq         # Inductance Lq (H)
Psi_PM = motor_params.Psi_PM    # Permanent magnet flux linkage (Wb)
p = motor_params.p              # Pole pairs
omega_e = motor_params.omega_e  # Electrical angular speed (rad/s)
B = motor_params.B
J = motor_params.J
T_L = motor_params.T_L
KT = motor_params.KT
Max_Speed = motor_params.Max_Speed

    # PI controller gains
Kp = motor_params.Kp
Ki = motor_params.Ki / Fs# Ki/FastHandlerFrequency

# PI Speed gains
Kp_spd = motor_params.Kp_spd
Ki_spd = motor_params.Ki_spd / Fslow # Ki/SlowHandlerFrequency
IqMax = motor_params.IqMax

#Params dq Obs
wn = motor_params.wn
epsilon = motor_params.epsilon

#Params Anlge Tracking
wn_ato = motor_params.wn_ato
eps_ato = motor_params.eps_ato

#----------------------------------------------
#-------------- Initialize Motor --------------
#----------------------------------------------
motor = IPMSMMotor(Rs, Ld, Lq, Psi_PM, p, B, J)


#----------------------------------------------
#-------------- Initialize PID's --------------
#----------------------------------------------

#dq Current Controllers
PI_iq = PI_Control(Kp, Ki, -256.0, 256.0)
PI_id = PI_Control(Kp, Ki, -256.0, 256.0)


#----------------------------------------------
#------------ Initialize Variables ------------
#----------------------------------------------
#INIT FOC PARAMS
Id = 0.0
Iq = 0.0
Ud = 0.0
Uq = 0.0
Id_ref = 0.0
Iq_ref = 0.3
c_Va = 0.0
c_Vb = 0.0
c_Vc = 0.0

Obs_State = 0
Alignment_Cnt = 0
Vq_Acc = 0
Vq_Alignment = 0



c_Ia = 0.0
c_Ib = 0.0
c_Ic = 0.0

#INIT SLIDING MODE OBBSERVER
i_gamma = 0
i_delta = 0
igamma_err = 0
idelta_err = 0
E_gamma = 0
E_delta = 0

#INIT ANGLE TRACKING
we = 0
theta = 0
theta_err = 0

#LOGGING
time_log = []
torque_log = []
mtheta_log = []
Iq_log = []
Id_log = []
Omega_log = []
Omega_Ref_log = []
thata_log = []
Ud_log = []
Uq_log = []

Ia_Log = []
Ib_Log = []
Ic_Log = []

TrCP = ClarkPark()

PI_sobs = PI_Control(3, 4800*dt, -600.0, 600.0)

#----------------------------------------------
#--------------- Speed Commands ---------------
#----------------------------------------------
Omega_Q = 0.0
Omega_D = 0.0
Omega_Est = 0.0
Theta_Est = 0.0
Theta_Est_Q = 0.0

#----------------------------------------------
#---------------- Run the Loop ----------------
#----------------------------------------------
for step in range(steps):
    t = step * dt
    
    #----------------------------------------------
    #-------------- Run Motor Model ---------------
    #----------------------------------------------
    #Update motor model 
    c_Ia, c_Ib, c_Ic = motor.update(c_Va, c_Vb, c_Vc, T_L, dt) 
    
    #----------------------------------------------
    #---------- Clark & Park Transforms -----------
    #----------------------------------------------
    c_Ialpha, c_Ibeta = TrCP.Clark(c_Ia, c_Ib, c_Ic)
    Id, Iq = TrCP.Park(c_Ialpha, c_Ibeta, theta)
    
    #----------------------------------------------
    #------------- Starting Observer --------------
    #----------------------------------------------
    if Obs_State == 0:
        if Iq >= Iq_ref:
            Alignment_Cnt += 1
            Vq_Acc += PI_iq.get_integ()
            
            if Alignment_Cnt >= 4000:
                Vq_Alignment = Vq_Acc / 4000
                Vq_Acc = 0
                Alignment_Cnt = 0
                
                Theta_Est = theta
                Theta_Est_Q = theta
                
                Obs_State = 1
            
            theta = theta = 30.0 * (np.pi / 180.0)
            we = 0.0
    
    elif Obs_State == 1:
        
        Omega_D = PI_id.get_integ() * (1.0 / Lq*Iq_ref)
    
        temp = PI_iq.get_integ() - Vq_Alignment
        Omega_Q = temp * (1.0 / Psi_PM)
    
        error = Omega_D - Omega_Q
        Omega_Est = PI_sobs.calc(error)
    
        Theta_Est += Omega_Est * dt
    
        Theta_Est_Q += Omega_Q * dt
        
        theta = Theta_Est_Q
        
        if Omega_Q >= 80.0: #approx. 800 rpm in rad/s el.
            
            Obs_State = 3
            
    
     
        
        
    #----------------------------------------------
    #---------- dq Current Controllers ------------
    #---------------------------------------------- 
    Id_err = Id_ref - Id
    Iq_err = Iq_ref - Iq
    
    vd_temp = PI_id.calc(Id_err)
    vq_temp = PI_iq.calc(Iq_err)
    
    #Decoupling Equations
    Ud = vd_temp
    Uq = vq_temp

    c_Ualpha, c_Ubeta = TrCP.Inv_Park(Ud, Uq, theta)
    c_Va, c_Vb, c_Vc  = TrCP.Inv_Clark(c_Ualpha, c_Ubeta)
    #----------------------------------------------
    #------------- Log Data & Graphs --------------
    #---------------------------------------------- 
    time_log.append(t)
    #mtheta_log.append(theta_err)
    torque_log.append(motor.get_torque()*1000.0)
    thata_log.append(theta) #motor.get_theta())
    Iq_log.append(Iq)
    Id_log.append(Id)
    Ud_log.append(Uq)
    Uq_log.append(Vq_Alignment)
    
    Ia_Log.append(c_Ia)
    Ib_Log.append(c_Ib)
    Ic_Log.append(c_Ic)


#----------------------------------------------
#------------ Display Final Data --------------
#---------------------------------------------- 
print("\n Last Theta Error: ",  theta_err)
print("\n Last Uq: ",  Vq_Alignment)
print("\n Last Ud: ",  vd_temp)

# Plot results
plt.subplot(3, 1, 1)
#plt.plot(time_log, Iq_log)
#plt.plot(time_log, Id_log)
plt.plot(time_log, Ia_Log)
plt.plot(time_log, Ib_Log)
plt.plot(time_log, Ic_Log)
#plt.title("dq-axis Current")
plt.ylabel("Idq (A)")

plt.subplot(3, 1, 2)
plt.plot(time_log, Ud_log)
plt.plot(time_log, Uq_log)
#plt.title("dq-axis Voltage")
plt.ylabel("Vout_dq (A)")

plt.subplot(3, 1, 3)
#plt.plot(time_log, mtheta_log)
plt.plot(time_log, thata_log)
#plt.title("Real & Estimated Rotor Positions")
plt.ylabel("Angle (Rad)")

plt.tight_layout()
plt.show()
