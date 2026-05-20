import numpy as np
import matplotlib.pyplot as plt
from IPMSM import IPMSMMotor
from PI import PI_Control
from ClrkPark import ClarkPark

import Drain_Params as motor_params
#import Drain_Params_W11377410 as motor_params
#import Wash_Params as motor_params
#import Wash_Params_W20007335 as motor_params



def Merge(k1_unmerged, k1, p_merge_step, k1_merge_coeff):
    if k1_merge_coeff > 1.0:
        k1_merge_coeff = 1.0
    
    output = k1_merge_coeff * k1_unmerged + (1.0 - k1_merge_coeff) * k1
    k1_merge_coeff += p_merge_step
    
    if output == k1:
        k1_merge_coeff = 0.0
    
    return output


#----------------------------------------------
#------------ Simulation Settings -------------
#----------------------------------------------
T_sim = 2.0
Fs = 8000.0
Fslow = 1000.0
dt = 1.0 / Fs
steps = int(T_sim / dt)

prm_minsp_o2c = 210.0
prm_k1min     = 11000.0
prm_smc       = 1600.0
prm_epsilon   = 2.0
prm_merge_step = dt


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
#--------------- Speed Commands ---------------
#----------------------------------------------
RPM2RAD_m = (np.pi / 30.0) # RPM to Rad/s mechanic
RPM2RAD_e = (p * RPM2RAD_m) # RPM to Rad/s electric


Set_Speed = 3500.0 #RPM
Speed_Accel = 4000.0 #RPM/s
Speed_Step = (Speed_Accel * RPM2RAD_e) * (1.0 / Fslow)
Speed_Target = (Set_Speed  * RPM2RAD_e)
Speed_Ref = 0.0

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

#Speed Controller
PI_spd = PI_Control(Kp_spd, Ki_spd, -IqMax*KT, IqMax*KT)

#Initialize Sliding Mode Observer Controllers
print(" *** DQ Observer Gains ")
Kp_obs =  2.0 * Ld * epsilon * wn * 2.0 * np.pi - Rs
print("Kp_obs =", Kp_obs)
Ki_obs = (wn * 2.0 * np.pi)**2 * Ld
print("Ki_obs =", Ki_obs)

PI_gamma = PI_Control(Kp_obs, Ki_obs*dt, -256.0, 256.0)
PI_delta = PI_Control(Kp_obs, Ki_obs*dt, -256.0, 256.0)

print("\n *** Angle Tracking Gains ")
Kp_ato = 2.0 * eps_ato * wn_ato * 2.0 * np.pi
Ki_ato = (wn_ato * 2.0 * np.pi)**2
k2 = Kp_ato / Fs
print("K2 =", k2)
print("K1_Max =", Ki_ato)

prm_k1max = Ki_ato
prm_k2    = k2

v_k1_coeff = (prm_k1max - prm_k1min) / (Max_Speed - prm_minsp_o2c);
v_k1_merge_coeff = 1.0
v_k1_unmerged = 0.0
v_k1 = prm_k1min

#Initialize Angle Tracking Controller
PI_ato = PI_Control(Kp_ato, Ki_ato*dt, -(Max_Speed*RPM2RAD_e), (Max_Speed*RPM2RAD_e))

print("\n")
#----------------------------------------------
#------------ Initialize Variables ------------
#----------------------------------------------
#INIT FOC PARAMS
Id = 0.0
Iq = 0.0
Ud = 0.0
Uq = 0.0
Id_ref = 0.0
Iq_ref = 0.0
c_Va = 0.0
c_Vb = 0.0
c_Vc = 0.0

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

Load_Torque = 0.0
Theta_err_sum = 0.0

w = 0.0
wf = 0.0
integ = 0.0

#----------------------------------------------
#---------------- Run the Loop ----------------
#----------------------------------------------
for step in range(steps):
    t = step * dt
    
    if t > 0.05:
        Load_Torque += (T_L / (1.0 * Fs))
        if Load_Torque > T_L:
            Load_Torque = T_L
    
    #----------------------------------------------
    #-------------- Run Motor Model ---------------
    #----------------------------------------------
    #Update motor model 
    c_Ia, c_Ib, c_Ic = motor.update(c_Va, c_Vb, c_Vc, Load_Torque, dt) 
    
    #Estimate Error Position
    err_theta = motor.get_theta() - theta
    #----------------------------------------------
    #----------- Sliding Mode Observer ------------
    #----------------------------------------------
    c_Ialpha, c_Ibeta = TrCP.Clark(c_Ia, c_Ib, c_Ic)
    Id, Iq = TrCP.Park(c_Ialpha, c_Ibeta, theta)
        
    #Gamma (Id)
    igamma_err = Id - i_gamma
    E_gamma = PI_gamma.calc(igamma_err)
    
    temp = ((Ld - dt*Rs)/Ld) * i_gamma
    temp += (dt/Ld)*(Ud + E_gamma)
    i_gamma = temp + (dt*Lq/Ld)*(we * Iq)
    
    #Delta (Iq)
    idelta_err = Iq - i_delta   
    E_delta = PI_delta.calc(idelta_err)
    
    temp = ((Ld - dt*Rs)/Ld) * i_delta
    temp += (dt/Ld)*(Uq + E_delta)
    i_delta = temp - (dt*Lq/Ld)*(we * Id)
    
    #Get Theta Error from BEMF Estimation  
    theta_err = np.arctan2(E_gamma, -E_delta)
    
    #----------------------------------------------
    #-------------- Angle Tracking ----------------
    #----------------------------------------------
    w += v_k1 *  theta_err * dt
    wf += (prm_smc / Fs)*(w - wf)
    integ += wf * dt     
    theta = prm_k2*wf + integ
    
    Theta_err_sum += theta_err
    we = wf
    
    # Rotor Position at range [-pi, pi]
    if theta >  np.pi:
        theta -= 2.0 * np.pi
    if theta < -np.pi:
        theta += 2.0 * np.pi

    #----------------------------------------------
    #--------- Speed Handler & Control ------------
    #----------------------------------------------   
    x = step % 8
    if(x == ((Fs / Fslow)-1)): #Speed Controller at Slow Handler
        
        #Speed Reference Generation
        if Speed_Ref < Speed_Target:        
            Speed_Ref += Speed_Step
        if Speed_Ref > Speed_Target:
            Speed_Ref = Speed_Target
       
        # Speed Control PI
        omega_e = motor.get_omega_e_rad_s_()
        Speed_err = Speed_Ref - omega_e
        T_ref = PI_spd.calc(Speed_err)
        
        # Generate Iq Reference from Torque
        temp = KT + (motor_params.Ld - motor_params.Lq)*Id_ref*Iq_ref
        Iq_ref = T_ref / temp
        
        # PI controllers
        if Speed_Ref > prm_minsp_o2c:
            v_k1_unmerged = prm_k1min + v_k1_coeff * (Speed_Ref - prm_minsp_o2c)
        else:
            v_k1_unmerged = prm_k1min
            v_k1_merge_coeff = 1.0
        
        v_k1 = Merge(v_k1_unmerged, v_k1, prm_merge_step, v_k1_merge_coeff)

    #----------------------------------------------
    #---------- dq Current Controllers ------------
    #---------------------------------------------- 
    Id_err = Id_ref - Id
    Iq_err = Iq_ref - Iq
    
    vd_temp = PI_id.calc(Id_err)
    vq_temp = PI_iq.calc(Iq_err)
    
    #Decoupling Equations
    Ud = vd_temp - Speed_Ref*(Lq * Iq)
    Uq = vq_temp + Speed_Ref*(Ld * Id + Psi_PM)

    c_Ualpha, c_Ubeta = TrCP.Inv_Park(Ud, Uq, theta)
    c_Va, c_Vb, c_Vc  = TrCP.Inv_Clark(c_Ualpha, c_Ubeta)
    #----------------------------------------------
    #------------- Log Data & Graphs --------------
    #---------------------------------------------- 
    time_log.append(t)
    mtheta_log.append(theta_err)
    #torque_log.append(motor.get_torque()*1000.0)
    torque_log.append(err_theta)
    thata_log.append(0.0) #motor.get_theta())
    Iq_log.append(Iq)
    Id_log.append(Id)
    Omega_log.append(Speed_Ref / RPM2RAD_e) #(motor.get_omega_m_rpm_())
    Omega_Ref_log.append(we / RPM2RAD_e)
    Ud_log.append(Ud)
    Uq_log.append(Uq)
    
    Ia_Log.append(c_Ia)
    Ib_Log.append(c_Ib)
    Ic_Log.append(c_Ic)


#----------------------------------------------
#------------ Display Final Data --------------
#---------------------------------------------- 
print("\n Last Theta Error: ",  theta_err)
print("\n Last Uq: ",  Uq)
print("\n Last Ud: ",  Ud)
print("\n Last Iq: ",  Iq)
print("\n Irms: ",  np.sqrt((Iq**2 + Id**2)/2.0))
print("\n Vout: ",  np.sqrt(Ud**2 + Uq**2))
print("\n Theta Error Avg: ",  Theta_err_sum / steps)

# Plot results
plt.title("FOC_Beko_Ato")
plt.subplot(5, 1, 1)
plt.plot(time_log, Omega_log)
plt.plot(time_log, Omega_Ref_log)
#plt.plot(time_log, Ia_Log)
#plt.plot(time_log, Ib_Log)
#plt.plot(time_log, Ic_Log)
#plt.title("Motor Speed (RPM)")
plt.ylabel("Speed (RPM)")
plt.xlabel("Time (s)")
plt.ylim(0, Set_Speed*1.10)

plt.subplot(5, 1, 5)
plt.plot(time_log, mtheta_log)
plt.plot(time_log, thata_log)
#plt.title("Real & Estimated Rotor Positions")
plt.ylabel("Angle (Rad)")
plt.ylim(-0.15, 0.15)

plt.subplot(5, 1, 2)
plt.plot(time_log, Iq_log)
plt.plot(time_log, Id_log)
#plt.title("dq-axis Current")
plt.ylabel("Idq (A)")

plt.subplot(5, 1, 3)
plt.plot(time_log, Ud_log)
plt.plot(time_log, Uq_log)
#plt.title("dq-axis Voltage")
plt.ylabel("Vout_dq (A)")

plt.subplot(5, 1, 4)
plt.plot(time_log, torque_log)
#plt.title("Electromagnetic Torque")
plt.ylabel("Torque (mNm)")


plt.tight_layout()
plt.show()
