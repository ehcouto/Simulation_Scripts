import numpy as np
import matplotlib.pyplot as plt
from IPMSM import IPMSMMotor
from PI import PI_Control

#plt.close() 

#----------------------------------------------
#------------ Simulation Settings -------------
#----------------------------------------------
T_sim = 5.0
Fs = 8000.0
Fslow = 1000.0
dt = 1.0 / Fs
steps = int(T_sim / dt)

WASH_PUMP = 0
DRAIN_PUMP = 1

Motor_Sel = DRAIN_PUMP

#----------------------------------------------
#-------------- Set Motor Params --------------
#----------------------------------------------

if Motor_Sel == WASH_PUMP:
    # Motor parameters
    Rs = 25.0         # Stator resistance (Ohm)
    Ld = 0.08 
    Lq = 0.08  # Inductance (H)
    Psi_PM = 0.07424    # Permanent magnet flux linkage (Wb)
    p = 3            # Pole pairs
    omega_e = 0.0   # Electrical angular speed (rad/s)
    B = 0.00002
    J = 0.00001
    T_L = 0.0
    KT = 3.0*p*Psi_PM/2.0
    Max_Speed = 3600.0 #RPM
    
    # PI controller gains
    Kp = 5.0
    Ki = 25.0 
    
    # PI Speed gains
    Kp_spd = 0.01
    Ki_spd = 0.03  
    IqMax = 2.0
    
    #Params dq Obs
    wn = 200.0
    epsilon = 0.85
    
    #Params Anlge Tracking
    wn_ato = 180.0
    eps_ato = 0.15


if Motor_Sel == DRAIN_PUMP:
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
    Kp = 200.0
    Ki = 25000.0 # Ki/FastHandlerFrequency
    
    # PI Speed gains
    Kp_spd = 0.010
    Ki_spd = 0.057 / Fslow # Ki/SlowHandlerFrequency
    IqMax = 0.6
    
    #Params dq Obs
    wn = 140.0
    epsilon = 0.85
    
    #Params Anlge Tracking
    wn_ato = 22.57
    eps_ato = 1.41


#----------------------------------------------
#--------------- Speed Commands ---------------
#----------------------------------------------
RPM2RAD_m = (np.pi / 30.0) # RPM to Rad/s mechanic
RPM2RAD_e = (p * RPM2RAD_m) # RPM to Rad/s electric

Speed_Target = 2800.0 #RPM
Speed_Accel = 1000.0 #RPM/s
Speed_Step = (Speed_Accel * RPM2RAD_e) * (1.0 / Fslow)
Speed_Target = (Speed_Target  * RPM2RAD_e)
Speed_Ref = 0.0

#----------------------------------------------
#-------------- Initialize Motor --------------
#----------------------------------------------
motor = IPMSMMotor(Rs, Ld, Lq, Psi_PM, p, omega_e, B, J)


#----------------------------------------------
#-------------- Initialize PID's --------------
#----------------------------------------------

#dq Current Controllers
PI_iq = PI_Control(Kp, Ki*dt, -256.0, 256.0)
PI_id = PI_Control(Kp, Ki*dt, -256.0, 256.0)

#Speed Controller
PI_spd = PI_Control(Kp_spd, Ki_spd*(1.0/Fslow), -IqMax*KT, IqMax*KT)

#Initialize Sliding Mode Observer Controllers
print(" *** DQ Observer Gains ")
Kp_obs =  2.0 * Ld * epsilon * wn * 2.0 * np.pi - Rs
print("Kp_obs =", Kp_obs)
Ki_obs = (wn * 2.0 * np.pi)**2 * Ld #/  (wn*4.0/(Max_Speed * p / 60.0))
print("Ki_obs =", Ki_obs)

PI_gamma = PI_Control(Kp_obs, Ki_obs*dt, -256.0, 256.0)
PI_delta = PI_Control(Kp_obs, Ki_obs*dt, -256.0, 256.0)

print("\n *** Angle Tracking Gains ")
Kp_ato = 2.0 * eps_ato * wn_ato * 2.0 * np.pi
print("Kp_ato =", Kp_ato)
Ki_ato = (wn_ato * 2.0 * np.pi)**2
print("Ki_ato =", Ki_ato)

#Initialize Angle Tracking Controller
PI_ato = PI_Control(Kp_ato, Ki_ato*dt, -(Max_Speed*RPM2RAD_e), (Max_Speed*RPM2RAD_e))


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

def Inv_Park(in_id, in_iq, in_therr):
    t_sin = np.sin(in_therr)
    t_cos = np.cos(in_therr)
    
    new_id = in_id*t_cos - in_iq*t_sin
    new_iq = in_id*t_sin + in_iq*t_cos
    
    return new_id, new_iq

#----------------------------------------------
#---------------- Run the Loop ----------------
#----------------------------------------------
for step in range(steps):
    t = step * dt
    
    #----------------------------------------------
    #-------------- Run Motor Model ---------------
    #----------------------------------------------
    #Update motor model 
    motor.update(Ud, Uq, T_L, dt)
    
    #Get dq motor currents
    Id, Iq = motor.get_currents()
       
    #Estimate Error Position
    err_theta = motor.get_theta() - theta
        
    #----------------------------------------------
    #----------- Sliding Mode Observer ------------
    #----------------------------------------------    
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
    we = PI_ato.calc(theta_err)    
    theta += we*dt
    
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
        Iq_ref = T_ref / KT

    #----------------------------------------------
    #---------- dq Current Controllers ------------
    #---------------------------------------------- 
    Id_err = Id_ref - Id
    Iq_err = Iq_ref - Iq
    
    vd_temp = PI_id.calc(Id_err)
    vq_temp = PI_iq.calc(Iq_err)
    
    #Decoupling Equations
    Ud = vd_temp + omega_e * Lq * Iq
    Uq = vq_temp - omega_e*(Ld * Id + Psi_PM)

    
    #----------------------------------------------
    #------------- Log Data & Graphs --------------
    #---------------------------------------------- 
    time_log.append(t)
    #err_theta_log.append(err_theta)
    mtheta_log.append(theta_err)
    torque_log.append(motor.get_torque()*1000.0)
    thata_log.append(0.0) #motor.get_theta())
    Iq_log.append(Iq)
    Id_log.append(Id)
    Omega_log.append(motor.get_omega_m_rpm_())
    #Omega_Ref_log.append((Speed_Ref * 60.0 /(2.0* np.pi * p)))
    Omega_Ref_log.append(we / RPM2RAD_e)
    Ud_log.append(Ud)
    Uq_log.append(Uq)


#----------------------------------------------
#------------ Display Final Data --------------
#---------------------------------------------- 
print("\n Last Theta Error: ",  theta_err)
print("\n Last Uq: ",  Uq)
print("\n Last Ud: ",  Ud)

# Plot results
plt.subplot(5, 1, 1)
plt.plot(time_log, Omega_log)
plt.plot(time_log, Omega_Ref_log)
#plt.title("Motor Speed (RPM)")
plt.ylabel("Speed (RPM)")
plt.xlabel("Time (s)")
plt.ylim(0, 3000)

plt.subplot(5, 1, 5)
plt.plot(time_log, mtheta_log)
plt.plot(time_log, thata_log)
#plt.title("Real & Estimated Rotor Positions")
plt.ylabel("Angle (Rad)")

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
