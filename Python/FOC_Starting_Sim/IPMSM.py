'''
Created on 3 ott 2025

@author: 93600004
'''
# motor_model.py

import numpy as np
from ClrkPark import ClarkPark 

ClPk = ClarkPark()

class IPMSMMotor:
    def __init__(self, Rs, Ld, Lq, Psi_PM, p, B, J):
        self.Rs = Rs
        self.Ld = Ld
        self.Lq = Lq
        self.Psi_PM = Psi_PM
        self.p = p
        self.J = J
        self.B = B
        
        #Variables Initialization
        self.m_Theta = 0.0
        self.m_Id = 0.0
        self.m_Iq = 0.0
        self.m_Omega_e = 0.0
        self.m_Te = 0.0
        self.m_Ud = 0.0
        self.m_Uq = 0.0
        self.m_Valpha = 0.0
        self.m_Vbeta = 0.0

    def update(self, in_Va, in_Vb, in_Vc, T_L, dt):
        
        #----------------------------------------------
        #------------- Convert Input Data -------------
        #----------------------------------------------
        self.m_Valpha, self.m_Vbeta = ClPk.Clark(in_Va, in_Vb, in_Vc) 
        self.m_Ud, self.m_Uq = ClPk.Park(self.m_Valpha, self.m_Vbeta, self.m_Theta)
        
        #----------------------------------------------
        #------------- Update Model Calc --------------
        #----------------------------------------------
        dId = (self.m_Ud - self.Rs * self.m_Id + self.m_Omega_e * self.Lq * self.m_Iq) / self.Ld
        dIq = (self.m_Uq - self.Rs * self.m_Iq - self.m_Omega_e * self.Ld * self.m_Id - self.m_Omega_e * self.Psi_PM) / self.Lq
        self.m_Id += dId * dt
        self.m_Iq += dIq * dt
        
        #----------------------------------------------
        #----------- Eletromagnetic Torque ------------
        #----------------------------------------------
        self.m_Te = 1.5 * self.p * (self.Psi_PM * self.m_Iq + (self.Ld - self.Lq) * self.m_Id * self.m_Iq)
       
        #----------------------------------------------
        #-------------- Motor Speed Calc --------------
        #---------------------------------------------- 
        dOmega_e = (self.m_Te - self.B*self.m_Omega_e -  T_L) / self.J
        self.m_Omega_e += dOmega_e * dt
        
        #----------------------------------------------
        #------------ Rotor Position Calc -------------
        #---------------------------------------------- 
        self.m_Theta += self.m_Omega_e * dt
        
        if self.m_Theta > np.pi:
            self.m_Theta -= 2.0*np.pi
        if self.m_Theta < -np.pi:
            self.m_Theta += 2.0*np.pi
           
        #----------------------------------------------
        #-------------- 3ph Current Calc --------------
        #---------------------------------------------- 
        m_Ialpha, m_Ibeta = ClPk.Inv_Park(self.m_Id, self.m_Iq, self.m_Theta)

        m_Ia, m_Ib, m_Ic = ClPk.Inv_Clark(m_Ialpha, m_Ibeta)
        
        return [m_Ia, m_Ib, m_Ic]

    def get_torque(self):
        return self.m_Te

    def get_currents(self):
        return self.m_Id, self.m_Iq
    
    def get_omega_e_rad_s_(self):
        return self.m_Omega_e
    
    def get_omega_m_rpm_(self):
        return self.m_Omega_e / (self.p * np.pi /30.0)  
    
    
    def get_theta(self):
        return self.m_Theta