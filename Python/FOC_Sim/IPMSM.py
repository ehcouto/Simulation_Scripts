'''
Created on 3 ott 2025

@author: 93600004
'''
# motor_model.py

import numpy as np

class IPMSMMotor:
    def __init__(self, Rs, Ld, Lq, Psi_PM, p, omega_e, B, J):
        self.Rs = Rs
        self.Ld = Ld
        self.Lq = Lq
        self.Psi_PM = Psi_PM
        self.p = p
        self.omega_e = omega_e
        self.Id = 0.0
        self.Iq = 0.0
        self.Te = 0.0
        self.J = J
        self.B = B
        self.theta = 0.0

    def update(self, Ud, Uq, T_L, dt):
        dId = (Ud - self.Rs * self.Id + self.omega_e * self.Lq * self.Iq) / self.Ld
        dIq = (Uq - self.Rs * self.Iq - self.omega_e * self.Ld * self.Id - self.omega_e * self.Psi_PM) / self.Lq
        self.Id += dId * dt
        self.Iq += dIq * dt
        
        self.Te = 1.5 * self.p * (self.Psi_PM * self.Iq + (self.Ld - self.Lq) * self.Id * self.Iq)
        
        dOmega_e = (self.Te - self.B*self.omega_e -  T_L) / self.J
        self.omega_e += dOmega_e * dt
        
        self.theta += self.omega_e * dt
        
        if self.theta > np.pi:
            self.theta -= 2.0*np.pi
        if self.theta < -np.pi:
            self.theta += 2.0*np.pi


    def get_torque(self):
        return self.Te

    def get_currents(self):
        return self.Id, self.Iq
    
    def get_omega_e_rad_s_(self):
        return self.omega_e
    
    def get_omega_m_rpm_(self):
        return self.omega_e * 60.0/ (self.p * 2.0 * np.pi)  
    
    
    def get_theta(self):
        return self.theta