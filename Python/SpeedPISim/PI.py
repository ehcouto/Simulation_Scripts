'''
Created on 3 ott 2025

@author: 93600004
'''
# motor_model.py

class PI_Control:
    def __init__(self, Kp, Ki, l_lim, h_lim):
        self.out = 0.0
        self.integ = 0.0
        self.Kp = Kp
        self.Ki = Ki
        self.l_lim = l_lim
        self.h_lim = h_lim

    def calc(self, error):
        temp_ki = error * self.Ki
        self.integ += temp_ki
        out = self.Kp * error + self.integ
        
        if(out > self.h_lim):
            out = self.h_lim
        elif(out < self.l_lim):
            out = self.l_lim
        
        self.out = out 
        return self.out
    
    def get_integ(self):
        return self.integ