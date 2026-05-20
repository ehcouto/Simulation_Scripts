'''
Created on 10 ott 2025

@author: 93600004
'''

import numpy as np

SQRT3_DIV_2 = np.sqrt(3.0) / 2.0
INV_SQRT3   = 1.0 /np.sqrt(3.0)
INV_3       = 1.0 / 3.0

class ClarkPark:
    '''
        Implements the Clark & Park Transforms
    '''


    def __init__(self):
        '''
        '''
    
    def Clark(self, in_a, in_b, in_c):
        x_Alpha = in_a
        x_Beta = (in_b - in_c)*(INV_SQRT3)
        
        return [x_Alpha, x_Beta]
    
    def Inv_Clark(self, in_alpha, in_beta):
        x_A =  in_alpha
        x_B = -in_alpha*(0.5) + in_beta*(SQRT3_DIV_2)
        x_C = -in_alpha*(0.5) - in_beta*(SQRT3_DIV_2)
        
        return [x_A, x_B, x_C]
    
    def Park(self, in_alpha, in_beta, angle_rads):
        '''
        Input = Alpha/Beta variables 
        angle_rads should be the angle in rad/s
        
        Output = dq variables
        '''
        tmp_sin = np.sin(angle_rads)
        tmp_cos = np.cos(angle_rads)
        
        x_D = in_alpha * tmp_cos + in_beta * tmp_sin
        x_Q = -in_alpha * tmp_sin + in_beta * tmp_cos
        
        return [x_D, x_Q]
    
    def Inv_Park(self, in_D, in_Q, angle_rads):
        '''
        Input = DQ variables 
        angle_rads should be the angle in rad/s
        
        Output = Alpha/Beta variables
        '''
        tmp_sin = np.sin(angle_rads)
        tmp_cos = np.cos(angle_rads)
        
        x_Alpha = in_D * tmp_cos - in_Q * tmp_sin
        x_Beta  = in_D * tmp_sin + in_Q * tmp_cos
        
        return [x_Alpha, x_Beta]