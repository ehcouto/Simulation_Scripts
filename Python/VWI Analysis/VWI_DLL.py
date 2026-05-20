import numpy as np

Kp = 1.0
Ki = 1.0
Integ_K_1 = 0.0

Upper_Limit = 2.8
Lower_Limit = 0.0
    
    
Err = -0.38-(-0.39);


for k in np.arange(1000): # for any active power input...
    #calculate proportional term
    prop_term = Kp * Err;
    
    #calculate integral term step k
    integ_term = Ki * Err;
    
    #prepare the output for saturation
    out = prop_term + integ_term  + Integ_K_1;
    
    #saturate and apply anti-windup to the state
    if out > Upper_Limit:
        Output = Upper_Limit;
        if integ_term > 0.0:
            integ_term = 0.0;
    elif out < Lower_Limit:
        Output = Lower_Limit;
        if (integ_term<0.0):
            integ_term = 0.0;
    else:
       Output = out;
    
    Integ_K_1 += integ_term;


print("Output:",Output)
print("\n")
print("Integral:",Integ_K_1)