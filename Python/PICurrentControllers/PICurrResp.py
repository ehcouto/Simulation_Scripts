


integ = 0.0


def PI(error):
    prop = error * 200.0
    tmp_i = error * 2500.0
    integ += tmp_i
    
    out = prop + integ
    
    if(out > 256.0):
        out = 256.0
    elif(out < -256.0):
        out = -256.0
    
    return out, integ



Ud, integ = PI(0.1)

print("Out: ", Ud)

