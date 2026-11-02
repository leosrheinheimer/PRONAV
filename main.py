import numpy as np
import math 
import matplotlib.pyplot as plt
import utility
from vehicle import Vehicle



#sim conditions
dt = 0.001
cutoff_time = 20
step_count = 0

#vehicles
target = Vehicle([100.0,0.0],[0.0,0.0])
pursuer = Vehicle([0.0,0.0],[20.0,0.0])

#logging

while True :
    #ticking
    target.tick(dt)
    pursuer.tick(dt)
    step_count += 1

    #logging 
    state_data = {
        'timestamp' : step_count * dt,
        'target_pos' : target.pos ,
        'pursuer_pos' : pursuer.pos,
        'dt' : dt,
        'step_count' : step_count
    }

    utility.run_log(50,state_data)

    #control
    los_rate_current = utility.calculate_los_rate(pursuer.pos, pursuer.v, target.pos, target.v)
    

    #simming
    if utility.closing_dist(pursuer.pos,target.pos) < 1 :
        print("TARGET INTERCEPTED")
        break 
    elif step_count / dt == cutoff_time :
        print("TARGET NOT INTERCEPTED IN TIME")
        break
