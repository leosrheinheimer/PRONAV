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

#constants
nav_constant = 3.0

#logging

while True :
    #logging 
    state_data = {
        'timestamp' : step_count * dt,
        'target_pos' : target.pos,
        'v_target' : target.v,
        'pursuer_pos' : pursuer.pos,
        'v_pursuer' : pursuer.v,
        'dt' : dt,
        'step_count' : step_count,
        'nav_constant' : 4.0,
        'position_vector' : np.subtract(target.pos, pursuer.pos),
        
        # AIM-9M Sidewinder Physical Parameters 
        'mass' : 85.5,  #kg
        'max_g' : 35.0,
        
        # Motor parameters
        'burn_time' : 5.2, # seconds
        'nominal_thrust' : 12800.0, # Newtons 
        
        # Aerodynamics
        'cross_sectional_area' : 0.05067, # m^2 (5 sq in circle)
        'drag_coefficient' : 0.3,        # Typical supersonic body Cd
        'air_density' : 1.225            # kg/m^3 (sea level)
    }

    utility.run_log(50,state_data)

    # update state 
    state_data['closing_velocity'] = utility.calculate_closing_velocity(state_data)
    state_data['los_rate'] = utility.calculate_los_rate(state_data) 


    #control
    pursuer.a = utility.calculate_dynamic_acceleration(state_data)

    #simming
    if utility.closing_dist(pursuer.pos,target.pos) < 1 :
        print("TARGET INTERCEPTED")
        break 
    elif state_data['timestamp'] >= cutoff_time :
        print("TARGET NOT INTERCEPTED IN TIME")
        break

    #ticking
    target.tick(dt)
    pursuer.tick(dt)
    step_count += 1
