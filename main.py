import numpy as np
import utility
import math
from vehicle import Vehicle
import plotter



#sim conditions
dt = 0.001
cutoff_time = 30
step_count = 0

#vehicles
target_v_hdg_polar_deg = [300, 45]
pursuer_v_hdg_polar_deg = [200, 45]

target = Vehicle([2000.0, 2000.0], utility.pol2cart(target_v_hdg_polar_deg[0], target_v_hdg_polar_deg[1]))
pursuer = Vehicle([0.0, 0.0], utility.pol2cart(pursuer_v_hdg_polar_deg[0], pursuer_v_hdg_polar_deg[1]))

#constants
nav_constant = 20.0

#logging

while True :
    #logging 
    state_data = {
        'timestamp' : pursuer.time,
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
        'max_g' : 30.0,
        
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

    #sinusoidal weaving accel, g limiter
    max_target_g = 10.0  
    max_target_accel = max_target_g * 9.81
    
    # Desired raw evasive acceleration components
    raw_ax = 80.0 * np.sin(0.8 * pursuer.time)
    raw_ay = 60.0 * np.cos(0.6 * pursuer.time)
    target_a_desired = np.array([raw_ax, raw_ay])
    
    # Clamp the total acceleration vector magnitude to the G-limit
    target_accel_mag = np.linalg.norm(target_a_desired)
    if target_accel_mag > max_target_accel:
        target.a = (target_a_desired / target_accel_mag) * max_target_accel
    else:
        target.a = target_a_desired

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

#calling plotter
plotter.plot_trajectories(target, pursuer)