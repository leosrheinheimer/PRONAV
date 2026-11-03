import numpy as np
import math


def run_log(hz,state_data):
    steps_per_log = int(round(1 / (hz * state_data['dt'])))
    if state_data['step_count'] % steps_per_log== 0 :
        print(f"time: t+ {state_data['timestamp']:.2f} seconds")
        print(f"distance to target: {closing_dist(state_data['pursuer_pos'],state_data['target_pos']):.2f} meters")
        print()

def calculate_los_rate(state):
    distance_vector = state['position_vector']
    v_relative = np.subtract(state['v_target'], state['v_pursuer'])
    r_3d = np.array([distance_vector[0], distance_vector[1], 0.0])
    v_3d = np.array([v_relative[0], v_relative[1], 0.0])

    distance_squared = np.sum(distance_vector**2) 
    cross_val = np.cross(r_3d, v_3d)
    
    los_rate = cross_val[2] / distance_squared
    return los_rate

def calculate_closing_velocity(state):
    distance_vector = state['position_vector']
    abs_distance = math.sqrt(distance_vector[0]**2 + distance_vector[1]**2)
    v_relative = np.subtract(state['v_target'], state['v_pursuer'])
    
    closing_v = -np.dot(distance_vector, v_relative) / abs_distance
    return closing_v

def calculate_dynamic_acceleration(state):
    distance_vector = state['position_vector']
    abs_distance = np.linalg.norm(distance_vector)
    
    if abs_distance == 0:
        return np.array([0.0, 0.0])
        
    v_pursuer = state['v_pursuer']
    v_mag = np.linalg.norm(v_pursuer)
    
    # Unit vectors
    u_r = distance_vector / abs_distance
    u_theta = np.array([-u_r[1], u_r[0]]) # Transverse direction for PN steering
    u_v = v_pursuer / v_mag if v_mag > 0 else np.array([1.0, 0.0]) # Heading direction
    
  
    # 1. Proportional Navigation Lateral Acceleration Magnitude
    nc = state['nav_constant']
    closing_v = state['closing_velocity']
    los_rate = state['los_rate']
    
    # --- SAFEGUARD: Check Closing Velocity & Seeker FOV ---
    cos_off_boresight = np.clip(np.dot(u_v, u_r), -1.0, 1.0)
    off_boresight_deg = np.degrees(np.arccos(cos_off_boresight))
    
    # Disable active PN steering if moving away (V_c <= 0) or target exceeds seeker FOV (60 deg)
    if closing_v <= 0 or off_boresight_deg > 60.0:
        a_lateral_mag = 0.0
    else:
        a_lateral_mag = nc * closing_v * los_rate
        
        # --- G-LIMITER ---
        g_0 = 9.81 # m/s^2
        max_lateral_accel = state['max_g'] * g_0
        if abs(a_lateral_mag) > max_lateral_accel:
            a_lateral_mag = np.sign(a_lateral_mag) * max_lateral_accel
        
    # Steering perpendicular to current velocity vector (Zero-Work turning)
    u_v_perp = np.array([-u_v[1], u_v[0]]) 
    a_lateral = a_lateral_mag * u_v_perp
    
    # 2. Longitudinal Forces (Thrust & Drag)
    # Check if motor is still active based on burn time
    if state['timestamp'] <= state['burn_time']:
        thrust = state['nominal_thrust']
    else:
        thrust = 0.0 # Motor burnout / coast phase
        
    mass = state['mass']
    rho = state['air_density']
    Cd = state['drag_coefficient']
    area = state['cross_sectional_area']
    
    # Aerodynamic Drag: D = 0.5 * rho * v^2 * Cd * A
    drag_mag = 0.5 * rho * (v_mag ** 2) * Cd * area
    
    f_thrust = thrust * u_v
    f_drag = -drag_mag * u_v
    
    # 3. Net Acceleration Vector (F = ma -> a = F/m)
    a_longitudinal = (f_thrust + f_drag) / mass
    a_net = a_longitudinal + a_lateral
    
    return a_net

def normalize_angle(angle) : 
    if angle < -180.0 : 
        normalized_angle = angle + 360.0
    elif angle > 180.0 : 
        normalized_angle = angle -360.0
    else :
        normalized_angle = angle
    return normalized_angle

def closing_dist (loc1 , loc2): 
    e_vector = np.subtract(loc1 , loc2) 
    e_dist = math.sqrt(e_vector[1]**2 + e_vector[0]**2)
    return e_dist

def error_vector(pursuer, target) :
    return np.subtract(target.pos,pursuer.pos)

def pol2cart(r,theta):
    x = r*np.cos(math.radians(theta))
    y = r*np.sin(math.radians(theta))
    return [x,y]
