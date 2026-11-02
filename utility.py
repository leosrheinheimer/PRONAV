import numpy as np
import math
import matplotlib.pyplot as plt 


def run_log(hz,state_data):
    steps_per_log = int(round(1 / (hz * state_data['dt'])))
    if state_data['step_count'] % steps_per_log== 0 :
        print(f"time: t+ {state_data['timestamp']:.2f} seconds")
        print(f"distance to target: {closing_dist(state_data['pursuer_pos'],state_data['target_pos']):.2f} meters")
        print()


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
    return np.subtract(target,pursuer)
