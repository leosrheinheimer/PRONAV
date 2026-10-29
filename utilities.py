import numpy as np
import math
import matplotlib as plt 

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