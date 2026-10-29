import numpy as np
import math
import matplotlib as plt 
import OOP practice as or

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


def graph () :
    plt.figure(figsize=(8, 8))
    plt.plot(seeker_x, seeker_y, label="Seeker Path", color="blue", linewidth=2)
    plt.plot(
        target_x, target_y, label="Target Path", color="red", linestyle="--", linewidth=2)

    plt.title("Proportional Navigation Trajectory")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()