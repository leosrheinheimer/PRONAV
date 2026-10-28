import math
import numpy as np
import matplotlib.pyplot as plt

t=0
t_hist = []
dt = 0.0025
timeout_seconds = 15
step_count = 0


#target
target_heading = 0
target_pos = np.array([1000.0,100.0])
target_hist = []
target_v = np.array([200.0,0.0])
target_a = np.array([0.0,0.0])


#seeker
seeker_heading = 0
seeker_pos = np.array([0.0,0.0])
seeker_hist = []
seeker_v = np.array([50.0,0.0])
seeker_a = np.array([0.0,0.0])
seeker_forward_accel = 10

#tracking
g_limit = 10
max_accel = g_limit * 9.81
los_angle = 0
N = 5

## FUNCTIONS

def distance_to_target() : 
    return math.sqrt(((target_pos[0] - seeker_pos[0])**2 + (target_pos[1] - seeker_pos[1])**2 ))

def relative_velocity() : 
    return math.sqrt(((target_v[0] - seeker_v[0])**2 + (target_v[1] - seeker_v[1])**2 ))

def closing_velocity() :
    r = error()
    v_relative = target_v - seeker_v 
    distance = np.linalg.norm(r)
    return -np.dot(r,v_relative) / distance 

def normalize_angle(angle) : 
    if angle < -180.0 : 
        normalized_angle = angle + 360.0
    elif angle > 180.0 : 
        normalized_angle = angle -360.0
    else :
        normalized_angle = angle
    return normalized_angle

def tick(p1,v1,a1,dt) : 
    p1 += v1 * dt
    v1 += a1 * dt

def error() :
    return np.subtract(target_pos, seeker_pos)

def accelerate(forward_accel,seeker_a): 
    seeker_a[0] = math.cos(seeker_heading) * forward_accel
    seeker_a[1] = math.sin(seeker_heading) * forward_accel


los_angle = math.degrees(math.atan2(error()[1],error()[0]))

while True :
##evasive maneuvers 
  #  target_a[0] = 30.0 * math.sin(0.8 * t)
   # target_a[1] = 20.0 * math.cos(0.5 * t)

##targeting system 
    prev_vc = closing_velocity()
    seeker_a = np.array([0,0])
    accelerate(seeker_forward_accel,seeker_a)
    prev_los_angle = los_angle
    seeker_heading = math.atan2(seeker_v[1],seeker_v[0])
    los_angle = math.degrees(math.atan2(error()[1],error()[0]))
    los_angle_rad = math.radians(los_angle)
    los_angle_diff = los_angle - prev_los_angle
    normalized_diff = normalize_angle(los_angle_diff)
    los_rate = normalized_diff / dt

##pronav

    if los_angle < 10 :
        seeker_forward_accel = 50
    else :
        seeker_forward_accel = 10
    
    commanded_accel = N * closing_velocity() * math.radians(los_rate)
    clamped_accel = max(-max_accel, min(commanded_accel,max_accel))
    steer_x = -clamped_accel * math.sin(los_angle_rad)
    steer_y = clamped_accel * math.cos(los_angle_rad)
    seeker_a[0] += steer_x
    seeker_a[1] += steer_y
    seeker_pos_prev = seeker_pos.copy()
    target_pos_prev = target_pos.copy()
    
##ticking
    t += dt
    step_count += 1
    tick(target_pos,target_v,target_a,dt)
    tick(seeker_pos,seeker_v,seeker_a,dt)

##print info
    if step_count % 100 == 0 :
        print(round(t,2))
        print(f"Distance To Target ={float(distance_to_target()):.3f}")
        print(f"target position = ({target_pos[0]:.1f},{target_pos[1]:.1f})")
        print(f"seeker position = ({seeker_pos[0]:.1f},{seeker_pos[1]:.1f})")
        print(f"closing V={closing_velocity():.4}")
        print(f"Distance to Target ={math.sqrt(error()[0]**2 + error()[1]**2):.4}")


##updating log
    if step_count % 10 == 0 :
        t_hist.append(t)
        seeker_hist.append(seeker_pos.copy())
        target_hist.append(target_pos.copy())

##outcomes
    if closing_velocity() <= 0 and prev_vc >0 and distance_to_target()<1 :
        miss_distance = distance_to_target()
        print(f"Target Intercepted In {t:.3} Seconds")
        print(f"Closest approach ={distance_to_target():.3}")
        step_fraction = prev_vc/(prev_vc-closing_velocity())
        seeker_pos_cpa = seeker_pos_prev + (seeker_pos - seeker_pos_prev) * step_fraction
        target_pos_cpa = target_pos_prev + (target_pos - target_pos_prev) * step_fraction
        closest_approach_vector = target_pos_cpa - seeker_pos_cpa
        real_closest_approach = math.sqrt(closest_approach_vector[0]**2 + closest_approach_vector[1]**2)
        print(f"REAL closest approach = {real_closest_approach:.2}")
        break 
    if t >= timeout_seconds : 
        print('Target Not Intercepted In Time')
        break

seeker_x = [pos[0] for pos in seeker_hist]
seeker_y = [pos[1] for pos in seeker_hist]

target_x = [pos[0] for pos in target_hist]
target_y = [pos[1] for pos in target_hist]

plt.figure(figsize=(8, 8))
plt.plot(seeker_x, seeker_y, label="Seeker Path", color="blue", linewidth=2)
plt.plot(
    target_x, target_y, label="Target Path", color="red", linestyle="--", linewidth=2
)

plt.title("Proportional Navigation Trajectory, N="+str(N))
plt.xlabel("X Position (meters)")
plt.ylabel("Y Position (meters)")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()