import numpy as np
import math 
import matplotlib as plt
import utility
from vehicle import Vehicle

dt = 0.001

target = Vehicle([100.0,0.0],[0.0,0.0])
pursuer = Vehicle([0.0,0.0],[20.0,0.0])

while True :
    #tick
    target.tick(dt)
    pursuer.tick(dt)

    print(f"error = {np.subtract(target.pos, pursuer.pos)}")
    print(f"error = {utility.error_vector(target.pos, pursuer.pos)}")
    if utility.closing_dist(pursuer.pos,target.pos) < 1 :
        print("TARGET INTERCEPTED")
        break 
    
