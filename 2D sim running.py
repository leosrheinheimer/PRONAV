import numpy as np
import math 
import time
import support


class Vehicle : 
    def __init__ (self, pos, v): 
        self.pos = np.array(pos, dtype=float)
        self.v = np.array(v, dtype=float)
        self.a = np.array([0.0,0.0], dtype = float)
        self.hdg = np.array([], dtype = float)

    def tick (self, dt) :
        self.pos += self.v * dt
        self.v += self.a * dt
        self.hdg = np.arctan2(self.v[1],self.v[0])

target = Vehicle([100.0,0.0],[0.0,0.0])
pursuer = Vehicle([0.0,0.0],[20.0,0.0])

dt = 0.001

while True :
    #print stuff 
    target.tick(dt)
    pursuer.tick(dt)
    #relative v calc 
    print(math.degrees(target.hdg))
    print(f"error = {np.subtract(target.pos, pursuer.pos)}")
    if support.closing_dist(pursuer.pos,target.pos) < 1 :
        print("TARGET INTERCEPTED")
        break 
    
