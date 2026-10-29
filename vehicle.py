import numpy as np

class Vehicle : 
    def __init__ (self, pos, v): 
        self.pos = np.array(pos, dtype=float)
        self.v = np.array(v, dtype=float)
        self.a = np.array([0.0,0.0], dtype = float)
        self.hdg = 0.0

    def tick (self, dt) :
        self.pos += self.v * dt
        self.v += self.a * dt
        self.hdg = np.arctan2(self.v[1],self.v[0])
