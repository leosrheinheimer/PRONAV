import numpy as np

class Vehicle : 
    def __init__ (self, pos, v): 
        self.pos = np.array(pos, dtype=float)
        self.v = np.array(v, dtype=float)
        self.a = np.array([0.0,0.0], dtype = float)
        self.hdg = 0.0
        self.history = []
        self.time = 0.0

    def tick (self, dt) :
        #logging
        self.history.append({
            'timestamp': self.time,
            'pos': self.pos.copy(),
            'v': self.v.copy(),
            'a': self.a.copy(),
            'hdg': self.hdg
        })
        #ticking
        self.pos += self.v * dt
        self.v += self.a * dt
        self.hdg = np.arctan2(self.v[1],self.v[0])

        #update internal clock
        self.time += dt

    
