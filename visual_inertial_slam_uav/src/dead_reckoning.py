import math
import numpy as np


def wrap(a):
    return (a + math.pi)%(2*math.pi)-math.pi


class DeadReckoning:
    def __init__(self, dt=0.05):
        self.dt=dt
        self.x=np.zeros(3,dtype=float)

    def step(self, v, w):
        th=self.x[2]+w*self.dt
        self.x[0]+=v*math.cos(th)*self.dt
        self.x[1]+=v*math.sin(th)*self.dt
        self.x[2]=wrap(th)
        return self.x.copy()
