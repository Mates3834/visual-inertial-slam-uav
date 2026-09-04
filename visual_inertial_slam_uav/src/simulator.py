from dataclasses import dataclass
import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


@dataclass
class UAVState:
    x: float
    y: float
    heading: float


class UAVSimulator:
    def __init__(self, dt=0.05, seed=7):
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.state = UAVState(0.0, 0.0, 0.0)

    def command(self, t):
        v = 3.0 + 0.4*np.sin(0.05*t)
        w = 0.10*np.sin(0.035*t) + 0.045*np.cos(0.08*t)
        return float(v), float(w)

    def step(self, v, w):
        dt = self.dt
        h = wrap(self.state.heading + w*dt)
        x = self.state.x + v*math.cos(h)*dt
        y = self.state.y + v*math.sin(h)*dt
        self.state = UAVState(x, y, h)
        return self.state

    def imu_measurement(self, v, w, sigma_v=0.08, sigma_w=0.01, gyro_bias=0.004):
        vm = v + self.rng.normal(0.0, sigma_v)
        wm = w + gyro_bias + self.rng.normal(0.0, sigma_w)
        return float(vm), float(wm)
