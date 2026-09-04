import math
import numpy as np


def rotation_world_to_body(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, s],[-s, c]], dtype=float)


class VisualLandmarkSensor:
    """
    Synthetic camera-like local landmark-position sensor.

    Measurements are relative 2D landmark coordinates in the UAV body frame.
    This approximates a calibrated stereo/depth-capable visual front end and
    avoids claiming monocular depth recovery.
    """
    def __init__(self, max_range=28.0, fov_deg=150.0, sigma_xy=0.25, seed=11):
        self.max_range=max_range
        self.fov=math.radians(fov_deg)
        self.sigma_xy=sigma_xy
        self.rng=np.random.default_rng(seed)

    def observe(self, state, landmarks):
        p=np.array([state.x,state.y],dtype=float)
        R=rotation_world_to_body(state.heading)
        out=[]
        for i,lm in enumerate(landmarks):
            rel_w=lm-p
            r=float(np.linalg.norm(rel_w))
            if r > self.max_range or r < 1e-6:
                continue
            bearing=((math.atan2(rel_w[1],rel_w[0])-state.heading+math.pi)%(2*math.pi))-math.pi
            if abs(bearing) > self.fov/2:
                continue
            z=R@rel_w + self.rng.normal(0.0,self.sigma_xy,size=2)
            out.append((i,z))
        return out
