import math
import numpy as np


def wrap(a):
    return (a + math.pi)%(2*math.pi)-math.pi


def Rwb(theta):
    c,s=math.cos(theta),math.sin(theta)
    return np.array([[c,s],[-s,c]],dtype=float)


class EKFSLAM:
    """
    EKF-SLAM with state:
      [x, y, theta, l1_x, l1_y, l2_x, l2_y, ...]

    The visual measurement for landmark j is its noisy relative 2D position in
    the vehicle body frame. Data association is known in this synthetic demo.
    """
    def __init__(self, n_landmarks, dt=0.05):
        self.n_landmarks=n_landmarks
        self.dt=dt
        self.n=3+2*n_landmarks
        self.x=np.zeros(self.n,dtype=float)
        self.P=np.eye(self.n)*1e-6
        for j in range(n_landmarks):
            idx=3+2*j
            self.P[idx:idx+2,idx:idx+2]=np.eye(2)*1e6
        self.initialized=np.zeros(n_landmarks,dtype=bool)
        self.Qu=np.diag([0.08**2,0.012**2])
        self.R=np.eye(2)*(0.25**2)

    def predict(self, v, w):
        dt=self.dt
        th=self.x[2]
        thn=wrap(th+w*dt)
        self.x[0]+=v*math.cos(thn)*dt
        self.x[1]+=v*math.sin(thn)*dt
        self.x[2]=thn

        F=np.eye(self.n)
        F[0,2]=-v*math.sin(thn)*dt
        F[1,2]= v*math.cos(thn)*dt

        G=np.zeros((self.n,2))
        G[0,0]=math.cos(thn)*dt
        G[1,0]=math.sin(thn)*dt
        G[0,1]=-v*math.sin(thn)*dt*dt
        G[1,1]= v*math.cos(thn)*dt*dt
        G[2,1]=dt

        self.P=F@self.P@F.T + G@self.Qu@G.T
        self.P=(self.P+self.P.T)/2

    def _initialize_landmark(self, j, z_body):
        idx=3+2*j
        th=self.x[2]
        c,s=math.cos(th),math.sin(th)
        Rbw=np.array([[c,-s],[s,c]],dtype=float)
        lm=self.x[:2]+Rbw@z_body
        self.x[idx:idx+2]=lm

        # Conservative but finite covariance after initialization.
        self.P[idx:idx+2,idx:idx+2]=np.eye(2)*1.5
        self.initialized[j]=True

    def update(self, j, z):
        if not self.initialized[j]:
            self._initialize_landmark(j,z)
            return

        idx=3+2*j
        px,py,th=self.x[:3]
        lx,ly=self.x[idx:idx+2]
        dx,dy=lx-px,ly-py
        c,s=math.cos(th),math.sin(th)

        zhat=np.array([c*dx+s*dy, -s*dx+c*dy])

        H=np.zeros((2,self.n))
        # d(body rel)/d vehicle position
        H[:,0:2]=np.array([[-c,-s],[s,-c]])
        # d(body rel)/d theta
        H[:,2]=np.array([-s*dx+c*dy, -c*dx-s*dy])
        # d(body rel)/d landmark
        H[:,idx:idx+2]=np.array([[c,s],[-s,c]])

        y=np.asarray(z)-zhat
        S=H@self.P@H.T+self.R
        K=self.P@H.T@np.linalg.inv(S)
        self.x=self.x+K@y
        self.x[2]=wrap(self.x[2])
        I=np.eye(self.n)
        self.P=(I-K@H)@self.P@(I-K@H).T + K@self.R@K.T
        self.P=(self.P+self.P.T)/2

    @property
    def pose(self):
        return self.x[:3].copy()

    def map_estimate(self):
        out=np.full((self.n_landmarks,2),np.nan)
        for j in range(self.n_landmarks):
            if self.initialized[j]:
                out[j]=self.x[3+2*j:3+2*j+2]
        return out
