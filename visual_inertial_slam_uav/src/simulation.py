import numpy as np
from .simulator import UAVSimulator
from .landmarks import default_landmarks
from .visual_sensor import VisualLandmarkSensor
from .dead_reckoning import DeadReckoning
from .ekf_slam import EKFSLAM
from .metrics import pose_metrics, landmark_rmse


def run(duration=45.0, dt=0.05, seed=7):
    sim=UAVSimulator(dt=dt,seed=seed)
    lms=default_landmarks()
    vis=VisualLandmarkSensor(seed=seed+10)
    dr=DeadReckoning(dt=dt)
    ekf=EKFSLAM(len(lms),dt=dt)

    truth=[]; dr_hist=[]; ekf_hist=[]; visible=[]

    for k in range(int(duration/dt)):
        t=k*dt
        v,w=sim.command(t)
        gt=sim.step(v,w)
        vm,wm=sim.imu_measurement(v,w)
        dr_pose=dr.step(vm,wm)
        ekf.predict(vm,wm)
        obs=vis.observe(gt,lms)
        for j,z in obs:
            ekf.update(j,z)

        truth.append([gt.x,gt.y,gt.heading])
        dr_hist.append(dr_pose)
        ekf_hist.append(ekf.pose)
        visible.append(len(obs))

    truth=np.asarray(truth)
    dr_hist=np.asarray(dr_hist)
    ekf_hist=np.asarray(ekf_hist)
    lm_est=ekf.map_estimate()

    return {
        "truth":truth,
        "dead_reckoning":dr_hist,
        "ekf":ekf_hist,
        "landmarks":lms,
        "landmark_estimates":lm_est,
        "visible_counts":np.asarray(visible),
        "dead_reckoning_metrics":pose_metrics(truth,dr_hist),
        "ekf_metrics":pose_metrics(truth,ekf_hist),
        "landmark_rmse":landmark_rmse(lms,lm_est),
        "initialized_landmarks":int(np.sum(ekf.initialized))
    }
