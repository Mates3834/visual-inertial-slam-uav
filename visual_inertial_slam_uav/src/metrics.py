import numpy as np


def wrap(a):
    return (a + np.pi)%(2*np.pi)-np.pi


def pose_metrics(truth, est):
    d=est[:,:2]-truth[:,:2]
    pos=np.linalg.norm(d,axis=1)
    h=wrap(est[:,2]-truth[:,2])
    return {
        "position_rmse": float(np.sqrt(np.mean(pos**2))),
        "heading_rmse_deg": float(np.degrees(np.sqrt(np.mean(h**2)))),
        "final_position_error": float(pos[-1])
    }


def landmark_rmse(true_landmarks, est_landmarks):
    mask=np.isfinite(est_landmarks[:,0])
    if not np.any(mask):
        return float("nan")
    e=est_landmarks[mask]-true_landmarks[mask]
    return float(np.sqrt(np.mean(np.sum(e**2,axis=1))))
