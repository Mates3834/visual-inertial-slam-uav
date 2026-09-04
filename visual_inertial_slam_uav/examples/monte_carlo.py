import numpy as np
from src.simulation import run

rows=[]
for seed in range(20):
    r=run(seed=seed)
    rows.append([
        r["dead_reckoning_metrics"]["position_rmse"],
        r["ekf_metrics"]["position_rmse"],
        r["landmark_rmse"]
    ])

a=np.asarray(rows)
print("20-run Monte Carlo")
print("Mean IMU-only position RMSE:",float(np.mean(a[:,0])))
print("Mean EKF-SLAM position RMSE:",float(np.mean(a[:,1])))
print("Mean landmark RMSE:",float(np.nanmean(a[:,2])))
