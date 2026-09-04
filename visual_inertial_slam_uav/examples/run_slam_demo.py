import matplotlib.pyplot as plt
from src.simulation import run

r=run()
print("IMU-only:",r["dead_reckoning_metrics"])
print("EKF-SLAM:",r["ekf_metrics"])
print("Landmark RMSE:",r["landmark_rmse"])
print("Initialized landmarks:",r["initialized_landmarks"])

plt.figure()
plt.plot(r["truth"][:,0],r["truth"][:,1],label="Ground truth")
plt.plot(r["dead_reckoning"][:,0],r["dead_reckoning"][:,1],label="IMU-only")
plt.plot(r["ekf"][:,0],r["ekf"][:,1],label="EKF-SLAM")
plt.scatter(r["landmarks"][:,0],r["landmarks"][:,1],marker="x",label="True landmarks")
mask=~(r["landmark_estimates"][:,0] != r["landmark_estimates"][:,0])
plt.scatter(r["landmark_estimates"][mask,0],r["landmark_estimates"][mask,1],
            marker="+",label="Estimated landmarks")
plt.axis("equal")
plt.grid(True)
plt.xlabel("x")
plt.ylabel("y")
plt.title("Visual-Inertial Landmark SLAM")
plt.legend()
plt.show()
