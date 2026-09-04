# Scope and limitations

This project is a generic synthetic SLAM study.

Implemented:
- planar UAV motion,
- noisy inertial velocity/yaw-rate propagation,
- local 2D landmark observations from a camera-like stereo/depth front end,
- known landmark data association,
- EKF-SLAM,
- pose and map error metrics.

Not implemented:
- monocular scale estimation,
- feature extraction/descriptors,
- optical flow,
- ORB/SIFT/SURF,
- RANSAC,
- loop closure,
- pose graph optimization,
- bundle adjustment,
- IMU preintegration,
- camera calibration,
- 3D landmarks,
- real datasets,
- ROS2,
- hardware experiments.
