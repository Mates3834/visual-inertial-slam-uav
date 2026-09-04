# Visual-Inertial Landmark SLAM for Autonomous UAV Navigation

A generic research-oriented simulation framework for studying **UAV localization
and landmark mapping in GPS-denied environments** using inertial propagation and
camera-like landmark observations.

The project includes:

- Planar UAV motion simulation
- IMU-like noisy velocity and yaw-rate propagation
- Camera-like local landmark observations
- EKF-SLAM state estimation
- Landmark initialization and mapping
- Known data association for controlled experiments
- IMU-only dead-reckoning baseline
- Pose RMSE / trajectory error evaluation
- Landmark-map RMSE evaluation
- Reproducible synthetic scenarios

The implementation is intentionally lightweight and educational. It is not a
production visual-inertial odometry or full bundle-adjustment SLAM system.

## Architecture

```text
UAV Ground Truth
      ↓
Synthetic IMU -------------------┐
      ↓                          │
Dead Reckoning                   │
                                 ↓
Synthetic Camera → Landmark Observations
                                 ↓
                           EKF-SLAM
                                 ↓
                     Pose + Landmark Map
                                 ↓
                     Quantitative Metrics
```

## Run

```bash
pip install -r requirements.txt
python examples/run_slam_demo.py
python examples/monte_carlo.py
```

## Main Metrics

- Position RMSE
- Heading RMSE
- Final position error
- IMU-only vs fused pose error
- Landmark map RMSE
- Number of initialized landmarks

## Scope

This repository uses synthetic measurements and known landmark associations.
It does not claim ORB/SIFT feature extraction, loop-closure detection, bundle
adjustment, real-camera calibration, real IMU preintegration, 3D mapping, or
hardware validation.
