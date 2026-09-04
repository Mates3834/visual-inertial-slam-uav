# Visual-Inertial Landmark SLAM for Autonomous UAV Navigation

A research-oriented simulation framework for **UAV localization and landmark mapping in GPS-denied environments** using inertial propagation and camera-like landmark observations.

The project studies how visual landmark measurements can reduce the drift produced by inertial dead reckoning while simultaneously estimating a map of the environment.

## 1. Motivation

Autonomous UAV navigation normally benefits from an external positioning source such as GNSS. In GPS-denied or degraded environments, navigation must instead rely on onboard sensing.

A simplified visual-inertial navigation pipeline can be represented as:

```text
UAV Motion
   ↓
IMU-like Measurements ─────────────┐
   ↓                               │
Dead Reckoning                     │
                                   ↓
Camera-like Landmark Observations → EKF-SLAM
                                   ↓
                          UAV Pose Estimate
                                   +
                           Landmark Map
```

Inertial propagation provides high-rate motion information but accumulates drift. Visual landmark observations provide geometric information that can correct this drift.

## 2. Implemented Framework

The current implementation contains:

- Planar UAV motion simulation
- Synthetic IMU-like velocity and yaw-rate measurements
- Gyroscope bias and Gaussian sensor noise
- Synthetic camera-like local landmark observations
- Limited sensor range and field of view
- IMU-only dead reckoning
- Extended Kalman Filter SLAM
- Landmark initialization
- Simultaneous UAV pose and landmark estimation
- Known landmark data association
- Pose and map accuracy metrics
- Monte Carlo evaluation

## 3. UAV State

The UAV pose is represented by:

```text
x = planar x position
y = planar y position
ψ = heading
```

The kinematic model is:

```text
x_dot = V cos(ψ)
y_dot = V sin(ψ)
ψ_dot = ω
```

where `V` is forward velocity and `ω` is yaw rate.

The simulation uses time-varying velocity and yaw-rate commands to generate a curved trajectory through the landmark environment.

## 4. Synthetic IMU Model

The inertial front end provides noisy velocity and yaw-rate measurements:

```text
V_m = V + n_v

ω_m = ω + b_g + n_ω
```

where:

- `n_v` is velocity measurement noise
- `n_ω` is yaw-rate measurement noise
- `b_g` is a small gyroscope bias

These measurements are used both by the dead-reckoning baseline and the EKF prediction stage.

## 5. IMU-Only Dead Reckoning

The baseline navigation estimate integrates the noisy inertial measurements without visual correction.

```text
IMU
 ↓
Motion Integration
 ↓
Pose Estimate
 ↓
Accumulating Drift
```

This provides a reference against which visual-inertial fusion can be evaluated.

## 6. Landmark Environment

The simulated environment contains a set of static point landmarks:

```text
L_i = [x_i, y_i]
```

The landmarks are generic synthetic features and do not represent a real geographic environment.

## 7. Camera-Like Landmark Sensor

The visual sensor produces noisy relative landmark positions in the UAV body frame.

For a landmark position `p_L` and UAV position `p_U`:

```text
p_rel_world = p_L - p_U
```

The relative vector is rotated into the body frame:

```text
z = R_world_to_body(ψ) p_rel_world + v
```

where `v` represents measurement noise.

The sensor model also applies:

- Maximum observation range
- Limited field of view

This approximates a calibrated stereo/depth-capable visual front end. It does **not** claim monocular depth estimation.

## 8. EKF-SLAM State

The complete EKF-SLAM state is:

```text
X =
[
x
y
ψ
l1_x
l1_y
l2_x
l2_y
...
lN_x
lN_y
]^T
```

The filter therefore estimates both:

```text
UAV Pose
   +
Landmark Positions
```

within one covariance framework.

## 9. EKF Prediction

The EKF prediction stage uses the noisy inertial measurements.

Conceptually:

```text
X(k|k-1) = f(X(k-1|k-1), u_k)
```

and covariance propagation is:

```text
P(k|k-1)
=
F P(k-1|k-1) F^T
+
G Q G^T
```

where:

- `F` is the state-transition Jacobian
- `G` maps control uncertainty
- `Q` represents inertial input uncertainty

## 10. Landmark Initialization

When a landmark is observed for the first time, its body-frame measurement is transformed into the world frame.

```text
p_landmark
=
p_UAV
+
R_body_to_world(ψ) z
```

The landmark is then inserted into the EKF state and assigned a finite initialization covariance.

## 11. EKF Measurement Update

For an initialized landmark, the predicted camera-like observation is generated from the current pose and landmark estimate.

The innovation is:

```text
y_k = z_k - h(X_k)
```

The innovation covariance is:

```text
S_k = H_k P_k H_k^T + R
```

and the Kalman gain is:

```text
K_k = P_k H_k^T S_k^-1
```

The state is updated using:

```text
X_k = X_k + K_k y_k
```

The covariance update uses a numerically robust Joseph-form expression.

## 12. Data Association

The current synthetic simulation uses **known landmark associations**.

This means the simulator directly provides the identity of each observed landmark.

The project therefore focuses on:

```text
State Estimation
+
Sensor Fusion
+
Mapping
```

rather than the separate computer-vision problem of matching image features to landmarks.

## 13. Evaluation Metrics

The framework reports:

### Position RMSE

```text
RMSE_position =
sqrt(mean(||p_est - p_true||²))
```

### Heading RMSE

```text
RMSE_heading =
sqrt(mean(wrap(ψ_est - ψ_true)²))
```

### Final Position Error

```text
e_final =
||p_est(T) - p_true(T)||
```

### Landmark Map RMSE

```text
RMSE_map =
sqrt(mean(||L_est - L_true||²))
```

### Initialized Landmarks

The number of landmarks successfully observed and initialized is also reported.

## 14. Comparison

The primary experiment compares:

```text
Ground Truth
     ↓
 ┌───┴──────────────┐
 ↓                  ↓
IMU-Only         Visual + IMU
Dead Reckoning     EKF-SLAM
 ↓                  ↓
Pose Error       Pose + Map Error
```

This allows the effect of visual landmark corrections on inertial drift to be evaluated directly.

## 15. Repository Structure

```text
visual_inertial_slam_uav/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── docs/
│   └── scope.md
│
├── src/
│   ├── __init__.py
│   ├── simulator.py
│   ├── landmarks.py
│   ├── visual_sensor.py
│   ├── dead_reckoning.py
│   ├── ekf_slam.py
│   ├── metrics.py
│   └── simulation.py
│
├── examples/
│   ├── run_slam_demo.py
│   └── monte_carlo.py
│
└── results/
```

## 16. Installation

```bash
git clone <repository-url>
cd visual-inertial-slam-uav
pip install -r requirements.txt
```

Dependencies:

```text
NumPy
Matplotlib
```

## 17. Running the SLAM Demo

```bash
python examples/run_slam_demo.py
```

The script reports:

```text
IMU-only pose metrics
EKF-SLAM pose metrics
Landmark map RMSE
Number of initialized landmarks
```

and plots:

- Ground-truth UAV trajectory
- IMU-only trajectory
- EKF-SLAM trajectory
- True landmarks
- Estimated landmarks

## 18. Monte Carlo Evaluation

Run:

```bash
python examples/monte_carlo.py
```

The current Monte Carlo script repeats the synthetic simulation over multiple random seeds and reports mean:

- IMU-only position RMSE
- EKF-SLAM position RMSE
- Landmark RMSE

This evaluates sensitivity to measurement-noise realizations.

## 19. Example Sanity Check

A short generic sanity run of the current implementation produced approximately:

| Metric | Value |
|---|---:|
| IMU-only position RMSE | 0.208 |
| EKF-SLAM position RMSE | 0.110 |
| Landmark RMSE | 0.271 |
| Initialized landmarks | 4 |

These values are only a software sanity-check result from the included synthetic configuration and should not be interpreted as experimental UAV performance.

## 20. Technologies

- Python
- NumPy
- Matplotlib
- Extended Kalman Filter
- SLAM
- Visual-Inertial Sensor Fusion
- State Estimation
- UAV Navigation
- Numerical Simulation
- Monte Carlo Evaluation

## 21. Research Areas

The project is related to:

- Autonomous UAV Navigation
- GPS-Denied Navigation
- Visual-Inertial Navigation
- Simultaneous Localization and Mapping
- State Estimation
- Sensor Fusion
- Robotics
- Autonomous Systems

## 22. Current Scope and Limitations

The current implementation is intentionally lightweight.

Implemented:

```text
2D UAV Motion
Synthetic IMU
Camera-like Relative Landmark Measurements
Known Data Association
EKF-SLAM
Landmark Mapping
Dead-Reckoning Baseline
Pose / Map Metrics
Monte Carlo Simulation
```

Not implemented:

```text
Real Camera Images
ORB / SIFT / SURF
Optical Flow
RANSAC
Unknown Data Association
Loop Closure
Pose Graph Optimization
Bundle Adjustment
IMU Preintegration
Camera Calibration
Monocular Scale Estimation
3D Landmark Mapping
ROS2 Integration
Real UAV Hardware
Real Flight Data
```

The repository should therefore be interpreted as an **EKF-based visual-inertial landmark SLAM research simulation**, not a complete production visual-SLAM stack.

## 23. Future Extensions

Potential extensions include:

```text
Feature Extraction
      ↓
Feature Matching
      ↓
RANSAC
      ↓
Visual Odometry
      ↓
IMU Preintegration
      ↓
Visual-Inertial SLAM
```

Additional research directions include:

- Unknown data association
- 3D landmark representation
- Stereo image simulation
- Real datasets
- Loop closure
- Pose graph optimization
- Bundle adjustment
- ROS2 integration
- Gazebo/PX4 simulation
- Real UAV flight experiments

## 24. Public Implementation Notice

This repository contains a **generic and sanitized research implementation**.

All trajectories, landmarks, sensor parameters, and measurements are synthetic.

The repository contains no:

- Real operational UAV parameters
- Restricted navigation data
- Real surveillance data
- Proprietary sensor configurations
- Platform-specific flight-control logic
- Confidential datasets

## 25. Status

**Research-oriented simulation framework / active development**

The project demonstrates the complete estimation pipeline:

```text
UAV Motion
    ↓
Inertial Propagation
    +
Visual Landmark Observation
    ↓
EKF-SLAM
    ↓
Pose Estimation
    +
Landmark Mapping
    ↓
Quantitative Evaluation
```

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Guidance, Navigation and Control
- State Estimation
- Sensor Fusion
- SLAM
- UAV Autonomy
- Marine Robotics
- Model Predictive Control
- Reinforcement Learning
- Multi-Agent Systems
