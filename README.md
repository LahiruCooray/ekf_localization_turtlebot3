# EKF Localization for TurtleBot3

Extended Kalman Filter for 2D localization using GPS and IMU sensor fusion in ROS 2 Jazzy + Gazebo Harmonic.

## Overview

This package implements a custom EKF for TurtleBot3 localization:

| Component | Source | Purpose |
|-----------|--------|---------|
| Prediction | cmd_vel | Motion model propagation |
| Position update | GPS | Absolute x, y correction |
| Orientation update | IMU | Absolute θ correction |

### State Vector

```
x = [x, y, θ]ᵀ
```

- x, y: Position in meters (ENU frame)
- θ: Orientation in radians

## Prerequisites

```bash
sudo apt install ros-jazzy-turtlebot3-gazebo \
                 ros-jazzy-turtlebot3-teleop \
                 ros-jazzy-turtlebot3-description \
                 ros-jazzy-ros-gz-bridge
```

## Installation

```bash
cd ~/repos/ekf_localization_turtlebot3
colcon build
source install/setup.bash
```

## Usage

### Launch Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch ekf_localization_tb3 ekf_localization.launch.py
```

### Teleop Control

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### View EKF Output

```bash
ros2 topic echo /ekf_pose
```

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| /cmd_vel | geometry_msgs/TwistStamped | Subscribe | Velocity commands |
| /gps/fix | sensor_msgs/NavSatFix | Subscribe | GPS position |
| /imu | sensor_msgs/Imu | Subscribe | IMU orientation |
| /ekf_pose | geometry_msgs/PoseWithCovarianceStamped | Publish | Fused pose |

## Parameters

Configure in `config/ekf_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| process_noise_x | 0.01 | Motion model uncertainty (m²) |
| process_noise_y | 0.01 | Motion model uncertainty (m²) |
| process_noise_theta | 0.01 | Motion model uncertainty (rad²) |
| gps_noise_x | 0.0001 | GPS measurement noise (m²) |
| gps_noise_y | 0.0001 | GPS measurement noise (m²) |
| imu_noise_theta | 0.001 | IMU measurement noise (rad²) |
| prediction_rate | 50.0 | Prediction frequency (Hz) |
| publish_tf | true | Broadcast TF transform |

## Package Structure

```
ekf_localization_tb3/
├── ekf_localization_tb3/
│   ├── ekf_core.py          # EKF algorithm
│   └── ekf_node.py          # ROS 2 node
├── config/
│   └── ekf_params.yaml      # Parameters
├── launch/
│   └── ekf_localization.launch.py
├── models/
│   └── turtlebot3_burger_gps/  # Custom model with GPS
├── worlds/
│   └── empty_gps.sdf        # World with NavSat support
└── package.xml
```

## Design Notes

See [EKF_DESIGN_NOTES.md](EKF_DESIGN_NOTES.md) for detailed design decisions and justifications.

See [GPS_INTEGRATION_NOTES.md](GPS_INTEGRATION_NOTES.md) for GPS configuration and coordinate handling.

## License

MIT
