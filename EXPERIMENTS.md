# Parameter Experiments

This document records experiments with different EKF parameters using the same trajectory.

## Experiment Setup

- **Trajectory:** Figure-8 pattern (60 seconds)
- **Robot:** TurtleBot3 Burger with GPS
- **Simulator:** Gazebo Harmonic

## Experiments

### Experiment 1: Baseline (RTK GPS)

**Config:** `ekf_baseline.yaml` + Gazebo GPS noise: 0.0000001° (~1 cm)

| Parameter | Value |
|-----------|-------|
| process_noise | 0.01 |
| gps_noise | 0.0001 |
| Gazebo GPS stddev | 0.0000001° (~1 cm) |

**Results:**

| Metric | Raw GPS | EKF Estimate |
|--------|---------|--------------|
| X (m) | 0.092 | 0.078 |
| Y (m) | 0.177 | 0.182 |
| 2D (m) | 0.200 | 0.198 |
| Yaw (deg) | 5.20 | 5.19 |

**Observations:**
- EKF closely tracks GPS with centimeter-level accuracy
- Position errors within ±20 cm (X drifts +15cm, Y drifts -30cm)
- Y shows systematic drift due to GPS projection offset
- Covariance bounds are consistent with actual errors

---

### Experiment 2: Poor GPS

**Config:** `ekf_poor_gps.yaml` + Gazebo GPS noise: 0.000005° (~55 cm)

| Parameter | Value |
|-----------|-------|
| process_noise | 0.01 |
| gps_noise | 0.25 |
| Gazebo GPS stddev | 0.000005° (~55 cm) |

**Results:**

| Metric | Raw GPS | EKF Estimate |
|--------|---------|--------------|
| X (m) | 0.590 | 0.555 |
| Y (m) | 0.642 | 0.663 |
| 2D (m) | 0.872 | 0.865 |
| Yaw (deg) | 0.13 | 0.16 |

**Observations:**
- EKF shows minimal improvement (~1%) over raw GPS
- Position errors fluctuate ±1.5m during operation
- Errors remain within ±2σ bounds (filter is consistent)
- Trajectory plot shows significant noise in EKF estimate

---

### Experiment 3: High Process Noise

**Config:** `ekf_high_process_noise.yaml`

| Parameter | Value |
|-----------|-------|
| process_noise | 0.1 |
| gps_noise | 0.0001 |

**Results:**

| Metric | RMSE |
|--------|------|
| X (m) | - |
| Y (m) | - |
| 2D (m) | - |

**Observations:**
- (To be filled after experiment)

---

### Experiment 4: Low Process Noise

**Config:** `ekf_low_process_noise.yaml`

| Parameter | Value |
|-----------|-------|
| process_noise | 0.001 |
| gps_noise | 0.0001 |

**Results:**

| Metric | RMSE |
|--------|------|
| X (m) | - |
| Y (m) | - |
| 2D (m) | - |

**Observations:**
- (To be filled after experiment)

---

## How to Run Experiments

```bash
# 1. Launch simulation with specific config
ros2 launch ekf_localization_tb3 ekf_localization.launch.py config:=ekf_baseline.yaml

# 2. Start data logger with experiment name
ros2 run ekf_localization_tb3 data_logger --ros-args -p experiment:=baseline

# 3. Run trajectory
ros2 run ekf_localization_tb3 trajectory_publisher --ros-args -p trajectory:=figure8 -p duration:=60.0

# 4. Wait for trajectory to complete, then Ctrl+C the logger

# 5. Generate plots
python3 ~/repos/ekf_localization_turtlebot3/src/ekf_localization_tb3/scripts/plot_ekf_data.py ~/repos/ekf_localization_turtlebot3/experiments/baseline/
```

## Comparison Summary

| Experiment | GPS Noise | Process Noise | 2D RMSE (m) | Observation |
|------------|-----------|---------------|-------------|-------------|
| Baseline | 0.0001 | 0.01 | 0.198 | Best accuracy, centimeter-level |
| Poor GPS | 0.25 | 0.01 | 0.865 | ~4x worse, high noise |
| High Proc Noise | 0.0001 | 0.1 | - | (To run) |
| Low Proc Noise | 0.0001 | 0.001 | - | (To run) |

