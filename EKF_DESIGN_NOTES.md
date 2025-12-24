# EKF Design Notes

This document describes the Extended Kalman Filter design for 2D robot localization.

## State Vector

The EKF maintains a 3-dimensional state:

```
x = [x, y, θ]ᵀ
```

| State | Description | Unit |
|-------|-------------|------|
| x | Position (East) | meters |
| y | Position (North) | meters |
| θ | Orientation (heading) | radians |

## System Architecture

### Prediction Step

**Source:** Velocity commands (`/cmd_vel`)

**Motion Model:**
```
x' = x + v × cos(θ) × Δt
y' = y + v × sin(θ) × Δt
θ' = θ + ω × Δt
```

Where:
- v = linear velocity (m/s)
- ω = angular velocity (rad/s)
- Δt = time step (0.02s at 50 Hz)

**Jacobian:**
```
F = | 1  0  -v×sin(θ)×Δt |
    | 0  1   v×cos(θ)×Δt |
    | 0  0        1      |
```

### Update Steps

| Sensor | Observable | Purpose |
|--------|------------|---------|
| GPS | x, y | Position correction |
| IMU | θ | Orientation correction |

## Design Decisions

### Why cmd_vel for Prediction?

The motion model uses velocity commands rather than odometry for prediction because:

1. **Decoupling:** Prediction is based on commanded motion, not measured motion
2. **Consistency:** The same motion model applies regardless of odometry quality
3. **Simplicity:** Single source for state propagation

### Why Not Use Odometry?

Wheel odometry was excluded from the update step because:

1. **Drift accumulation:** Odometry integrates errors over time
2. **No absolute reference:** Without SLAM or loop closure, drift cannot be corrected
3. **Redundancy:** GPS provides absolute position, odometry would be redundant

### Why Use GPS for Position?

GPS is used exclusively for position updates because:

1. **Absolute reference:** No drift accumulation
2. **Global consistency:** Position is referenced to a fixed coordinate frame
3. **Simplicity:** Direct measurement of x, y without integration

### Why Use IMU for Orientation?

IMU orientation is used because:

1. **Absolute reference:** Orientation relative to gravity and magnetic north
2. **High frequency:** Can update faster than GPS
3. **No integration:** Direct measurement, not integrated from angular velocity

## Covariance Matrices

### Process Noise (Q)

```
Q = diag(σ²ₓ, σ²ᵧ, σ²θ)
```

Default values: diag(0.01, 0.01, 0.01)

Represents uncertainty in the motion model.

### GPS Measurement Noise (R_gps)

```
R_gps = diag(σ²ₓ, σ²ᵧ)
```

Default values: diag(0.0001, 0.0001) = (0.01m)²

Represents GPS measurement uncertainty (~1 cm for RTK).

### IMU Measurement Noise (R_imu)

```
R_imu = [σ²θ]
```

Default value: 0.001 rad²

Represents IMU orientation uncertainty.

## Observation Matrices

### GPS Observation (H_gps)

```
H_gps = | 1  0  0 |
        | 0  1  0 |
```

Observes x and y from state.

### IMU Observation (H_imu)

```
H_imu = | 0  0  1 |
```

Observes θ from state.

## Assumptions

1. Robot operates on a flat surface (2D assumption)
2. IMU provides absolute orientation (magnetometer calibrated)
3. GPS provides centimeter-level accuracy (RTK GNSS)
4. Motion model accurately represents differential drive kinematics
5. Sensor noise is Gaussian with zero mean
