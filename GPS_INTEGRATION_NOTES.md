# GPS Integration Notes

This document describes the GPS (GNSS) integration for the EKF localization system.

## Sensor Configuration

### NavSat Sensor in Gazebo

A NavSat sensor is attached to the TurtleBot3 model to simulate GPS measurements. The sensor is configured with RTK-like accuracy.

**Noise Parameters:**

| Parameter | Value | Equivalent |
|-----------|-------|------------|
| Horizontal stddev | 0.0000001° | ~1.1 cm |
| Vertical stddev | 0.0000003° | ~3.3 cm |

These values approximate centimeter-level RTK GNSS performance.

### World Spherical Coordinates

The simulation world uses WGS84 spherical coordinates with the origin at Colombo, Sri Lanka:

- Latitude: 6.9271°
- Longitude: 79.8612°
- Elevation: 0 m

## Coordinate Conversion

GPS measurements are converted from geodetic (lat/lon) to local Cartesian (x, y) coordinates using equirectangular projection:

```
x = R × (lon - lon₀) × cos(lat₀)
y = R × (lat - lat₀)
```

Where R = 6,371,000 m (Earth radius). This projection is accurate for small areas (< 10 km).

The first GPS fix is used as the local origin (x=0, y=0).

## Covariance Handling

### Issue

Gazebo's NavSat sensor does not populate the `position_covariance` field in the `NavSatFix` message. All covariance values are zero with `position_covariance_type = 0` (unknown).

### Solution

GPS measurement uncertainty is modeled at the estimator level. The EKF configuration includes `gps_noise_x` and `gps_noise_y` parameters that define the measurement noise covariance:

```yaml
gps_noise_x: 0.0001  # (0.01 m)² = 1 cm standard deviation
gps_noise_y: 0.0001  # (0.01 m)² = 1 cm standard deviation
```

This approach is valid because:
1. Sensor noise is still simulated in Gazebo
2. The EKF is informed of the expected measurement uncertainty
3. Sensor fusion operates correctly

## Assumptions

1. GPS provides absolute position without drift
2. RTK-like accuracy is assumed (centimeter-level)
3. Equirectangular projection is valid for the operating area
4. GPS updates are received at 10 Hz
