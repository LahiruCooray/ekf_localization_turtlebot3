"""Extended Kalman Filter for 2D robot localization."""

import numpy as np


class EKFCore:
    """EKF with state [x, y, theta] using velocity prediction and GPS/IMU updates."""

    def __init__(
        self,
        initial_state: np.ndarray = None,
        initial_covariance: np.ndarray = None,
        process_noise: np.ndarray = None,
        imu_noise: np.ndarray = None,
        gps_noise: np.ndarray = None,
    ):
        self.x = initial_state if initial_state is not None else np.zeros(3)
        self.P = initial_covariance if initial_covariance is not None else np.eye(3) * 0.1
        self.Q = process_noise if process_noise is not None else np.diag([0.1, 0.1, 0.05])
        self.R_imu = imu_noise if imu_noise is not None else np.array([[0.01]])
        self.R_gps = gps_noise if gps_noise is not None else np.diag([0.5, 0.5])

        # Observation matrices
        self.H_imu = np.array([[0.0, 0.0, 1.0]])  # Observe theta
        self.H_gps = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # Observe x, y

    def predict(self, v: float, omega: float, dt: float) -> np.ndarray:
        """Predict state using velocity motion model from cmd_vel."""
        theta = self.x[2]

        # Motion model: x += v*cos(θ)*dt, y += v*sin(θ)*dt, θ += ω*dt
        dx = np.array([
            v * np.cos(theta) * dt,
            v * np.sin(theta) * dt,
            omega * dt
        ])
        self.x = self.x + dx
        self.x[2] = self._normalize_angle(self.x[2])

        # Jacobian of motion model
        F = np.array([
            [1.0, 0.0, -v * np.sin(theta) * dt],
            [0.0, 1.0, v * np.cos(theta) * dt],
            [0.0, 0.0, 1.0]
        ])
        self.P = F @ self.P @ F.T + self.Q

        return self.x.copy()

    def update_imu(self, z_theta: float) -> np.ndarray:
        """Update orientation using IMU measurement."""
        z = np.array([z_theta])
        y = z - self.H_imu @ self.x
        y[0] = self._normalize_angle(y[0])

        S = self.H_imu @ self.P @ self.H_imu.T + self.R_imu
        K = self.P @ self.H_imu.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.x[2] = self._normalize_angle(self.x[2])
        self.P = (np.eye(3) - K @ self.H_imu) @ self.P

        return self.x.copy()

    def update_gps(self, z_x: float, z_y: float) -> np.ndarray:
        """Update position using GPS measurement."""
        z = np.array([z_x, z_y])
        y = z - self.H_gps @ self.x

        S = self.H_gps @ self.P @ self.H_gps.T + self.R_gps
        K = self.P @ self.H_gps.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.x[2] = self._normalize_angle(self.x[2])
        self.P = (np.eye(3) - K @ self.H_gps) @ self.P

        return self.x.copy()

    def get_state(self) -> np.ndarray:
        """Return current state estimate."""
        return self.x.copy()

    def get_covariance(self) -> np.ndarray:
        """Return current covariance matrix."""
        return self.P.copy()

    def set_state(self, state: np.ndarray):
        """Set state directly."""
        self.x = state.copy()

    def set_covariance(self, covariance: np.ndarray):
        """Set covariance directly."""
        self.P = covariance.copy()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
