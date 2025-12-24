"""Data logger for EKF performance analysis with ground truth."""

import csv
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from quaternion."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


class DataLogger(Node):
    """Log EKF, ground truth (odom), GPS, and IMU data for visualization."""

    def __init__(self):
        super().__init__('data_logger')

        # Create output directory in workspace
        self.output_dir = os.path.expanduser(
            '~/repos/ekf_localization_turtlebot3/ekf_logs'
        )
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'ekf_data_{timestamp}.csv')

        # Open CSV file
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'gt_x', 'gt_y', 'gt_theta',
            'ekf_x', 'ekf_y', 'ekf_theta',
            'ekf_cov_x', 'ekf_cov_y', 'ekf_cov_theta',
            'gps_x', 'gps_y',
            'imu_theta'
        ])

        # Origins for zero-referencing
        self.gps_origin = None
        self.gps_initialized = False
        self.odom_origin = None
        self.odom_initialized = False

        # Latest sensor values (relative to origin)
        self.latest_gt_x = 0.0
        self.latest_gt_y = 0.0
        self.latest_gt_theta = 0.0
        self.latest_gps_x = 0.0
        self.latest_gps_y = 0.0
        self.latest_imu_theta = 0.0
        self.start_time = None

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped, '/ekf_pose', self.ekf_callback, 10
        )
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10
        )
        self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )

        self.get_logger().info(f'Logging to: {self.csv_path}')
        self.get_logger().info('Using /odom as ground truth (zero-referenced)')

    def odom_callback(self, msg: Odometry):
        """Store ground truth from Gazebo odometry (zero-referenced)."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        raw_theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Set origin on first reading
        if not self.odom_initialized:
            self.odom_origin = (raw_x, raw_y, raw_theta)
            self.odom_initialized = True
            self.get_logger().info(
                f'Odom origin: x={raw_x:.2f}, y={raw_y:.2f}, theta={raw_theta:.2f}'
            )

        # Store relative to origin
        self.latest_gt_x = raw_x - self.odom_origin[0]
        self.latest_gt_y = raw_y - self.odom_origin[1]
        self.latest_gt_theta = raw_theta - self.odom_origin[2]

    def ekf_callback(self, msg: PoseWithCovarianceStamped):
        """Log all data when EKF publishes."""
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # EKF state
        ekf_x = msg.pose.pose.position.x
        ekf_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        ekf_theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # EKF covariance
        cov = msg.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_theta = cov[35]

        self.csv_writer.writerow([
            f'{elapsed:.3f}',
            f'{self.latest_gt_x:.6f}', f'{self.latest_gt_y:.6f}',
            f'{self.latest_gt_theta:.6f}',
            f'{ekf_x:.6f}', f'{ekf_y:.6f}', f'{ekf_theta:.6f}',
            f'{cov_x:.8f}', f'{cov_y:.8f}', f'{cov_theta:.8f}',
            f'{self.latest_gps_x:.6f}', f'{self.latest_gps_y:.6f}',
            f'{self.latest_imu_theta:.6f}'
        ])

    def gps_callback(self, msg: NavSatFix):
        """Convert GPS to local coordinates (zero-referenced)."""
        if msg.status.status < 0:
            return

        lat, lon = msg.latitude, msg.longitude

        if not self.gps_initialized:
            self.gps_origin = (lat, lon)
            self.gps_initialized = True
            return

        lat0, lon0 = self.gps_origin
        R = 6371000.0

        lat_rad = np.radians(lat)
        lat0_rad = np.radians(lat0)
        lon_rad = np.radians(lon)
        lon0_rad = np.radians(lon0)

        self.latest_gps_x = R * (lon_rad - lon0_rad) * np.cos(lat0_rad)
        self.latest_gps_y = R * (lat_rad - lat0_rad)

    def imu_callback(self, msg: Imu):
        """Extract IMU orientation."""
        q = msg.orientation
        self.latest_imu_theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def destroy_node(self):
        """Close CSV file on shutdown."""
        self.csv_file.close()
        self.get_logger().info(f'Data saved to: {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
