"""EKF Localization ROS 2 Node for TurtleBot3 with GPS."""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from ekf_localization_tb3.ekf_core import EKFCore


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert Euler angles to quaternion (x, y, z, w)."""
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.copysign(np.pi / 2, sinp) if np.abs(sinp) >= 1 else np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


class EKFLocalizationNode(Node):
    """ROS 2 Node for EKF localization using cmd_vel, GPS, and IMU."""

    def __init__(self):
        super().__init__('ekf_localization_node')
        self._declare_parameters()
        self._init_ekf()
        self._init_subscribers()
        self._init_publishers()

        self.dt = 1.0 / self.get_parameter('prediction_rate').value
        self.prediction_timer = self.create_timer(self.dt, self._prediction_callback)

        self.get_logger().info('EKF started (prediction: cmd_vel, update: GPS + IMU)')

    def _declare_parameters(self):
        """Declare ROS parameters."""
        self.declare_parameter('process_noise_x', 0.1)
        self.declare_parameter('process_noise_y', 0.1)
        self.declare_parameter('process_noise_theta', 0.05)
        self.declare_parameter('imu_noise_theta', 0.01)
        self.declare_parameter('gps_noise_x', 0.5)
        self.declare_parameter('gps_noise_y', 0.5)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('prediction_rate', 50.0)

    def _init_ekf(self):
        """Initialize EKF with parameters."""
        initial_state = np.array([
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
            self.get_parameter('initial_theta').value,
        ])
        process_noise = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value,
        ])
        imu_noise = np.array([[self.get_parameter('imu_noise_theta').value]])
        gps_noise = np.diag([
            self.get_parameter('gps_noise_x').value,
            self.get_parameter('gps_noise_y').value,
        ])

        self.ekf = EKFCore(
            initial_state=initial_state,
            process_noise=process_noise,
            imu_noise=imu_noise,
            gps_noise=gps_noise,
        )

        self.gps_origin = None
        self.gps_initialized = False
        self.current_v = 0.0
        self.current_omega = 0.0
        self.publish_tf = self.get_parameter('publish_tf').value
        self.last_stamp = None

    def _init_subscribers(self):
        """Initialize topic subscribers."""
        self.create_subscription(TwistStamped, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(Imu, '/imu', self._imu_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._gps_callback, 10)

    def _init_publishers(self):
        """Initialize topic publishers and TF broadcaster."""
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

    def _cmd_vel_callback(self, msg: TwistStamped):
        """Store velocity commands for prediction."""
        self.current_v = msg.twist.linear.x
        self.current_omega = msg.twist.angular.z

    def _prediction_callback(self):
        """Run EKF prediction step at fixed rate using cmd_vel."""
        self.ekf.predict(self.current_v, self.current_omega, self.dt)

    def _imu_callback(self, msg: Imu):
        """Update EKF with IMU orientation (theta correction)."""
        q = msg.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.ekf.update_imu(yaw)
        self._publish_pose(msg.header.stamp)

    def _gps_callback(self, msg: NavSatFix):
        """Update EKF with GPS position (x, y correction)."""
        if msg.status.status < 0:
            return

        lat, lon = msg.latitude, msg.longitude

        if not self.gps_initialized:
            self.gps_origin = (lat, lon)
            self.gps_initialized = True
            self.get_logger().info(f'GPS origin: lat={lat:.6f}, lon={lon:.6f}')
            return

        x, y = self._latlon_to_local(lat, lon)
        self.ekf.update_gps(x, y)
        self._publish_pose(msg.header.stamp)

    def _latlon_to_local(self, lat: float, lon: float) -> tuple:
        """Convert lat/lon to local coordinates using equirectangular projection."""
        if self.gps_origin is None:
            return 0.0, 0.0

        lat0, lon0 = self.gps_origin
        R = 6371000.0  # Earth radius in meters

        lat_rad, lat0_rad = np.radians(lat), np.radians(lat0)
        lon_rad, lon0_rad = np.radians(lon), np.radians(lon0)

        x = R * (lon_rad - lon0_rad) * np.cos(lat0_rad)
        y = R * (lat_rad - lat0_rad)
        return x, y

    def _publish_pose(self, stamp):
        """Publish EKF estimated pose."""
        state = self.ekf.get_state()
        cov = self.ekf.get_covariance()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = state[0]
        msg.pose.pose.position.y = state[1]
        msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, state[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Fill 6x6 covariance (x, y, z, roll, pitch, yaw)
        cov_36 = [0.0] * 36
        cov_36[0], cov_36[1], cov_36[5] = cov[0, 0], cov[0, 1], cov[0, 2]
        cov_36[6], cov_36[7], cov_36[11] = cov[1, 0], cov[1, 1], cov[1, 2]
        cov_36[30], cov_36[31], cov_36[35] = cov[2, 0], cov[2, 1], cov[2, 2]
        msg.pose.covariance = cov_36

        self.pose_pub.publish(msg)
        self.last_stamp = stamp

        if self.publish_tf:
            self._broadcast_tf(state, stamp)

    def _broadcast_tf(self, state, stamp):
        """Broadcast EKF pose as TF transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint_ekf'
        t.transform.translation.x = state[0]
        t.transform.translation.y = state[1]
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, state[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()

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
