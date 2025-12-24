"""Trajectory publisher for repeatable EKF experiments."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math


class TrajectoryPublisher(Node):
    """Publish a predefined trajectory for consistent experiments."""

    def __init__(self):
        super().__init__('trajectory_publisher')

        # Parameters
        self.declare_parameter('trajectory', 'figure8')  # figure8, circle, straight, zigzag
        self.declare_parameter('linear_speed', 0.15)  # m/s
        self.declare_parameter('duration', 60.0)  # seconds

        self.trajectory = self.get_parameter('trajectory').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.duration = self.get_parameter('duration').value

        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.start_time = self.get_clock().now()
        self.elapsed = 0.0

        self.get_logger().info(f'Starting {self.trajectory} trajectory for {self.duration}s')

    def timer_callback(self):
        """Publish velocity commands based on trajectory type."""
        now = self.get_clock().now()
        self.elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.elapsed > self.duration:
            self._stop()
            self.get_logger().info('Trajectory complete!')
            self.timer.cancel()
            return

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_footprint'

        v, omega = self._get_velocity(self.elapsed)
        msg.twist.linear.x = v
        msg.twist.angular.z = omega

        self.publisher.publish(msg)

    def _get_velocity(self, t):
        """Get velocity commands based on trajectory type."""
        v = self.linear_speed

        if self.trajectory == 'straight':
            # Straight line
            return v, 0.0

        elif self.trajectory == 'circle':
            # Circle (constant angular velocity)
            omega = 0.3  # rad/s
            return v, omega

        elif self.trajectory == 'figure8':
            # Figure-8 pattern using sinusoidal angular velocity
            period = 20.0  # seconds per loop
            omega = 0.5 * math.sin(2 * math.pi * t / period)
            return v, omega

        elif self.trajectory == 'zigzag':
            # Alternating left-right turns
            period = 5.0
            if int(t / period) % 2 == 0:
                omega = 0.4
            else:
                omega = -0.4
            return v, omega

        elif self.trajectory == 'square':
            # Square pattern: straight, turn, straight, turn...
            segment = 5.0  # seconds per segment
            phase = int(t / segment) % 4
            if phase == 0 or phase == 2:
                return v, 0.0  # straight
            else:
                return 0.0, 0.5  # turn in place

        else:
            return 0.0, 0.0

    def _stop(self):
        """Stop the robot."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
