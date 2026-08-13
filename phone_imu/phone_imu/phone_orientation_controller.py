import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw in radians."""

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


class PhoneOrientationController(Node):

    def __init__(self):

        super().__init__('phone_orientation_controller')

        # -------------------------------------------------
        # Subscribers
        # -------------------------------------------------

        self.phone_sub = self.create_subscription(
            Imu,
            '/phone/imu',
            self.phone_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # -------------------------------------------------
        # Phone orientation
        # -------------------------------------------------

        self.phone_yaw = 0.0
        self.phone_yaw_initial = None

        self.last_phone_time = None

        # -------------------------------------------------
        # LIMO orientation
        # -------------------------------------------------

        self.limo_yaw = 0.0
        self.limo_yaw_initial = None

        self.last_odom_time = None

        # -------------------------------------------------
        # Controller parameters
        # -------------------------------------------------

        self.Kp = 2.0

        # Start conservatively
        self.max_angular_velocity = 0.40

        # Ignore very small heading errors
        self.deadband = math.radians(2.0)

        # Stop robot if phone data becomes unavailable
        self.phone_timeout = 0.5

        # Change to True if LIMO rotates in wrong direction
        self.invert_command = False

        # -------------------------------------------------
        # Control timer
        # -------------------------------------------------

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Phone orientation controller started'
        )

        self.get_logger().warn(
            'REAL /cmd_vel control is ACTIVE'
        )

    # =====================================================
    # PHONE IMU CALLBACK
    # =====================================================

    def phone_callback(self, msg):

        now = time.monotonic()

        if self.last_phone_time is None:

            self.last_phone_time = now

            return

        dt = now - self.last_phone_time

        self.last_phone_time = now

        # Reject abnormal time interval
        if dt <= 0.0 or dt > 0.5:
            return

        # Phone gyro Z
        gyro_z = msg.angular_velocity.z

        # Integrate gyro
        self.phone_yaw += gyro_z * dt

        self.phone_yaw = wrap_angle(
            self.phone_yaw
        )

        # Capture initial phone orientation
        if self.phone_yaw_initial is None:

            self.phone_yaw_initial = self.phone_yaw

            self.get_logger().info(
                'Initial phone yaw = '
                f'{math.degrees(self.phone_yaw_initial):.2f} deg'
            )

    # =====================================================
    # LIMO ODOM CALLBACK
    # =====================================================

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        self.limo_yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.last_odom_time = time.monotonic()

        # Capture initial LIMO orientation
        if self.limo_yaw_initial is None:

            self.limo_yaw_initial = self.limo_yaw

            self.get_logger().info(
                'Initial LIMO yaw = '
                f'{math.degrees(self.limo_yaw_initial):.2f} deg'
            )

    # =====================================================
    # PUBLISH ZERO VELOCITY
    # =====================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # =====================================================
    # CONTROL LOOP
    # =====================================================

    def control_loop(self):

        # -------------------------------------------------
        # Check initialization
        # -------------------------------------------------

        if self.phone_yaw_initial is None:
            self.stop_robot()
            return

        if self.limo_yaw_initial is None:
            self.stop_robot()
            return

        # -------------------------------------------------
        # Check phone communication
        # -------------------------------------------------

        if self.last_phone_time is None:
            self.stop_robot()
            return

        phone_age = time.monotonic() - self.last_phone_time

        if phone_age > self.phone_timeout:

            self.get_logger().warn(
                'Phone IMU timeout - stopping LIMO'
            )

            self.stop_robot()

            return

        # -------------------------------------------------
        # Phone relative rotation
        # -------------------------------------------------

        phone_delta = wrap_angle(
            self.phone_yaw -
            self.phone_yaw_initial
        )

        # -------------------------------------------------
        # Target LIMO orientation
        # -------------------------------------------------

        target_yaw = wrap_angle(
            self.limo_yaw_initial +
            phone_delta
        )

        # -------------------------------------------------
        # Heading error
        # -------------------------------------------------

        error = wrap_angle(
            target_yaw -
            self.limo_yaw
        )

        # -------------------------------------------------
        # Deadband
        # -------------------------------------------------

        if abs(error) < self.deadband:

            angular_cmd = 0.0

        else:

            angular_cmd = self.Kp * error

        # -------------------------------------------------
        # Limit angular velocity
        # -------------------------------------------------

        angular_cmd = max(
            -self.max_angular_velocity,
            min(
                self.max_angular_velocity,
                angular_cmd
            )
        )

        # -------------------------------------------------
        # Optional direction inversion
        # -------------------------------------------------

        if self.invert_command:

            angular_cmd = -angular_cmd

        # -------------------------------------------------
        # Construct command
        # -------------------------------------------------

        cmd = Twist()

        # No forward motion
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        # No roll/pitch motion
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0

        # Heading control
        cmd.angular.z = angular_cmd

        # -------------------------------------------------
        # Publish
        # -------------------------------------------------

        self.cmd_pub.publish(cmd)

        # -------------------------------------------------
        # Display
        # -------------------------------------------------

        self.get_logger().info(
            f'Phone Δ={math.degrees(phone_delta):+6.1f}° | '
            f'Target={math.degrees(target_yaw):+6.1f}° | '
            f'LIMO={math.degrees(self.limo_yaw):+6.1f}° | '
            f'Error={math.degrees(error):+6.1f}° | '
            f'cmd_w={angular_cmd:+.3f}'
        )

    # =====================================================
    # SHUTDOWN
    # =====================================================

    def destroy_node(self):

        self.get_logger().info(
            'Stopping LIMO before shutdown'
        )

        # Send several zero commands
        for _ in range(5):
            self.stop_robot()
            time.sleep(0.02)

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = PhoneOrientationController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
