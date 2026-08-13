import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def quaternion_to_yaw(x, y, z, w):
    """
    Convert quaternion to yaw angle in radians.
    """

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


def wrap_angle(angle):
    """
    Wrap angle to [-pi, pi].
    """

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


class PhoneOrientationDryRun(Node):

    def __init__(self):

        super().__init__('phone_orientation_dryrun')

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
        # Phone yaw
        # -------------------------------------------------

        self.phone_yaw = 0.0
        self.phone_yaw_initial = None

        self.last_time = None

        # -------------------------------------------------
        # LIMO yaw
        # -------------------------------------------------

        self.limo_yaw = 0.0
        self.limo_yaw_initial = None

        # -------------------------------------------------
        # Control parameters
        # -------------------------------------------------

        self.Kp = 1.0

        self.max_angular_velocity = 0.3

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Phone orientation DRY-RUN controller started'
        )

    # =====================================================
    # PHONE IMU
    # =====================================================

    def phone_callback(self, msg):

        now = time.monotonic()

        if self.last_time is None:

            self.last_time = now

            return

        dt = now - self.last_time

        self.last_time = now

        # Phone gyro Z
        gyro_z = msg.angular_velocity.z

        # Integrate gyro
        self.phone_yaw += gyro_z * dt

        self.phone_yaw = wrap_angle(
            self.phone_yaw
        )

        # Save initial phone yaw
        if self.phone_yaw_initial is None:

            self.phone_yaw_initial = self.phone_yaw

            self.get_logger().info(
                f'Initial phone yaw = '
                f'{math.degrees(self.phone_yaw_initial):.2f} deg'
            )

    # =====================================================
    # LIMO ODOM
    # =====================================================

    def odom_callback(self, msg):

        q = msg.pose.pose.orientation

        self.limo_yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        # Save initial LIMO yaw
        if self.limo_yaw_initial is None:

            self.limo_yaw_initial = self.limo_yaw

            self.get_logger().info(
                f'Initial LIMO yaw = '
                f'{math.degrees(self.limo_yaw_initial):.2f} deg'
            )

    # =====================================================
    # CONTROL LOOP
    # =====================================================

    def control_loop(self):

        if self.phone_yaw_initial is None:

            return

        if self.limo_yaw_initial is None:

            return

        # -------------------------------------------------
        # Phone relative movement
        # -------------------------------------------------

        phone_delta = wrap_angle(
            self.phone_yaw -
            self.phone_yaw_initial
        )

        # -------------------------------------------------
        # Target LIMO heading
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
        # P controller
        # -------------------------------------------------

        angular_cmd = self.Kp * error

        # Limit command
        angular_cmd = max(
            -self.max_angular_velocity,
            min(
                self.max_angular_velocity,
                angular_cmd
            )
        )

        # -------------------------------------------------
        # DRY RUN
        # -------------------------------------------------

        self.get_logger().info(
            f'Phone Δ={math.degrees(phone_delta):+6.1f}° | '
            f'Target={math.degrees(target_yaw):+6.1f}° | '
            f'LIMO={math.degrees(self.limo_yaw):+6.1f}° | '
            f'Error={math.degrees(error):+6.1f}° | '
            f'cmd_w={angular_cmd:+.3f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PhoneOrientationDryRun()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
