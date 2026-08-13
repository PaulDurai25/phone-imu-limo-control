import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class PhoneYawTest(Node):

    def __init__(self):
        super().__init__('phone_yaw_test')

        self.subscription = self.create_subscription(
            Imu,
            '/phone/imu',
            self.imu_callback,
            10
        )

        self.yaw = 0.0
        self.last_time = None

        self.get_logger().info('Phone yaw test started')

    def imu_callback(self, msg):

        now = time.monotonic()

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        # Gyroscope Z-axis
        gyro_z = msg.angular_velocity.z

        # Integrate angular velocity
        self.yaw += gyro_z * dt

        # Wrap yaw to [-pi, +pi]
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        yaw_deg = math.degrees(self.yaw)

        self.get_logger().info(
            f'gyro_z={gyro_z:+.3f} rad/s | '
            f'yaw={yaw_deg:+.2f} deg'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PhoneYawTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
