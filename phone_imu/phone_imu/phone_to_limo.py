import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class PhoneToLimo(Node):

    def __init__(self):
        super().__init__('phone_to_limo')

        self.imu_sub = self.create_subscription(
            Imu,
            '/phone/imu',
            self.imu_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Maximum LIMO angular velocity
        self.max_angular = 0.3

        # Gyroscope deadband
        self.deadband = 0.05

        self.get_logger().info(
            'Phone → LIMO yaw controller started'
        )

    def imu_callback(self, msg):

        gyro_z = msg.angular_velocity.z

        # Deadband
        if abs(gyro_z) < self.deadband:
            angular_z = 0.0
        else:
            # Scale phone angular velocity
            angular_z = 0.15 * gyro_z

            # Limit
            angular_z = max(
                -self.max_angular,
                min(self.max_angular, angular_z)
            )

        cmd = Twist()

        # No forward motion
        cmd.linear.x = 0.0

        # Rotation only
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = PhoneToLimo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop robot
        stop = Twist()
        node.cmd_pub.publish(stop)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
