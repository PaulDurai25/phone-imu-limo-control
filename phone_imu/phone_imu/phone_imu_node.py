import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class PhoneImuNode(Node):

    def __init__(self):
        super().__init__('phone_imu_node')

        self.declare_parameter(
            'phone_ip',
            '192.168.0.107'
        )

        self.declare_parameter(
            'phone_port',
            8080
        )

        self.declare_parameter(
            'publish_rate',
            20.0
        )

        phone_ip = self.get_parameter(
            'phone_ip'
        ).value

        phone_port = self.get_parameter(
            'phone_port'
        ).value

        publish_rate = self.get_parameter(
            'publish_rate'
        ).value

        self.url = (
            f'http://{phone_ip}:{phone_port}/get'
        )

        self.http = requests.Session()

        self.publisher = self.create_publisher(
            Imu,
            '/phone/imu',
            10
        )

        period = 1.0 / publish_rate

        self.timer = self.create_timer(
            period,
            self.read_imu
        )

        self.get_logger().info(
            'Android Phone IMU node started'
        )

        self.get_logger().info(
            f'phyphox: {self.url}'
        )

        self.get_logger().info(
            f'Publishing /phone/imu at '
            f'{publish_rate} Hz'
        )

    def read_imu(self):

        try:

            params = (
                'accX&accY&accZ&'
                'gyrX&gyrY&gyrZ'
            )

            response = self.http.get(
                self.url + '?' + params,
                timeout=1.0
            )

            response.raise_for_status()

            data = response.json()

            buffers = data.get(
                'buffer',
                {}
            )

            required = [
                'accX',
                'accY',
                'accZ',
                'gyrX',
                'gyrY',
                'gyrZ'
            ]

            for name in required:

                if name not in buffers:
                    self.get_logger().warning(
                        f'Missing buffer: {name}'
                    )
                    return

            ax = buffers['accX']['buffer'][0]
            ay = buffers['accY']['buffer'][0]
            az = buffers['accZ']['buffer'][0]

            gx = buffers['gyrX']['buffer'][0]
            gy = buffers['gyrY']['buffer'][0]
            gz = buffers['gyrZ']['buffer'][0]

            msg = Imu()

            # Linux reception timestamp
            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.header.frame_id = (
                'phone_imu_link'
            )

            # Orientation not available yet
            msg.orientation_covariance[0] = -1.0

            # Angular velocity [rad/s]
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            # Linear acceleration [m/s^2]
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            self.publisher.publish(msg)

        except requests.exceptions.RequestException as e:

            self.get_logger().warning(
                f'phyphox connection error: {e}'
            )

        except (KeyError, IndexError, TypeError) as e:

            self.get_logger().warning(
                f'Invalid IMU data: {e}'
            )

        except Exception as e:

            self.get_logger().warning(
                f'Unexpected error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = PhoneImuNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
