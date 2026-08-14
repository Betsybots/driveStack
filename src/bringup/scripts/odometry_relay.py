#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryRelay(Node):
    def __init__(self):
        super().__init__('odometry_relay')
        input_topic = self.declare_parameter('input_topic', '/lidar/odometry').value
        output_topic = self.declare_parameter('output_topic', '/odometry/filtered').value

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, input_topic, self.publisher.publish, 10)

        self.get_logger().info(
            "Relaying LiDAR odometry from '%s' to '%s'" % (input_topic, output_topic))


def main(args=None):
    rclpy.init(args=args)
    node = OdometryRelay()
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
