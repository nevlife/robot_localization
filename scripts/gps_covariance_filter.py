#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GpsCovarianceFilter(Node):
    def __init__(self):
        super().__init__('gps_covariance_filter')
        self.declare_parameter('cov_threshold', 0.5)
        self.threshold = self.get_parameter('cov_threshold').value

        self.sub = self.create_subscription(
            Odometry, 'odometry/gps_raw', self.callback, 10)
        self.pub = self.create_publisher(Odometry, 'odometry/gps', 10)

        self.get_logger().info(
            f'GPS covariance filter started. threshold={self.threshold} m²')

    def callback(self, msg):
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]

        if cov_x < self.threshold and cov_y < self.threshold:
            self.pub.publish(msg)
        else:
            self.get_logger().warn(
                f'GPS rejected: cov_x={cov_x:.3f}, cov_y={cov_y:.3f} (threshold={self.threshold})',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = GpsCovarianceFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
