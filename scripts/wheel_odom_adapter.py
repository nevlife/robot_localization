#!/usr/bin/env python3
"""Adapt the Hunter wheel speed (std_msgs/Float64 on /hunter/velocity) into a
nav_msgs/Odometry twist source that robot_localization can fuse.

Only twist.linear.x carries information (Ackermann base, no lateral speed,
yaw rate comes from the IMU), so the pose block is left zero with huge
covariance and the EKF configs fuse vx only.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class WheelOdomAdapter(Node):
    def __init__(self):
        super().__init__('wheel_odom_adapter')
        self.declare_parameter('input_topic', '/hunter/velocity')
        self.declare_parameter('output_topic', 'odometry/wheel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('vx_variance', 0.04)   # (0.2 m/s)^2 — wheel slip margin

        gp = self.get_parameter
        self.base_frame = gp('base_frame').value
        self.vx_var = float(gp('vx_variance').value)

        self.pub = self.create_publisher(Odometry, gp('output_topic').value, 10)
        self.create_subscription(Float64, gp('input_topic').value, self.cb, 20)
        self.get_logger().info(
            f"wheel odom adapter: {gp('input_topic').value} -> "
            f"{gp('output_topic').value} (vx var {self.vx_var})")

    def cb(self, msg: Float64):
        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.header.frame_id = 'odom'
        od.child_frame_id = self.base_frame
        od.twist.twist.linear.x = float(msg.data)
        cov = [0.0] * 36
        cov[0] = self.vx_var          # vx
        cov[7] = 0.05                 # vy: effectively "measured as 0" (no lateral slip assumption)
        cov[35] = 1e6                 # yaw rate: not measured here (IMU owns it)
        od.twist.covariance = cov
        self.pub.publish(od)


def main():
    rclpy.init()
    n = WheelOdomAdapter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
