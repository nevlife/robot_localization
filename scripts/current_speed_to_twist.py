#!/usr/bin/env python3
"""current_speed_to_twist: wrap a scalar wheel/CAN speed into a Twist for robot_localization.

robot_localization only accepts nav_msgs/Odometry, geometry_msgs/Twist(WithCovariance)Stamped
and sensor_msgs/Imu. The vehicle publishes a bare scalar forward speed, so this node
wraps it into a body-frame TwistWithCovarianceStamped that the UKF fuses as a Vx-only
velocity source.

Subscribes: /current_speed  (std_msgs/Float64)                    forward speed [m/s], base_link
Publishes:  /wheel/twist    (geometry_msgs/TwistWithCovarianceStamped)

Parameters:
  input_topic  (str,   default /current_speed)
  output_topic (str,   default /wheel/twist)
  frame_id     (str,   default base_link)   frame the speed is expressed in
  vx_variance  (float, default 0.01)        measurement variance on Vx [m^2/s^2]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped

# Large variance marks the unmeasured velocity components so the filter ignores them.
UNMEASURED_VARIANCE = 1e6


class CurrentSpeedToTwist(Node):
    def __init__(self):
        super().__init__('current_speed_to_twist')

        self.declare_parameter('input_topic', '/current_speed')
        self.declare_parameter('output_topic', '/wheel/twist')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('vx_variance', 0.01)

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.vx_variance = self.get_parameter('vx_variance').get_parameter_value().double_value

        self.pub = self.create_publisher(TwistWithCovarianceStamped, out_topic, 10)
        self.sub = self.create_subscription(Float64, in_topic, self.on_speed, 10)

        self.get_logger().info(
            "current_speed_to_twist: {} (Float64) -> {} (TwistWithCovarianceStamped), frame={}".format(
                in_topic, out_topic, self.frame_id))

    def on_speed(self, msg):
        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        # Preserve sign so reverse motion integrates correctly.
        out.twist.twist.linear.x = float(msg.data)

        # Row-major 6x6 covariance over (vx, vy, vz, vroll, vpitch, vyaw).
        cov = [0.0] * 36
        cov[0] = self.vx_variance        # vx
        cov[7] = UNMEASURED_VARIANCE     # vy
        cov[14] = UNMEASURED_VARIANCE    # vz
        cov[21] = UNMEASURED_VARIANCE    # vroll
        cov[28] = UNMEASURED_VARIANCE    # vpitch
        cov[35] = UNMEASURED_VARIANCE    # vyaw
        out.twist.covariance = cov

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CurrentSpeedToTwist()
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
