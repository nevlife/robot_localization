#!/usr/bin/env python3
"""scaled_nhc_twist: wrap a scalar wheel/CAN speed into a base_link Twist with a
nonholonomic lateral constraint, for robot_localization.

The vehicle publishes a bare scalar forward speed (std_msgs/Float64). robot_localization
only accepts Odometry / Twist(WithCovariance)Stamped / Imu, so this node wraps the speed
into a body-frame TwistWithCovarianceStamped that the UKF fuses as:
  - Vx : scaled forward speed (wheel_scale * raw), sign preserved (reverse ok)
  - Vy : 0 pseudo-measurement (nonholonomic constraint: no lateral slip in base_link)

Everything else (Vz, Vroll, Vpitch, Vyaw) is marked unmeasured with a large variance so
the filter ignores it. This matches scv_1.yaml, whose twist0_config fuses (Vx, Vy) from
/wheel/twist_nhc.

Subscribes: input_topic  (std_msgs/Float64)                    forward speed [m/s], base_link
Publishes:  output_topic (geometry_msgs/TwistWithCovarianceStamped)

Parameters:
  input_topic  (str,   default /current_speed)
  output_topic (str,   default /wheel/twist_nhc)
  frame_id     (str,   default base_link)   frame the speed is expressed in
  wheel_scale  (float, default 0.925)       multiplies raw speed (per-robot calibration)
  vx_var       (float, default 0.05)        measurement variance on Vx [m^2/s^2]
  nhc_var      (float, default 0.001)       lateral pseudo-measurement variance (small = strong NHC)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped

# Large variance marks the unmeasured velocity components so the filter ignores them.
UNMEASURED_VARIANCE = 1e6


class ScaledNhcTwist(Node):
    def __init__(self):
        super().__init__('scaled_nhc_twist')

        self.declare_parameter('input_topic', '/current_speed')
        self.declare_parameter('output_topic', '/wheel/twist_nhc')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('wheel_scale', 0.925)
        self.declare_parameter('vx_var', 0.05)
        self.declare_parameter('nhc_var', 0.001)

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.wheel_scale = self.get_parameter('wheel_scale').get_parameter_value().double_value
        self.vx_var = self.get_parameter('vx_var').get_parameter_value().double_value
        self.nhc_var = self.get_parameter('nhc_var').get_parameter_value().double_value

        self.pub = self.create_publisher(TwistWithCovarianceStamped, out_topic, 10)
        self.sub = self.create_subscription(Float64, in_topic, self.on_speed, 10)

        self.get_logger().info(
            "scaled_nhc_twist: {} (Float64) -> {} (TwistWithCovarianceStamped), "
            "frame={}, wheel_scale={}, vx_var={}, nhc_var={}".format(
                in_topic, out_topic, self.frame_id,
                self.wheel_scale, self.vx_var, self.nhc_var))

    def on_speed(self, msg):
        out = TwistWithCovarianceStamped()
        # Float64 has no header; stamp with the node clock so timestamps follow /clock
        # (bag replay) when use_sim_time is set.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        # Sign preserved so reverse motion integrates correctly.
        out.twist.twist.linear.x = self.wheel_scale * float(msg.data)
        # Nonholonomic constraint: zero lateral velocity in base_link.
        out.twist.twist.linear.y = 0.0

        # Row-major 6x6 covariance over (vx, vy, vz, vroll, vpitch, vyaw).
        cov = [0.0] * 36
        cov[0] = self.vx_var             # vx
        cov[7] = self.nhc_var            # vy (strong NHC)
        cov[14] = UNMEASURED_VARIANCE    # vz
        cov[21] = UNMEASURED_VARIANCE    # vroll
        cov[28] = UNMEASURED_VARIANCE    # vpitch
        cov[35] = UNMEASURED_VARIANCE    # vyaw
        out.twist.covariance = cov

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScaledNhcTwist()
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
