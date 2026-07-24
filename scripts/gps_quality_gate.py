#!/usr/bin/env python3
"""gps_quality_gate: pass GPS through to the global filter ONLY while the fix is good.

Design: gyro-integrated yaw is the continuous heading backbone (no magnetometer, no
continuous GPS dependence). GPS is fused opportunistically - position AND an absolute
course-heading - but ONLY during good-quality sections. In bad-GPS stretches (campus
multipath, tree cover, urban canyon) nothing is emitted, so the estimator coasts on
wheel + IMU + gyro and GPS never drags it off.

Gate (per NavSatFix):
  - status.status  >= min_status                 (reject NO_FIX)
  - horizontal std  = sqrt(max(cov_E, cov_N))     <= max_pos_std   (needs a known covariance)
Good fixes are republished on out_fix_topic for navsat_transform.

Heading (per fix_velocity, only while the latest fix is good):
  - horizontal speed >= min_speed                 (course undefined at low speed)
  - speed std        <= max_speed_std
  yaw = atan2(v_north, v_east)  (ENU, robot_localization convention), published as an
  Imu (yaw-only) on out_heading_topic. This is the magnetometer replacement, gated.

ASSUMPTION: fix_velocity is ENU (linear.x = East, linear.y = North). ublox publishes ENU.

Subscribes: fix_topic          (sensor_msgs/NavSatFix)
            fix_velocity_topic  (geometry_msgs/TwistWithCovarianceStamped)
            /current_speed      (std_msgs/Float64, only if use_reverse_flip)
Publishes:  out_fix_topic       (sensor_msgs/NavSatFix)   good fixes only
            out_heading_topic   (sensor_msgs/Imu)         gated course heading

Parameters:
  fix_topic          (str,   default /ublox_gps_node/fix)
  fix_velocity_topic (str,   default /ublox_gps_node/fix_velocity)
  out_fix_topic      (str,   default /gps/fix_gated)
  out_heading_topic  (str,   default /gps/heading)
  frame_id           (str,   default base_link)
  min_status         (int,   default 0)     NavSatStatus: -1 NO_FIX, 0 FIX, 1 SBAS, 2 GBAS
  max_pos_std        (float, default 1.0)   [m] horizontal std gate (RTK: set ~0.1)
  require_cov        (bool,  default true)  reject fixes whose covariance is UNKNOWN
  min_speed          (float, default 0.5)   [m/s] below this no heading
  max_speed_std      (float, default 0.5)   [m/s] velocity std gate for heading
  yaw_var            (float, default 0.02)  [rad^2] heading variance when emitted
  east_index         (int,   default 0)     fix_velocity linear component that is East
  north_index        (int,   default 1)     fix_velocity linear component that is North
  use_reverse_flip   (bool,  default false) add pi to course when /current_speed < 0
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu

BIG_VARIANCE = 1e6
COVARIANCE_TYPE_UNKNOWN = 0


class GpsQualityGate(Node):
    def __init__(self):
        super().__init__('gps_quality_gate')

        self.declare_parameter('fix_topic', '/ublox_gps_node/fix')
        self.declare_parameter('fix_velocity_topic', '/ublox_gps_node/fix_velocity')
        self.declare_parameter('out_fix_topic', '/gps/fix_gated')
        self.declare_parameter('out_heading_topic', '/gps/heading')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('min_status', 0)
        self.declare_parameter('max_pos_std', 1.0)
        self.declare_parameter('require_cov', True)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('max_speed_std', 0.5)
        self.declare_parameter('yaw_var', 0.02)
        self.declare_parameter('east_index', 0)
        self.declare_parameter('north_index', 1)
        self.declare_parameter('use_reverse_flip', False)

        gp = self.get_parameter
        fix_topic = gp('fix_topic').get_parameter_value().string_value
        fix_vel_topic = gp('fix_velocity_topic').get_parameter_value().string_value
        out_fix_topic = gp('out_fix_topic').get_parameter_value().string_value
        out_heading_topic = gp('out_heading_topic').get_parameter_value().string_value
        self.frame_id = gp('frame_id').get_parameter_value().string_value
        self.min_status = gp('min_status').get_parameter_value().integer_value
        self.max_pos_std = gp('max_pos_std').get_parameter_value().double_value
        self.require_cov = gp('require_cov').get_parameter_value().bool_value
        self.min_speed = gp('min_speed').get_parameter_value().double_value
        self.max_speed_std = gp('max_speed_std').get_parameter_value().double_value
        self.yaw_var = gp('yaw_var').get_parameter_value().double_value
        self.east_index = gp('east_index').get_parameter_value().integer_value
        self.north_index = gp('north_index').get_parameter_value().integer_value
        self.use_reverse_flip = gp('use_reverse_flip').get_parameter_value().bool_value

        self.fix_good = False
        self.reverse = False

        self.fix_pub = self.create_publisher(NavSatFix, out_fix_topic, 10)
        self.heading_pub = self.create_publisher(Imu, out_heading_topic, 10)
        self.create_subscription(NavSatFix, fix_topic, self.on_fix, 10)
        self.create_subscription(TwistWithCovarianceStamped, fix_vel_topic, self.on_fix_velocity, 10)
        if self.use_reverse_flip:
            self.create_subscription(Float64, '/current_speed', self.on_speed, 10)

        self.get_logger().info(
            "gps_quality_gate: fix<={} vel<={} -> fix>={} heading>={}, "
            "min_status={}, max_pos_std={} m, min_speed={} m/s".format(
                fix_topic, fix_vel_topic, out_fix_topic, out_heading_topic,
                self.min_status, self.max_pos_std, self.min_speed))

    def on_speed(self, msg):
        self.reverse = float(msg.data) < 0.0

    def fix_is_good(self, msg):
        if msg.status.status < self.min_status:
            return False
        if msg.position_covariance_type == COVARIANCE_TYPE_UNKNOWN:
            return not self.require_cov
        # ENU covariance, row-major: [0]=East var, [4]=North var.
        hvar = max(msg.position_covariance[0], msg.position_covariance[4])
        if hvar <= 0.0:
            return not self.require_cov
        return math.sqrt(hvar) <= self.max_pos_std

    def on_fix(self, msg):
        self.fix_good = self.fix_is_good(msg)
        if self.fix_good:
            self.fix_pub.publish(msg)

    def on_fix_velocity(self, msg):
        if not self.fix_good:
            return

        lin = msg.twist.twist.linear
        comp = [lin.x, lin.y, lin.z]
        v_east = comp[self.east_index]
        v_north = comp[self.north_index]

        speed = math.hypot(v_east, v_north)
        if speed < self.min_speed:
            return

        cov = msg.twist.covariance
        speed_var = max(cov[0], cov[7])  # vx, vy variances
        if speed_var > 0.0 and math.sqrt(speed_var) > self.max_speed_std:
            return

        yaw = math.atan2(v_north, v_east)
        if self.use_reverse_flip and self.reverse:
            yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))

        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = math.sin(yaw * 0.5)
        out.orientation.w = math.cos(yaw * 0.5)
        out.orientation_covariance = [
            BIG_VARIANCE, 0.0, 0.0,
            0.0, BIG_VARIANCE, 0.0,
            0.0, 0.0, self.yaw_var,
        ]
        out.angular_velocity_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        out.linear_acceleration_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.heading_pub.publish(out)


def main():
    rclpy.init()
    node = GpsQualityGate()
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
