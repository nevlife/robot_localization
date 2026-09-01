#!/usr/bin/env python3
"""Sanity gate between the GNSS driver and navsat_transform.

Field failure 2026-07-14 (bag field_20260714_145901): the receiver's
cold-start garbage position (~110 km away) reached the map EKF before the
first sane fix. With no rejection threshold (deliberate, see
scv_dual_ekf.yaml) and no wheel-velocity observation that run, the filter
was kicked to -168 km and never recovered.

The EKF keeps its no-rejection design (RTK-outage jumps of a few metres
must converge, not deadlock); insanity is instead removed at the source.
A fix passes only if:
  * status >= min_status (driver-level no-fix already filtered by navsat,
    but kept here so the radius check always sees a claimed-valid fix)
  * lat/lon finite and non-zero
  * position covariance below max_cov_m2 (when the driver reports one)
  * within max_radius_m of the map datum — the decisive cold-start check:
    the datum is static ground truth, so anything farther than the site
    radius is physically impossible, not merely inaccurate
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class GpsFixGate(Node):
    def __init__(self):
        super().__init__('gps_fix_gate')
        self.declare_parameter('input_topic', '/ublox_gps_node/fix')
        self.declare_parameter('output_topic', '/gps/fix_gated')
        self.declare_parameter('datum_lat', 35.91361)
        self.declare_parameter('datum_lon', 128.80308)
        self.declare_parameter('max_radius_m', 5000.0)
        self.declare_parameter('max_cov_m2', 100.0)
        self.declare_parameter('min_status', int(NavSatStatus.STATUS_FIX))

        self.datum_lat = self.get_parameter('datum_lat').value
        self.datum_lon = self.get_parameter('datum_lon').value
        self.max_radius = self.get_parameter('max_radius_m').value
        self.max_cov = self.get_parameter('max_cov_m2').value
        self.min_status = self.get_parameter('min_status').value

        self.n_pass = 0
        self.n_drop = 0
        self.passed_once = False

        self.pub = self.create_publisher(
            NavSatFix, self.get_parameter('output_topic').value, 10)
        self.sub = self.create_subscription(
            NavSatFix, self.get_parameter('input_topic').value, self.cb, 10)
        self.get_logger().info(
            f'gating {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}: '
            f'radius {self.max_radius:.0f} m around '
            f'({self.datum_lat:.5f}, {self.datum_lon:.5f}), '
            f'cov <= {self.max_cov:.0f} m2, status >= {self.min_status}')

    def cb(self, msg: NavSatFix):
        reason = None
        if msg.status.status < self.min_status:
            reason = f'status {msg.status.status} < {self.min_status}'
        elif not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)) \
                or (msg.latitude == 0.0 and msg.longitude == 0.0):
            reason = 'non-finite/zero position'
        elif (msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN
                and msg.position_covariance[0] > self.max_cov):
            reason = f'cov {msg.position_covariance[0]:.1f} > {self.max_cov:.0f} m2'
        else:
            d = haversine_m(msg.latitude, msg.longitude,
                            self.datum_lat, self.datum_lon)
            if d > self.max_radius:
                reason = (f'{d / 1000:.1f} km from datum '
                          f'(> {self.max_radius / 1000:.1f} km) — cold-start garbage?')

        if reason is None:
            self.n_pass += 1
            if not self.passed_once:
                self.passed_once = True
                self.get_logger().info(
                    f'first sane fix passed after {self.n_drop} dropped '
                    f'({msg.latitude:.6f}, {msg.longitude:.6f})')
            self.pub.publish(msg)
        else:
            self.n_drop += 1
            self.get_logger().warn(
                f'fix DROPPED: {reason} (pass {self.n_pass} / drop {self.n_drop})',
                throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = GpsFixGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
