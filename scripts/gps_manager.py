#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from robot_localization.srv import SetPose
import math


class GpsManager(Node):
    """
    GPS 관리 노드.

    역할:
    - 하드 게이트 없이 GPS를 EKF에 직접 투입 (공분산이 칼만 게인 조절)
    - GPS 품질 상태 퍼블리시 (다른 노드에서 활용 가능)
    - map EKF 드리프트 감지 → 고품질 GPS 복귀 시 SetPose로 리셋

    토픽:
      subscribe: odometry/gps_raw  (navsat_transform 출력)
      subscribe: odometry/global   (map EKF 출력, 드리프트 감지용)
      publish:   odometry/gps      (map EKF 입력, 필터링 없이 그대로)
      publish:   gps/quality       (GPS 품질 상태 문자열)
    """

    # GPS 품질 상태
    QUALITY_FIX    = 'RTK_FIX'
    QUALITY_FLOAT  = 'RTK_FLOAT'
    QUALITY_DGPS   = 'DGPS'
    QUALITY_SINGLE = 'SINGLE'
    QUALITY_NONE   = 'NO_FIX'

    def __init__(self):
        super().__init__('gps_manager')

        # 파라미터 선언
        self.declare_parameter('rtk_fix_threshold',   0.05)   # m² RTK Fix 기준
        self.declare_parameter('rtk_float_threshold', 0.5)    # m² RTK Float 기준
        self.declare_parameter('dgps_threshold',      5.0)    # m² DGPS 기준

        self.declare_parameter('reset_cov_threshold', 0.05)   # m² SetPose 허용 품질 (RTK Fix만)
        self.declare_parameter('reset_drift_threshold', 3.0)  # m  드리프트 감지 거리
        self.declare_parameter('reset_cooldown',      10.0)   # s  SetPose 최소 간격

        self.fix_thr   = self.get_parameter('rtk_fix_threshold').value
        self.float_thr = self.get_parameter('rtk_float_threshold').value
        self.dgps_thr  = self.get_parameter('dgps_threshold').value

        self.reset_cov_thr   = self.get_parameter('reset_cov_threshold').value
        self.drift_thr       = self.get_parameter('reset_drift_threshold').value
        self.reset_cooldown  = self.get_parameter('reset_cooldown').value

        # 구독 / 발행
        self.sub_gps = self.create_subscription(
            Odometry, 'odometry/gps_raw', self.gps_callback, 10)
        self.sub_ekf = self.create_subscription(
            Odometry, 'odometry/global', self.ekf_callback, 10)

        self.pub_gps     = self.create_publisher(Odometry, 'odometry/gps', 10)
        self.pub_quality = self.create_publisher(String, 'gps/quality', 10)

        self.set_pose_cli = self.create_client(
            SetPose, '/ekf_filter_node_map/set_pose')

        # 내부 상태
        self.last_ekf_pose   = None   # 최근 map EKF 위치
        self.last_reset_time = None   # 마지막 SetPose 호출 시각
        self.current_quality = self.QUALITY_NONE
        self.gps_was_lost    = False  # GPS 유실 이력 (복귀 감지용)

        self.get_logger().info('GPS manager started.')

    # ------------------------------------------------------------------
    def gps_callback(self, msg: Odometry):
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        cov   = max(cov_x, cov_y)

        # 품질 분류
        quality = self._classify(cov)
        self._publish_quality(quality)

        # RTK Fix 품질일 때만 map EKF에 투입
        if cov <= self.reset_cov_thr:
            self.pub_gps.publish(msg)

            # GPS 유실됐다가 복귀한 경우 드리프트 확인 후 리셋
            if self.gps_was_lost and self.last_ekf_pose is not None:
                drift = self._calc_drift(msg, self.last_ekf_pose)
                if drift > self.drift_thr and self._can_reset():
                    self.get_logger().warn(
                        f'GPS recovered. Drift {drift:.2f}m. Resetting map EKF.')
                    self._set_pose(msg)
            self.gps_was_lost = False
        else:
            self.gps_was_lost = True

    def ekf_callback(self, msg: Odometry):
        self.last_ekf_pose = msg

    # ------------------------------------------------------------------
    def _classify(self, cov: float) -> str:
        if cov <= self.fix_thr:
            return self.QUALITY_FIX
        elif cov <= self.float_thr:
            return self.QUALITY_FLOAT
        elif cov <= self.dgps_thr:
            return self.QUALITY_DGPS
        else:
            return self.QUALITY_SINGLE

    def _publish_quality(self, quality: str):
        if quality != self.current_quality:
            self.get_logger().info(f'GPS quality: {self.current_quality} → {quality}')
            self.current_quality = quality
        msg = String()
        msg.data = quality
        self.pub_quality.publish(msg)

    def _calc_drift(self, gps_msg: Odometry, ekf_msg: Odometry) -> float:
        dx = gps_msg.pose.pose.position.x - ekf_msg.pose.pose.position.x
        dy = gps_msg.pose.pose.position.y - ekf_msg.pose.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _can_reset(self) -> bool:
        now = self.get_clock().now()
        if self.last_reset_time is None:
            return True
        elapsed = (now - self.last_reset_time).nanoseconds / 1e9
        return elapsed >= self.reset_cooldown

    def _set_pose(self, gps_msg: Odometry):
        if not self.set_pose_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('set_pose service not available')
            return

        pose = PoseWithCovarianceStamped()
        pose.header = gps_msg.header
        pose.pose   = gps_msg.pose

        req = SetPose.Request()
        req.pose = pose
        self.set_pose_cli.call_async(req)
        self.last_reset_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = GpsManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
