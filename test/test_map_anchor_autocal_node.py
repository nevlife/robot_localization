#!/usr/bin/env python3
"""map_anchor yaw autocal 노드 통합 테스트 (실제 구독 경로).

합성 토픽(/vectornav/imu, /odom, /gps/fix_gated, odometry/gps)을 발행해
YawAutocal 공급 사슬 전체(on_fix 품질 → on_odom 속도 → on_gps 측정)를
검증한다. 90도 주입 시 corr -90도 수렴 + ENGAGED(th 회전·재앵커)까지.
챔버 실험에서 측정 0회가 나왔을 때 노드 배선/시뮬 환경을 가르는 용도.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

sys.path.insert(0, '/home/ppub/scv_ws/src/robot_localization/scripts')
from map_anchor_node import MapAnchorNode  # noqa: E402

FAULT = math.radians(90.0)

rclpy.init()
anchor = MapAnchorNode()
anchor.yaw_offset = FAULT          # 결함 주입 (파라미터 경로와 등가)

pub = Node('feeder')
p_imu = pub.create_publisher(Imu, '/vectornav/imu', 10)
p_odom = pub.create_publisher(Odometry, '/odom', 10)
p_fix = pub.create_publisher(NavSatFix, '/gps/fix_gated', 10)
p_gps = pub.create_publisher(Odometry, 'odometry/gps', 10)

ex = rclpy.executors.SingleThreadedExecutor()
ex.add_node(anchor)
ex.add_node(pub)

t0 = time.monotonic()
V = 1.0                            # 동진 1 m/s, 실헤딩 0
while time.monotonic() - t0 < 12.0:
    t = time.monotonic() - t0
    x = V * t

    im = Imu()
    im.orientation.w = 1.0         # 실제 yaw 0 → uncal = +90도 (오염)
    p_imu.publish(im)

    od = Odometry()
    od.pose.pose.position.x = x
    od.pose.pose.orientation.w = 1.0
    od.twist.twist.linear.x = V
    p_odom.publish(od)

    fx = NavSatFix()
    fx.status.status = 2
    fx.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    fx.position_covariance = [0.0004, 0.0, 0.0,
                              0.0, 0.0004, 0.0,
                              0.0, 0.0, 0.0016]
    p_fix.publish(fx)

    gp = Odometry()
    gp.pose.pose.position.x = x    # map 상 실이동 = 동진
    p_gps.publish(gp)

    ex.spin_once(timeout_sec=0.0)
    time.sleep(0.05)               # 20 Hz

P = F = 0


def check(name, ok, detail=''):
    global P, F
    P, F = (P + 1, F) if ok else (P, F + 1)
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


cal = anchor.autocal
check('autocal 활성 파라미터로 생성됨', cal is not None)
check('측정 발생 (공급 사슬 전체 동작)', cal is not None and cal.n >= 6,
      f'(n={getattr(cal, "n", 0)})')
check('corr -90도 수렴', cal is not None and
      abs(math.degrees(cal.corr) + 90.0) < 6.0,
      f'(corr {math.degrees(cal.corr):.1f}deg)' if cal else '')
check('ENGAGED (active + 20도 초과 즉시 적용)', anchor._cal_active)
check('보정 후 imu_yaw ~ 실헤딩 0',
      anchor.imu_yaw is not None and
      abs(math.degrees(anchor.imu_yaw)) < 8.0,
      f'(imu_yaw {math.degrees(anchor.imu_yaw):.1f}deg)'
      if anchor.imu_yaw is not None else '')

anchor.destroy_node()
pub.destroy_node()
rclpy.shutdown()
print(f"\n{P} passed, {F} failed")
sys.exit(1 if F else 0)
