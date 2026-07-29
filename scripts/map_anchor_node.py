#!/usr/bin/env python3
"""map->odom anchoring WITHOUT an EKF — replaces ekf_filter_node_map.

Why this exists (2026-07-18 field failure, root-caused by bag replay):
robot_localization's map EKF diverged 8.9 km on a STATIONARY vehicle with a
sane GPS and a sane IMU. Mechanism: navsat's output stamps trail the 100 Hz
IMU stream by 60-120 ms, so every GPS correction is applied out-of-order
(correct without predict), and the filter's UNOBSERVED velocity/acceleration
states (published twist showed 0 while the internal state held vx=-5 m/s,
ax=-1 m/s^2!) are excited through cross-covariance until position runs away.
No parameter combination fixed it (fastlio removal, accel fusion,
smooth_lagged_data, process-noise stiffening); restamping bounded it but left
multi-metre transients.

Anchoring needs no derivative states at all: the map frame differs from the
odom frame by a slowly-varying rigid transform. So estimate exactly that and
nothing else:

    yaw:  th_map_odom = angle_ema( imu_absolute_yaw - odom_yaw )
    xy :  t_map_odom  = ema( gps_map_position - R(th) * odom_position )

Unconditionally stable by construction (pure low-pass on direct
measurements). Field-bag replay: mean error vs navsat 0.14 m, second-half
0.05 m, max 0.42 m (the EKF: 8,877 m). Fine relative motion stays with the
proven ekf_odom; during GPS outages the anchor simply freezes, which is the
desired behaviour (FAST-LIO carries the run — same design as before).

Publishes: map->odom TF + /odometry/global (map-frame pose of base_link, at
odom rate so downstream liveliness checks — e.g. the C4 interlock — behave
identically to the EKF it replaces).
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def anchor_target(gx, gy, ox, oy, th):
    """map->odom translation implied by one absolute observation (gx, gy)
    of the robot whose odom-frame position is (ox, oy), given yaw th."""
    c, s = math.cos(th), math.sin(th)
    return gx - (c * ox - s * oy), gy - (s * ox + c * oy)


def select_source(rtk_recent, pcd_fresh, pcd_ever):
    """Anchor-source policy (pure, unit-tested):
    RTK sample > (PCD fresh ? PCD : PCD_HOLD, once PCD ever seen) > plain GPS.
    Returns one of 'GPS_RTK', 'PCD', 'PCD_HOLD', 'GPS'."""
    if rtk_recent:
        return 'GPS_RTK'
    if pcd_ever:
        return 'PCD' if pcd_fresh else 'PCD_HOLD'
    return 'GPS'


def gate_rtk(resid_m, streak, gap_s, thresh_m, need, reset_gap_s=5.0):
    """RTK re-trust gate (pure, unit-tested).

    A receiver's own quality flags must not restore snap authority: real
    receivers report status 2 / tiny covariance while multipathing metres
    off (cov 0.14 at 110 km error on the 2026-07-14 cold start; field
    report 2026-07-30: false "recovered" GPS re-dragged the anchor away
    from the SLAM-consistent pose and warped the graph map). An RTK sample
    regains authority only by CONSISTENCY with the current estimate:
    `need` consecutive samples whose anchor innovation is <= thresh_m,
    with no inter-sample gap longer than reset_gap_s. A single outlier
    revokes trust instantly — asymmetric hysteresis (fast to revoke, slow
    to grant). Returns (trusted, new_streak)."""
    if gap_s > reset_gap_s or resid_m > thresh_m:
        streak = 0
    if resid_m <= thresh_m:
        streak += 1
    return streak >= need, streak


class MapAnchorNode(Node):
    def __init__(self):
        super().__init__('map_anchor')
        # EMA time constants (s); gains derived per-sample from actual dt
        self.declare_parameter('xy_tau', 2.0)
        self.declare_parameter('yaw_tau', 5.0)
        # innovation slew limit [m] per GPS sample: multipath bias JUMPS (real
        # receivers under-report covariance while multipathing) drag a plain
        # EMA by metres; a rate limit turns them into bounded creep while
        # still converging metre-scale legitimate offsets within ~1 s @10 Hz
        self.declare_parameter('max_step_m', 0.5)
        # ---- hybrid anchoring (mirrors main's GPS<->PCD switching) -------
        # When a PCD map-matching localizer is running (main-line
        # lio_feature_map_localizer, reused as-is), its pose is a far better
        # absolute anchor than a degraded (non-RTK) fix: measured on bag 82,
        # PCD 0.21 m vs plain-GPS-anchored 1.23 m median. Source priority per
        # observation: RTK fix > fresh PCD pose > plain fix > freeze.
        self.declare_parameter('pcd_pose_topic', '')       # '' disables hybrid
        # their map frame -> our map frame translation:
        # (their georef UTM origin) - (our datum node[0] UTM)
        self.declare_parameter('pcd_offset_e', 45.518)
        self.declare_parameter('pcd_offset_n', 26.015)
        self.declare_parameter('pcd_timeout', 3.0)         # [s] freshness —
                                                           # rides matcher
                                                           # MayLost cycles
        # ---- RTK re-trust (innovation) gate ------------------------------
        # thresh aligned with the chamber's GPS_DIVERGE_M; <=0 disables the
        # gate (pre-2026-07-30 behaviour) for A/B and field rollback. NB a
        # false fix whose bias stays UNDER the threshold is indistinguishable
        # from odom drift by consistency alone — that residual systematic
        # component is the online frame-calibration work item, not the gate's.
        self.declare_parameter('rtk_gate_m', 3.0)
        self.declare_parameter('rtk_gate_n', 5)
        self.declare_parameter('rtk_gate_reset_s', 5.0)
        # magnetic/mounting declination: map yaw = imu yaw + offset (rad)
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('gps_topic', 'odometry/gps')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/vectornav/imu')

        self.xy_tau = self.get_parameter('xy_tau').value
        self.yaw_tau = self.get_parameter('yaw_tau').value
        self.max_step = self.get_parameter('max_step_m').value
        self.pcd_topic = self.get_parameter('pcd_pose_topic').value
        self.pcd_off = (self.get_parameter('pcd_offset_e').value,
                        self.get_parameter('pcd_offset_n').value)
        self.pcd_timeout = self.get_parameter('pcd_timeout').value
        self.rtk_gate_m = self.get_parameter('rtk_gate_m').value
        self.rtk_gate_n = self.get_parameter('rtk_gate_n').value
        self.rtk_gate_reset = self.get_parameter('rtk_gate_reset_s').value
        self.rtk_streak = 0
        self.rtk_ok = False         # RTK currently holds snap authority
        self.last_rtk_eval_t = None
        self.last_rtk_t = None
        self.last_pcd = None        # (t, x_our, y_our, yaw)
        self.mode = 'INIT'
        self.yaw_offset = self.get_parameter('yaw_offset').value

        self.odom = None            # (x, y, yaw) in odom frame
        self.imu_yaw = None         # absolute yaw (map convention)
        self.t = None               # map->odom translation [x, y]
        self.th = 0.0               # map->odom yaw
        self.th_init = False
        self.anchored = False       # first GPS sample received
        self.last_gps_t = None

        self.tf = TransformBroadcaster(self)
        self.pub_global = self.create_publisher(Odometry, 'odometry/global', 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.on_odom, 30)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self.on_imu, 50)
        self.create_subscription(
            Odometry, self.get_parameter('gps_topic').value, self.on_gps, 30)
        # RTK detection from the gated fix stream (sane fixes only)
        self.create_subscription(NavSatFix, '/gps/fix_gated', self.on_fix, 30)
        self.pub_mode = self.create_publisher(String, 'map_anchor/mode', 10)
        if self.pcd_topic:
            self.create_subscription(Odometry, self.pcd_topic, self.on_pcd, 30)
        self.get_logger().info(
            f'EKF-free map anchor: xy_tau={self.xy_tau}s yaw_tau={self.yaw_tau}s'
            + (f' hybrid(pcd={self.pcd_topic}, off={self.pcd_off})' if self.pcd_topic else ''))

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.mode = mode
        self.get_logger().info(f'anchor source -> {mode}')
        self.pub_mode.publish(String(data=mode))

    def on_fix(self, m):
        if m.status.status == 2:
            self.last_rtk_t = self.now_s()

    def rtk_recent(self, window=0.3):
        return self.last_rtk_t is not None and \
            self.now_s() - self.last_rtk_t < window

    def snap_anchor(self, gx, gy, g=0.5):
        ox, oy, _ = self.odom
        tx, ty = anchor_target(gx, gy, ox, oy, self.th)
        if not self.anchored:
            self.t = [tx, ty]
            self.anchored = True
        else:
            dx = g * (tx - self.t[0])
            dy = g * (ty - self.t[1])
            # slew-limit snaps too: alternating RTK<->plain observations
            # differ by the plain-fix bias (~1 m) — unlimited snapping made
            # the anchor JUMP at every mode boundary (29 jumps >1 m on bag
            # 150709 vs main's 0). 2x the EMA slew still converges an RTK
            # correction in 2-3 samples.
            step = math.hypot(dx, dy)
            lim = 2.0 * self.max_step
            if step > lim:
                dx *= lim / step
                dy *= lim / step
            self.t[0] += dx
            self.t[1] += dy

    def pcd_fresh(self):
        return self.last_pcd is not None and \
            self.now_s() - self.last_pcd[0] < self.pcd_timeout

    def on_pcd(self, m):
        """PCD map-matcher pose (their map frame) -> our map frame.

        Corrections are SPARSE (the matcher publishes only accepted
        registrations — measured 5/44 acceptance on bag 82) but each one is
        an absolute high-quality snap. So: apply it as a direct anchor
        correction (fast EMA), and afterwards HOLD — do not let plain-fix
        GPS re-drag the anchor between corrections (main's PCD_ACTIVE
        semantics: candidates + local odometry bridge the gaps)."""
        p = m.pose.pose.position
        self.last_pcd = (self.now_s(),
                         p.x + self.pcd_off[0], p.y + self.pcd_off[1],
                         yaw_of(m.pose.pose.orientation))
        # yield only to TRUSTED RTK — a suspect (gate-rejected) RTK stream
        # must not silence the matcher, it is the reference we hold against
        if self.odom is None or not self.th_init or \
                (self.rtk_recent() and self.rtk_ok):
            return
        self.set_mode('PCD')
        self.snap_anchor(self.last_pcd[1], self.last_pcd[2], g=0.5)

    def on_imu(self, m):
        self.imu_yaw = wrap(yaw_of(m.orientation) + self.yaw_offset)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y, yaw_of(m.pose.pose.orientation))
        # yaw anchor updates at odom rate (imu+odom both live even without GPS)
        if self.imu_yaw is not None:
            target = wrap(self.imu_yaw - self.odom[2])
            if not self.th_init:
                self.th = target
                self.th_init = True
            else:
                # ~30 Hz odom -> per-sample gain from tau
                g = min(1.0, (1.0 / 30.0) / self.yaw_tau)
                self.th = wrap(self.th + g * wrap(target - self.th))
        # Pre-anchor bootstrap: publish with a provisional t=(0,0) as soon as
        # yaw is known. navsat_transform (use_odometry_yaw) needs OUR yaw
        # before it can produce odometry/gps — waiting for GPS first would
        # deadlock the chain. The first GPS sample SNAPS t (no EMA from the
        # provisional value), same as the EKF's first-measurement init; until
        # then downstream must not drive anyway (C4 gates on /gps/fix_gated).
        if self.t is None and self.th_init:
            self.t = [0.0, 0.0]
        if self.t is not None:
            self.publish(m.header.stamp)

    def on_gps(self, m):
        if self.odom is None or not self.th_init:
            return
        # hybrid: while GPS is degraded (no recent RTK) and the PCD matcher is
        # alive, the PCD pose owns the anchor — skip plain-fix updates so the
        # noisier source cannot fight the better one (mirrors main's
        # GPS_ACTIVE -> PCD_ACTIVE handover)
        gx, gy = m.pose.pose.position.x, m.pose.pose.position.y
        src = select_source(self.rtk_recent(),
                            self.pcd_fresh(),
                            self.pcd_topic and self.last_pcd is not None)
        if src == 'GPS_RTK':
            # RTK-fixed sample (cm-class) — but only a TRUSTED one may snap.
            # Trust is earned by innovation consistency (gate_rtk), not by
            # the receiver's own status flag: false fixes after signal
            # recovery re-dragged the anchor and warped the graph map on the
            # 2026-07-30 field run.
            if not self.anchored or self.rtk_gate_m <= 0.0:
                trusted = True
            else:
                ox, oy, _ = self.odom
                tx, ty = anchor_target(gx, gy, ox, oy, self.th)
                resid = math.hypot(tx - self.t[0], ty - self.t[1])
                now = self.now_s()
                gap = 0.0 if self.last_rtk_eval_t is None \
                    else now - self.last_rtk_eval_t
                self.last_rtk_eval_t = now
                trusted, self.rtk_streak = gate_rtk(
                    resid, self.rtk_streak, gap,
                    self.rtk_gate_m, self.rtk_gate_n, self.rtk_gate_reset)
            self.rtk_ok = trusted
            if trusted:
                self.set_mode('GPS_RTK')
                # snap directly — smoothing across surrounding plain-fix
                # bias costs ~1 m at exactly the moments GPS is at its best
                # (measured 1.23 m EMA vs 0.14 m snapped)
                self.snap_anchor(gx, gy, g=0.5)
            elif self.pcd_topic and self.last_pcd is not None:
                # matcher keeps the anchor; suspect stream observable for
                # field debugging and the paper's ablation
                self.set_mode('GPS_SUSPECT')
            else:
                # no alternative absolute reference: bounded EMA creep — a
                # genuine offset (odom drift) still converges via the slew
                # path and the shrinking innovation then earns trust back
                self.set_mode('GPS_SUSPECT')
                self.update_anchor(gx, gy)
        elif src == 'GPS':
            self.set_mode(src)
            self.update_anchor(gx, gy)
        else:
            # PCD / PCD_HOLD: the matcher owns the anchor — plain fixes must
            # not re-drag it between sparse corrections
            self.set_mode(src)

    def update_anchor(self, gx, gy):
        now = self.now_s()
        dt = 0.1 if self.last_gps_t is None else max(1e-3, now - self.last_gps_t)
        self.last_gps_t = now
        ox, oy, _ = self.odom
        tx, ty = anchor_target(gx, gy, ox, oy, self.th)
        if not self.anchored:
            self.t = [tx, ty]       # snap over the provisional bootstrap value
            self.anchored = True
            self.get_logger().info(
                f'map frame anchored: t=({tx:.2f}, {ty:.2f}) th={math.degrees(self.th):.1f} deg')
        else:
            g = min(1.0, dt / self.xy_tau)
            dx = g * (tx - self.t[0])
            dy = g * (ty - self.t[1])
            step = math.hypot(dx, dy)
            if step > self.max_step:
                scale = self.max_step / step
                dx *= scale
                dy *= scale
            self.t[0] += dx
            self.t[1] += dy

    def publish(self, stamp):
        tfm = TransformStamped()
        tfm.header.stamp = stamp
        tfm.header.frame_id = 'map'
        tfm.child_frame_id = 'odom'
        tfm.transform.translation.x = self.t[0]
        tfm.transform.translation.y = self.t[1]
        tfm.transform.rotation.z = math.sin(self.th / 2.0)
        tfm.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf.sendTransform(tfm)

        ox, oy, oyaw = self.odom
        c, s = math.cos(self.th), math.sin(self.th)
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'map'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = c * ox - s * oy + self.t[0]
        out.pose.pose.position.y = s * ox + c * oy + self.t[1]
        myaw = wrap(oyaw + self.th)
        out.pose.pose.orientation.z = math.sin(myaw / 2.0)
        out.pose.pose.orientation.w = math.cos(myaw / 2.0)
        self.pub_global.publish(out)


def main():
    rclpy.init()
    node = MapAnchorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
