#!/usr/bin/env python3
"""Unit tests for map_anchor's pure helpers (no ROS)."""
import math
import sys

sys.path.insert(0, '/home/ppub/scv_ws/src/robot_localization/scripts')
from map_anchor_node import anchor_target, select_source, wrap  # noqa

P = F = 0


def check(name, ok, detail=''):
    global P, F
    P, F = (P + 1, F) if ok else (P, F + 1)
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}")


# --- select_source: 소스 우선순위 정책 --------------------------------
cases = [
    # (rtk, pcd_fresh, pcd_ever) -> expected
    ((True,  True,  True),  'GPS_RTK'),   # RTK가 항상 최우선
    ((True,  False, False), 'GPS_RTK'),
    ((False, True,  True),  'PCD'),       # RTK 없고 PCD 신선 → PCD
    ((False, False, True),  'PCD_HOLD'),  # PCD 이력 있으면 plain GPS로 안 돌아감
    ((False, False, False), 'GPS'),       # PCD 이력 없으면 plain GPS 유지
    ((False, True,  False), 'GPS'),       # fresh인데 ever=False는 불가 조합 → GPS
]
for (r, pf, pe), exp in cases:
    got = select_source(r, pf, pe)
    check(f'select_source{ (r, pf, pe) } -> {exp}', got == exp, f'(got {got})')

# --- anchor_target: 관측→map->odom 변환 (역변환 왕복) -----------------
for th in (0.0, 0.7, -2.0, math.pi):
    ox, oy = 3.2, -1.4
    tx_true, ty_true = 10.0, -5.0
    c, s = math.cos(th), math.sin(th)
    gx = c * ox - s * oy + tx_true
    gy = s * ox + c * oy + ty_true
    tx, ty = anchor_target(gx, gy, ox, oy, th)
    ok = math.hypot(tx - tx_true, ty - ty_true) < 1e-9
    check(f'anchor_target roundtrip th={th:.2f}', ok)

# --- wrap ----------------------------------------------------------------
check('wrap(3π) == π', abs(wrap(3 * math.pi) - math.pi) < 1e-9 or
      abs(wrap(3 * math.pi) + math.pi) < 1e-9)

print(f"\n{P} passed, {F} failed")
sys.exit(1 if F else 0)
