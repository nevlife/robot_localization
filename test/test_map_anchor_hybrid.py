#!/usr/bin/env python3
"""Unit tests for map_anchor's pure helpers (no ROS)."""
import math
import sys

import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from map_anchor_node import anchor_target, gate_rtk, select_source, wrap  # noqa

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

# --- gate_rtk: RTK 재신뢰 혁신 게이트 ------------------------------------
TH, N = 3.0, 5
streak, granted_at = 0, None
for i in range(1, 7):
    trusted, streak = gate_rtk(0.2, streak, 0.1, TH, N)
    if trusted and granted_at is None:
        granted_at = i
check('gate: 5연속 정합에서 신뢰 획득', granted_at == N, f'(granted at {granted_at})')
trusted, streak = gate_rtk(4.5, streak, 0.1, TH, N)
check('gate: 이상치 1개로 즉시 박탈', not trusted and streak == 0, f'(streak {streak})')
trusted, streak = gate_rtk(0.1, streak, 0.1, TH, N)
check('gate: 박탈 후 1개 정합으로는 미복귀', not trusted and streak == 1)
streak = 4
trusted, streak = gate_rtk(0.1, streak, 6.0, TH, N, reset_gap_s=5.0)
check('gate: 6s 공백에서 streak 리셋', not trusted and streak == 1, f'(streak {streak})')
trusted, streak = gate_rtk(TH, 4, 0.1, TH, N)
check('gate: resid==thresh 통과(5번째로 신뢰)', trusted and streak == 5)

# --- wrap ----------------------------------------------------------------
check('wrap(3π) == π', abs(wrap(3 * math.pi) - math.pi) < 1e-9 or
      abs(wrap(3 * math.pi) + math.pi) < 1e-9)

print(f"\n{P} passed, {F} failed")
sys.exit(1 if F else 0)
