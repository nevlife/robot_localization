#!/usr/bin/env python3
"""Unit tests for map_anchor's pure helpers (no ROS)."""
import math
import sys

import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from map_anchor_node import (anchor_gain, anchor_target, gate_rtk,  # noqa
                             select_source, slew_limit, wrap)

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

# 1) 정합 샘플 need개 연속 → 신뢰 획득 (그 전까지는 불신)
streak, granted_at = 0, None
for i in range(1, 7):
    trusted, streak = gate_rtk(0.2, streak, 0.1, TH, N)
    if trusted and granted_at is None:
        granted_at = i
check('gate: 5연속 정합에서 신뢰 획득', granted_at == N, f'(granted at {granted_at})')

# 2) 신뢰 후 이상치 1개 → 즉시 박탈 + streak 리셋 (비대칭 히스테리시스)
trusted, streak = gate_rtk(4.5, streak, 0.1, TH, N)
check('gate: 이상치 1개로 즉시 박탈', not trusted and streak == 0, f'(streak {streak})')

# 3) 박탈 후 다시 need개 정합 필요 (1개로는 부족)
trusted, streak = gate_rtk(0.1, streak, 0.1, TH, N)
check('gate: 박탈 후 1개 정합으로는 미복귀', not trusted and streak == 1)

# 4) 샘플 간 공백 > reset_gap → streak 리셋
streak = 4
trusted, streak = gate_rtk(0.1, streak, 6.0, TH, N, reset_gap_s=5.0)
check('gate: 6s 공백에서 streak 리셋', not trusted and streak == 1, f'(streak {streak})')

# 5) 경계값: resid == thresh 는 통과
trusted, streak = gate_rtk(TH, 4, 0.1, TH, N)
check('gate: resid==thresh 통과(5번째로 신뢰)', trusted and streak == 5)

# --- anchor_gain: 공분산 역가중 이득 -------------------------------------
g_good = anchor_gain(0.1, 2.0, 0.3)      # cov < cov_ref → 공칭 이득
check('gain: 양호 fix는 공칭 이득', abs(g_good - 0.05) < 1e-9, f'({g_good:.4f})')
g_bad = anchor_gain(0.1, 2.0, 100.0)     # cov 100 → 100배 느리게
check('gain: cov=100은 1/100 이득', abs(g_bad - 0.0005) < 1e-9, f'({g_bad:.5f})')
check('gain: cov 클수록 단조 감소',
      anchor_gain(0.1, 2.0, 10.0) > anchor_gain(0.1, 2.0, 400.0))
check('gain: cov 미보고(None)는 공칭', anchor_gain(0.1, 2.0, None) == g_good)
check('gain: 상한 1.0', anchor_gain(100.0, 2.0, 0.1) == 1.0)

# --- slew_limit: 절대 이동률 상한 ----------------------------------------
dx, dy = slew_limit(3.0, 4.0, 0.5, 0.5, 0.1)   # rate 0.5 m/s × 0.1 s = 0.05 m
check('slew: 이동률 상한 적용', abs(math.hypot(dx, dy) - 0.05) < 1e-9,
      f'({math.hypot(dx, dy):.4f})')
dx, dy = slew_limit(0.01, 0.0, 0.5, 0.5, 0.1)  # 상한 이하 → 그대로
check('slew: 상한 이하는 무변경', abs(dx - 0.01) < 1e-12 and dy == 0.0)
dx, dy = slew_limit(3.0, 4.0, 0.5, 0.0, 0.1)   # rate 비활성 → per-sample만
check('slew: rate=0이면 per-sample 상한만', abs(math.hypot(dx, dy) - 0.5) < 1e-9)
# 10 Hz에서 앵커가 차량 최고속(1.3 m/s)을 넘지 못하는지 — 7/30 실측 5.0 m/s
worst = max(math.hypot(*slew_limit(99.0, 99.0, 0.5, 0.5, 0.1)) / 0.1
            for _ in range(1))
check('slew: 10Hz 최악에도 앵커 속도 <= 0.5 m/s', worst <= 0.5 + 1e-9,
      f'({worst:.2f} m/s)')

# --- wrap ----------------------------------------------------------------
check('wrap(3π) == π', abs(wrap(3 * math.pi) - math.pi) < 1e-9 or
      abs(wrap(3 * math.pi) + math.pi) < 1e-9)

print(f"\n{P} passed, {F} failed")
sys.exit(1 if F else 0)
