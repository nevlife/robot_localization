#!/usr/bin/env python3
"""Unit tests for map_anchor's pure helpers (no ROS)."""
import math
import sys

import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from map_anchor_node import (YawAutocal, anchor_gain, anchor_target,  # noqa
                             gate_rtk, select_source, slew_limit, wrap)

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

# --- YawAutocal: 온라인 yaw 오프셋 추정 ----------------------------------
# 시나리오: IMU 가 167도 어긋난 차량(2026-08-04 실측)이 동쪽으로 직진.
# 실제 진행방위 0, IMU 헤딩 = -167도 → 추정기는 +167도 근방으로 수렴해야
# 하고, ±180 경계를 넘나드는 노이즈에서도 벡터 EMA 평균이 무너지지 않아야.
import random
random.seed(7)
TRUE = math.radians(167.0)
cal = YawAutocal(d_min=1.5, alpha=0.3, n_apply=6)
t = 0.0
x = 0.0
meas = 0
for k in range(400):
    t += 0.1
    x += 0.10                     # 1.0 m/s 동진
    hdg = wrap(0.0 - TRUE + random.gauss(0.0, math.radians(3)))
    gx = x + random.gauss(0.0, 0.02)
    gy = random.gauss(0.0, 0.02)
    if cal.add(t, gx, gy, hdg, x, wz=0.0) is not None:
        meas += 1
check('autocal 167deg 수렴', cal.active and
      abs(wrap(cal.corr - TRUE)) < math.radians(5),
      f'(corr {math.degrees(cal.corr):.1f}deg, meas {meas})')

# 후진 게이트: 후진(v<=0)은 창 전체를 무효화해야 한다 — 후진 변위 방위는
# 헤딩의 정반대라 추정을 180도 오염시킨다
cal2 = YawAutocal()
t = x = 0.0
polluted = 0
for k in range(100):
    t += 0.1
    x -= 0.10                     # 후진
    if cal2.add(t, x, 0.0, 0.0, x, wz=0.0) is not None:
        polluted += 1
check('autocal 후진 게이트 (측정 0)', polluted == 0 and cal2.n == 0)

# 순후진 뒤 재전진: 후진 잔재가 첫 전진 측정을 오염시키지 않아야 한다
# (후진 변위 방위는 헤딩의 정반대 — 이월되면 180도 오염)
cal3 = YawAutocal()
t = x = 0.0
cal3.add(t, x, 0.0, 0.0, x, wz=0.0)
for k in range(8):                # 0.8 m 순후진 (도중 창 폐기 발동)
    t += 0.1
    x -= 0.1
    cal3.add(t, x, 0.0, 0.0, x, wz=0.0)
first = None
for k in range(20):               # 재전진 — 첫 측정의 오차각 확인
    t += 0.1
    x += 0.1
    e = cal3.add(t, x, 0.0, 0.0, x, wz=0.0)
    if e is not None:
        first = e
        break
check('autocal 순후진 후 재전진 첫 측정 무오염',
      first is not None and abs(math.degrees(first)) < 5.0,
      f'(err {math.degrees(first):.1f}deg)' if first is not None else '(측정 없음)')

# 전진+후진 혼합 창: chord 가 odo 의 0.8배 미달이라 측정이 안 나와야 한다
cal3m = YawAutocal()
t = x = 0.0
mixed = 0
for k in range(12):               # 1.2 m 전진
    t += 0.1
    x += 0.1
    if cal3m.add(t, x, 0.0, 0.0, x, wz=0.0) is not None:
        mixed += 1
# 측정은 났을 수 있음(순전진 구간) — 이제 0.5 m 후진 후 다시 0.5 m 전진:
# chord ~1.2 인데 odo ~1.2+(-0.5)+0.5 중 창 내 혼합 구간은 odo < 0.8*chord
cal3m.buf.clear(); cal3m.n = 0
for k in range(5):
    t += 0.1
    x -= 0.1
    cal3m.add(t, x, 0.0, 0.0, x, wz=0.0)
for k in range(5):
    t += 0.1
    x += 0.1
    cal3m.add(t, x, 0.0, 0.0, x, wz=0.0)
check('autocal 혼합(왕복) 창 측정 없음', cal3m.n == 0)

# EKF twist 노이즈 내성: 실제 전진 1 m/s 인데 twist 가 -0.1~+0.4 로 요동
# (챔버 실측 재현) — 순간 부호로 창을 버리면 측정 0회가 되는 상황.
# odo 는 잡음 평균만큼 과소평가되지만 0.8 게이트 안에서 측정이 성립해야 한다.
cal_ns = YawAutocal(alpha=0.3, n_apply=3)
t = x = mtr = 0.0
random.seed(11)
for k in range(300):
    t += 0.1
    x += 0.10
    v_noisy = 1.0 + random.gauss(0.0, 0.35)   # 간헐 음수 포함
    mtr += v_noisy * 0.1
    cal_ns.add(t, x, 0.0, wrap(0.0 - TRUE), mtr, wz=0.0)
check('autocal twist 노이즈 내성 (측정 발생+수렴)', cal_ns.n >= 3 and
      abs(wrap(cal_ns.corr - TRUE)) < math.radians(5),
      f'(n {cal_ns.n}, corr {math.degrees(cal_ns.corr):.1f}deg)')

# GPS 점프 기각: 창 중간 5 m 점프 → chord 폭증하지만 odo 불변 → 무측정
cal_j = YawAutocal()
t = x = 0.0
jmp = 0
for k in range(6):
    t += 0.1
    x += 0.1
    cal_j.add(t, x, 0.0, 0.0, x, wz=0.0)
if cal_j.add(t + 0.1, x + 5.0, 0.0, 0.0, x + 0.1, wz=0.0) is not None:
    jmp += 1
check('autocal GPS 점프 기각', jmp == 0)

# 급회전 표본은 보류하되 창은 유지한다 (완만한 곡선은 유효 표본 —
# 등곡률 호에서 chord 방위 = 중간점 방위)
cal3b = YawAutocal()
t = 0.0
cal3b.add(t, 0.0, 0.0, 0.0, 0.0, wz=0.0)
t += 0.1
cal3b.add(t, 0.1, 0.0, 0.0, 0.1, wz=1.0)     # 급회전 → 보류
check('autocal 급회전 보류 (창 유지)', len(cal3b.buf) == 1)

# 곡선 주행: 반경 5 m 호를 따라 돌 때 중간점 헤딩 대조로 오프셋이 맞아야
cal_arc = YawAutocal(alpha=0.4, n_apply=3)
R, w = 5.0, 0.15                  # v = 0.75 m/s
t = 0.0
for k in range(2000):
    t += 0.1
    a = w * t
    hdg_true = wrap(a + math.pi / 2)          # 접선 방위
    cal_arc.add(t, R * math.cos(a), R * math.sin(a),
                wrap(hdg_true - TRUE), R * w * t, wz=w)
check('autocal 곡선 주행 수렴', cal_arc.n > 3 and
      abs(wrap(cal_arc.corr - TRUE)) < math.radians(5),
      f'(corr {math.degrees(cal_arc.corr):.1f}deg, n {cal_arc.n})')

# 정지/저속 게이트: GPS 노이즈만 있는 정지 상태에서 측정이 나오면 안 된다
cal4 = YawAutocal()
t = 0.0
still = 0
for k in range(100):
    t += 0.1
    if cal4.add(t, random.gauss(0, 0.3), random.gauss(0, 0.3), 0.0,
                0.05 * t, wz=0.0) is not None:
        still += 1
check('autocal 정지 게이트 (측정 0)', still == 0)

# 품질 적응 기저선: 열화 fix(cov 0.04, sigma 20 cm)는 d_req=2 m 로 늘어나
# 0.9 m 변위로는 측정이 성립하지 않아야 (RTK 였다면 성립하는 거리)
cal6 = YawAutocal()
t = x = 0.0
early = 0
while x < 0.9:
    t += 0.1
    x += 0.1
    if cal6.add(t, x, 0.0, 0.0, x, wz=0.0, cov=0.04) is not None:
        early += 1
check('autocal 품질 적응 d_req (열화 fix 0.9m 측정 0)', early == 0)
while x < 2.5:                    # 10*sqrt(0.04)=2.0 m 넘기면 성립
    t += 0.1
    x += 0.1
    cal6.add(t, x, 0.0, 0.0, x, wz=0.0, cov=0.04)
check('autocal 품질 적응 d_req (2m 초과 시 측정)', cal6.n >= 1)

# n_apply 전에는 active 금지 (표본 부족 상태의 섣부른 적용 방지)
cal5 = YawAutocal(n_apply=6)
t = x = 0.0
while cal5.n < 5:
    t += 0.1
    x += 0.1
    cal5.add(t, x, 0.0, 0.0, x, wz=0.0)
check('autocal n_apply 전 inactive', not cal5.active and cal5.n == 5)

# 제자리 선회 오염 차단: 28초에 걸쳐 240도 회전(평균 0.15 rad/s 로
# wz_max 0.5 아래)하는 동안 GPS 는 조금씩 표류한다. 2026-08-06 필드에서
# 이 조합이 meas +75/-59도를 만들어 corr 을 +12.6도까지 밀어올렸다.
cal_spin = YawAutocal()
t = 0.0
spin = 0
random.seed(3)
for k in range(280):          # 0.1초 간격 28초
    t += 0.1
    yaw = math.radians(240.0 * k / 280.0)      # 서서히 240도 회전
    x = random.gauss(0, 0.25)                  # 제자리 GPS 표류
    y = random.gauss(0, 0.25)
    if cal_spin.add(t, x, y, wrap(yaw), 0.05 * t, wz=0.15, cov=0.02) is not None:
        spin += 1
check('제자리 선회 중 측정 차단', spin == 0, f'(측정 {spin}회)')

# 완만한 곡선(창 내 yaw 변화 작음)은 계속 유효해야 한다 — 과차단 방지
cal_ok = YawAutocal(alpha=0.4, n_apply=3)
t = 0.0
R, w = 12.0, 0.06            # 반경 12 m, 0.72 m/s → 창 12초에 약 41도
for k in range(1500):
    t += 0.1
    a = w * t
    cal_ok.add(t, R * math.cos(a), R * math.sin(a),
               wrap(wrap(a + math.pi / 2) - TRUE), R * w * t, wz=w, cov=0.02)
check('완만한 곡선은 계속 측정됨', cal_ok.n > 5,
      f'(측정 {cal_ok.n}회, corr {math.degrees(cal_ok.corr):.1f}deg)')

# --- wrap ----------------------------------------------------------------
check('wrap(3π) == π', abs(wrap(3 * math.pi) - math.pi) < 1e-9 or
      abs(wrap(3 * math.pi) + math.pi) < 1e-9)

print(f"\n{P} passed, {F} failed")
sys.exit(1 if F else 0)
