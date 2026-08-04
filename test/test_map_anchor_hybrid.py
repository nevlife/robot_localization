#!/usr/bin/env python3
"""Unit tests for map_anchor's pure helpers (no ROS)."""
import math
import sys

sys.path.insert(0, '/home/ppub/scv_ws/src/robot_localization/scripts')
from map_anchor_node import YawAutocal, anchor_target, select_source, wrap  # noqa

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

# --- wrap ----------------------------------------------------------------
check('wrap(3π) == π', abs(wrap(3 * math.pi) - math.pi) < 1e-9 or
      abs(wrap(3 * math.pi) + math.pi) < 1e-9)

print(f"\n{P} passed, {F} failed")
sys.exit(1 if F else 0)
