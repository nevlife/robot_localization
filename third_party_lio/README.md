# 대체 LiDAR-관성 오도메트리 (Faster-LIO / RKO-LIO)

SCV 는 FAST-LIO2 를 기본 오도메트리로 쓰지만, 같은 자리에 끼울 수 있는
구현 두 가지를 함께 통합해 두었다. 알고리즘 교체는 런치 인자 하나다.

```bash
ros2 launch robot_localization scv_dual_ekf.launch.py lio_source:=fastlio   # 기본
ros2 launch robot_localization scv_dual_ekf.launch.py lio_source:=fasterlio
ros2 launch robot_localization scv_dual_ekf.launch.py lio_source:=rko
# 필드 기동에서도 그대로 통과된다
field_bringup.sh --record lio_source:=rko
```

## 계약 (세 구현이 반드시 지켜야 하는 것)

1. `nav_msgs/Odometry` 를 **`/odometry/fast_lio`** 로 발행 (odom 프레임).
   ekf_odom 이 이 토픽을 differential 모드로 흡수하므로 토픽 이름을 바꾸면
   하류 전체가 조용히 멈춘다.
2. **`odom -> base_link` TF 를 발행하지 않는다.** 그 변환의 소유자는
   ekf_odom 이다. 두 발행자가 겹치면 TF 트리가 진동한다.
3. 입력은 `/velodyne_points` + `/vectornav/imu`.

상위 스택(map_anchor, behavior, MPPI)은 `/odom` 만 보므로 교체의 영향을
받지 않는다.

## 설치 (새 장비/워크스페이스)

```bash
./install_lio_alternatives.sh [<ws>/src/localization]
cd <ws> && colcon build --packages-select rko_lio faster_lio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DRKO_LIO_BUILD_ROS=ON \
                 -DRKO_LIO_FETCH_CONTENT_DEPS=ON
```

선행 조건: `libgoogle-glog-dev`, `libgflags-dev` (Faster-LIO).
RKO-LIO 의 Sophus/tbb/robin-map 등은 `RKO_LIO_FETCH_CONTENT_DEPS=ON` 이
알아서 받아온다(배포판에 `ros-humble-sophus` 가 없어 이 경로가 사실상 필수).

## 상류에 가한 수술과 그 이유

| 대상 | 수술 | 이유 |
|---|---|---|
| Faster-LIO | `livox_ros_driver2` 필수 의존 제거, 관련 코드는 `FASTER_LIO_WITH_LIVOX` 가드로 비활성 | SCV 는 velodyne 전용. 코드는 지우지 않고 가드만 씌워 상류 갱신 시 재적용이 쉽다 |
| Faster-LIO | `common_lib.h` 에 `<deque>` 추가 | 상류 누락 — Humble/GCC 11 에서 컴파일 실패 |
| RKO-LIO | `publish_odom_tf` 파라미터 신설 | 기본 동작이 odom→base_link TF 를 쏜다(계약 2 위반) |

패치 파일(`*.patch`)과 SCV 설정(`scv*.yaml`, `velodyne_scv.yaml`)이 이
디렉토리에 보관돼 있다 — 서드파티 클론 안의 수정은 우리 저장소에 남지
않으므로, 재현 가능한 원본은 항상 여기다.

## 설정에서 걸린 함정 (실측)

- **velodyne `time` 필드는 스캔 끝 기준 "음수 상대 초"** 다. RKO-LIO 의
  자동 감지가 이를 절대 시각으로 오인하면 모든 스캔이 `±194 s delta`
  예외로 버려진다 → `lidar_timestamps.force_relative: true` 필수.
  Faster-LIO 쪽 대응 항목은 `time_scale: 1.0`(기본 nclt 값 1e-3 아님).
- **RKO-LIO 설정 파일이 두 벌인 이유**: 자체 런치(`odometry.launch.py`)는
  평문 YAML 을 받고, `Node(parameters=[...])` 는 `ros__parameters` 구조를
  요구한다. 평문을 Node 에 넘기면 rcl 파싱 실패로 노드가 즉사한다(차량
  실측). 값이 갈라지지 않게 두 파일을 항상 같이 고칠 것.
- **온라인 모드 영구 락아웃 (RKO-LIO)**: 처리가 실시간을 못 따라가면
  IMU 큐(`keep_last(100)` = 1 s)가 넘쳐 "0 IMU message(s) in interval" →
  프레임 드랍 → `lidar_state.time` 정지 → 이후 **모든** 프레임이 1 s 델타
  예외. 한 번 빠지면 스스로 복구하지 못한다(실측: 5 초 만에 정지 후 락).
  실차 투입 전 CPU 여유를 반드시 확인할 것. bag 평가는 `mode:=offline`.
- **extrinsic**: 세 구현 모두 현행 FAST-LIO2 값 (0.34, 0, 0.17)+I 로 맞춰
  비교 변수를 줄였다. 이 값 자체와 URDF 계산값 (0.070, 0, 0.195) 의 27 cm
  불일치 판정은 별도 과제다(자기 앵커 복원과는 무관한 문제).

## 벤치마크 시 주의

같은 bag 을 `ros2 bag play --rate 1.0` 으로 돌리면 PC 성능에 따라 스캔이
버려져(측정: FAST-LIO2 가 10 Hz 입력에 3.5 Hz 출력) **알고리즘이 아니라
그 PC 의 실시간성을 재게 된다.** 0.3 배속 이하로 재생하거나 오프라인
모드를 쓰고, 세 구현의 **공통 시간창**에서만 오차를 비교할 것.
