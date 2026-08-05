#!/bin/bash
# 대체 LIO(Faster-LIO, RKO-LIO) 설치 — 새 워크스페이스/장비에서 재현용.
#
# 왜 이 스크립트가 필요한가: 두 알고리즘은 상류 저장소를 클론해 쓰는데,
# SCV 에 맞추려면 각각 수술이 필요하다(아래). 클론 안의 수정은 우리
# 저장소에 남지 않으므로 패치·설정을 여기에 보관하고 여기서 적용한다.
#
#   Faster-LIO : livox_ros_driver2 강제 의존 제거(SCV 는 velodyne 전용).
#                코드 경로는 FASTER_LIO_WITH_LIVOX 가드로 비활성만 하고
#                삭제하지 않는다. + common_lib.h 의 <deque> 누락 보완.
#   RKO-LIO    : publish_odom_tf 파라미터 신설 — 기본 동작이 odom->base_link
#                TF 를 쏘는데 SCV 는 EKF 가 그 변환의 소유자다.
#
# 사용: install_lio_alternatives.sh [워크스페이스_src_경로]
#       기본값 /home/scv/SCV_park/src/localization
set -o pipefail
SRC=${1:-/home/scv/SCV_park/src/localization}
HERE="$(cd "$(dirname "$0")" && pwd)"

command -v git >/dev/null || { echo "git 필요"; exit 1; }
dpkg -l libgoogle-glog-dev 2>/dev/null | grep -q ^ii || {
  echo "libgoogle-glog-dev 미설치 — sudo apt install libgoogle-glog-dev libgflags-dev"; exit 1; }

mkdir -p "$SRC" && cd "$SRC" || exit 1

# --- RKO-LIO --------------------------------------------------------------
[ -d rko_lio ] || git clone --depth 1 https://github.com/PRBonn/rko_lio.git
( cd rko_lio && git apply --check "$HERE/rko_lio_publish_odom_tf.patch" 2>/dev/null \
    && git apply "$HERE/rko_lio_publish_odom_tf.patch" && echo "[rko] 패치 적용" \
    || echo "[rko] 패치 생략(이미 적용됐거나 상류 변경)" )
cp "$HERE/scv.yaml" "$HERE/scv_ros_params.yaml" rko_lio/config/

# --- Faster-LIO -----------------------------------------------------------
[ -d faster-lio-ros2 ] || git clone --depth 1 https://github.com/RollingOat/faster-lio-ros2.git
( cd faster-lio-ros2 && git apply --check "$HERE/faster_lio_no_livox.patch" 2>/dev/null \
    && git apply "$HERE/faster_lio_no_livox.patch" && echo "[faster] 패치 적용" \
    || echo "[faster] 패치 생략(이미 적용됐거나 상류 변경)" )
cp "$HERE/velodyne_scv.yaml" faster-lio-ros2/faster-lio/config/
# 번들된 livox 메시지 패키지는 빌드하지 않는다
touch faster-lio-ros2/faster-lio/thirdparty/livox_ros_driver/COLCON_IGNORE

WS="$(cd "$SRC/../.." && pwd)"
echo
echo "다음: cd $WS && colcon build --packages-select rko_lio faster_lio \\"
echo "        --cmake-args -DCMAKE_BUILD_TYPE=Release -DRKO_LIO_BUILD_ROS=ON \\"
echo "                     -DRKO_LIO_FETCH_CONTENT_DEPS=ON"
echo "확인: ros2 launch robot_localization scv_dual_ekf.launch.py lio_source:=rko"
echo "      (fastlio | fasterlio | rko — 셋 다 /odometry/fast_lio 발행)"
