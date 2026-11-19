# 모든 터미널이 같은 그래프를 보도록 하는 환경. source 한다.
#
# 순서가 곧 우선순위다: Autoware(브리지 인터페이스·A*) → hero-release(ioniq_carla_bridge)
# → monoscale → hero_ws. hero-release/sim/record_run.sh 와 같은 사슬이다.
#
# DDS 설정도 녹화 스크립트와 같게 둔다. 카메라 프레임은 Cyclone 조각 크기와 소켓
# 버퍼에 걸려 있고, 다른 설정의 터미널은 조용히 이미지를 잃는다.

HERO_COACH_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
HERO_WS=${HERO_WS:-$(cd "$HERO_COACH_ROOT/../.." && pwd)}
HERO_RELEASE=${HERO_RELEASE:-/home/i/ros2_ws/hero-release}
MONOSCALE=${MONOSCALE:-/home/i/monoscale}
AUTOWARE=${AUTOWARE:-/home/i/autoware}

set +u
source /opt/ros/humble/setup.bash
source "$AUTOWARE/install/setup.bash" > /dev/null 2>&1
source "$HERO_RELEASE/install/setup.bash"
source "$MONOSCALE/install/setup.bash"
source "$HERO_WS/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HERO_RELEASE/sim/config/cyclonedds.xml
