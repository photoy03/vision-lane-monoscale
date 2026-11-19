#!/usr/bin/env bash
# 키보드로 CARLA 자차를 몬다. 자기 터미널에서 돌려야 한다 — 런치된 노드는 키를 못 읽는다.
#
# keyboard_teleop 은 legacy ros2_ws 의 carla_vision_autopark 에 있다. 그 작업공간은
# ioniq_carla_bridge 의 다른 사본도 들고 있어서, 이 터미널에만 겹친다.
set -u
here=$(dirname "$(readlink -f "$0")")
source "$here/env.sh"
source "${LEGACY_WS:-/home/i/ros2_ws}/install/setup.bash"
exec ros2 run carla_vision_autopark keyboard_teleop --ros-args -p use_sim_time:=true
