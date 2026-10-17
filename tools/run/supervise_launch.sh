#!/usr/bin/env bash
# Run a pipeline launch and start it again whenever it stops on its own -- for
# example clock_guard stopping it after the ROS clock jumped back. Ctrl-C ends.
#   bash tools/run/supervise_launch.sh live.launch.py device:=... imu_device:=...
support_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/humble/setup.bash
source "$support_root/install/setup.bash"
set -u  # after the ROS setup scripts, which read unset variables
stop=0
trap 'stop=1' INT TERM
while true; do
  ros2 launch vision_mapping_bringup "$@" &
  child=$!
  wait "$child"
  code=$?
  [[ $stop -eq 1 ]] && { kill -INT "$child" 2>/dev/null; wait "$child" 2>/dev/null; exit 0; }
  echo "[supervise_launch] pipeline stopped (exit $code); starting it again in 3 s" >&2
  sleep 3
done
