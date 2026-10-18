#!/usr/bin/env bash
# Run a pipeline launch and start it again only when the pipeline itself asked
# for it -- clock_guard after the ROS clock jumped back -- never after a crash,
# a configuration error or Ctrl-C. Each start gets its own run ID; the request
# must name that ID, so a file left by an earlier run cannot cause a restart.
#   bash tools/run/supervise_launch.sh live.launch.py device:=... imu_device:=...
# Restart limit: VISION_SUPERVISE_MAX_RESTARTS (5) within VISION_SUPERVISE_WINDOW_SEC (600).
# A full restart is not seamless: poses pause and the map starts over.
# Each start gets VISION_RESTART_INDEX (0 = first start) and, after a restart,
# VISION_RESTART_REASON (the request that caused it, one line): map_assembler
# reports "recovering" instead of "starting" until its first fused map.
support_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
source /opt/ros/humble/setup.bash
source "$support_root/install/setup.bash"
set -u  # after the ROS setup scripts, which read unset variables

# The command is overridable for tests only.
read -r -a launch_command <<< "${VISION_SUPERVISE_COMMAND:-ros2 launch vision_mapping_bringup}"
max_restarts=${VISION_SUPERVISE_MAX_RESTARTS:-5}
window_sec=${VISION_SUPERVISE_WINDOW_SEC:-600}
state_dir=$(mktemp -d "${TMPDIR:-/tmp}/vision_supervise_XXXXXX")
export VISION_RESTART_REQUEST_DIR=$state_dir

stop=0
child=0
on_stop() {
  stop=1
  [[ $child -ne 0 ]] && kill -INT "$child" 2>/dev/null
}
trap on_stop INT TERM

say() { echo "[supervise_launch] $*" >&2; }

restarts=()   # start times of recent restarts
start=0
reason=""
while true; do
  start=$((start + 1))
  export VISION_RUN_ID="run_$(date +%Y%m%d_%H%M%S)_$$_${start}"
  export VISION_RESTART_INDEX=$((start - 1))
  export VISION_RESTART_REASON=$reason
  rm -f "$state_dir"/*.restart "$state_dir"/*.tmp
  say "starting $VISION_RUN_ID"
  "${launch_command[@]}" "$@" &
  child=$!
  wait "$child"; code=$?
  if [[ $stop -eq 1 ]]; then
    # Give the launch 10 s for its own shutdown, then insist.
    for _ in $(seq 100); do kill -0 "$child" 2>/dev/null || break; sleep 0.1; done
    kill -TERM "$child" 2>/dev/null
  fi
  while kill -0 "$child" 2>/dev/null; do wait "$child"; code=$?; done
  child=0
  if [[ $stop -eq 1 ]]; then say "stopped by signal"; rm -rf "$state_dir"; exit 0; fi

  request="$state_dir/$VISION_RUN_ID.restart"
  if [[ ! -f $request ]] || ! grep -q "\"run_id\": \"$VISION_RUN_ID\"" "$request"; then
    say "pipeline stopped without a restart request (launch exit $code); not restarting"
    rm -rf "$state_dir"; exit "$(( code == 0 ? 1 : code ))"
  fi
  say "restart requested: $(cat "$request")"
  reason=$(tr '\n' ' ' < "$request")

  now=$(date +%s)
  recent=()
  for t in "${restarts[@]+"${restarts[@]}"}"; do (( now - t < window_sec )) && recent+=("$t"); done
  restarts=("${recent[@]+"${recent[@]}"}" "$now")
  if (( ${#restarts[@]} > max_restarts )); then
    say "more than $max_restarts restarts within ${window_sec}s; giving up"
    rm -rf "$state_dir"; exit 2
  fi
  # Wait 3 s, but leave at once on Ctrl-C.
  for _ in $(seq 30); do
    sleep 0.1
    if [[ $stop -eq 1 ]]; then say "stopped by signal"; rm -rf "$state_dir"; exit 0; fi
  done
done
