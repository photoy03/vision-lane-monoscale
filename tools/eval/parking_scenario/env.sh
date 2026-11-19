# Common settings for the parking-scenario evaluation scripts. Override any of them in the environment.
#   WS    colcon workspace (built)                 default: the workspace this script is in
#   BAGS  folder with 20261113_172234 / _171835    default WS/rosbag, else ~/Desktop/ROSBAG
#   WORK  where converted bags and run outputs go  default ~/vision_lane_runs/parking_work  (needs ~15 GB)
_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WS=${WS:-$(cd "$_here/../../.." && pwd)}
if [ -z "$BAGS" ]; then BAGS=$WS/rosbag; [ -d "$BAGS" ] || BAGS=$HOME/Desktop/ROSBAG; fi
export BAGS
export WORK=${WORK:-$HOME/vision_lane_runs/parking_work}
export EVAL=$WS/tools/eval/parking_scenario
# machine-specific CUDA runtime (tools/build/local.bash, not committed); absent on the laptop
[ -f $WS/tools/build/local.bash ] && source $WS/tools/build/local.bash
[ -n "$VISION_CUDA_ROOT" ] && [ -d "$VISION_CUDA_ROOT/lib64" ] && export LD_LIBRARY_PATH="$VISION_CUDA_ROOT/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
source /opt/ros/humble/setup.bash
# velodyne_msgs built from source when the apt package is not installed (no sudo): DEPS/install
DEPS=${DEPS:-$HOME/vision_lane_deps}; [ -f $DEPS/install/setup.bash ] && source $DEPS/install/setup.bash
source $WS/install/setup.bash
mkdir -p $WORK
