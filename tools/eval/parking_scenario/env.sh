# Common settings for the parking-scenario evaluation scripts. Override any of them in the environment.
#   WS    colcon workspace (built)                 default ~/vision_support_ws
#   BAGS  folder with 20261113_172234 / _171835    default ~/Desktop/ROSBAG
#   WORK  where converted bags and run outputs go  default ~/vision_lane_runs/parking_work  (needs ~15 GB; use an external disk)
export WS=${WS:-$HOME/vision_support_ws}
export BAGS=${BAGS:-$HOME/Desktop/ROSBAG}
export WORK=${WORK:-$HOME/vision_lane_runs/parking_work}
export EVAL=$WS/tools/eval/parking_scenario
source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
mkdir -p $WORK
