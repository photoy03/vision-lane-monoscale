#!/bin/bash
# Play the recorded HUD parking view into the HUD and save screenshots. usage: hud_shot.sh RUN START_OFFSET_S OUT.png
cd /home/jaehun/vision_support_ws; source /opt/ros/humble/setup.bash; source install/setup.bash; export ROS_DOMAIN_ID=97
R=/home/jaehun/vision_lane_runs/scenario_20261119
ros2 bag play $R/$1/outputs_bag --topics /hero/parking_view --start-offset $2 > /dev/null 2>&1 & bp=$!
DISPLAY=:0 HERO_HUD_SCREENSHOT=$3 HERO_HUD_SCREENSHOT_DELAY_MS=${4:-4000} timeout 30 ros2 run hero_hud hud_node --ros-args --params-file install/hero_hud/share/hero_hud/config/hud.yaml -p display.mirror_horizontal:=false -p parking.plan_view:=true 2>&1 | tail -1
kill $bp 2>/dev/null
