# Launch arguments of parking_scenario.launch.py for the "nearest visible slot" behaviour, as evaluated on the
# two 11/13 recordings (docs/PARKING_SCENARIO_HANDOVER_KO.md, section 11). Every one of them is an option that
# is off by default in the launch file, so the behaviour only exists where this list is passed.
# Sourced by tools/run/parking_live.sh (vehicle) and tools/eval/parking_scenario/run_hud.sh (recorded inputs).
PARKING_PRESET="select_mode:=nearest select_max_distance_m:=15.0 select_switch_margin_m:=1.0 \
line_max_yaw_rate_dps:=8.0 max_plan_distance_m:=25.0 \
planner_step_m:=0.6 planner_xy_resolution_m:=0.4 planner_footprint_check:=true \
planner_native:=true planner_cost_table:=false planner_heuristic_weight:=1.2 planner_time_limit_sec:=0.025 \
display_predict_hz:=30.0 display_ego_smooth_sec:=0.15 display_align_rows:=true display_fill_slots:=2"
# HUD plan view range from the rear axle (the HUD's own defaults are 16 / 9 m; the selected slot is often
# behind the car, which those cut off).
PARKING_HUD_AHEAD_M=12.0
PARKING_HUD_BEHIND_M=14.0
