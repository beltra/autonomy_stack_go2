#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source "$SCRIPT_DIR/launch_common.sh"
setup_workspace
tune_ros2_network_buffers

echo "Starting RViz for local planner and route planner..."
# ros2 run rviz2 rviz2 -d src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz &
ros2 run rviz2 rviz2 -d src/route_planner/far_planner/rviz/default.rviz &

wait
