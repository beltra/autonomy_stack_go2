#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source "$SCRIPT_DIR/launch_common.sh"
setup_workspace

./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
sleep 3
ros2 launch vehicle_simulator system_simulation_with_route_planner.launch
