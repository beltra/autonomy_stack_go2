#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source "$SCRIPT_DIR/launch_common.sh"
setup_workspace
tune_ros2_network_buffers

ros2 launch vehicle_simulator system_real_robot.launch
