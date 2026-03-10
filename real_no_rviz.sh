#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source "$SCRIPT_DIR/launch_common.sh"
setup_workspace
tune_ros2_network_buffers

function cleanup() {
    echo "Sending StandDown command for clean exit..."
    ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 0, api_id: 1005}}, parameter: ''}"
}

trap cleanup EXIT

echo "Starting system (SLAM) without RViz..."
ros2 launch vehicle_simulator system_real_robot.launch rviz:=false
