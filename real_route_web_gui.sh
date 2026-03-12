#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source "$SCRIPT_DIR/launch_common.sh"
setup_workspace
tune_ros2_network_buffers

function cleanup() {
    echo ""
    echo "Sending StandDown command for clean exit..."
    ros2 topic pub --once /api/sport/request unitree_api/msg/Request "{header: {identity: {id: 0, api_id: 1005}}}" 2>/dev/null
    echo "Shutting down all processes..."
    trap '' SIGINT SIGTERM
    kill -INT 0 2>/dev/null
    sleep 2
    kill -9 0 2>/dev/null
    wait 2>/dev/null
    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "Starting system (SLAM and Route Planner) without RViz + Web GUI..."
echo ""
echo "============================================================"
echo "  Web GUI will be available at:"
echo "    http://<ROBOT_IP>:8080"
echo ""
echo "  Connect from your browser using the robot's Wi-Fi IP."
echo "  rosbridge WebSocket on port 9090."
echo "============================================================"
echo ""

# Launch main system without RViz
ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch rviz:=false &

# Wait a moment for the main system to start
sleep 5

# Launch web GUI (rosbridge + HTTP server + bridge node)
ros2 launch far_web_gui far_web_gui.launch.py &

# Wait for both
wait
