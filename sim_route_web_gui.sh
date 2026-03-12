#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source "$SCRIPT_DIR/launch_common.sh"
setup_workspace

function cleanup() {
    echo ""
    echo "Shutting down all processes..."
    trap '' SIGINT SIGTERM   # prevent recursive signals
    kill -INT 0 2>/dev/null  # SIGINT to entire process group
    sleep 2
    kill -9 0 2>/dev/null    # force-kill stragglers
    wait 2>/dev/null
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "Starting simulation (Route Planner) + Web GUI..."
echo ""
echo "============================================================"
echo "  Web GUI will be available at:"
echo "    http://localhost:8080"
echo ""
echo "  rosbridge WebSocket on port 9090."
echo "============================================================"
echo ""

# Launch simulation with route planner
./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64 &
sleep 3
ros2 launch vehicle_simulator system_simulation_with_route_planner.launch &

# Wait for system to initialize
sleep 5

# Launch web GUI
ros2 launch far_web_gui far_web_gui.launch.py &

wait
