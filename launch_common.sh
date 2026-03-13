#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

setup_workspace() {
    cd "$SCRIPT_DIR" || exit 1
    source ./install/setup.bash
}

# tune_ros2_network_buffers() {
#     # Ensure OS packet buffering is tuned for ROS 2 over Ethernet
#     sudo sysctl -w net.core.rmem_max=2147483647
#     sudo sysctl -w net.core.rmem_default=2147483647
#     sudo sysctl -w net.ipv4.ipfrag_time=3
#     sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
# }
