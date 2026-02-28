#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR"
source ./install/setup.bash

export UNITY_MULTI_LEVEL_MAP=${UNITY_MULTI_LEVEL_MAP:-true}
export ENABLE_MULTI_LAYER_ROUTE=${ENABLE_MULTI_LAYER_ROUTE:-true}
export ENABLE_STAIRS_AUTONOMY=${ENABLE_STAIRS_AUTONOMY:-true}
export STAIRS_MOTION_DRY_RUN=${STAIRS_MOTION_DRY_RUN:-true}
export STAIRS_MOTION_BACKEND=${STAIRS_MOTION_BACKEND:-legacy_sport_api}

./system_simulation_with_route_planner.sh