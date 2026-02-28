#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash
ENABLE_STAIRS_AUTONOMY=${ENABLE_STAIRS_AUTONOMY:-false}
STAIRS_MOTION_DRY_RUN=${STAIRS_MOTION_DRY_RUN:-true}
STAIRS_MOTION_BACKEND=${STAIRS_MOTION_BACKEND:-legacy_sport_api}
UNITY_MULTI_LEVEL_MAP=${UNITY_MULTI_LEVEL_MAP:-false}
UNITY_ENV_BINARY_DEFAULT=./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64
UNITY_ENV_BINARY_MULTI=./src/base_autonomy/vehicle_simulator/mesh/unity/environment_multi_level/Model.x86_64
UNITY_ENV_BINARY=${UNITY_ENV_BINARY:-$UNITY_ENV_BINARY_DEFAULT}
if [ "$UNITY_MULTI_LEVEL_MAP" = "true" ] && [ -f "$UNITY_ENV_BINARY_MULTI" ]; then
	UNITY_ENV_BINARY=$UNITY_ENV_BINARY_MULTI
fi

$UNITY_ENV_BINARY &
sleep 3 
ros2 launch vehicle_simulator system_simulation.launch \
	enable_stairs_autonomy:=${ENABLE_STAIRS_AUTONOMY} \
	stairs_motion_dry_run:=${STAIRS_MOTION_DRY_RUN} \
	stairs_motion_backend:=${STAIRS_MOTION_BACKEND}
