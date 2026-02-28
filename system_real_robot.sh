#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source ./install/setup.bash
ENABLE_STAIRS_AUTONOMY=${ENABLE_STAIRS_AUTONOMY:-false}
STAIRS_MOTION_DRY_RUN=${STAIRS_MOTION_DRY_RUN:-true}
STAIRS_MOTION_BACKEND=${STAIRS_MOTION_BACKEND:-legacy_sport_api}

ros2 launch vehicle_simulator system_real_robot.launch \
	enable_stairs_autonomy:=${ENABLE_STAIRS_AUTONOMY} \
	stairs_motion_dry_run:=${STAIRS_MOTION_DRY_RUN} \
	stairs_motion_backend:=${STAIRS_MOTION_BACKEND}
