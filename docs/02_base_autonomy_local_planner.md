# Base Autonomy: `local_planner`

**Location:** `src/base_autonomy/local_planner/`
**When Used:** All modes (simulation and real robot, with or without route planner).

## Overview

This package contains two nodes — `localPlanner` and `pathFollower` — that together implement **local obstacle-avoidant path planning and path tracking** for the Go2 robot.

---

## Node 1: `localPlanner`

### Algorithm Description

The local planner uses a **pre-computed path set** approach for rapid collision-free path selection:

1. **Pre-computed Paths:** At startup, 343 candidate paths (organized in 7 groups) are loaded from PLY files in the `paths/` directory. These paths represent a discretized set of possible trajectories fanning out from the robot.

2. **Obstacle Checking:** For each incoming scan, the planner transforms all obstacle points into the vehicle frame. It evaluates each candidate path at 36 rotation directions (10° increments covering 360°), checking which paths are blocked by obstacles using a **grid-voxel correspondence table**. A path is considered blocked if the number of obstacle points mapped to it exceeds `pointPerPathThre`.

3. **Path Scoring:** Free (unblocked) paths are scored based on:
   - Alignment with the desired direction (joystick or goal)
   - Rotation direction weight (preference for forward motion)
   - Path group weight (preference for straighter paths)
   - Terrain cost penalty (if `useCost` is enabled)

4. **Multi-Scale Search:** If no path is found at the current scale, the planner iteratively reduces `pathScale` (by `pathScaleStep`) and `pathRange` (by `pathRangeStep`) to search for shorter collision-free paths.

5. **Output:** The winning path is published on `/path` in the vehicle frame.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Current robot pose |
| `/registered_scan` | `PointCloud2` | Registered LiDAR scan (used when `useTerrainAnalysis=false`) |
| `/terrain_map` | `PointCloud2` | Terrain analysis output (used when `useTerrainAnalysis=true`) |
| `/joy` | `Joy` | Joystick commands |
| `/way_point` | `PointStamped` | Goal waypoint |
| `/speed` | `Float32` | Commanded speed (from external source) |
| `/navigation_boundary` | `PolygonStamped` | Navigation boundary polygon |
| `/added_obstacles` | `PointCloud2` | Externally added obstacles |
| `/check_obstacle` | `Bool` | Enable/disable obstacle checking |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/path` | `Path` | Selected collision-free path (vehicle frame) |
| `/free_paths` | `PointCloud2` | Visualization of all free paths |

### Configuration Parameters

| Parameter | Default (Launch) | Description |
|-----------|-----------------|-------------|
| `pathFolder` | `$(find local_planner)/paths` | Directory containing pre-computed path files |
| `vehicleLength` | `0.3` | Robot length for collision checking (m) |
| `vehicleWidth` | `0.7` | Robot width for collision checking (m) |
| `sensorOffsetX` | `0.0` (sim) / `0.3` (real) | Sensor-to-vehicle-center X offset (m) |
| `sensorOffsetY` | `0.0` | Sensor-to-vehicle-center Y offset (m) |
| `twoWayDrive` | `false` | Allow backward driving |
| `laserVoxelSize` | `0.05` | Voxel size for raw scan downsampling (m) |
| `terrainVoxelSize` | `0.2` | Voxel size for terrain cloud downsampling (m) |
| `useTerrainAnalysis` | `true` | Use terrain_analysis output instead of raw scan |
| `checkObstacle` | `true` | Enable obstacle checking |
| `checkRotObstacle` | `false` | Check rotational obstacles |
| `adjacentRange` | `3.0` | Planning range around the robot (m) |
| `obstacleHeightThre` | `0.3` | Height above ground to classify as obstacle (m) |
| `groundHeightThre` | `0.1` | Max height to classify as ground (m) |
| `costHeightThre` | `0.1` | Height threshold for cost penalty |
| `costScore` | `0.02` | Minimum cost score penalty |
| `useCost` | `false` | Enable terrain cost scoring |
| `pointPerPathThre` | `2` | Min obstacle points to block a path |
| `minRelZ` / `maxRelZ` | `-0.5` / `0.25` | Relative Z bounds for point filtering (m) |
| `maxSpeed` | `1.0` | Maximum allowed speed (m/s) |
| `dirWeight` | `0.02` | Direction alignment weight in scoring |
| `dirThre` | `90.0` | Max angular difference to consider (degrees) |
| `pathScale` | `0.75` | Initial path scale factor |
| `minPathScale` | `0.5` | Minimum path scale factor |
| `pathScaleStep` | `0.25` | Path scale reduction step |
| `pathScaleBySpeed` | `true` | Scale paths by current speed |
| `minPathRange` | `1.0` | Minimum path range (m) |
| `pathRangeStep` | `0.5` | Path range reduction step (m) |
| `pathRangeBySpeed` | `true` | Adjust path range by speed |
| `pathCropByGoal` | `true` | Crop paths to goal distance |
| `autonomyMode` | `false` | Start in autonomous waypoint mode |
| `autonomySpeed` | `1.0` | Speed in autonomous mode (m/s) |
| `joyToSpeedDelay` | `2.0` | Delay before external speed overrides joystick (s) |
| `joyToCheckObstacleDelay` | `5.0` | Delay before external obstacle checking override (s) |
| `goalClearRange` | `0.5` | Extra range past goal for path evaluation (m) |
| `goalX` / `goalY` | `0.0` | Initial goal position |

---

## Node 2: `pathFollower`

### Algorithm Description

The path follower implements a **pure-pursuit-like path tracking** controller:

1. **Look-Ahead Point:** Finds the path point at `lookAheadDis` ahead of the current position.
2. **Direction Error:** Computes the angular difference between the vehicle heading and the direction to the look-ahead point.
3. **Yaw Rate Control:** Proportional control on the heading error (`yawRateGain` × dirDiff), clamped to `maxYawRate`.
4. **Speed Control:** Acceleration-limited speed ramping. Slows down near the goal (`slowDwnDisThre`). Can slow on high inclination rates.
5. **Omnidirectional Motion:** Decomposes velocity into forward (cos) and lateral (sin) components based on direction error, enabling omnidirectional walking.
6. **Two-Way Drive:** Can switch between forward and backward driving with hysteresis (`switchTimeThre`).
7. **Safety Stops:** Responds to `/stop` topic for safety (bit-masked: 1=forward, 2=backward, 4=CW, 8=CCW).

### Real Robot Interface

On the real robot (`is_real_robot=true`), the pathFollower:
- Sends `RecoveryStand` and `StandUp` commands at startup
- Activates `ClassicWalk` mode (better for stairs/slopes, firmware ≥ V1.1.6)
- Publishes velocity commands via the Unitree Sport API (`/api/sport/request`)
- Deactivates ClassicWalk on exit if `disableClassicWalkOnExit=true`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Current robot pose |
| `/path` | `Path` | Path to follow (from localPlanner) |
| `/joy` | `Joy` | Joystick commands |
| `/speed` | `Float32` | External speed override |
| `/stop` | `Int8` | Safety stop flags (bitmask) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `TwistStamped` | Velocity command (linear.x, linear.y, angular.z) |
| `/api/sport/request` | `Request` | Unitree Go2 Sport API request (real robot) |

### Configuration Parameters

| Parameter | Default (Launch) | Description |
|-----------|-----------------|-------------|
| `sensorOffsetX/Y` | From launch | Sensor offset from vehicle center (m) |
| `pubSkipNum` | `1` | Publish every Nth control cycle |
| `twoWayDrive` | `false` | Allow backward driving |
| `lookAheadDis` | `0.5` | Look-ahead distance for path tracking (m) |
| `yawRateGain` | `1.5` | Proportional gain for yaw rate control |
| `stopYawRateGain` | `1.5` | Yaw rate gain when stopped |
| `maxYawRate` | `80.0` | Maximum yaw rate (deg/s) |
| `maxSpeed` | `1.0` | Maximum speed (m/s) |
| `maxAccel` | `2.0` | Maximum acceleration (m/s²) |
| `switchTimeThre` | `1.0` | Min time between fwd/bwd switches (s) |
| `dirDiffThre` | `0.4` | Heading error threshold to start moving (rad) |
| `omniDirDiffThre` | `1.5` | Relaxed heading threshold near goal (rad) |
| `stopDisThre` | `0.3` | Distance to goal to stop (m) |
| `slowDwnDisThre` | `0.75` | Distance to goal to start slowing (m) |
| `useInclRateToSlow` | `false` | Slow on high inclination rate |
| `inclRateThre` | `120.0` | Inclination rate threshold (deg/s) |
| `slowRate1/2` | `0.25/0.5` | Slow-down speed multipliers |
| `slowTime1/2` | `2.0/2.0` | Duration of slow-down phases (s) |
| `useInclToStop` | `false` | Stop on high inclination |
| `inclThre` | `45.0` | Inclination threshold to trigger stop (deg) |
| `stopTime` | `5.0` | Duration to stay stopped after inclination trigger (s) |
| `noRotAtStop` | `false` | Disable yaw at zero speed |
| `noRotAtGoal` | `true` | Disable yaw when at goal |
| `goalCloseDis` | `0.4` | Distance threshold for "close to goal" behavior |
| `is_real_robot` | `true` | Enable real robot interface (Sport API) |
| `enableClassicWalk` | `true` | Activate ClassicWalk at startup |
| `disableClassicWalkOnExit` | `false` | Deactivate ClassicWalk on shutdown |

## Static TF Publishers (from `local_planner.launch`)

| Parent | Child | Transform | Description |
|--------|-------|-----------|-------------|
| `sensor` | `vehicle` | `(-sensorOffsetX, -sensorOffsetY, 0)` | Vehicle center relative to sensor |
| `sensor` | `camera` | `(0, 0, cameraOffsetZ)` rotated | Camera frame |
