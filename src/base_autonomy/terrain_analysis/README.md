# terrain_analysis

This package runs the `terrainAnalysis` node, which converts a registered point cloud into a local terrain map with per-point elevation above the estimated ground.

The node subscribes to:

- `/state_estimation` (`nav_msgs/msg/Odometry`)
- `/registered_scan` (`sensor_msgs/msg/PointCloud2`)
- `/joy` (`sensor_msgs/msg/Joy`)
- `/map_clearing` (`std_msgs/msg/Float32`)

The node publishes:

- `/terrain_map` (`sensor_msgs/msg/PointCloud2`)

The main launch defaults are defined in [launch/terrain_analysis.launch](launch/terrain_analysis.launch), while the fallback in-code defaults are defined in [src/terrainAnalysis.cpp](src/terrainAnalysis.cpp).

## Processing summary

At a high level, the node does the following:

1. Crops the incoming registered scan around the robot using a distance-dependent vertical band.
2. Accumulates the cropped points into a rolling terrain voxel map.
3. Periodically downsamples and decays old voxel contents.
4. Builds a finer planar grid around the robot and estimates ground height per cell.
5. Optionally suppresses dynamic obstacles.
6. Emits terrain points whose intensity stores elevation above the estimated ground.
7. Optionally marks selected no-data regions as obstacles.

Two fixed internal grid resolutions are used by the current implementation:

- Terrain accumulation voxel: `1.0 m`, `21 x 21` cells.
- Planar elevation grid: `0.2 m`, `51 x 51` cells.

These are hard-coded and are **not** runtime parameters.

## Parameter reference

Unless noted otherwise, distances are in meters, times are in seconds, and angles are in degrees.

### Scan filtering and map decay

| Parameter | Type | Launch default | Code default | Meaning | Tuning guidance |
| --- | --- | --- | --- | --- | --- |
| `scanVoxelSize` | double | `0.05` | `0.05` | Leaf size used when downsampling each terrain voxel before reinsertion into the rolling map. Smaller values preserve detail; larger values reduce memory and CPU. | Increase for sparse sensors or CPU pressure. Decrease for fine foothold/curb detail. |
| `decayTime` | double | `2.0` | `2.0` | Maximum age of stored points before they are removed during voxel refresh. Point age is tracked in the point intensity field. | Lower values make the map react faster to change; higher values keep static structure longer. |
| `noDecayDis` | double | `0.0` | `4.0` | Radius around the vehicle inside which points are exempt from time decay. The same distance is also used after startup or map clearing to decide when no-data obstacle marking becomes active. | Set above `0` to retain very near terrain longer. Larger values also delay no-data obstacle activation until the robot has moved that far. |
| `clearingDis` | double | `8.0` | `8.0` | When clearing is triggered, points within this radius are removed from the rolling terrain map. | Use a value slightly larger than the local planning radius if you want a full local reset. |
| `voxelPointUpdateThre` | int | `100` | `100` | A terrain voxel is refreshed once this many new raw points have been inserted into it. | Lower values refresh more often but cost more CPU. |
| `voxelTimeUpdateThre` | double | `2.0` | `2.0` | A terrain voxel is refreshed if this amount of time has passed since its last update, even if `voxelPointUpdateThre` was not reached. | Lower values reduce stale data at the cost of more frequent filtering. |

### Input height gate

Incoming points are accepted only if their relative height satisfies

$$
minRelZ - disRatioZ \cdot d < z - z_{veh} < maxRelZ + disRatioZ \cdot d
$$

where $d$ is the horizontal distance to the robot.

| Parameter | Type | Launch default | Code default | Meaning | Tuning guidance |
| --- | --- | --- | --- | --- | --- |
| `minRelZ` | double | `-1.5` | `-1.5` | Minimum allowed point height relative to the vehicle for cropping and terrain evaluation. | Decrease if the sensor regularly sees deep holes or descending stairs. |
| `maxRelZ` | double | `0.5` | `0.2` | Maximum allowed point height relative to the vehicle for cropping and terrain evaluation. | Increase if you need to preserve tall positive relief close to the robot. |
| `disRatioZ` | double | `0.2` | `0.2` | Expands the allowable vertical band with distance from the robot. | Increase for pitched terrain or long-range sensors; decrease to reject more vertical clutter. |

### Ground estimation

The node estimates one ground elevation value per planar cell using either sorting plus a quantile, or the raw minimum.

| Parameter | Type | Launch default | Code default | Meaning | Tuning guidance |
| --- | --- | --- | --- | --- | --- |
| `useSorting` | bool | `true` | `true` | If `true`, ground is estimated from a quantile of the per-cell elevation samples. If `false`, the minimum sample is used. | Keep `true` for noisy lidars. Use `false` only when the minimum is known to be reliable. |
| `quantileZ` | double | `0.25` | `0.25` | Quantile used when `useSorting` is enabled. `0.0` is the minimum; `0.5` is the median. | Lower values bias toward the lowest visible surface. Higher values resist outliers but can lift the ground estimate. |
| `limitGroundLift` | bool | `false` | `false` | If `true`, the selected quantile cannot exceed the minimum observed elevation by more than `maxGroundLift`. | Enable when the quantile method tends to lift ground over grass, brush, or sparse returns. |
| `maxGroundLift` | double | `0.15` | `0.15` | Maximum allowed difference between the cell minimum and the chosen quantile when `limitGroundLift` is enabled. | Smaller values keep the ground conservative; larger values allow smoother but potentially elevated ground. |

### Obstacle elevation extraction

After ground estimation, the output terrain map keeps points whose elevation above the local ground satisfies:

- `disZ >= 0`
- `disZ < vehicleHeight`
- enough samples exist in the planar cell (`minBlockPointNum`)

If `considerDrop` is enabled, the node uses $|disZ|$ instead of `disZ` before this test.

| Parameter | Type | Launch default | Code default | Meaning | Tuning guidance |
| --- | --- | --- | --- | --- | --- |
| `considerDrop` | bool | `false` | `false` | If `true`, negative deviation below the estimated ground is treated the same as positive deviation by taking the absolute value of the elevation difference. | Enable only if holes, steps down, or pits should be treated as obstacles in the output map. |
| `minBlockPointNum` | int | `10` | `10` | Minimum number of elevation samples required in a planar cell before points from that cell can be emitted as valid terrain output. Also reused by the no-data obstacle logic. | Increase to reject sparse noise; decrease if the map becomes too holey with low-density scans. |
| `vehicleHeight` | double | `1.5` | `1.5` | Maximum elevation above the estimated ground that is still reported in the output map. Output point intensity is set to this relative elevation. | Set slightly above the tallest obstacle the local planner should consider. |

### Dynamic obstacle suppression

When enabled, the node counts points per planar cell that look like dynamic obstacle hits. Cells with at least `minDyObsPointNum` such hits are suppressed from the final terrain output.

This logic uses two passes:

1. Count suspicious points from the accumulated terrain cloud.
2. Reset cells to zero if the newest scan still contains points that pass the dynamic-obstacle entrance test.

| Parameter | Type | Launch default | Code default | Meaning | Tuning guidance |
| --- | --- | --- | --- | --- | --- |
| `clearDyObs` | bool | `false` | `false` | Enables dynamic obstacle suppression. | Turn on only if moving people/objects are leaving persistent false obstacles. |
| `minDyObsDis` | double | `0.3` | `0.3` | Points closer than this are immediately treated as dynamic-obstacle evidence without the angle checks. | Raise slightly if self-hits or body reflections are common. |
| `minDyObsAngle` | double | `0.0` | `0.0` | Minimum vertical angle of a point, computed using `minDyObsRelZ`, required to be considered dynamic-obstacle evidence. | Increase to ignore low, ground-like returns. |
| `minDyObsRelZ` | double | `-0.3` | `-0.5` | Reference relative height used in the dynamic-obstacle entrance-angle test. | Make less negative to be stricter; more negative to admit more points. |
| `absDyObsRelZThre` | double | `0.2` | `0.2` | A point can count as dynamic-obstacle evidence if its roll/pitch compensated relative height magnitude is below this threshold, even if it falls outside the vertical FOV gate. | Useful when you want to catch near-horizontal clutter around the sensor. |
| `minDyObsVFOV` | double | `-16.0` | `-16.0` | Minimum roll/pitch compensated vertical angle for dynamic-obstacle evidence. | Narrow the interval to be more selective. |
| `maxDyObsVFOV` | double | `16.0` | `16.0` | Maximum roll/pitch compensated vertical angle for dynamic-obstacle evidence. | Narrow the interval to reject steep rays or ceiling-like returns. |
| `minDyObsPointNum` | int | `1` | `1` | Number of dynamic-obstacle evidence points needed in a planar cell before output from that cell is suppressed. Also used as the immediate increment for very near points (`dis < minDyObsDis`). | Increase to reduce false suppression from single noisy hits. |

### No-data obstacle generation

When enabled, the node can synthesize obstacle points in a rectangular area around the robot if the map there has insufficient data or is too far below the vehicle.

This logic only runs after `noDataInited == 2`, which means the robot has moved at least `noDecayDis` meters since startup or since the last clearing event.

The rectangular gate is defined in the robot body frame after compensating for yaw:

- forward/backward axis: `noDataAreaMinX` to `noDataAreaMaxX`
- lateral axis: `noDataAreaMinY` to `noDataAreaMaxY`

Cells selected as no-data edges are expanded inward by `noDataBlockSkipNum` iterations before being emitted as four corner points with intensity equal to `vehicleHeight`.

| Parameter | Type | Launch default | Code default | Meaning | Tuning guidance |
| --- | --- | --- | --- | --- | --- |
| `noDataObstacle` | bool | `true` | `false` | Enables generation of artificial obstacle points in poorly observed cells inside the configured rectangular region. | Keep enabled when planners must behave conservatively in blind spots near the robot. |
| `noDataBlockSkipNum` | int | `0` | `0` | Number of inward expansion iterations before a no-data cell is emitted as an obstacle. Larger values keep cells near observed boundaries free. | Increase if the generated no-data wall is too aggressive near valid terrain. |
| `maxElevBelowVeh` | double | `-0.6` | `-0.6` | A cell inside the no-data region is marked as blocked if its estimated elevation is lower than this threshold relative to the vehicle, even if some points exist. | Make more negative to allow deeper dips; make less negative to classify drops more aggressively as blocked. |
| `noDataAreaMinX` | double | `0.3` | `0.3` | Minimum forward coordinate of the no-data obstacle region in the robot frame. | Increase to ignore the area too close to the body. |
| `noDataAreaMaxX` | double | `1.8` | `1.8` | Maximum forward coordinate of the no-data obstacle region in the robot frame. | Increase to extend conservative blocking farther ahead. |
| `noDataAreaMinY` | double | `-0.9` | `-0.9` | Minimum lateral coordinate of the no-data obstacle region in the robot frame. | Adjust to match the robot width and nearby blind zones. |
| `noDataAreaMaxY` | double | `0.9` | `0.9` | Maximum lateral coordinate of the no-data obstacle region in the robot frame. | Adjust symmetrically with `noDataAreaMinY` unless a sensor blind spot is asymmetric. |

## Parameter interactions that matter

### `noDecayDis` affects two subsystems

`noDecayDis` is easy to misread because it is reused in two places:

1. **Decay bypass:** points within this radius are kept even if they are older than `decayTime`.
2. **No-data obstacle activation:** no-data obstacle generation starts only after the robot has moved at least this far following startup or clearing.

If you set `noDecayDis` to `0.0`, near points are no longer protected from decay and no-data obstacle marking can activate immediately after the first sufficient motion update.

### `minRelZ`, `maxRelZ`, and `disRatioZ` define the vertical acceptance band

These parameters do not just filter the initial scan. They also affect which stored points are kept during terrain voxel refresh and which points participate in later terrain evaluation. Overly tight values can make the map appear empty.

### `useSorting`, `quantileZ`, `limitGroundLift`, and `maxGroundLift` must be tuned together

- Low `quantileZ` gives a conservative ground estimate.
- High `quantileZ` can smooth rough terrain but may raise the estimated ground.
- `limitGroundLift` and `maxGroundLift` cap that upward drift.

For most outdoor lidar stacks, `useSorting = true` with a low quantile is the safest starting point.

### `clearDyObs` can remove valid static structure

The dynamic obstacle logic is intentionally simple and local. If enabled with permissive thresholds, it can suppress thin static objects such as chair legs, poles, brush, or close-range robot self-returns.

### `noDataObstacle` is intentionally conservative

When `noDataObstacle` is on, blind spots in the configured body-frame rectangle are converted into artificial obstacles. This is useful for legged navigation, but it can also make narrow passages appear blocked if the sensor coverage is poor.

## Recommended tuning workflow

1. Start with `clearDyObs = false` and `noDataObstacle = false`.
2. Tune the vertical gate: `minRelZ`, `maxRelZ`, `disRatioZ`.
3. Tune the ground estimator: `useSorting`, `quantileZ`, `limitGroundLift`, `maxGroundLift`.
4. Tune persistence and refresh: `scanVoxelSize`, `decayTime`, `voxelPointUpdateThre`, `voxelTimeUpdateThre`.
5. Re-enable `noDataObstacle` and shape the blind-zone rectangle.
6. Enable `clearDyObs` only if moving obstacles are clearly polluting the local terrain map.

## Current launch profile

The launch file [launch/terrain_analysis.launch](launch/terrain_analysis.launch) currently chooses a conservative near-range configuration with:

- strong local detail (`scanVoxelSize = 0.05`)
- short map memory (`decayTime = 2.0`)
- no near-range decay exemption (`noDecayDis = 0.0`)
- quantile-based ground estimation (`useSorting = true`, `quantileZ = 0.25`)
- dynamic-obstacle suppression disabled (`clearDyObs = false`)
- no-data obstacle generation enabled (`noDataObstacle = true`)

If you change the behavior of [src/terrainAnalysis.cpp](src/terrainAnalysis.cpp), this document should be updated together with the launch file.