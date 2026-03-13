# Base Autonomy: `terrain_analysis` & `terrain_analysis_ext`

**Location:** `src/base_autonomy/terrain_analysis/` and `src/base_autonomy/terrain_analysis_ext/`

---

## `terrain_analysis`

**When Used:** All modes (simulation and real robot).

### Overview

Performs **near-field terrain traversability analysis**. It accumulates registered LiDAR scans into a rolling voxel grid, estimates ground elevation for each planar cell, and computes the height of each point above the estimated ground. The output is a terrain map where each point's `intensity` field encodes its height above ground — enabling the local planner to distinguish between traversable ground and obstacles.

### Algorithm Description

1. **Rolling Voxel Grid:** Maintains a 21×21 grid of 1.0m terrain voxels. Each voxel stores accumulated point clouds. The grid shifts as the vehicle moves to keep the vehicle centered.

2. **Scan Accumulation:** Incoming registered scans are distributed into the appropriate voxel based on XY position. Points are time-stamped and filtered by relative Z bounds.

3. **Voxel Maintenance:** Each voxel is periodically downsampled (when it exceeds `voxelPointUpdateThre` points or `voxelTimeUpdateThre` seconds). Old points (beyond `decayTime`) are removed, except those within `noDecayDis` of the vehicle.

4. **Ground Estimation:** For a 51×51 grid of 0.2m planar voxels (within ±5 terrain voxels of the vehicle), ground elevation is estimated by:
   - **Sorting mode** (`useSorting=true`, default): Takes the `quantileZ` percentile (default 25th) of Z values as ground elevation.
   - **Min mode** (`useSorting=false`): Takes the minimum Z value.
   - `limitGroundLift`: Prevents ground estimate from jumping up more than `maxGroundLift`.

5. **Dynamic Obstacle Filtering** (`clearDyObs=false` by default): Can detect and remove dynamic obstacles based on point density and viewing angle in the sensor frame.

6. **No-Data Obstacle** (`noDataObstacle=false` by default): Fills areas with no LiDAR data (in the forward direction) as obstacles, with edge erosion (`noDataBlockSkipNum`).

7. **Output:** Points are published with `intensity = height_above_ground`. Ground points have low intensity; obstacles have high intensity (>0.3m for the local planner's default threshold).

8. **Map Clearing:** Pressing joystick button 5 or publishing to `/map_clearing` clears accumulated data within `clearingDis`.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Vehicle pose |
| `/registered_scan` | `PointCloud2` | Registered LiDAR scan |
| `/joy` | `Joy` | Joystick (button 5 = clear terrain map) |
| `/map_clearing` | `Float32` | Clear terrain map within given radius |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/terrain_map` | `PointCloud2` | Terrain map with elevation-coded intensity |

### Configuration Parameters (from `terrain_analysis.launch`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scanVoxelSize` | `0.05` | Scan downsampling voxel size (m) |
| `decayTime` | `2.0` | Point decay time (s) |
| `noDecayDis` | `4.0` | No-decay radius around vehicle (m) |
| `clearingDis` | `8.0` | Clearing radius when triggered (m) |
| `useSorting` | `true` | Use sorted quantile for ground estimation |
| `quantileZ` | `0.25` | Quantile for ground elevation |
| `considerDrop` | `false` | Consider downward drops as obstacles |
| `limitGroundLift` | `false` | Limit ground elevation lift |
| `maxGroundLift` | `0.15` | Maximum ground lift (m) |
| `clearDyObs` | `false` | Clear dynamic obstacles |
| `minDyObsDis` | `0.3` | Minimum dynamic obstacle distance (m) |
| `minDyObsPointNum` | `1` | Minimum points for dynamic obstacle detection |
| `noDataObstacle` | `false` | Treat no-data areas as obstacles |
| `noDataBlockSkipNum` | `0` | Edge erosion iterations for no-data areas |
| `minBlockPointNum` | `10` | Minimum points in a voxel to be considered valid |
| `vehicleHeight` | `1.5` | Maximum obstacle height to report (m) |
| `voxelPointUpdateThre` | `100` | Points before forced voxel update |
| `voxelTimeUpdateThre` | `2.0` | Time before forced voxel update (s) |
| `minRelZ` / `maxRelZ` | `-1.5` / `0.2` | Relative Z bounds for point inclusion (m) |
| `disRatioZ` | `0.2` | Z bound expansion ratio with distance |

---

## `terrain_analysis_ext`

**When Used:** Simulation (always) and real-robot route planner mode. Disabled in base real-robot mode.

### Overview

Extends the terrain map to a **larger area** (80m × 80m), providing far-field traversability information needed by the route planner. It uses a coarser resolution and adds **terrain connectivity checking** to filter out ceiling structures that would otherwise appear as obstacles.

### Algorithm Description

1. **Extended Rolling Voxel Grid:** 41×41 grid of 2.0m terrain voxels (vs. 21×21 × 1.0m for `terrain_analysis`).

2. **Larger Coverage:** Processes points from a 101×101 grid of 0.4m planar voxels (±10 terrain voxels = ±20m from vehicle).

3. **Terrain Connectivity Check** (`checkTerrainConn=true`):
   - BFS flood-fill from the vehicle position outward
   - Connected voxels must have elevation difference < `terrainConnThre` (0.5m)
   - Voxels with elevation difference > `ceilingFilteringThre` (2.0m) are marked as ceiling
   - Only connected terrain is included in the output

4. **Far-Field Map Composition:**
   - Points beyond `localTerrainMapRadius` (4.0m) come from the extended analysis
   - Points within `localTerrainMapRadius` come from the local `terrain_map` topic
   - This creates a seamless composite map

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Vehicle pose |
| `/registered_scan` | `PointCloud2` | Registered LiDAR scan |
| `/terrain_map` | `PointCloud2` | Local terrain map (from `terrain_analysis`) |
| `/joy` | `Joy` | Joystick (button 5 = clear) |
| `/cloud_clearing` | `Float32` | Clear extended terrain map |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/terrain_map_ext` | `PointCloud2` | Extended terrain map with elevation-coded intensity |

### Configuration Parameters (from `terrain_analysis_ext.launch`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scanVoxelSize` | `0.1` | Scan downsampling voxel size (m) |
| `decayTime` | `10.0` | Point decay time (s) |
| `noDecayDis` | `0.0` | No-decay radius — all points decay |
| `clearingDis` | `30.0` | Clearing radius (m) |
| `useSorting` | `false` | Use minimum Z for ground estimation |
| `quantileZ` | `0.25` | Quantile (unused when sorting=false) |
| `vehicleHeight` | `1.5` | Max obstacle height (m) |
| `lowerBoundZ` | `-1.5` | Lower Z bound relative to vehicle (m) |
| `upperBoundZ` | `1.0` | Upper Z bound relative to vehicle (m) |
| `disRatioZ` | `0.1` | Z bound expansion ratio |
| `checkTerrainConn` | `true` | Enable terrain connectivity checking |
| `terrainUnderVehicle` | `-0.75` | Assumed ground Z relative to vehicle (m) |
| `terrainConnThre` | `0.5` | Max elevation diff for connectivity (m) |
| `ceilingFilteringThre` | `2.0` | Elevation diff to mark as ceiling (m) |
| `localTerrainMapRadius` | `4.0` | Radius where local terrain_map is used (m) |
| `voxelPointUpdateThre` | `100` | Points before forced voxel update |
| `voxelTimeUpdateThre` | `2.0` | Time before forced voxel update (s) |
