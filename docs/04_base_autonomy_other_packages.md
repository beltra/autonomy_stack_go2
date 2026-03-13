# Base Autonomy: `vehicle_simulator`, `sensor_scan_generation`, `visualization_tools`, `waypoint_example`

---

## `vehicle_simulator`

**Location:** `src/base_autonomy/vehicle_simulator/`
**When Used:** Simulation only. Provides the simulated robot dynamics and bridges with Unity.

### Overview

A simple **kinematic vehicle simulator** that integrates velocity commands to produce odometry. Also interfaces with the Unity simulation environment for rendering and LiDAR simulation.

### Algorithm

1. **Kinematic Integration (200 Hz):** Integrates `/cmd_vel` commands using a simple Euler step:
   - `vehicleYaw += 0.005 × yawRate`
   - `vehicleX += 0.005 × (cos(yaw) × fwdSpeed - sin(yaw) × leftSpeed)`
   - `vehicleZ = terrainZ + vehicleHeight`

2. **Terrain Adaptation (optional):**
   - **Z adjustment** (`adjustZ`): Smoothly tracks average ground elevation within `terrainRadiusZ`
   - **Inclination adjustment** (`adjustIncl`): Fits a plane to nearby ground points using RANSAC-like iterative least squares to estimate roll/pitch

3. **Unity Bridge:** Publishes `/unity_sim/set_model_state` so the Unity environment tracks the simulated robot position.

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/terrain_map` | `PointCloud2` | Terrain for Z/inclination adjustment |
| `/cmd_vel` | `TwistStamped` | Velocity commands from pathFollower |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Simulated robot pose (200 Hz) |
| `/unity_sim/set_model_state` | `PoseStamped` | Robot pose for Unity rendering |
| TF: `map → sensor` | `TransformStamped` | TF broadcast (200 Hz) |

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `vehicleHeight` | `0.366` | Vehicle height above terrain (m) |
| `sensorOffsetX/Y` | `0.0` | Sensor offset from vehicle center |
| `adjustZ` | `false` | Enable Z adjustment from terrain |
| `adjustIncl` | `false` | Enable inclination adjustment |
| `terrainRadiusZ` | `1.0` | Radius for Z estimation (m) |
| `terrainRadiusIncl` | `2.0` | Radius for inclination estimation (m) |
| `smoothRateZ` | `0.5` | Low-pass filter rate for Z |
| `smoothRateIncl` | `0.5` | Low-pass filter rate for inclination |
| `InclFittingThre` | `0.2` | Outlier threshold for plane fitting (m) |
| `maxIncl` | `30.0` | Maximum allowed inclination (deg) |

### Additional Simulation Nodes

**`sim_image_repub`**: Decompresses images from Unity's compressed topics to raw format:
- `/camera/image/compressed` → `/camera/image/raw`
- `/camera/semantic_image/compressed` → `/camera/semantic_image/raw`
- `/camera/depth/compressed` → `/camera/depth/raw`

---

## `sensor_scan_generation`

**Location:** `src/base_autonomy/sensor_scan_generation/`
**When Used:** Simulation only (commented out in real-robot launch).

### Overview

Transforms registered point clouds from the **map frame** back to the **sensor frame at scan time**, and publishes the scan and corresponding odometry for visualization purposes.

### Algorithm

Uses ROS 2 message filters to synchronize `/state_estimation` and `/registered_scan`. For each synchronized pair:
1. Computes the inverse of the current odometry transform
2. Transforms each point from map frame to sensor frame
3. Publishes the sensor-frame point cloud and the per-scan odometry

### Subscribed Topics (synchronized)
| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Current pose |
| `/registered_scan` | `PointCloud2` | Registered scan in map frame |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/sensor_scan` | `PointCloud2` | Scan in sensor frame |
| `/state_estimation_at_scan` | `Odometry` | Odometry at scan time |
| TF: `map → sensor_at_scan` | `TransformStamped` | Transform at scan time |

### Parameters
No configurable parameters.

---

## `visualization_tools`

**Location:** `src/base_autonomy/visualization_tools/`
**When Used:** Simulation only (commented out in real-robot launch).

### Overview

Provides **exploration metrics** and **map visualization** for simulation environments:
- Accumulates and publishes the overall map (from PLY file)
- Tracks and publishes explored area and explored volume
- Records trajectory with traveling distance
- Logs metrics to files (explored volume, traveling distance, runtime, time duration)

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Vehicle pose |
| `/registered_scan` | `PointCloud2` | Registered scans |
| `/runtime` | `Float32` | Planner runtime metric |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/overall_map` | `PointCloud2` | Pre-loaded environment map (colored) |
| `/explored_areas` | `PointCloud2` | Accumulated explored area |
| `/trajectory` | `PointCloud2` | Vehicle trajectory points |
| `/explored_volume` | `Float32` | Total explored volume (m³) |
| `/traveling_distance` | `Float32` | Total distance traveled (m) |
| `/time_duration` | `Float32` | Elapsed time (s) |

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `mapFile` | — | Path to environment PLY map file |
| `overallMapVoxelSize` | `0.5` | Map display voxel size (m) |
| `exploredAreaVoxelSize` | `0.3` | Explored area voxel size (m) |
| `exploredVolumeVoxelSize` | `0.5` | Explored volume voxel size (m) |
| `transInterval` | `0.2` | Min translation for trajectory update (m) |
| `yawInterval` | `10.0` | Min yaw for trajectory update (deg) |

---

## `waypoint_example`

**Location:** `src/base_autonomy/waypoint_example/`
**When Used:** On-demand via separate launch (`ros2 launch waypoint_example waypoint_example.launch`).

### Overview

A **programmatic waypoint sequencer** that reads waypoints and a navigation boundary from PLY files, then publishes them sequentially. The vehicle navigates to each waypoint in order, with optional wait times at each waypoint.

### Algorithm

1. Reads waypoints from a PLY file at startup
2. Monitors vehicle position via `/state_estimation`
3. When the vehicle reaches within `waypointXYRadius` (and `waypointZBound`) of the current waypoint, starts waiting
4. After `waitTime` seconds, advances to the next waypoint
5. Publishes the current waypoint, speed, and boundary at `frameRate` Hz

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/state_estimation` | `Odometry` | Vehicle pose |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/way_point` | `PointStamped` | Current target waypoint |
| `/speed` | `Float32` | Desired speed |
| `/navigation_boundary` | `PolygonStamped` | Navigation boundary polygon |

### Key Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `waypoint_file_dir` | — | Path to waypoints PLY file |
| `boundary_file_dir` | — | Path to boundary PLY file |
| `waypointXYRadius` | `0.5` | Waypoint reached radius (m) |
| `waypointZBound` | `5.0` | Waypoint reached Z tolerance (m) |
| `waitTime` | `0.0` | Wait time at each waypoint (s) |
| `frameRate` | `5.0` | Publication rate (Hz) |
| `speed` | `1.0` | Desired speed (m/s) |
| `sendSpeed` | `true` | Publish speed messages |
| `sendBoundary` | `true` | Publish boundary messages |
