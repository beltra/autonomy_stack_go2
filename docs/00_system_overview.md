# Autonomy Stack for Unitree Go2 — System Overview

## 1. Introduction

This autonomy stack provides full autonomous navigation capability for the **Unitree Go2 EDU** quadruped robot. Given a goal point, the system navigates Go2 autonomously while building a map in real-time. It can also operate in a smart-joystick mode where the operator guides the navigation while the system handles collision avoidance.

**Sensors Used:** Built-in L1 LiDAR and its internal IMU (no external sensors required).

## 2. Architecture Overview

The stack is organized into **four package groups** containing **24 ROS 2 packages**:

```
src/
├── slam/                   # SLAM module (1 package)
│   └── point_lio_unilidar  # Point-LIO based LiDAR-inertial odometry
│
├── base_autonomy/          # Core navigation (7 packages)
│   ├── local_planner       # Local path planning + path following
│   ├── terrain_analysis    # Near-field terrain traversability
│   ├── terrain_analysis_ext # Far-field terrain map extension
│   ├── vehicle_simulator   # Kinematic simulator + Unity bridge
│   ├── sensor_scan_generation # Point cloud frame transformation
│   ├── visualization_tools # Map/trajectory/metrics visualization
│   └── waypoint_example    # Programmatic waypoint sequencer
│
├── route_planner/          # Global route planning (4 packages)
│   ├── far_planner         # FAR Planner (visibility graph navigation)
│   ├── graph_decoder       # Visibility graph message decoder
│   ├── boundary_handler    # Boundary polygon → V-graph extraction
│   └── visibility_graph_msg # Custom ROS message definitions
│
└── utilities/              # Support & hardware interface (12 packages)
    ├── transform_sensors   # IMU/LiDAR coordinate transform + calibration
    ├── calibrate_imu       # IMU bias calibration tool
    ├── go2_h264_repub      # H.264 camera stream → ROS images
    ├── go2_sport_api       # Unitree Go2 sport-mode API bridge
    ├── unitree_api         # Unitree API message definitions
    ├── unitree_go          # Unitree Go message definitions
    ├── ROS-TCP-Endpoint    # Unity ↔ ROS TCP bridge (simulation)
    ├── far_web_gui         # Web GUI for FAR Planner
    ├── goalpoint_rviz_plugin  # RVIZ goal-point tool
    ├── waypoint_rviz_plugin   # RVIZ waypoint tool
    ├── teleop_rviz_plugin     # RVIZ teleoperation panel
    └── teleop_rviz_plugin_plus # Enhanced RVIZ teleop panel
```

## 3. Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          SENSOR INPUTS                                  │
│  /utlidar/cloud (PointCloud2)     /utlidar/imu (Imu)     /joy (Joy)   │
└──────────┬──────────────────────────┬─────────────────────┬─────────────┘
           │                          │                     │
           ▼                          ▼                     │
   ┌───────────────────────────────────────────┐            │
   │       transform_sensors                    │            │
   │  /utlidar/transformed_cloud               │            │
   │  /utlidar/transformed_imu                 │            │
   └──────────┬──────────────────┬─────────────┘            │
              │                  │                          │
              ▼                  ▼                          │
   ┌──────────────────────────────────────────┐             │
   │     point_lio_unilidar (SLAM)            │             │
   │  OUT: /registered_scan (PointCloud2)     │             │
   │  OUT: /state_estimation (Odometry)       │             │
   └──────────┬──────────────────┬────────────┘             │
              │                  │                          │
      ┌───────┴──────┐   ┌──────┴───────┐                  │
      ▼              ▼   ▼              ▼                  │
┌──────────┐  ┌───────────────┐  ┌──────────────────┐      │
│terrain_  │  │terrain_       │  │sensor_scan_      │      │
│analysis  │  │analysis_ext   │  │generation        │      │
│          │  │               │  │                  │      │
│/terrain_ │  │/terrain_      │  │/sensor_scan      │      │
│map       │  │map_ext        │  │/state_estimation_│      │
│          │  │               │  │at_scan           │      │
└────┬─────┘  └───────┬───────┘  └──────────────────┘      │
     │                │                                     │
     ▼                ▼                                     │
┌─────────────────────────────────────┐                     │
│        local_planner                │◄────────────────────┘
│  localPlanner  → /path             │
│  pathFollower  → /cmd_vel           │
│                → /api/sport/request │
└─────────────────────────────────────┘
              ▲
              │ (in route mode)
┌─────────────┴──────────────────┐
│        far_planner             │
│  → /way_point (to local_planner)
│  → /navigation_boundary        │
│  Uses: /terrain_map_ext,       │
│        /terrain_map,           │
│        /state_estimation       │
└────────────────────────────────┘
```

## 4. Operating Modes

### 4.1 Smart Joystick Mode (Default)
The vehicle follows joystick commands while avoiding collisions. The `localPlanner` evaluates a pre-computed set of 343 candidate paths, selects the best collision-free path closest to the joystick direction, and the `pathFollower` tracks it.

### 4.2 Waypoint Mode
A goal waypoint is set (via RVIZ or programmatically). The `localPlanner` steers toward the waypoint while avoiding obstacles. The `pathFollower` tracks the computed path and sends velocity commands.

### 4.3 Manual Mode
Direct joystick control with no collision avoidance. The `pathFollower` directly maps joystick axes to velocity commands.

### 4.4 Route Planner Mode
The `far_planner` builds a visibility graph of the environment and plans long-range routes. It sends intermediate waypoints to `local_planner`, which handles local obstacle avoidance.

## 5. Launch Configurations

| Script | Launches | Use Case |
|--------|----------|----------|
| `sim.sh` | `system_simulation.launch` | Simulation with Unity, base autonomy only |
| `sim_route.sh` | `system_simulation_with_route_planner.launch` | Simulation with Unity + route planner |
| `real.sh` | `system_real_robot.launch` | Real Go2, base autonomy only |
| `real_route.sh` | `system_real_robot_with_route_planner.launch` | Real Go2 + route planner |

### Simulation Launch Includes
- `joy_node` (joystick driver)
- `local_planner` (localPlanner + pathFollower)
- `terrain_analysis`
- `terrain_analysis_ext`
- `vehicle_simulator` (kinematic sim)
- `sensor_scan_generation`
- `visualization_tools`
- `ros_tcp_endpoint` (Unity bridge)
- `sim_image_repub`
- `rviz2`

### Real Robot Launch Includes
- `joy_node` (joystick driver)
- `point_lio_unilidar` (SLAM via `mapping_utlidar.launch`)
- `local_planner`
- `terrain_analysis`
- Static TF publishers (`map→camera_init`, `aft_mapped→sensor`)
- `rviz2`

### Route Planner Additions
Both sim and real route variants additionally include:
- `far_planner.launch` (which includes `graph_decoder`)
- `terrain_analysis_ext` (enabled on real robot for route mode)

## 6. TF Tree

```
map
├── camera_init (identity, real robot only)
├── sensor (from SLAM or simulator)
│   ├── vehicle (static: offset by -sensorOffsetX, -sensorOffsetY)
│   └── camera (static: rotated -π/2 about X and Z, offset cameraOffsetZ)
├── sensor_at_scan (from sensor_scan_generation)
└── aft_mapped → sensor (identity, real robot only)
```

## 7. Key ROS Topics

| Topic | Type | Producer | Consumer(s) |
|-------|------|----------|-------------|
| `/state_estimation` | `Odometry` | SLAM / vehicle_simulator | All navigation nodes |
| `/registered_scan` | `PointCloud2` | SLAM / vehicle_simulator | terrain_analysis, local_planner, far_planner |
| `/terrain_map` | `PointCloud2` | terrain_analysis | local_planner, terrain_analysis_ext, far_planner |
| `/terrain_map_ext` | `PointCloud2` | terrain_analysis_ext | far_planner |
| `/path` | `Path` | localPlanner | pathFollower |
| `/cmd_vel` | `TwistStamped` | pathFollower | vehicle_simulator (sim) |
| `/api/sport/request` | `Request` | pathFollower | Go2 sport API (real) |
| `/way_point` | `PointStamped` | RVIZ / far_planner | localPlanner |
| `/joy` | `Joy` | joy_node | localPlanner, pathFollower, terrain_analysis |
| `/speed` | `Float32` | waypoint_example / far_planner | localPlanner, pathFollower |
| `/navigation_boundary` | `PolygonStamped` | waypoint_example / far_planner | localPlanner |
| `/free_paths` | `PointCloud2` | localPlanner | RVIZ (visualization) |
| `/sensor_scan` | `PointCloud2` | sensor_scan_generation | RVIZ |
| `/map_clearing` | `Float32` | external | terrain_analysis |
| `/stop` | `Int8` | external | pathFollower |
