# Utilities Packages

---

## `transform_sensors`

**Location:** `src/utilities/transform_sensors/`
**When Used:** Real-robot mode (launched by `mapping_utlidar.launch`).

### Overview

A Python ROS 2 node that transforms the raw Unitree L1 LiDAR and IMU data from the sensor's native frame into a standard body frame suitable for SLAM. Applies **IMU calibration corrections** (bias subtraction and cross-axis coupling compensation).

### Algorithm

**Point Cloud Transform:**
1. Applies a rotation to convert from the LiDAR's native frame (tilted 15.1° from vertical) to a z-up body frame
2. Applies a Z offset (`cam_offset = 0.046825m`) to account for the camera-to-LiDAR mounting
3. Filters out self-reflection points from the robot's body using a bounding box filter:
   - X: [-0.7, -0.1], Y: [-0.3, 0.3], Z: [-0.646, -0.047]
4. Adjusts timestamps by computing an offset between the node's clock and the sensor's timestamps

**IMU Transform:**
1. Coordinate conversion: negates Y and Z angular velocity and acceleration (sensor frame convention)
2. Applies 15.1° rotation about Y axis to align with z-up frame
3. Subtracts calibrated biases (`ang_bias_x/y/z`, `acc_bias_x/y/z`)
4. Applies cross-axis coupling compensation: `x += ang_z2x_proj × z`, `y += ang_z2y_proj × z`
5. Publishes two IMU outputs:
   - **`/utlidar/transformed_raw_imu`**: Full corrected IMU with orientation
   - **`/utlidar/transformed_imu`**: Angular-velocity-only IMU (orientation and acceleration zeroed) — this is what SLAM uses

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/utlidar/imu` | `Imu` | Raw IMU from L1 LiDAR |
| `/utlidar/cloud` | `PointCloud2` | Raw LiDAR point cloud |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/utlidar/transformed_imu` | `Imu` | Corrected IMU (angular velocity only) |
| `/utlidar/transformed_raw_imu` | `Imu` | Corrected IMU (full data) |
| `/utlidar/transformed_cloud` | `PointCloud2` | Transformed point cloud in body frame |

### Calibration File
Reads `imu_calib_data.yaml` from the workspace root (or parent directory). Contains:
- `acc_bias_x/y/z`: Accelerometer biases
- `ang_bias_x/y/z`: Gyroscope biases
- `ang_z2x_proj`: Cross-axis coupling Z→X
- `ang_z2y_proj`: Cross-axis coupling Z→Y

---

## `calibrate_imu`

**Location:** `src/utilities/calibrate_imu/`
**When Used:** Run once per Go2 before first use: `ros2 run calibrate_imu calibrate_imu`

### Overview

Automated IMU calibration tool. Commands the Go2 through a calibration sequence and estimates IMU biases and cross-axis coupling parameters.

### Calibration Procedure (State Machine)

| Time (s) | State | Action | Data Collection |
|----------|-------|--------|-----------------|
| 0–2 | Adjusting | Robot steps in place | — |
| 2–5 | Settling | Robot stands still | — |
| 5–15 | Static | Robot stands still | Collecting static IMU data (bias estimation) |
| 15–35 | Spinning | Robot spins at 1.396 rad/s (80°/s) | Collecting rotation data (cross-axis) |
| 35–37 | Writing | Stop, compute, save | — |

### Bias Estimation Algorithm
1. **Static phase:** Average all IMU readings to get `acc_bias_x/y/z` and `ang_bias_x/y/z`. Subtract 9.81 from `acc_bias_z` (gravity).
2. **Rotation phase:** Average angular velocities during spinning. After subtracting bias, compute cross-axis coupling ratios:
   - `ang_z2x_proj = -ang_rot_x / ang_rot_z`
   - `ang_z2y_proj = -ang_rot_y / ang_rot_z`

### Output
Saves `imu_calib_data.yaml` to the workspace root.

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/utlidar/imu` | `Imu` | Raw IMU data |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/api/sport/request` | `Request` | Commands to Go2 (move, stop) |
| `/cmd_vel` | `TwistStamped` | Debug velocity output |

---

## `go2_h264_repub`

**Location:** `src/utilities/unitree_pkgs/go2_h264_repub/`
**When Used:** Manually launched for camera access: `ros2 run go2_h264_repub go2_h264_repub`

### Overview

Captures the H.264 video stream multicast from Go2's front camera using GStreamer, decodes it, and publishes as standard ROS image messages.

### Pipeline
```
UDP multicast (230.1.1.1:1720) → RTP H.264 depay → H.264 decode → BGR conversion → ROS Image
```

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image/raw` | `Image` | Decoded camera image (1280×720, BGR) |

### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `multicast_iface` | `eth0` | Network interface for multicast reception |

---

## `go2_sport_api`

**Location:** `src/utilities/unitree_pkgs/go2_sport_api/`
**When Used:** Provides the API bridge library used by `pathFollower` and `calibrate_imu`.

### Overview

Contains the **SportClient** class (`ros2_sport_client.h/cpp`) that constructs Unitree Sport API request messages. Supports commands:

- `Move(req, vx, vy, vyaw)` — Set velocity (API 1008)
- `StopMove(req)` — Stop movement (API 1002)
- `RecoveryStand(req)` — Recover from fallen state (API 1006)
- `StandUp(req)` — Stand up (API 1004)
- `StandDown(req)` — Sit down (API 1005)
- `ClassicWalk(req, enable)` — Toggle ClassicWalk mode for better terrain adaptability

Also includes `motor_crc.h/cpp` for CRC computation for motor commands.

### Standalone Node: `vel_ctrl_repub`

A standalone velocity controller that reads joystick input and sends directly to the Go2 Sport API (bypassing the autonomy stack). Used for manual joystick control without the autonomy system.

#### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `Joy` | Joystick input |

#### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/api/sport/request` | `Request` | Go2 Sport API commands |

---

## `unitree_api` and `unitree_go`

**Location:** `src/utilities/unitree_pkgs/unitree_api/` and `src/utilities/unitree_pkgs/unitree_go/`

### Overview

**Message definition packages** containing Unitree-specific ROS 2 message and service types:

- **`unitree_api`**: Defines `Request.msg` (API request message with header identity and parameter string)
- **`unitree_go`**: Defines various Go2-specific messages including `SportModeState.msg`, `LowState.msg`, `IMUState.msg`, etc.

These packages produce no nodes — they provide message types used by other packages.

---

## `ROS-TCP-Endpoint`

**Location:** `src/utilities/ROS-TCP-Endpoint/`
**When Used:** Simulation only. Bridges ROS 2 and Unity.

### Overview

[Unity Robotics' ROS-TCP-Endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) provides a TCP server that connects Unity's simulation environment to ROS 2. Unity sends simulated sensor data (LiDAR, camera, etc.) and receives robot state updates.

### Configuration (from system launch)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROS_IP` | `0.0.0.0` | TCP server bind address |
| `ROS_TCP_PORT` | `10000` | TCP server port |

---

## `far_web_gui`

**Location:** `src/utilities/far_web_gui/`
**When Used:** Optional, launched via `*_web_gui.sh` scripts.

### Overview

A web-based GUI for the FAR Planner that provides a browser-based interface for setting goal points and monitoring the planner state. Includes a Python bridge (`graph_file_bridge.py`) for loading/saving graph files.

---

## RVIZ Plugins

### `goalpoint_rviz_plugin`
**Location:** `src/utilities/goalpoint_rviz_plugin/`

RVIZ tool for setting goal points. Clicking in RVIZ publishes a `PointStamped` message to `/goal_point`, which the `far_planner` uses as the navigation target.

### `waypoint_rviz_plugin`
**Location:** `src/utilities/waypoint_rviz_plugin/`

RVIZ tool for setting local waypoints. Clicking publishes to `/way_point`, which the `localPlanner` uses as the immediate goal.

### `teleop_rviz_plugin`
**Location:** `src/utilities/teleop_rviz_plugin/`

RVIZ panel with a virtual joystick pad. Publishes `Joy` messages to `/joy` for controlling the robot's speed and direction through the RVIZ interface.

### `teleop_rviz_plugin_plus`
**Location:** `src/utilities/teleop_rviz_plugin_plus/`

Enhanced teleoperation panel with additional controls including:
- Speed/direction joystick
- Navigation mode buttons (autonomy, manual, waypoint)
- Resume navigation
- Clear terrain map
- Reset visibility graph
