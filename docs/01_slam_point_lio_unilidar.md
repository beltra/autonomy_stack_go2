# SLAM Module: `point_lio_unilidar`

**Location:** `src/slam/point_lio_unilidar/`
**When Used:** Real-robot mode only (not used in simulation — the vehicle simulator provides odometry directly).

## Overview

This package implements **Point-LIO** (Point-by-point LiDAR-Inertial Odometry), a tightly-coupled LiDAR-inertial SLAM system based on the [Point-LIO](https://github.com/hku-mars/Point-LIO) algorithm, customized for the Unitree L1 LiDAR. It processes each LiDAR point individually (rather than full sweeps), fusing it with IMU data through an iterated Kalman filter on manifolds (IKFoM) to produce high-frequency, low-latency state estimation.

## Algorithm Description

1. **Preprocessing** (`preprocess.cpp`): Raw LiDAR points from `/utlidar/transformed_cloud` are parsed and organized. The lidar type is set to `5` (Unitree UT lidar), with 18 scan lines and a blind zone of 0.5m.

2. **State Estimation** (`Estimator.cpp`): Uses an Iterated Error-State Kalman Filter on the SE(3) manifold (IKFoM toolkit). The state vector includes position, rotation, velocity, IMU biases, and gravity direction. The filter propagates at IMU frequency and updates point-by-point as LiDAR measurements arrive.

3. **Laser Mapping** (`laserMapping.cpp`): The main node that:
   - Maintains an incremental KD-tree (`ikd-Tree`) for the global map
   - Performs point-to-plane matching against the map for state correction
   - Voxel-grid downsamples the map to control memory
   - Publishes registered scans and odometry

4. **FOV Checker** (`FOV_Checker.cpp`): Manages the field-of-view to determine which map points are visible and need to be maintained.

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/utlidar/transformed_cloud` | `PointCloud2` | LiDAR point cloud (after coordinate transform) |
| `/utlidar/transformed_imu` | `PointCloud2` | IMU data (after coordinate transform + bias correction) |

## Published Topics

| Topic | Type | Remapped To | Description |
|-------|------|-------------|-------------|
| `/cloud_registered` | `PointCloud2` | `/registered_scan` | Point cloud registered in map frame |
| `/aft_mapped_to_init` | `Odometry` | `/state_estimation` | 6-DoF pose estimation |
| `/Laser_map` | `PointCloud2` | — | Full accumulated map (for visualization) |
| `/cur_cloud_body` | `PointCloud2` | — | Current scan in body frame |

> **Important:** The topic remappings `/cloud_registered → /registered_scan` and `/aft_mapped_to_init → /state_estimation` are defined in `mapping_utlidar.launch`.

## Configuration Parameters (`config/utlidar.yaml`)

### Common
| Parameter | Default | Description |
|-----------|---------|-------------|
| `lid_topic` | `/utlidar/transformed_cloud` | LiDAR input topic |
| `imu_topic` | `/utlidar/transformed_imu` | IMU input topic |
| `con_frame` | `false` | Combine multiple LiDAR frames |
| `con_frame_num` | `1` | Number of frames to combine |
| `cut_frame` | `false` | Cut one frame into sub-frames |
| `time_lag_imu_to_lidar` | `0.0` | Time offset between IMU and LiDAR |

### Preprocessing
| Parameter | Default | Description |
|-----------|---------|-------------|
| `lidar_type` | `5` | Unitree UT lidar type |
| `scan_line` | `18` | Number of scan lines |
| `timestamp_unit` | `0` | Time unit: 0=second, 1=ms, 2=μs, 3=ns |
| `blind` | `0.5` | Minimum range (meters) — points closer are discarded |

### Mapping / Filter
| Parameter | Default | Description |
|-----------|---------|-------------|
| `imu_en` | `true` | Enable IMU fusion |
| `imu_time_inte` | `0.01` | IMU time interval (100 Hz) |
| `satu_acc` | `30.0` | Accelerometer saturation threshold |
| `satu_gyro` | `35.0` | Gyroscope saturation threshold |
| `acc_norm` | `9.81` | Expected gravity norm (m/s²) |
| `lidar_meas_cov` | `0.01` | LiDAR measurement covariance |
| `acc_cov_output` | `500.0` | Accelerometer output covariance |
| `gyr_cov_output` | `1000.0` | Gyroscope output covariance |
| `b_acc_cov` | `0.0001` | Accelerometer bias covariance |
| `b_gyr_cov` | `0.0001` | Gyroscope bias covariance |
| `plane_thr` | `0.1` | Plane fitting threshold (smaller = flatter) |
| `fov_degree` | `180.0` | Sensor field of view (degrees) |
| `det_range` | `100.0` | Maximum detection range (meters) |
| `gravity_align` | `false` | Align z-axis with gravity direction |
| `extrinsic_T` | `[0.008, 0.015, -0.007]` | IMU-to-LiDAR translation |
| `extrinsic_R` | Identity | IMU-to-LiDAR rotation matrix |

### Launch Parameters (from `mapping_utlidar.launch`)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_imu_as_input` | `false` | Use IMU as process model input (overridden in launch) |
| `prop_at_freq_of_imu` | `true` | Propagate state at IMU frequency |
| `check_satu` | `true` | Check for IMU saturation |
| `point_filter_num` | `1` | Point decimation factor (1 = keep all) |
| `space_down_sample` | `true` | Spatial downsampling |
| `filter_size_surf` | `0.1` | Surface voxel filter size (m) |
| `filter_size_map` | `0.1` | Map voxel filter size (m) |
| `cube_side_length` | `1000.0` | Map cube side length (m) |

## Dependencies
- Launches `transform_sensors` (transform_everything node) to transform raw sensor data before ingestion.
- Requires `imu_calib_data.yaml` (produced by `calibrate_imu`) in the workspace root.

## Supported LiDAR Configurations
The package includes configs for multiple LiDARs. Only `utlidar.yaml` is used on Go2:
- `utlidar.yaml` — Unitree L1 LiDAR (Go2 built-in)
- `unilidar.yaml` — Unitree L2 LiDAR
- `avia.yaml` — Livox Avia
- `horizon.yaml` — Livox Horizon
- `velody16.yaml` — Velodyne VLP-16
- `ouster64.yaml` — Ouster OS1-64
