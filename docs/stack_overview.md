# Autonomy Stack Go2 — panoramica completa

## 1) Obiettivo del sistema
Lo stack implementa navigazione autonoma per Unitree Go2 (simulazione e robot reale) con questa pipeline:

1. acquisizione sensori (`/utlidar/*`, joystick, camera)
2. trasformazioni e calibrazioni IMU/LiDAR
3. SLAM LIO (`point_lio_unilidar`) → posa e nuvola registrata
4. mappa locale traversabile (`terrain_analysis`, opzionalmente `terrain_analysis_ext`)
5. pianificazione locale (`local_planner`) + inseguimento traiettoria (`pathFollower`)
6. pianificazione globale route-based (`far_planner`) con visibility graph (opzionale)
7. interfacce operatore (RViz plugin, Web GUI)

## 2) Modalità operative e quando i pacchetti sono usati

- **Simulazione base** (`system_simulation.launch`):
  - `vehicle_simulator`, `terrain_analysis`, `local_planner`, `sensor_scan_generation`, `visualization_tools`, RViz plugin.
- **Simulazione con route planner** (`system_simulation_with_route_planner.launch`):
  - come sopra + `far_planner`, `graph_decoder`.
- **Robot reale base** (`system_real_robot.launch`):
  - `transform_sensors`, `point_lio_unilidar`, `terrain_analysis`, `local_planner`.
- **Robot reale con route planner** (`system_real_robot_with_route_planner.launch`):
  - come sopra + `terrain_analysis_ext`, `far_planner`, `graph_decoder`.
- **GUI web remota** (`far_web_gui.launch.py`):
  - `rosbridge_websocket`, `rosapi`, `graph_file_bridge`.

## 3) Flussi dati principali (topic)

### 3.1 Sensori e pre-processing
- `/utlidar/imu`, `/utlidar/cloud` (driver robot)
- `transform_sensors` pubblica:
  - `/utlidar/transformed_imu`
  - `/utlidar/transformed_raw_imu`
  - `/utlidar/transformed_cloud`

### 3.2 SLAM
- `point_lio_unilidar` sottoscrive dati trasformati e pubblica:
  - `/state_estimation` (odometria mondo)
  - `/registered_scan` (nuvola registrata nel frame mappa)

### 3.3 Mapping traversabilità
- `terrain_analysis`: `/state_estimation` + `/registered_scan` → `/terrain_map`
- `terrain_analysis_ext`: `/terrain_map` + input recenti → `/terrain_map_ext`

### 3.4 Pianificazione locale
- `localPlanner` usa `/terrain_map` o `/registered_scan` (+ comandi utente) e pubblica:
  - `/path` (traiettoria locale)
  - `/free_paths` (debug)
- `pathFollower` usa `/path` + stato + joy e pubblica:
  - `/cmd_vel`
  - `/api/sport/request` (su robot reale)

### 3.5 Pianificazione globale (route planner)
- `far_planner` usa:
  - `/state_estimation`, `/terrain_map_ext`, `/terrain_map`, `/registered_scan`, `/goal_point`
- `far_planner` pubblica:
  - `/way_point`, `/navigation_boundary`
  - `/robot_vgraph`
  - stato: `/far_reach_goal_status`
  - visualizzazione: `/viz_graph_topic`, `/viz_path_topic`, `/viz_node_topic`, ecc.
- `graph_decoder` converte `/robot_vgraph` e gestisce I/O file `.vgh`.

### 3.6 UI e controllo
- Plugin RViz e Web GUI pubblicano comandi su:
  - `/joy`, `/goal_point`, `/way_point`
  - `/planning_attemptable`, `/update_visibility_graph`, `/reset_visibility_graph`

## 4) Algoritmi chiave

- **SLAM**: LIO (fusion IMU + scan matching con mappa incrementale voxel/kd-tree).
- **Terrain mapping**: accumulo voxel locale, stima piano/ground locale, elevazione relativa ostacoli in `intensity`.
- **Local planning**: selezione traiettoria da libreria di path precomputati + filtro collisione + costo direzione.
- **Path following**: look-ahead + controllo yaw-rate + limiti accelerazione/sicurezza + mode switching.
- **FAR planner**:
  - estrazione contorni ostacolo da immagine occupancy
  - costruzione visibility graph dinamico
  - ricerca path su grafo con modalità free-space vs attemptable
  - pubblicazione waypoint locale al planner di basso livello.

## 5) Parametri “globalmente sensibili”

I parametri che cambiano di più il comportamento del sistema:

- `point_lio_unilidar`: topic input IMU/LiDAR, covarianze IMU, filtri voxel.
- `terrain_analysis`: `scanVoxelSize`, `decayTime`, `quantileZ`, `noDataObstacle`, limiti Z.
- `local_planner`: `maxSpeed`, `adjacentRange`, `obstacleHeightThre`, scaling path.
- `pathFollower`: `lookAheadDis`, `maxYawRate`, `maxAccel`, soglie stop/slow.
- `far_planner`: `main_run_freq`, `voxel_dim`, `sensor_range`, `terrain_range`, `is_attempt_autoswitch`, `suppress_nonessential_topics`.

## 6) Pacchetti e documenti dedicati

La documentazione per singolo pacchetto è in:

- [docs/packages/local_planner.md](packages/local_planner.md)
- [docs/packages/sensor_scan_generation.md](packages/sensor_scan_generation.md)
- [docs/packages/terrain_analysis.md](packages/terrain_analysis.md)
- [docs/packages/terrain_analysis_ext.md](packages/terrain_analysis_ext.md)
- [docs/packages/vehicle_simulator.md](packages/vehicle_simulator.md)
- [docs/packages/visualization_tools.md](packages/visualization_tools.md)
- [docs/packages/waypoint_example.md](packages/waypoint_example.md)
- [docs/packages/boundary_handler.md](packages/boundary_handler.md)
- [docs/packages/far_planner.md](packages/far_planner.md)
- [docs/packages/graph_decoder.md](packages/graph_decoder.md)
- [docs/packages/visibility_graph_msg.md](packages/visibility_graph_msg.md)
- [docs/packages/point_lio_unilidar.md](packages/point_lio_unilidar.md)
- [docs/packages/calibrate_imu.md](packages/calibrate_imu.md)
- [docs/packages/far_web_gui.md](packages/far_web_gui.md)
- [docs/packages/ros_tcp_endpoint.md](packages/ros_tcp_endpoint.md)
- [docs/packages/goalpoint_rviz_plugin.md](packages/goalpoint_rviz_plugin.md)
- [docs/packages/teleop_rviz_plugin.md](packages/teleop_rviz_plugin.md)
- [docs/packages/teleop_rviz_plugin_plus.md](packages/teleop_rviz_plugin_plus.md)
- [docs/packages/transform_sensors.md](packages/transform_sensors.md)
- [docs/packages/go2_h264_repub.md](packages/go2_h264_repub.md)
- [docs/packages/go2_sport_api.md](packages/go2_sport_api.md)
- [docs/packages/unitree_api.md](packages/unitree_api.md)
- [docs/packages/unitree_go.md](packages/unitree_go.md)
- [docs/packages/waypoint_rviz_plugin.md](packages/waypoint_rviz_plugin.md)
