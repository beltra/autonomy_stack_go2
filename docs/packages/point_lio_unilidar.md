# point_lio_unilidar

## Ruolo
SLAM/LIO principale dello stack (adattamento Point-LIO per Unitree L1).

## Quando viene usato
Su robot reale (e playback bag), opzionalmente anche in simulazione avanzata.

## Topic
### Input (tipico config Go2)
- `/utlidar/transformed_cloud`
- `/utlidar/transformed_imu`

### Output principali
- `/state_estimation` (remap da `/aft_mapped_to_init`)
- `/registered_scan` (remap da `/cloud_registered`)
- debug opzionale: `/Laser_map`, `/cloud_effected`, `/path`, `/planner_normal`

## Dati calcolati
- stato inerziale (posa/velocità/bias)
- mappa locale incrementale e scan deskewed/registrata
- odometria ad alta frequenza robusta a vibrazioni

## Algoritmo
Filtro iterativo LIO con propagazione IMU, correzione LiDAR su mappe locali kd-tree e update stato.

## Parametri principali
- topic sensori: `common.lid_topic`, `common.imu_topic`
- pre-process LiDAR: `preprocess.*` (`lidar_type`, `scan_line`, `blind`)
- stima IMU: `mapping.*` (covarianze, gravità, estrinseci, saturazioni)
- publish/save: `publish.*`, `pcd_save.*`

Config Go2:
[src/slam/point_lio_unilidar/config/utlidar.yaml](../../src/slam/point_lio_unilidar/config/utlidar.yaml)

Launch Go2:
[src/slam/point_lio_unilidar/launch/mapping_utlidar.launch](../../src/slam/point_lio_unilidar/launch/mapping_utlidar.launch)
