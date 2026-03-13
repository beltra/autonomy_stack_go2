# transform_sensors

## Ruolo
Trasforma e ripulisce stream LiDAR/IMU grezzi in frame `body`, applicando calibrazione IMU.

## Quando viene usato
Nel launch SLAM reale (`mapping_utlidar.launch`).

## Topic
- Subscribe: `/utlidar/imu`, `/utlidar/cloud`
- Publish:
  - `/utlidar/transformed_raw_imu`
  - `/utlidar/transformed_imu`
  - `/utlidar/transformed_cloud`

## Dati calcolati
- compensazione timestamp su clock ROS locale
- trasformazione frame e offset camera
- filtro punti in box robot (self points)
- applicazione bias/calibrazione letta da `imu_calib_data.yaml`

## Algoritmo
Pipeline Python (`rclpy` + `transforms3d`):
1. converte IMU/LiDAR nel frame body,
2. corregge bias,
3. pubblica IMU “raw corrected” e IMU “flattened” per LIO,
4. filtra nuvola da punti del robot.

## Parametri
Nessun parametro ROS pubblico rilevante nel nodo corrente; usa file `imu_calib_data.yaml` trovato nella root workspace o parent.

Sorgente:
[src/utilities/transform_sensors/transform_sensors/transform_everything.py](../../src/utilities/transform_sensors/transform_sensors/transform_everything.py).
