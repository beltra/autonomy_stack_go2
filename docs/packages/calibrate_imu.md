# calibrate_imu

## Ruolo
Calibra bias IMU del LiDAR L1 e salva `imu_calib_data.yaml` usato da `transform_sensors`.

## Quando viene usato
Una tantum per ogni robot (o quando si vuole ricalibrare).

## Topic
- Subscribe: `/utlidar/imu`
- Publish: `/cmd_vel`, `/api/sport/request`

## Dati calcolati
- bias accelerometri: `acc_bias_*`
- bias giroscopi: `ang_bias_*`
- compensazione cross-axis gyro: `ang_z2x_proj`, `ang_z2y_proj`

## Algoritmo
State machine temporale:
1. fase statica (stima bias)
2. rotazione attorno a Z (stima proiezioni coupling)
3. scrittura YAML.

## Parametri
Nessun parametro ROS significativo; sequenza temporale hard-coded nel sorgente.

Sorgente:
[src/utilities/calibrate_imu/src/calibrate_imu.cpp](../../src/utilities/calibrate_imu/src/calibrate_imu.cpp).
