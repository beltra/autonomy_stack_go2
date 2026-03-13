# vehicle_simulator

## Ruolo
Simula dinamica base del robot e ripubblica immagini compresse Unity come immagini raw.

## Quando viene usato
Solo in simulazione (`system_simulation*.launch`).

## Nodi
- `vehicleSimulator`
- `sim_image_repub`

## Topic
### vehicleSimulator
- Subscribe: `/cmd_vel`, `/terrain_map`
- Publish: `/state_estimation`, `/unity_sim/set_model_state`

### sim_image_repub
- Subscribe: `/camera/image/compressed`, `/camera/semantic_image/compressed`, `/camera/depth/compressed`
- Publish: `/camera/image/raw`, `/camera/semantic_image/raw`, `/camera/depth/raw`

## Dati calcolati
- integrazione pose/velocità a 200 Hz
- adattamento opzionale quota e inclinazione al terreno (`adjustZ`, `adjustIncl`)
- conversione frame immagini compressi -> `sensor_msgs/Image`

## Parametri principali
- stato iniziale: `vehicleX`, `vehicleY`, `vehicleZ`, `vehicleYaw`, `terrainZ`
- modello terreno: `terrainVoxelSize`, `groundHeightThre`, `terrainRadiusZ`, `terrainRadiusIncl`
- fitting assetto: `adjustZ`, `adjustIncl`, `InclFittingThre`, `maxIncl`
- repub camera: `camera_in_topic`, `camera_raw_out_topic`, `sem_*`, `depth_*`

Launch file:
[src/base_autonomy/vehicle_simulator/launch/vehicle_simulator.launch](../../src/base_autonomy/vehicle_simulator/launch/vehicle_simulator.launch).
