# ros_tcp_endpoint

## Ruolo
Bridge TCP ROS2 <-> Unity (pacchetto Unity Robotics) usato nella simulazione con ROS-TCP-Endpoint.

## Quando viene usato
Nei launch simulazione che avviano `default_server_endpoint` (porta tipica 10000).

## Topic/servizi
Il bridge è dinamico: inoltra topic/service richiesti dal client Unity (non ha un set fisso hard-coded di topic applicativi).

## Dati calcolati
- serializzazione/deserializzazione messaggi ROS su socket TCP
- dispatch publisher/subscriber/service lato ROS

## Parametri principali
- `ROS_IP`
- `ROS_TCP_PORT`

Launch app stack:
[src/base_autonomy/vehicle_simulator/launch/system_simulation.launch](../../src/base_autonomy/vehicle_simulator/launch/system_simulation.launch)

Package upstream readme:
[src/utilities/ROS-TCP-Endpoint/README.md](../../src/utilities/ROS-TCP-Endpoint/README.md)
