# go2_sport_api

## Ruolo
Bridge comandi velocità -> API sport Unitree (publisher di `unitree_api/Request`).

## Quando viene usato
Robot reale; in questo stack è usato come backend comandi moto.

## Topic
- Subscribe: `/joy` (in questa implementazione), opzionale `/cmd_vel`
- Publish: `/api/sport/request`

## Dati calcolati
- mappatura assi joystick a `vx`, `vy`, `vyaw`
- serializzazione command request Unitree SportClient

## Algoritmo
Loop a 100 Hz: legge ultimo comando joystick, costruisce request `Move`, pubblica verso API robot.

## Parametri
Nessun parametro ROS nel nodo attuale (costanti hard-coded `maxSpeedYaw`, `maxSpeedLateral`).

Sorgente:
[src/utilities/unitree_pkgs/go2_sport_api/src/vel_ctrl_repub.cpp](../../src/utilities/unitree_pkgs/go2_sport_api/src/vel_ctrl_repub.cpp)
