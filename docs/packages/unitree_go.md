# unitree_go

## Ruolo
Pacchetto messaggi ROS2 per stati/comandi Go2 (sport mode, low-level, sensori, video).

## Quando viene usato
Come dipendenza messaggi nel control stack (specialmente `go2_sport_api`).

## Messaggi rilevanti nello stack
- `SportModeState.msg` (stato locomozione)
- `SportModeCmd.msg` (comando locomozione)
- `IMUState.msg`, `LidarState.msg`
- `LowCmd.msg`, `LowState.msg`
- `WirelessController.msg`
- `Go2FrontVideoData.msg`

## Dati trasmessi
Contratto dati completo robot: stato dinamico, sensori, battery, comandi gait, telemetria.

## Parametri
Nessuno (package interface).
