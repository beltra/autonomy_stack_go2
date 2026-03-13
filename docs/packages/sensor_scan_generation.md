# sensor_scan_generation

## Ruolo
Sincronizza odometria e scan registrata e ripubblica i dati nel frame del sensore al timestamp dello scan.

## Quando viene usato
Principalmente in simulazione e pipeline con strumenti di visualizzazione/analisi che richiedono coerenza temporale scan-odom.

## Topic
- Subscribe: `/state_estimation`, `/registered_scan`
- Publish: `/state_estimation_at_scan`, `/sensor_scan`

## Dati calcolati
- Trasforma ciascun punto da mappa a `sensor_at_scan` usando l’odometria sincronizzata.
- Pubblica anche TF `map -> sensor_at_scan`.

## Algoritmo
ApproximateTime sync tra odometria e point cloud, inversione della posa per riportare lo scan nel frame sensore al tempo corretto.

## Parametri
Nessun parametro runtime rilevante nel nodo; launch minimale:
[src/base_autonomy/sensor_scan_generation/launch/sensor_scan_generation.launch](../../src/base_autonomy/sensor_scan_generation/launch/sensor_scan_generation.launch).
