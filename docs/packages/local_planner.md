# local_planner

## Ruolo
Pacchetto di navigazione locale a due nodi:
- `localPlanner`: genera una traiettoria locale collision-free.
- `pathFollower`: converte la traiettoria in comando velocità robot.

## Quando viene usato
Sempre nello stack base e route-planner (sim e real).

## Topic
### Subscribe
- `/state_estimation`
- `/registered_scan` (se `useTerrainAnalysis=false`)
- `/terrain_map` (se `useTerrainAnalysis=true`)
- `/joy`, `/speed`, `/way_point`, `/navigation_boundary`, `/added_obstacles`, `/check_obstacle`
- `pathFollower` legge anche `/path` e `/stop`

### Publish
- `/path` (path locale selezionato)
- `/free_paths` (debug candidate path)
- `/stacked_scans` (debug)
- `/cmd_vel`
- `/api/sport/request` (robot reale)

## Dati calcolati e trasmessi
- nuvola locale filtrata e/o mappa terrain locale
- score dei path candidati
- path migliore (`nav_msgs/Path`)
- comando cinematico (`geometry_msgs/TwistStamped`) + request Unitree API

## Algoritmo
1. Carica una libreria di primitive di traiettoria da `pathFolder`.
2. Filtra i punti ostacolo nel raggio locale (`adjacentRange`) e downsample.
3. Per ogni path candidato calcola collisioni, costo direzione e costo opzionale di elevazione.
4. Seleziona path con score massimo e pubblica `/path`.
5. `pathFollower` applica look-ahead, controllo yaw-rate, limiti dinamici e logica modalità (manual/smart joystick/waypoint).

## Parametri principali
- Geometria: `vehicleLength`, `vehicleWidth`, `sensorOffsetX`, `sensorOffsetY`
- Collisione: `adjacentRange`, `obstacleHeightThre`, `groundHeightThre`, `checkObstacle`, `checkRotObstacle`
- Scoring: `dirWeight`, `dirThre`, `useCost`, `costHeightThre`, `costScore`
- Traiettoria: `pathScale*`, `pathRange*`, `pathCropByGoal`, `pointPerPathThre`
- Velocità/controllo: `maxSpeed`, `maxYawRate`, `maxAccel`, `lookAheadDis`
- Sicurezza: `stopDisThre`, `slowDwnDisThre`, `useInclRateToSlow`, `useInclToStop`
- Modalità: `autonomyMode`, `autonomySpeed`, `twoWayDrive`, `goalCloseDis`
- Robot reale: `is_real_robot`, `enableClassicWalk`, `disableClassicWalkOnExit`

I default operativi sono in [src/base_autonomy/local_planner/launch/local_planner.launch](../../src/base_autonomy/local_planner/launch/local_planner.launch).
