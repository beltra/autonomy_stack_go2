# terrain_analysis

## Ruolo
Costruisce una mappa locale traversabile (`/terrain_map`) a partire da scan registrati e posa robot.

## Quando viene usato
Modulo core dello stack base sia in simulazione che su robot reale.

## Topic
- Subscribe: `/state_estimation`, `/registered_scan`, `/joy`, `/map_clearing`
- Publish: `/terrain_map`

## Dati calcolati
- voxel map locale con decadimento temporale
- stima suolo locale per cella
- altezza ostacolo relativa al suolo (in `intensity`)
- opzionale generazione ostacoli “no-data” in zone cieche

## Algoritmo
Pipeline: crop scan -> accumulo voxel rolling -> stima quota suolo (min/quantile) -> estrazione ostacoli -> pubblicazione nuvola terrain.

## Parametri
Parametri chiave:
- filtraggio: `scanVoxelSize`, `minRelZ`, `maxRelZ`, `disRatioZ`
- memoria: `decayTime`, `voxelPointUpdateThre`, `voxelTimeUpdateThre`
- suolo: `useSorting`, `quantileZ`, `limitGroundLift`, `maxGroundLift`
- ostacoli/no-data: `vehicleHeight`, `noDataObstacle`, `noDataArea*`, `maxElevBelowVeh`
- dinamici: `clearDyObs`, `minDyObs*`, `maxDyObsVFOV`

Default launch:
[src/base_autonomy/terrain_analysis/launch/terrain_analysis.launch](../../src/base_autonomy/terrain_analysis/launch/terrain_analysis.launch).

Documentazione dettagliata dei parametri già presente in:
[src/base_autonomy/terrain_analysis/README.md](../../src/base_autonomy/terrain_analysis/README.md).
