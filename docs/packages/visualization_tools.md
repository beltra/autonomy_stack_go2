# visualization_tools

## Ruolo
Visualizza metriche esplorazione e produce log di traiettoria/metriche.

## Quando viene usato
Tipicamente in simulazione per analisi performance (copertura, distanza, durata).

## Topic
- Subscribe: `/state_estimation`, `/registered_scan`, `/runtime`
- Publish: `/overall_map`, `/explored_areas`, `/trajectory`, `/explored_volume`, `/traveling_distance`, `/time_duration`

## Dati calcolati
- volume esplorato voxelizzato
- area esplorata in 2.5D
- distanza percorsa e durata missione
- log su file (`metricFile`, `trajFile` con timestamp)

## Algoritmo
Accumula point cloud osservate, downsample periodico, pubblica mappe aggregate e metriche numeriche.

## Parametri principali
- input file: `metricFile`, `trajFile`, `mapFile`
- voxelizzazione: `overallMapVoxelSize`, `exploredAreaVoxelSize`, `exploredVolumeVoxelSize`
- soglie update: `transInterval`, `yawInterval`, `overallMapDisplayInterval`, `exploredAreaDisplayInterval`

Launch:
[src/base_autonomy/visualization_tools/launch/visualization_tools.launch](../../src/base_autonomy/visualization_tools/launch/visualization_tools.launch).
