# terrain_analysis_ext

## Ruolo
Estende la mappa terrain con scala più ampia e connettività locale, pubblicando `/terrain_map_ext` per il route planner.

## Quando viene usato
Soprattutto in modalità con `far_planner` (route planner), in particolare real-route.

## Topic
- Subscribe: `/state_estimation`, `/registered_scan`, `/terrain_map`, `/cloud_clearing`, `/joy`
- Publish: `/terrain_map_ext`

## Dati calcolati
- mappa estesa con punti elevati rispetto al suolo
- filtro ceiling/noise
- stima connettività del terreno vicino al robot (opzionale)

## Algoritmo
Come `terrain_analysis` ma su griglia più ampia (`terrainVoxelWidth=41`, `planarVoxelWidth=101`), con logica aggiuntiva di connettività flood-fill locale e fusione con `terrain_map` locale.

## Parametri principali
- risoluzione/memoria: `scanVoxelSize`, `decayTime`, `voxelPointUpdateThre`, `voxelTimeUpdateThre`
- limiti verticali: `lowerBoundZ`, `upperBoundZ`, `disRatioZ`, `ceilingFilteringThre`
- connettività: `checkTerrainConn`, `terrainConnThre`, `terrainUnderVehicle`
- output locale: `localTerrainMapRadius`, `vehicleHeight`

Default launch:
[src/base_autonomy/terrain_analysis_ext/launch/terrain_analysis_ext.launch](../../src/base_autonomy/terrain_analysis_ext/launch/terrain_analysis_ext.launch).
