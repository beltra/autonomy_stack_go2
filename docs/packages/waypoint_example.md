# waypoint_example

## Ruolo
Nodo demo che invia una sequenza di waypoint da file, opzionalmente velocità e boundary.

## Quando viene usato
Per test missioni predefinite senza interazione manuale RViz.

## Topic
- Subscribe: `/state_estimation`
- Publish: `/way_point`, `/speed`, `/navigation_boundary`

## Dati calcolati
- waypoint corrente in funzione della distanza dal robot
- transizione waypoint con attesa (`waitTime`)
- pubblicazione periodica a `frameRate`

## Algoritmo
Legge file PLY waypoint/boundary, monitora distanza dal target corrente, avanza all’obiettivo successivo quando raggiunto.

## Parametri
- file: `waypoint_file_dir`, `boundary_file_dir`
- logica avanzamento: `waypointXYRadius`, `waypointZBound`, `waitTime`
- output: `frameRate`, `speed`, `sendSpeed`, `sendBoundary`

Launch:
[src/base_autonomy/waypoint_example/launch/waypoint_example.launch](../../src/base_autonomy/waypoint_example/launch/waypoint_example.launch).
