# teleop_rviz_plugin_plus

## Ruolo
Pannello RViz esteso con controllo FAR planner (attemptable, update graph, reset, read/save grafo).

## Quando viene usato
Modalità route planner con necessità di debug/controllo grafo in RViz.

## Topic
- Publish: `/joy`, `/planning_attemptable`, `/update_visibility_graph`, `/reset_visibility_graph`, `/read_file_dir`, `/save_file_dir`

## Dati trasmessi
- comandi modalità di navigazione
- trigger reset/update grafo
- path file `.vgh` da leggere/salvare

## Algoritmo
UI Qt che converte click/checkbox in messaggi ROS standard (`Bool`, `Empty`, `String`, `Joy`).

## Parametri
Nessuno (plugin UI).
