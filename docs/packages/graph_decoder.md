# graph_decoder

## Ruolo
Decodifica/serializza il visibility graph per visualizzazione e import/export file `.vgh`.

## Quando viene usato
In route planner, incluso automaticamente da `far_planner.launch`.

## Topic e servizi
- Subscribe: `/robot_vgraph`, `/read_file_dir`, `/save_file_dir`
- Publish: `decoded_vgraph`, `/graph_decoder_viz`
- Service server: `/request_graph_service` (`std_srvs/Trigger`)

## Dati calcolati
- conversione `visibility_graph_msg/Graph` <-> rappresentazione interna
- serializzazione testo `.vgh` per salvataggio
- parsing `.vgh` per ricaricare grafo

## Algoritmo
Mantiene una mappa id->indice nodo, ricostruisce connessioni (`connect/poly/contour/traj`) e pubblica marker RViz.

## Parametri
- `world_frame`
- `visual_scale_ratio`

Config:
[src/route_planner/graph_decoder/config/default.yaml](../../src/route_planner/graph_decoder/config/default.yaml).
