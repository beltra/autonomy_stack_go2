# boundary_handler

## Ruolo
Costruisce un visibility graph statico da boundary polygon e traiettoria seed (tool offline/di setup).

## Quando viene usato
Workflow di preparazione mappe/graph file; non è nel loop online standard di navigazione.

## Topic
- Publish: `/graph_decoder_viz`, `/free_p_viz`

## Dati calcolati
- nodi boundary con classificazione convesso/concavo
- archi di visibilità validi nel poligono
- file grafo `.vgh` salvato su disco

## Algoritmo
1. Legge boundary PLY.
2. Crea nodi sui vertici e connessioni contigue.
3. Testa visibilità tra coppie nodi (vincoli direzionali + collisione segmenti poligonali).
4. Salva il grafo serializzato.

## Parametri
- `folder_path`, `boundary_file`, `traj_file`, `graph_file`
- `world_frame`, `visual_scale_ratio`, `height_tolz`

Config:
[src/route_planner/boundary_handler/config/default.yaml](../../src/route_planner/boundary_handler/config/default.yaml).
