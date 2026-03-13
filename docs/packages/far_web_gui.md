# far_web_gui

## Ruolo
Interfaccia web remota per controllo FAR planner via browser (senza RViz locale).

## Quando viene usato
Modalità route con accesso Wi-Fi/remoto.

## Componenti
- launch include `rosbridge_websocket` + `rosapi`
- nodo Python `graph_file_bridge.py`
- frontend statico in `www/`

## Topic e servizi
- Web GUI sottoscrive (via rosbridge): `/state_estimation`, `/far_reach_goal_status`, `/viz_graph_topic`, `/viz_path_topic`, `/viz_node_topic`
- Web GUI pubblica: `/goal_point`, `/joy`, `/planning_attemptable`, `/update_visibility_graph`, `/reset_visibility_graph`, `/web_gui/upload_graph`
- Service: `/web_gui/download_graph` (bridge verso `/request_graph_service`)
- Topic file-bridge: `/read_file_dir`, ascolto `/decoded_vgraph`

## Dati calcolati
- serializzazione graph ROS -> testo `.vgh` in risposta service
- upload `.vgh` browser -> file temporaneo -> trigger lettura decoder

## Parametri principali
- launch: `ws_port`, `http_port`, `suppress_nonessential_topics`
- bridge node: `http_port`, `web_dir`

Launch:
[src/utilities/far_web_gui/launch/far_web_gui.launch.py](../../src/utilities/far_web_gui/launch/far_web_gui.launch.py)
