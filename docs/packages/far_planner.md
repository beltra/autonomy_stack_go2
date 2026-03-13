# far_planner

## Ruolo
Planner globale dinamico basato su visibility graph (FAR Planner). Produce waypoint locali e boundary per il livello locale.

## Quando viene usato
Nelle modalità `*_route*` (sim e real).

## Topic
### Subscribe
- `/odom_world` (remap tipico da `/state_estimation`)
- `/terrain_cloud` (remap da `/terrain_map_ext`)
- `/scan_cloud` (remap da `/terrain_map`)
- `/terrain_local_cloud` (remap da `/registered_scan`)
- `/goal_point`, `/joy`
- `/planning_attemptable`, `/update_visibility_graph`, `/reset_visibility_graph`
- `/read_file_dir`, `/save_file_dir` (se non soppressi)

### Publish
- controllo missione: `/way_point`, `/navigation_boundary`, `/far_reach_goal_status`
- grafo: `/robot_vgraph`
- visual/debug: `/viz_graph_topic`, `/viz_path_topic`, `/viz_node_topic`, `/viz_contour_topic`, `/viz_poly_topic`, `/viz_grid_map_topic`, `/viz_viewpoint_extend_topic`
- timing/debug: `/runtime`, `/planning_time`, `/far_traverse_time`, `/FAR_*_debug`

## Dati calcolati
- contorni ostacoli da occupancy image
- nodi e archi del visibility graph dinamico
- path globale e waypoint target locale
- stato raggiungimento goal e tempi ciclo

## Algoritmo
1. `ScanHandler`: discretizzazione 3D scan/obs/ray e rilevamento ostacoli dinamici.
2. `ContourDetector`: estrazione contorni ostacolo (OpenCV + semplificazione poligonale).
3. `DynamicGraph`: aggiorna nodi/archi visibili, votazione robusta connessioni, gestione frontiere.
4. `GraphPlanner`: ricerca traversabilità e path con modalità free-space vs attemptable, momentum path, autoswitch.
5. Pubblica waypoint e boundary al local planner.

## Parametri principali
- core: `main_run_freq`, `voxel_dim`, `robot_dim`, `sensor_range`, `terrain_range`, `local_planner_range`, `vehicle_height`
- modalità: `is_static_env`, `is_multi_layer`, `is_attempt_autoswitch`, `is_viewpoint_extend`
- output: `suppress_nonessential_topics`, `is_viz_freespace`, `visualize_ratio`
- namespace config: `graph/*`, `g_planner/*`, `map_handler/*`, `util/*`, `c_detector/*`, `graph_msger/*`

Config:
- [src/route_planner/far_planner/config/default.yaml](../../src/route_planner/far_planner/config/default.yaml)
- [src/route_planner/far_planner/config/matterport.yaml](../../src/route_planner/far_planner/config/matterport.yaml)

Launch:
[src/route_planner/far_planner/launch/far_planner.launch](../../src/route_planner/far_planner/launch/far_planner.launch).
