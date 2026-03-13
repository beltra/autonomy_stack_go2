# visibility_graph_msg

## Ruolo
Pacchetto interfaccia che definisce i messaggi del visibility graph.

## Quando viene usato
Usato da `far_planner`, `graph_decoder`, `far_web_gui` e strumenti I/O grafo.

## Messaggi
- `Graph.msg`
  - header
  - `robot_id`
  - array `nodes`
  - `size`
- `Node.msg`
  - metadati nodo (`id`, `freetype`, flag `is_*`)
  - geometria (`position`, `surface_dirs`)
  - connettività (`connect_nodes`, `poly_connects`, `contour_connects`, `trajectory_connects`)

## Dati trasmessi
Rappresentazione completa del grafo di navigazione, usata sia online (topic) sia offline (serializzazione `.vgh`).

## Parametri
Nessuno (solo definizioni messaggi).
