# goalpoint_rviz_plugin

## Ruolo
Tool RViz per inviare goal globale (`/goal_point`) con click mappa.

## Quando viene usato
In modalità route planner: utente seleziona destinazione finale.

## Topic
- Subscribe: `/state_estimation` (per quota Z attuale)
- Publish: `/goal_point`, `/joy`

## Dati trasmessi
- `geometry_msgs/PointStamped` in frame `map`
- messaggio `/joy` di supporto per riportare stack in modalità navigazione waypoint

## Parametri
Nessun parametro runtime significativo; topic configurabile via proprietà tool RViz.
