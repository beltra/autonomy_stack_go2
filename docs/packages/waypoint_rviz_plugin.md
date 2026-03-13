# waypoint_rviz_plugin

## Ruolo
Tool RViz per impostare waypoint locale (`/way_point`) con click.

## Quando viene usato
Modalità base waypoint (senza goal globale route planner).

## Topic
- Subscribe: `/state_estimation`
- Publish: `/way_point`, `/joy`

## Dati trasmessi
- waypoint in frame `map`
- messaggio `/joy` per confermare modalità navigation.

## Parametri
Nessuno runtime rilevante; topic configurabile via proprietà tool RViz.
