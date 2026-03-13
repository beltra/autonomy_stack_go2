# unitree_api

## Ruolo
Pacchetto messaggi per request/response API Unitree (canale comandi robot).

## Quando viene usato
Da `pathFollower`, `calibrate_imu`, `go2_sport_api` per inviare comandi sport al robot.

## Messaggi principali
- `Request.msg` (`header`, `parameter`, `binary`)
- `Response.msg` (`header`, `data`, `binary`)
- strutture header: `RequestHeader`, `RequestIdentity`, `RequestLease`, `RequestPolicy`, `ResponseHeader`, `ResponseStatus`

## Dati trasmessi
Envelope tipizzato per API Unitree con policy priorità e payload testuale/binario.

## Parametri
Nessuno (package interface).
