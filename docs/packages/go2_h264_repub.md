# go2_h264_repub

## Ruolo
Riceve stream H.264 multicast della camera Go2 e lo ripubblica come immagini ROS raw.

## Quando viene usato
Robot reale, quando si vuole feed camera su `/camera/image/raw`.

## Topic
- Publish: `/camera/image/raw`

## Dati calcolati
- decode H.264 via pipeline GStreamer
- conversione frame OpenCV -> `sensor_msgs/Image` (`bgr8`)

## Parametri
- `multicast_iface` (interfaccia di rete per il multicast)

Sorgente:
[src/utilities/unitree_pkgs/go2_h264_repub/src/go2_h264_repub.cpp](../../src/utilities/unitree_pkgs/go2_h264_repub/src/go2_h264_repub.cpp)
