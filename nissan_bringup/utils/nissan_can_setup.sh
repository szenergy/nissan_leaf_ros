modprobe can
modprobe can-raw
modprobe can-dev
modprobe mttcan
ifconfig can1 down
ip link set can1 up type can bitrate 500000
ifconfig can0 down
ip link set can0 up type can bitrate 500000

