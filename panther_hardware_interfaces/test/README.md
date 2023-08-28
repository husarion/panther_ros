
## Setup
mock can config for tests:

sudo modprobe vcan
sudo ip link add dev panther_can type vcan
sudo ip link set up panther_can
sudo ip link set panther_can down
sudo ip link set panther_can txqueuelen 1000
sudo ip link set panther_can up

## Runing tests

colcon build --packages-select panther_hardware_interfaces --symlink-install
colcon test --event-handlers console_direct+ --packages-select panther_hardware_interfaces
colcon test-result --verbose --all

## Updating config
Copy eds file to config and run 
`dcfgen panther_can.yaml -r`
Remove master.dcf