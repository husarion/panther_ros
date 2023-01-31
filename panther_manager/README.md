# panther_manager

A package containing nodes responsible for logging the state of the Husarion Panther robot.

## ROS Nodes

### system_status_node.py

Publishes stats status of the internal computer. Stats include CPU utilization and temperature, as well as disc and RAM usage.

#### Publishes

- `/panther/system_status` [*panther_msgs/SystemStatus*]: information about internal computer CPU temperature, utilization and disc and RAM usage.