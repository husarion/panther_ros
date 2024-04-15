[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_diagnostics

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

Package containing nodes monitoring and publishing the Built-in Computer status of Husarion Panther robot.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## ROS Nodes

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)
[//]: # (ROS_API_NODE_NAME_START)

### system_status_node

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

Publishes system state of the Built-in Computer such as CPU usage, RAM memory usage, disk usage and  CPU temperature.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishes

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/diagnostics` [*diagnostic_msgs/DiagnosticArray*]: system status diagnostic messages.
- `/system_status` [*panther_msgs/SystemStatus*]: system status statistics.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Parameters

[//]: # (ROS_API_NODE_PARAMETERS_START)
- `~frame_id` [*string*, default: **build_in_computer**]: Frame where computer is located.
- `~publish_rate` [*double*, default: **0.25**]: System status publish rate in seconds.
- `~disk_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for disk usage warning in percentage.
- `~memory_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for memory usage warning in percentage.
- `~cpu_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for CPU usage warning in percentage.
- `~cpu_temperature_warn_threshold` [*float*, default: **80.0**]: Threshold for CPU temperature warning in degrees Celsius.

[//]: # (ROS_API_NODE_PARAMETERS_END)
[//]: # (ROS_API_NODE_END)
