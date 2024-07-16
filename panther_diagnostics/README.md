# panther_diagnostics

Package containing nodes monitoring and publishing the Built-in Computer status of Husarion Panther robot.

## Launch Files

`system_status.launch.py` - launch a node that analyzes the state of the most important components in the robot

### system_status.launch.py - Arguments

| Argument    | Description <br/> ***Type:*** `Default`                                         |
| ----------- | ------------------------------------------------------------------------------- |
| `namespace` | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)` |

### system_status.launch.py - Nodes

`system_monitor`

## ROS Nodes

### system_monitor

Publishes system state of the Built-in Computer such as CPU usage, RAM memory usage, disk usage and  CPU temperature.

#### Publishes

- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: system status diagnostic messages.
- `system_status` [*panther_msgs/SystemStatus*]: system status statistics.

#### Parameters

- `~frame_id` [*string*, default: **build_in_computer**]: Frame where computer is located.
- `~publish_rate` [*double*, default: **0.25**]: System status publish rate in seconds.
- `~disk_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for disk usage warning in percentage.
- `~memory_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for memory usage warning in percentage.
- `~cpu_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for CPU usage warning in percentage.
- `~cpu_temperature_warn_threshold` [*float*, default: **80.0**]: Threshold for CPU temperature warning in degrees Celsius.
