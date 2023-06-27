# panther_battery

Package containing nodes monitoring and publishing internal battery state of the Husarion Panther robot.

## ROS Nodes

### roboteq_republisher_node.py

Node publishing Panther battery state read from motor controllers. Used in Panther versions 1.06 and below.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: battery state.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

#### Parameters

- `~battery_timeout` [*float*, default: **1.0**]: specifies the timeout in seconds. If no new battery messages are received within this duration, the node will publish an unknown battery state.
- `~batery_voltage_window_len` [*int*, default: **10**]: moving average window length used to smooth out battery voltage readings.
- `~batery_current_window_len` [*int*, default: **10**]: moving average window length used to smooth out battery current readings.