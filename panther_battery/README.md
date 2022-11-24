# panther_battery

Package containing nodes monitoring and publishing internal battery state of the Husarion Panther robot.

## ROS Nodes

### adc_node.py

Publishes battery state read from ADC unit. Available from Panther version 1.2.

#### Publishes

- `battery` [*sensor_msgs/BatteryState*]: average values of both batteries if panther has two batteries. In case of single battery values only for the single one.
- `battery_1` [*sensor_msgs/BatteryState*]: first battery state. Published if second battery detected.
- `battery_2` [*sensor_msgs/BatteryState*]: second battery state. Published if second battery detected.

#### Subscribes

- `motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

#### Parameters

- `~loop_rate` [*float*, default: **20**]: rate in Hz at which battery state will be computed and published.

### roboteq_republisher_node.py

Node publishing Panther battery state read from motor controllers. Used in Panther versions 1.06 and below.

#### Publishes

- `battery` [*sensor_msgs/BatteryState*]: battery state.

#### Subscribes

- `motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.
