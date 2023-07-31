# panther_battery

Package containing nodes monitoring and publishing internal battery state of the Husarion Panther robot.

## ROS Nodes

### adc_node.py

Publishes battery state read from ADC unit. Available from Panther version 1.2. Voltage, current and temperature are smoothed out using a moving average.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: average values of both batteries if panther has two batteries. In case of single battery values only for the single one.
- `/panther/battery_1` [*sensor_msgs/BatteryState*]: first battery state. Published if second battery detected.
- `/panther/battery_2` [*sensor_msgs/BatteryState*]: second battery state. Published if second battery detected.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

- `/panther/hardware/io_state` [*panther_msgs/IOState*]: checks if charger is connected. Later fuses the information with charging current.

#### Parameters 

- `~fatal_bat_temp` [*float*, default: **62.0**]: The temperature of the battery at which the battery health state is set to `POWER_SUPPLY_HEALTH_OVERHEAT`.


### roboteq_republisher_node.py

Node publishing Panther battery state read from motor controllers. Used in Panther versions 1.06 and below. Voltage and current measurement are smoothed with moveing average. Current accounts only for motor controllers.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: battery state estimated from motor controllers.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.