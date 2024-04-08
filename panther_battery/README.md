[//]: # (ROS_API_PACKAGE_START)
[//]: # (ROS_API_PACKAGE_NAME_START)

# panther_battery

[//]: # (ROS_API_PACKAGE_NAME_END)
[//]: # (ROS_API_PACKAGE_DESCRIPTION_START)

Package containing nodes monitoring and publishing the internal battery state of the Husarion Panther robot.

[//]: # (ROS_API_PACKAGE_DESCRIPTION_END)

## ROS Nodes

[//]: # (ROS_API_NODE_START)
[//]: # (ROS_API_NODE_COMPATIBLE_1_0)
[//]: # (ROS_API_NODE_COMPATIBLE_1_2)
[//]: # (ROS_API_NODE_NAME_START)

### battery_node

[//]: # (ROS_API_NODE_NAME_END)
[//]: # (ROS_API_NODE_DESCRIPTION_START)

Publishes battery state read from ADC unit for Panther version 1.2 and above, or based on Roboteq motor controllers' data for earlier versions of the robot.

[//]: # (ROS_API_NODE_DESCRIPTION_END)

#### Publishes

[//]: # (ROS_API_NODE_PUBLISHERS_START)

- `/diagnostics` [*diagnostic_msgs/DiagnosticArray*]: battery diagnostic messages.
- `~/battery` [*sensor_msgs/BatteryState*]: mean values of both batteries if Panther has two batteries. Otherwise, the state of the single battery will be published.
- `~/battery_1_raw` [*sensor_msgs/BatteryState*]: first battery raw state.
- `~/battery_2_raw` [*sensor_msgs/BatteryState*]: second battery raw state. Published if second battery detected.

[//]: # (ROS_API_NODE_PUBLISHERS_END)

#### Subscribes

[//]: # (ROS_API_NODE_SUBSCRIBERS_START)

- `~/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags. Subscribed if using Roboteq motor controllers data.

[//]: # (ROS_API_NODE_SUBSCRIBERS_END)

#### Parameters

[//]: # (ROS_API_NODE_PARAMETERS_START)

- `~/adc/device0` [*string*, default: **/sys/bus/iio/devices/iio:device0**]: ADC nr 0 device IIO path. Used with Panther version 1.2 and above.
- `~/adc/device1` [*string*, default: **/sys/bus/iio/devices/iio:device1**]: ADC nr 1 device IIO path. Used with Panther version 1.2 and above.
- `~/adc/ma_window_len/charge` [*int*, default: **10**]: window length of a moving average, used to smooth out battery charge readings. Used with Panther version 1.2 and above.
- `~/adc/ma_window_len/temp` [*int*, default: **10**]: window length of a moving average, used to smooth out battery temperature readings. Used with Panther version 1.2 and above.
- `~/battery_timeout` [*float*, default: **1.0**]: specifies the timeout in seconds. If the node fails to read battery data exceeding this duration, the node will publish an unknown battery state.
- `~/ma_window_len/voltage` [*int*, default: **10**]: window length of a moving average, used to smooth out battery voltage readings.
- `~/ma_window_len/current` [*int*, default: **10**]: window length of a moving average, used to smooth out battery current readings.
- `~/panther_version` [*float*, default: **1.2**]: Panther robot version. Used to initialize correct Battery node interface.
- `~/roboteq/driver_state_timeout` [*float*, default: **0.2**]: specifies timeout in seconds after which driver state messages will be considered old. Used with Panther version 1.06 and earlier.

[//]: # (ROS_API_NODE_PARAMETERS_END)
[//]: # (ROS_API_NODE_END)
