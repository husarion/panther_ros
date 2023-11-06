# panther_battery

Package containing nodes monitoring and publishing internal battery state of the Husarion Panther robot.

## ROS Nodes

### adc_node

Publishes battery state read from ADC unit. Available from Panther version 1.2.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: mean values of both batteries if Panther has two batteries. Otherwise, the state of the single battery will be published.
- `/panther/battery_1_raw` [*sensor_msgs/BatteryState*]: first battery state. Published if second battery detected.
- `/panther/battery_2_raw` [*sensor_msgs/BatteryState*]: second battery state. Published if second battery detected.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

#### Parameters

- `~adc0_device` [*string*, default: **/sys/bus/iio/devices/iio:device0**]: ADC nr 0 device IIO path.
- `~adc1_device` [*string*, default: **/sys/bus/iio/devices/iio:device1**]: ADC nr 1 device IIO path.
- `~battery_timeout` [*float*, default: **1.0**]: specifies the timeout in seconds. If node fails to read ADC battery data exceeding this duration, the node will publish an unknown battery state.
- `~high_bat_temp` [*float*, default: **55.0**]: the temperature of the battery at which is is considered to overheat.
- `~ma_window_len/charge` [*int*, default: **10**]: window length of a moving average, used to smooth out battery charge readings.
- `~ma_window_len/current` [*int*, default: **10**]: window length of a moving average, used to smooth out battery current readings.
- `~ma_window_len/temp` [*int*, default: **10**]: window length of a moving average, used to smooth out battery temperature readings.
- `~ma_window_len/voltage` [*int*, default: **10**]: window length of a moving average, used to smooth out battery voltage readings.

### roboteq_republisher_node

Node publishing Panther battery state read from motor controllers. Used in Panther versions 1.06 and below.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: battery state.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.

#### Parameters

- `~battery_current_window_len` [*int*, default: **10**]: window length of a moving average, used to smooth out battery current readings.
- `~battery_timeout` [*float*, default: **1.0**]: specifies the timeout in seconds. If no new battery messages are received within this duration, the node will publish an unknown battery state.
- `~batery_voltage_window_len` [*int*, default: **10**]: window length of a moving average, used to smooth out battery voltage readings.
