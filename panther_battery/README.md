# panther_battery

The package containing nodes monitoring and publishing the internal battery state of the Husarion Panther robot.

## Launch Files

This package contains:

- [`battery.launch.py`](#batterylaunchpy---arguments) - is responsible for activating battery node, which dealing with reading and publishing battery data.

### battery.launch.py - Arguments

| Argument    | Description <br/> ***Type:*** `Default`                                         |
| ----------- | ------------------------------------------------------------------------------- |
| `namespace` | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)` |

### battery.launch.py - Nodes

| Node name            | *Type*                             |
| ---------------- | ---------------------------------- |
| `battery_driver` | [*panther_batter/battery_node*](.) |

## ROS Nodes

- `battery_node`: Publishes battery state read from ADC unit for Panther version 1.2 and above, or based on Roboteq motor controllers' data for earlier versions of the robot.

### battery_node

#### Publishes

- `_battery/battery_1_status_raw` [*sensor_msgs/BatteryState*]: first battery raw state.
- `_battery/battery_2_status_raw` [*sensor_msgs/BatteryState*]: second battery raw state. Published if second battery detected.
- `battery/battery_status` [*sensor_msgs/BatteryState*]: mean values of both batteries if Panther has two batteries. Otherwise, the state of the single battery will be published.
- `battery/charging_status` [*panther_msgs/ChargingStatus*]: battery charging status.
- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: battery diagnostic messages.

#### Subscribes

- `hardware/io_state` [*panther_msgs/IOState*]: current state of IO.
- `hardware/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags. Subscribed if using Roboteq motor controllers data.

#### Parameters

- `~/adc/device0` [*string*, default: **/dev/adc0**]: ADC nr 0 IIO device. Used with Panther version 1.2 and above.
- `~/adc/device1` [*string*, default: **/dev/adc1**]: ADC nr 1 IIO device. Used with Panther version 1.2 and above.
- `~/adc/path` [*string*, default: **/sys/bus/iio/devices/**]: path of ADC devices mount.
- `~/adc/ma_window_len/charge` [*int*, default: **10**]: window length of a moving average, used to smooth out battery charge readings. Used with Panther version 1.2 and above.
- `~/adc/ma_window_len/temp` [*int*, default: **10**]: window length of a moving average, used to smooth out battery temperature readings. Used with Panther version 1.2 and above.
- `~/battery_timeout` [*float*, default: **1.0**]: specifies the timeout in seconds. If the node fails to read battery data exceeding this duration, the node will publish an unknown battery state.
- `~/ma_window_len/voltage` [*int*, default: **10**]: window length of a moving average, used to smooth out battery voltage readings.
- `~/ma_window_len/current` [*int*, default: **10**]: window length of a moving average, used to smooth out battery current readings.
- `~/roboteq/driver_state_timeout` [*float*, default: **0.2**]: specifies timeout in seconds after which driver state messages will be considered old. Used with Panther version 1.06 and earlier.
