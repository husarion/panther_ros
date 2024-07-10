# panther_battery

Package containing nodes monitoring and publishing the internal battery state of the Husarion Panther robot.

## Launch Files

This package contains:

- [`battery.launch.py`](#batterylaunchpy---arguments) - is responsible for activating battery node, which dealing with reading and publishing battery data.

### battery.launch.py - Arguments

| Argument    | Description <br/> ***Type:*** `Default`                                         |
| ----------- | ------------------------------------------------------------------------------- |
| `namespace` | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)` |

### battery.launch.py - Executable

| Executable     | *Type*                             |
| -------------- | ---------------------------------- |
| `battery_node` | [*panther_batter/battery_node*](.) |

## Running nodes

- [`battery_node`](#battery_node---configuration): Publishes battery state read from ADC unit for Panther version 1.2 and above, or based on Roboteq motor controllers' data for earlier versions of the robot.

### battery_node - Configuration

- `~/adc/device0` [*string*, default: **/sys/bus/iio/devices/iio:device0**]: ADC nr 0 device IIO path.*
- `~/adc/device1` [*string*, default: **/sys/bus/iio/devices/iio:device1**]: ADC nr 1 device IIO path.*
- `~/adc/ma_window_len/charge` [*int*, default: **10**]: window length of a moving average, used to smooth out battery charge readings.*
- `~/adc/ma_window_len/temp` [*int*, default: **10**]: window length of a moving average, used to smooth out battery temperature readings.*
- `~/battery_timeout` [*float*, default: **1.0**]: specifies the timeout in seconds. If the node fails to read battery data exceeding this duration, the node will publish an unknown battery state.
- `~/ma_window_len/voltage` [*int*, default: **10**]: window length of a moving average, used to smooth out battery voltage readings.
- `~/ma_window_len/current` [*int*, default: **10**]: window length of a moving average, used to smooth out battery current readings.
- `~/roboteq/driver_state_timeout` [*float*, default: **0.2**]: specifies timeout in seconds after which driver state messages will be considered old. Used with Panther version 1.06 and earlier.

> \* - Used with Panther version 1.2 and above.
