# panther_battery

Package containing nodes monitoring and publishing internal battery state of the Husarion Panther robot.

## ROS Nodes

### adc_node.py

Publishes battery state read from ADC unit and thermistors. Available from Panther version 1.2. Voltage, current and temperature are smoothed out using a moving average.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: average values of both batteries if panther has two batteries. In case of single battery values only for the single one.
- `/panther/battery_1` [*sensor_msgs/BatteryState*]: first battery state. Published if second battery detected.
- `/panther/battery_2` [*sensor_msgs/BatteryState*]: second battery state. Published if second battery detected.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.
- `/panther/hardware/io_state` [*panther_msgs/IOState*]: checks if charger is connected. Later fuses the information with charging current.

#### Battery statuses

The battery status is described with two [Battery State](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/BatteryState.html) message fields: `power_supply_status` and `power_supply_health`. **This information is subsequently retrieved in the panther_manager node, where decisions related to safety procedures are made**.

Safety thresholds used in calculating the various values describing the state of the battery:

| Name                       | Value | Unit       |
| -------------------------- | ----- | ---------- |
| `BAT_CHARGING_CURR_THRESH` | 0.1   | **A**      |
| `V_BAT_FATAL_MIN`          | 27.0  | **V**      |
| `V_BAT_FATAL_MAX`          | 43.0  | **V**      |
| `V_BAT_FULL`               | 41.4  | **V**      |
| `V_BAT_MIN`                | 32.0  | **V**      |
| `LOW_BAT_TEMP`             | -10.0 | **&deg;C** |
| `OVERHEAT_BAT_TEMP`        | 45.0  | **&deg;C** |

The method of calculating individual values:

- for single battery:

| Value                                       | Variable name                                                 | Method of calculating                            | Comment                                                                                          | Published on |
| ------------------------------------------- | ------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------------------------ | ------------ |
| Voltage                                     | `V_bat`                                                       | Measured with ADC unit                           | -                                                                                                | `/battery`   |
| Temperature                                 | `temp_bat`                                                    | Measured with thermistor                         | -                                                                                                | `/battery`   |
| Each channel current (charge and discharge) | `I_bat_1`, `I_bat_2`, <br> `I_charge_bat_1`, `I_charge_bat_2` | Measured with ADC unit                           | Despite the single battery, two ADC unit channels are taken into consideration | -            |
| Current (discharge)                         | `I_bat`                                                       | `-(I_bat_1 + I_bat_2) + I_charge_bat_1`          | As above                                                                                         | `/battery`   |
| Current (charge)                            | `I_charge`                                                    | `I_charge_bat_1 + I_charge_bat_2`                | As above                                                                                         | -            |
| Percentage                                  | `percentage`                                                  | `(V_bat - V_BAT_MIN) / (V_BAT_FULL - V_BAT_MIN)` | -                                                                                                | `/battery`   |  |

- for double battery:

| Value                       | Variable name                               | Method of calculating                                                       | Published on                           |
| --------------------------- | ------------------------------------------- | --------------------------------------------------------------------------- | -------------------------------------- |
| Channel voltage             | `V_bat_1`, `V_bat_2`                        | Measured with ADC unit separately for each battery                          | `/battery_1`, `/battery_2`             |
| Mean voltage                | `V_bat`                                     | `V_bat_1 + V_bat_2`                                                         | `/battery`                             |
| Channel temperature         | `temp_bat_1`, `temp_bat_2`                  | Measured with thermistor separately for each battery                        | `/battery_1`, `/battery_2`             |
| Channel current (discharge) | `I_bat_1`, `I_bat_2`                        | Measured with ADC unit separately for each battery                          | -                                      |
| Channel current (charge)    | `I_charge_bat_1`, `I_charge_bat_2`          | Measured with ADC unit separately for each battery                          | -                                      |
| Mean current (discharge)    | `I_bat` *Mean value of two ADC channels*    | `-(I_bat_1 + I_bat_2) + I_charge_bat_1 + I_charge_bat_2,`                   | `/battery`                             |
| Current (discharge)         | `I_bat`                                     | `-I_bat_? + I_charge_bat_?`                                                 | `/battery_1`, `/battery_2`             |
| Mean current (charge)       | `I_charge` *Mean value of two ADC channels* | `I_charge_bat_1 + I_charge_bat_2`                                           | -                                      |
| Current (charge)            | `I_charge`                                  | `I_charge_bat_x`                                                            | -                                      |
| Percentage                  | `percentage`                                | `(V_bat - V_BAT_MIN) / (V_BAT_FULL - V_BAT_MIN)` *For `V_bat` or `V_bat_?`* | `/battery`, `/battery_1`, `/battery_2` |

##### Value of `power_supply_health` Field

| Status                | Method of verification                                  | Condition                      | Procedure                                                                                                                                     |
| --------------------- | ------------------------------------------------------- | ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------- |
| GOOD                  | None of these conditions are met                        | n/a                            | -                                                                                                                                             |
| UNKNOWN               | Check the last time of Roboteq driver message receive  | data not received              | -                                                                                                                                             |
| OVERHEAT              | Battery temperature measurement with ADC & thermistor   | `temp_bat > OVERHEAT_BAT_TEMP` | The `adc_node` issues a warning. This status is then read in the `panther_manager` safety behavior tree.                                      |
| DEAD                  | Mean battery voltage  (moving average, window size: 10) | `V_bat < V_BAT_FATAL_MIN`      | `adc_node` logs warning. This status is then read in the `panther_manager` safety behavior tree.                                              |
| OVERVOLTAGE           | Mean battery voltage  (moving average, window size: 10) | `V_bat > V_BAT_FATAL_MAX`      | After 2 seconds of high voltage being present, the warning is logged. This status is then read in the `panther_manager` safety behavior tree. |
| COLD                  | Battery temperature measurement with ADC & thermistor   | `temp_bat < LOW_BAT_TEMP`      | `adc_node` logs warning                                                                                                                       |
| UNSPEC_FAILURE        | -                                                       | -                              | -                                                                                                                                             |
| WATCHDOG_TIMER_EXPIRE | -                                                       | -                              | -                                                                                                                                             |
| SAFETY_TIMER_EXPIRE   | -                                                       | -                              | -                                                                                                                                             |

##### Value of `power_supply_status` Field

| State        | Condition                                                   | Comment                                                                                                                                                                                                     |
| ------------ | ----------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| UNKNOWN      | None of these conditions are met                            | -                                                                                                                                                                                                           |
| CHARGING     | Charger connected and `I_charge > BAT_CHARGING_CURR_THRESH` | The state is published on the `/battery` topic. If the robot has two batteries, it is also published on `/battery` (where the threshold is `2 * BAT_CHARGING_CURR_THRESH`), `/battery_1`, and `/battery_2`. |
| FULL         | Batter percentage                                           | when charge is 100%, but 100% is equal to `V_BAT_FULL`                                                                                                                                                      |
| NOT_CHARGING | Charger connected but none of above conditions are met      |                                                                                                                                                                                                             |
| DISCHARGING  | none of above conditions are met                            |                                                                                                                                                                                                             |

### roboteq_republisher_node.py

Node publishing Panther battery state read from motor controllers. Used in Panther versions 1.06 and below. Voltage and current measurement are smoothed out using a moving average. Current accounts only for motor controllers.

#### Publishes

- `/panther/battery` [*sensor_msgs/BatteryState*]: battery state estimated from motor controllers.

#### Subscribes

- `/panther/driver/motor_controllers_state` [*panther_msgs/DriverState*]: current motor controllers' state and error flags.
