#!/usr/bin/python3

from collections import defaultdict
import math
from threading import Lock
from typing import Dict, Optional, Union

import rospy

from sensor_msgs.msg import BatteryState

from panther_msgs.msg import DriverState, IOState


class ADCNode:
    BAT_CHARGING_CURR_THRESH = 0.1
    BAT02_DETECT_THRESH = 3.03
    V_BAT_FATAL_MIN = 27.0
    V_BAT_FATAL_MAX = 43.0
    V_BAT_FULL = 41.4
    V_BAT_MIN = 32.0
    LOW_BAT_TEMP = -10.0
    OVERHEAT_BAT_TEMP = 45.0

    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._driver_battery_last_info_time: Optional[float] = None
        self._I_driv: Optional[float] = None
        self._V_driv: Optional[float] = None

        self._mean_length = 20
        self._V_bat_hist = defaultdict(lambda: [37.0] * self._mean_length)
        self._V_bat_mean = defaultdict(lambda: 37.0)
        self._I_bat_charge_hist = defaultdict(lambda: [0.0] * self._mean_length)
        self._I_bat_charge_mean = defaultdict(lambda: 0.0)

        self._A = 298.15
        self._B = 3977.0
        self._R1 = 10000.0
        self._R0 = 10000.0
        self._u_supply = 3.28

        self._battery_count = self._check_battery_count()

        self._I_bat_charging_thresh = {}
        self._charger_connected = False

        self._lock = Lock()

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._motor_controllers_state_sub = rospy.Subscriber(
            'driver/motor_controllers_state', DriverState, self._motor_controllers_state_cb
        )
        self._io_state_sub = rospy.Subscriber('hardware/io_state', IOState, self._io_state_cb)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._battery_pub = rospy.Publisher('battery', BatteryState, queue_size=1)

        if self._battery_count == 2:
            self._battery_1_pub = rospy.Publisher('battery_1', BatteryState, queue_size=1)
            self._battery_2_pub = rospy.Publisher('battery_2', BatteryState, queue_size=1)

            self._I_bat_charging_thresh.update(
                {
                    self._battery_pub: 2 * self.BAT_CHARGING_CURR_THRESH,
                    self._battery_1_pub: self.BAT_CHARGING_CURR_THRESH,
                    self._battery_2_pub: self.BAT_CHARGING_CURR_THRESH,
                }
            )
        else:
            self._I_bat_charging_thresh.update({self._battery_pub: self.BAT_CHARGING_CURR_THRESH})

        # -------------------------------
        #   Timers
        # -------------------------------

        # Running at 10 Hz
        self._battery_timer = rospy.Timer(rospy.Duration(0.1), self._battery_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Battery count: {self._battery_count}')
        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _motor_controllers_state_cb(self, driver_state: DriverState) -> None:
        with self._lock:
            self._driver_battery_last_info_time = rospy.get_time()

            self._V_driv = (driver_state.front.voltage + driver_state.rear.voltage) / 2.0
            self._I_driv = driver_state.front.current + driver_state.rear.current

    def _io_state_cb(self, io_state: IOState) -> None:
        with self._lock:
            self._charger_connected = io_state.charger_connected

    def _battery_timer_cb(self, *args) -> None:
        try:
            V_bat_1 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device1/in_voltage0_raw', LSB=0.02504255, offset=0.0
            )
            V_bat_2 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device1/in_voltage3_raw', LSB=0.02504255, offset=0.0
            )
            V_temp_bat_1 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device0/in_voltage1_raw', LSB=0.002, offset=0.0
            )
            V_temp_bat_2 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device0/in_voltage0_raw', LSB=0.002, offset=0.0
            )
            I_charge_bat_1 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device0/in_voltage3_raw', LSB=0.005, offset=0.0
            )
            I_charge_bat_2 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device0/in_voltage2_raw', LSB=0.005, offset=0.0
            )
            I_bat_1 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device1/in_voltage2_raw', LSB=0.04, offset=625.0
            )
            I_bat_2 = self._get_adc_measurement(
                path='/sys/bus/iio/devices/iio:device1/in_voltage1_raw', LSB=0.04, offset=625.0
            )
        except:
            rospy.logerr(f'[{rospy.get_name()}] Battery ADC measurement error')
            return

        if self._battery_count == 2:
            temp_bat_1 = self._voltage_to_deg(V_temp_bat_1)
            temp_bat_2 = self._voltage_to_deg(V_temp_bat_2)

            self._publish_battery_msg(
                self._battery_1_pub, V_bat_1, temp_bat_1, -I_bat_1 + I_charge_bat_1, I_charge_bat_1
            )
            self._publish_battery_msg(
                self._battery_2_pub, V_bat_2, temp_bat_2, -I_bat_2 + I_charge_bat_2, I_charge_bat_2
            )

            self._publish_battery_msg(
                self._battery_pub,
                (V_bat_1 + V_bat_2) / 2.0,
                (temp_bat_1 + temp_bat_2) / 2.0,
                -(I_bat_1 + I_bat_2) + I_charge_bat_1 + I_charge_bat_2,
                I_charge_bat_1 + I_charge_bat_2,
            )

        else:
            temp_bat_1 = self._voltage_to_deg(V_temp_bat_1)
            self._publish_battery_msg(
                self._battery_pub,
                V_bat_1,
                temp_bat_1,
                -(I_bat_1 + I_bat_2) + I_charge_bat_1,
                I_charge_bat_1,
            )

    def _check_battery_count(self) -> int:
        trials_num = 10
        V_temp_sum = 0.0

        try:
            for i in range(trials_num):
                V_temp_sum += self._get_adc_measurement(
                    path='/sys/bus/iio/devices/iio:device0/in_voltage0_raw', LSB=0.002, offset=0.0
                )
                rospy.sleep(0.2)

            V_temp_bat_2 = V_temp_sum / trials_num

        except:
            rospy.logerr(
                f'[{rospy.get_name()}] Battery ADC measurement error excep. '
                f'The number of batteries cannot be determined. The single battery was adopted.'
            )
            return 1

        return 1 if V_temp_bat_2 > ADCNode.BAT02_DETECT_THRESH else 2

    def _get_adc_measurement(self, path: str, offset: float, LSB: float) -> float:
        raw_value = self._read_file(path)
        return (raw_value - offset) * LSB

    def _voltage_to_deg(self, V_temp: float) -> float:
        if V_temp == 0 or V_temp >= self._u_supply:
            rospy.logerr(f'[{rospy.get_name()}] Temperature measurement error')
            return float('nan')

        R_therm = (V_temp * self._R1) / (self._u_supply - V_temp)

        return (self._A * self._B / (self._A * math.log(R_therm / self._R0) + self._B)) - 273.15

    def _publish_battery_msg(
        self, bat_pub: rospy.Publisher, V_bat: float, temp_bat: float, I_bat: float, I_charge: float
    ) -> None:
        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = V_bat
        battery_msg.temperature = temp_bat
        battery_msg.current = I_bat
        battery_msg.percentage = self._clamp(
            (battery_msg.voltage - self.V_BAT_MIN) / (self.V_BAT_FULL - self.V_BAT_MIN),
            0.0,
            1.0,
        )
        battery_msg.capacity = 20.0
        battery_msg.design_capacity = 20.0
        battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery_msg.present = True

        V_bat_mean = self._count_mean(bat_pub, V_bat, self._V_bat_mean, self._V_bat_hist)
        I_bat_mean = self._count_mean(
            bat_pub, I_charge, self._I_bat_charge_mean, self._I_bat_charge_hist
        )

        with self._lock:
            # check battery status
            if self._charger_connected:
                if battery_msg.percentage == 1.0:
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
                    rospy.loginfo_throttle(
                        60.0,
                        f'[{rospy.get_name()}] The battery is fully charged. '
                        f'The robot can be disconnected from the charger.',
                    )
                elif I_bat_mean > self._I_bat_charging_thresh[bat_pub]:
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    rospy.loginfo_throttle(
                        1800.0,
                        f'[{rospy.get_name()}] Robot charging process update. '
                        f'Battery percentage: {battery_msg.percentage * 100}%',
                    )
                else:
                    battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                    rospy.logwarn_throttle(
                        5.0,
                        f'[{rospy.get_name()}] The charger has been plugged in, '
                        f'but the charging process has not started. '
                        f'Verify whether the charger is connected to a power source.',
                    )
            else:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

            # check battery health
            error_msg = None
            if V_bat_mean < self.V_BAT_FATAL_MIN:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
                error_msg = 'The battery voltage is critically low!'
            elif V_bat_mean > self.V_BAT_FATAL_MAX:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
                error_msg = 'The battery overvoltage!'
            elif temp_bat >= self.OVERHEAT_BAT_TEMP:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
                error_msg = 'The battery is overheating!'
            elif temp_bat <= self.LOW_BAT_TEMP:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_COLD
                error_msg = 'The battery is too cold! It may result in reduced effectiveness.'
            elif self._driver_battery_last_info_time is None:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            else:
                battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

        if error_msg is not None:
            rospy.logerr_throttle_identical(10.0, f'[{rospy.get_name()}] {error_msg}')

        bat_pub.publish(battery_msg)

    def _count_mean(
        self,
        label: Union[rospy.Publisher, str],
        new_val: float,
        mean_dict: dict,
        hist_dict: Dict[Union[rospy.Publisher, str], list],
    ) -> float:
        # Updates the average by adding the newest and removing the oldest component of mean value,
        # in order to avoid recalculating the entire sum every time.
        mean_dict[label] += (new_val - hist_dict[label][0]) / self._mean_length

        hist_dict[label].pop(0)
        hist_dict[label].append(new_val)

        return mean_dict[label]

    @staticmethod
    def _read_file(path: str) -> int:
        with open(path, 'r') as file:
            data = file.read().rstrip()

        return int(data)

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main():
    adc_node = ADCNode('adc_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
