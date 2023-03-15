#!/usr/bin/python3

import math

import rospy

from sensor_msgs.msg import BatteryState

from panther_msgs.msg import DriverState, IOState


class ADCNode:
    BAT02_DETECT_THRESH = 3.03

    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._V_driv_front = None
        self._V_driv_rear = None
        self._I_driv_front = None
        self._I_driv_rear = None

        self._A = 298.15
        self._B = 3977.0
        self._R1 = 10000.0
        self._R0 = 10000.0
        self._u_supply = 3.28

        self._battery_count = self._check_battery_count()
        self._battery_charging = True  # const for now, later this will be evaluated

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

        # -------------------------------
        #   Timers
        # -------------------------------

        # Running at 10 Hz
        self._battery_timer = rospy.Timer(rospy.Duration(0.1), self._battery_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Battery count: {self._battery_count}')
        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _motor_controllers_state_cb(self, driver_state: DriverState) -> None:
        self._V_driv_front = driver_state.front.voltage
        self._V_driv_rear = driver_state.front.current
        self._I_driv_front = driver_state.rear.voltage
        self._I_driv_rear = driver_state.rear.current

    def _io_state_cb(self, io_state: IOState) -> None:
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
                self._battery_1_pub, V_bat_1, temp_bat_1, -I_bat_1 + I_charge_bat_1
            )
            self._publish_battery_msg(
                self._battery_2_pub, V_bat_2, temp_bat_2, -I_bat_2 + I_charge_bat_2
            )

            V_bat_avereage = (V_bat_1 + V_bat_2) / 2.0
            temp_bat_average = (temp_bat_1 + temp_bat_2) / 2.0
            I_bat_sum = I_bat_1 + I_bat_2
            I_charge_bat_average = (I_charge_bat_1 + I_charge_bat_2) / 2.0

            self._publish_battery_msg(
                self._battery_pub,
                V_bat_avereage,
                temp_bat_average,
                -I_bat_sum + I_charge_bat_average,
            )

        else:
            temp_bat_1 = self._voltage_to_deg(V_temp_bat_1)
            self._publish_battery_msg(
                self._battery_pub, V_bat_1, temp_bat_1, -(I_bat_1 + I_bat_2) + I_charge_bat_1
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
        self, bat_pub: rospy.Publisher, V_bat: float, temp_bat: float, I_bat: float
    ) -> None:
        battery_msg = BatteryState()
        battery_msg.header.stamp = rospy.Time.now()
        battery_msg.voltage = V_bat
        battery_msg.temperature = temp_bat
        battery_msg.current = I_bat
        battery_msg.percentage = (battery_msg.voltage - 32.0) / 10.0
        battery_msg.capacity = 20.0
        battery_msg.design_capacity = 20.0
        battery_msg.charge = battery_msg.percentage * battery_msg.design_capacity
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery_msg.present = True

        # check battery status
        if self._charger_connected:
            if battery_msg.percentage >= 1.0:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif self._battery_charging:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        bat_pub.publish(battery_msg)

    @staticmethod
    def _read_file(path: str) -> int:
        with open(path, 'r') as file:
            data = file.read().rstrip()

        return int(data)


def main():
    adc_node = ADCNode('adc_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
