#!/usr/bin/python3

import math

import rospy

from sensor_msgs.msg import BatteryState

from panther_msgs.msg import DriverState

BAT02_DETECT_THRESH = 3.03

class ADCNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        loop_rate = rospy.get_param('~loop_rate', 20)
        
        self._V_driv_front = float('nan')
        self._V_driv_rear = float('nan')
        self._I_driv_front = float('nan')
        self._I_driv_rear = float('nan')
        
        self._A = 298.15
        self._B = 3977.0
        self._R1 = 10000.0
        self._R0 = 10000.0
        self._u_supply = 3.28

        self._battery_count = self._check_battery_count()

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        self._battery_driv_sub = rospy.Subscriber('motor_controllers_state', DriverState, self._battery_driv_cb)

        self._battery_publisher = rospy.Publisher('battery', BatteryState, queue_size=1)

        if self._battery_count == 2:
            self._battery_1_publisher = rospy.Publisher('battery_1', BatteryState, queue_size=1)
            self._battery_2_publisher = rospy.Publisher('battery_2', BatteryState, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        rospy.Timer(rospy.Duration(1.0 / loop_rate), self._battery_timer_cb)

        rospy.loginfo(f'[{rospy.get_name()}] Battery count: {self._battery_count}')
        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _battery_driv_cb(self, msg) -> None:
        self._V_driv_front = msg.front.voltage
        self._V_driv_rear = msg.front.current
        self._I_driv_front = msg.rear.voltage
        self._I_driv_rear = msg.rear.current

    def _battery_timer_cb(self, *args) -> None:
        try:
            V_bat_1 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device1/in_voltage0_raw", LSB=0.02504255, offset=0
            )
            V_bat_2 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device1/in_voltage3_raw", LSB=0.02504255, offset=0
            )
            V_temp_bat_1 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device0/in_voltage1_raw", LSB=0.002, offset=0
            )
            V_temp_bat_2 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device0/in_voltage0_raw", LSB=0.002, offset=0
            )
            I_charge_bat_1 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device0/in_voltage3_raw", LSB=0.005, offset=0
            )
            I_charge_bat_2 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device0/in_voltage2_raw", LSB=0.005, offset=0
            )
            I_bat_1 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device1/in_voltage2_raw", LSB=0.04, offset=625
            )
            I_bat_2 = self._get_adc_measurement(
                path="/sys/bus/iio/devices/iio:device1/in_voltage1_raw", LSB=0.04, offset=625
            )
        except:
            rospy.logerr(f'[{rospy.get_name()}] Battery ADC measurement error excep')
            return
        
        if self._battery_count == 2:
            temp_bat_1 = self._voltage_to_deg(V_temp_bat_1)
            temp_bat_2 = self._voltage_to_deg(V_temp_bat_2)

            self._publish_battery_msg(
                self._battery_1_publisher, True, V_bat_1, temp_bat_1, -I_bat_1 + I_charge_bat_1
            )
            self._publish_battery_msg(
                self._battery_2_publisher, True, V_bat_2, temp_bat_2, -I_bat_2 + I_charge_bat_2
            )

            V_bat_avereage = (V_bat_1 + V_bat_2) / 2.0
            temp_average = (temp_bat_1 + temp_bat_2) / 2.0
            I_bat_average = (I_bat_1 + I_bat_2) / 2.0
            I_charge_bat_average = (I_charge_bat_1 + I_charge_bat_2) / 2.0

            self._publish_battery_msg(
                self._battery_publisher, True, V_bat_avereage, temp_average, -I_bat_average + I_charge_bat_average
            )

        else: 
            temp_bat_1 = self._voltage_to_deg(V_temp_bat_1)
            self._publish_battery_msg(self._battery_publisher, True, V_bat_1, temp_bat_1, I_bat_1)

    def _check_battery_count(self) -> int:
        try:
            V_temp_bat_2 = self._get_adc_measurement(
                    path="/sys/bus/iio/devices/iio:device0/in_voltage0_raw", LSB=0.002, offset=0
                )
        except:
            rospy.logerr(f'[{rospy.get_name()}] Battery ADC measurement error excep')
        
        return 1 if V_temp_bat_2 > BAT02_DETECT_THRESH else 2

    def _get_adc_measurement(self, path, offset, LSB) -> float:
        raw_value = self._read_file(path)
        value = (raw_value - offset) * LSB

        return value

    def _voltage_to_deg(self, V_temp) -> float:
        if V_temp == 0 or V_temp >= self._u_supply:
            rospy.logerr(f'[{rospy.get_name()}] Temperature measurement error')
            return float('nan')

        R_therm = (V_temp * self._R1) / (self._u_supply - V_temp)

        return (self._A * self._B / (self._A * math.log(R_therm / self._R0) + self._B)) - 273.15

    @staticmethod
    def _read_file(path) -> int:
        with open(path, 'r') as file:
            data = file.read().rstrip()
        file.close()

        return int(data)

    @staticmethod
    def _publish_battery_msg(bat_pub, present, V_bat=float('nan'), temp_bat=float('nan'), I_bat=float('nan')) -> None:
        battery_msg = BatteryState()
        if present:
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
        else:
            battery_msg.header.stamp = rospy.Time.now()
            battery_msg.voltage = float('nan')
            battery_msg.temperature = float('nan')
            battery_msg.current = float('nan')
            battery_msg.percentage = float('nan')
            battery_msg.capacity = float('nan')
            battery_msg.design_capacity = float('nan')
            battery_msg.charge = float('nan')
            battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
            battery_msg.present = False

        bat_pub.publish(battery_msg)


def main():
    adc_node = ADCNode('adc_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
