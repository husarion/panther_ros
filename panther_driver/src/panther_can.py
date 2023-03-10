#!/usr/bin/python3

import canopen
from dataclasses import dataclass
from threading import Lock
from typing import Any, Dict, Generator, List

import rospy


@dataclass
class ControllerChannels:
    LEFT_WHEEL: int = 2
    RIGHT_WHEEL: int = 1
    VOLT_CHANNEL: int = 2

    def __setattr__(self, name, value):
        raise AttributeError(f'can\'t reassign constant {name}')


@dataclass
class MotorController:
    def __init__(self, can_node_id: int, eds_file: str) -> None:
        self._last_time_callback = {}

        self.wheel_pos = [0.0, 0.0]
        self.wheel_vel = [0.0, 0.0]
        self.wheel_curr = [0.0, 0.0]
        self.battery_data = [0.0, 0.0] # V, I
        self.runtime_stat_flag = [0, 0]
        self.temperature = 0.0
        self.fault_flags = 0
        self.script_flags = 0

        self.can_node = canopen.RemoteNode(can_node_id, eds_file)

    def __setattr__(self, name: str, value: Any) -> None:
        super().__setattr__(name, value)
        if name not in ['_last_time_callback', 'can_node']:    
            self._last_time_callback[name] = rospy.Time.now()

    @property
    def last_time_callback(self) -> Dict[str, rospy.Time]:
        return self._last_time_callback


class PantherCAN:
    def __init__(self, eds_file: str, can_interface: str) -> None:
        self._lock = Lock()
        self._network = canopen.Network()

        self._motor_controllers = [
            MotorController(1, eds_file),   # front
            MotorController(2, eds_file)    # rear
        ]
        
        self._network.connect(channel=can_interface, bustype='socketcan')

        self._network.add_node(self._motor_controllers[0].can_node)
        self._network.add_node(self._motor_controllers[1].can_node)
        
        self._wheels = [
            ControllerChannels.LEFT_WHEEL,
            ControllerChannels.RIGHT_WHEEL
        ]
        
        self._robot_driver_initialized = False
        self._can_init_time = rospy.Time.now()
        
        rospy.loginfo(f'[{rospy.get_name()}] Connected to the CAN bus.')
    
    def can_connection_error(self) -> Generator[bool, None, None]:
        time_now = rospy.Time.now()
            
        for controller in self._motor_controllers:
            if self._robot_driver_initialized:
                faults = [time_now - cb_time > rospy.Duration(0.2) for cb_time in controller.last_time_callback.values()]
                yield any(faults)
            else:
                self._robot_driver_initialized = time_now - self._can_init_time > rospy.Duration(5.0)
                yield False

    def write_wheels_enc_velocity(self, vel: list) -> None:
        with self._lock:
            for motor_controller, enc_vel in zip(self._motor_controllers, [vel[:2], vel[2:]]):
                for i, wheel in enumerate(self._wheels):
                    try:
                        motor_controller.can_node.sdo['Cmd_CANGO'][wheel].raw = enc_vel[i]
                    except:
                        rospy.logdebug(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                            f'occurred while setting wheels velocity'
                        )

    def query_battery_data(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.battery_data = [
                        sum(
                            [
                                float(motor_controller.can_node.sdo['Qry_BATAMPS'][wheel].raw) / 10.0 
                                for wheel in self._wheels
                            ]
                        ),
                        float(
                            motor_controller.can_node.sdo['Qry_VOLTS'][ControllerChannels.VOLT_CHANNEL].raw
                        ) / 10.0
                    ]
                except canopen.SdoCommunicationError:
                    pass
                
                for battery_data in motor_controller.battery_data:
                    yield battery_data
                    
    def query_driver_temperature_data(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:  
                    motor_controller.temperature = float(motor_controller.can_node.sdo['Qry_TEMP'][1].raw)
                except canopen.SdoCommunicationError:
                    pass 
                
                yield motor_controller.temperature

    def _turn_on_roboteq_emergency_stop(self) -> None:
        with self._lock:
            for motor_controller in self._motor_controllers:
                motor_controller.can_node.sdo['Cmd_ESTOP'].raw = 1

    def _turn_off_roboteq_emergency_stop(self) -> None:
        with self._lock:
            for motor_controller in self._motor_controllers:
                motor_controller.can_node.sdo['Cmd_MGO'].raw = 1


class PantherCANSDO(PantherCAN):
    def __init__(self, eds_file, can_interface):
        super().__init__(eds_file, can_interface)

    def query_wheels_enc_pose(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.wheel_pos = [
                        motor_controller.can_node.sdo['Qry_ABCNTR'][wheel].raw 
                        for wheel in self._wheels
                    ]
                except canopen.SdoCommunicationError:
                    pass

                for pos in motor_controller.wheel_pos:
                    yield pos

    def query_wheels_enc_velocity(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.wheel_vel = [
                        motor_controller.can_node.sdo['Qry_ABSPEED'][wheel].raw
                        for wheel in self._wheels
                    ]
                except canopen.SdoCommunicationError:
                    pass

                for vel in motor_controller.wheel_vel:
                    yield vel

    def query_motor_current(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    # division by 10 is needed according to documentation
                    motor_controller.wheel_curr = [
                        motor_controller.can_node.sdo['Qry_MOTAMPS'][wheel].raw / 10.0
                        for wheel in self._wheels 
                    ]
                except canopen.SdoCommunicationError:
                    pass

                for curr in motor_controller.wheel_curr:
                    yield curr

    def query_fault_flags(self) -> Generator[int, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.fault_flags = motor_controller.can_node.sdo['Qry_FLTFLAG'].raw
                except canopen.SdoCommunicationError:
                    pass

                yield motor_controller.fault_flags

    # mockup for compatybility with new driver version
    def query_script_flags(self) -> Generator[int, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                # dummy assignment (compatybility)
                motor_controller.script_flags = 0
                yield motor_controller.script_flags

    def query_runtime_stat_flag(self) -> Generator[int, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.runtime_stat_flag = [
                        int.from_bytes(motor_controller.can_node.sdo['Qry_MOTFLAGS'][wheel].data, 'little')
                        for wheel in self._wheels
                    ]
                except canopen.SdoCommunicationError:
                    pass
            
                for flag in motor_controller.runtime_stat_flag:
                    yield flag


class PantherCANPDO(PantherCAN):
    def __init__(self, eds_file, can_interface):
        super().__init__(eds_file, can_interface)

        number_of_retries = 0
        while not self._configure_tpdo() and not rospy.is_shutdown():
            if number_of_retries >= 5:
                rospy.logerr(f'[{rospy.get_name()}] Failed to configure TPDO.')
                rospy.logerr(f'[{rospy.get_name()}] Check CAN connection. Shutting down.')
                rospy.signal_shutdown('Failed to configure TPDO')
                return
            rospy.loginfo(f'[{rospy.get_name()}] Retrying TPDO configuration.')
            number_of_retries += 1
            rospy.sleep(1.0)

        self.restart_roboteq_script()

    def query_wheels_enc_pose(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for pos in motor_controller.wheel_pos:
                    yield pos

    def query_wheels_enc_velocity(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for vel in motor_controller.wheel_vel:
                    yield vel

    def query_motor_current(self) -> Generator[float, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for curr in motor_controller.wheel_curr:
                    yield curr / 10.0

    def query_fault_flags(self) -> Generator[int, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                yield motor_controller.fault_flags

    def query_script_flags(self) -> Generator[int, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                yield motor_controller.script_flags

    def query_runtime_stat_flag(self) -> Generator[int, None, None]:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for flag in motor_controller.runtime_stat_flag:
                    yield flag

    def _configure_tpdo(self):
        with self._lock:
            for motor_controller in self._motor_controllers:

                try:
                    motor_controller.can_node.tpdo.read()
                except canopen.SdoCommunicationError:
                    rospy.loginfo(f'[{rospy.get_name()}] Failed to read TPDO configuration.')
                    return False

                motor_controller.can_node.tpdo[1].add_callback(
                    lambda message, motor_controller=motor_controller: self._update_wheels_pos_cb(
                        message, motor_controller
                    )
                )
                motor_controller.can_node.tpdo[2].add_callback(
                    lambda message, motor_controller=motor_controller: self._update_wheels_speed_cb(
                        message, motor_controller
                    )
                )
                motor_controller.can_node.tpdo[3].add_callback(
                    lambda message, motor_controller=motor_controller: self._update_wheels_current_cb(
                        message, motor_controller
                    )
                )
                motor_controller.can_node.tpdo[4].add_callback(
                    lambda message, motor_controller=motor_controller: self._update_wheels_flags_cb(
                        message, motor_controller
                    )
                )

            rospy.loginfo(f'[{rospy.get_name()}] TPDO configured successfuly.')
            return True

    def _update_wheels_pos_cb(self, message, motor_controller):
        motor_controller.wheel_pos = [m.raw for m in reversed(message)]

    def _update_wheels_speed_cb(self, message, motor_controller):
        motor_controller.wheel_vel = [m.raw for m in reversed(message)]

    def _update_wheels_current_cb(self, message, motor_controller):
        motor_controller.wheel_curr = [m.raw for m in reversed(message)]

    def _update_wheels_flags_cb(self, message, motor_controller):
        motor_controller.fault_flags = message[0].data[0]
        motor_controller.script_flags = message[0].data[2]
        motor_controller.runtime_stat_flag = [m for m in message[1].data[:2]]

    def restart_roboteq_script(self) -> bool:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.can_node.sdo['Cmd_BRUN'].raw = 2
                except:
                    rospy.logdebug(
                        f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                        f'occurred while restarting roboteq script'
                    )
                    return False
        return True
