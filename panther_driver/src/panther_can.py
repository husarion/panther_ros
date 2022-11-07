#!/usr/bin/python3

import canopen
from collections.abc import Generator
from dataclasses import dataclass
from time import time
from threading import Lock

import rospy


@dataclass
class ControllerChannels:
    LEFT_WHEEL = 2
    RIGHT_WHEEL = 1
    VOLT_CHANNEL = 2
    AMP_CHANNEL = 1 

    def __setattr__(self, name, value):
        raise AttributeError(f'can\'t reassign constant {name}')


@dataclass
class MotorController:
    wheel_pos = [0.0, 0.0]
    wheel_vel = [0.0, 0.0]
    wheel_curr = [0.0, 0.0]
    battery_data = [0.0, 0.0] # V, I
    runtime_stat_flag = [0, 0]
    fault_flags = 0

    def __init__(self, can_node_id, eds_file) -> None:
        self.can_node = canopen.RemoteNode(can_node_id, eds_file)


class PantherCAN:
    def __init__(self, eds_file, can_interface) -> None:
        self._max_err_per_sec = 2
        self._err_times = [0] * self._max_err_per_sec
        
        self._lock = Lock()
        self._network = canopen.Network()

        self._motor_controllers = [
            MotorController(1, eds_file),   # front
            MotorController(2, eds_file)    # rear
        ]
        
        self._network.connect(channel=can_interface, bustype='socketcan')

        self._network.add_node(self._motor_controllers[0].can_node)
        self._network.add_node(self._motor_controllers[1].can_node)
        
        self._channels = ControllerChannels()
        self._wheels = [
            self._channels.LEFT_WHEEL,
            self._channels.RIGHT_WHEEL
        ]

        rospy.loginfo(f'[{rospy.get_name()}] Connected to the CAN bus.')
    
    def can_connection_correct(self) -> bool:
        if time() - self._err_times[-1] >= 2.0:
            return True

        return (self._err_times[-1] - self._err_times[0] >= 1.0)

    def write_wheels_enc_velocity(self, vel: list) -> None:
        with self._lock:
            for motor_controller, enc_vel in zip(self._motor_controllers, [vel[:2], vel[2:]]):
                for i, wheel in enumerate(self._wheels):
                    try:
                        motor_controller.can_node.sdo['Cmd_CANGO'][wheel].raw = enc_vel[i]
                    except:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                            f'occurred while setting wheels velocity'
                        )
                        self._error_handle()

    def query_wheels_enc_pose(self) -> Generator:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate(self._wheels):
                    try:
                        motor_controller.wheel_pos[i] = motor_controller.can_node.sdo['Qry_ABCNTR'][wheel].raw
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                            f'occurred while reading wheels position'
                        )
                        self._error_handle()
                    yield motor_controller.wheel_pos[i]

    def query_wheels_enc_velocity(self) -> Generator:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate(self._wheels):
                    try:
                        motor_controller.wheel_vel[i] = motor_controller.can_node.sdo['Qry_ABSPEED'][wheel].raw
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                            f'occurred while reading wheels velocity'
                        )
                        self._error_handle()
                    yield motor_controller.wheel_vel[i]

    def query_motor_current(self) -> Generator:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate(self._wheels):
                    try:
                        # division by 10 is needed according to documentation
                        motor_controller.wheel_curr[i] = motor_controller.can_node.sdo['Qry_MOTAMPS'][wheel].raw / 10.0
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                            f'occurred while reading motor current'
                        )
                        self._error_handle()
                    yield motor_controller.wheel_curr[i]

    def query_battery_data(self) -> Generator: 
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    # division by 10 is needed according to documentation
                    motor_controller.battery_data[0] = \
                        float(
                            motor_controller.can_node.sdo['Qry_VOLTS'][self._channels.VOLT_CHANNEL].raw
                        ) / 10.0
                    motor_controller.battery_data[1] = \
                        float(
                            motor_controller.can_node.sdo['Qry_BATAMPS'][self._channels.AMP_CHANNEL].raw
                        ) / 10.0
                except canopen.SdoCommunicationError:
                    rospy.logwarn(
                        f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                        f'occurred while reading battery data'
                    )       
                    self._error_handle()
                
                for battery_data in motor_controller.battery_data:
                    yield battery_data

    def query_fault_flags(self) -> Generator:
        with self._lock:
            for motor_controller in self._motor_controllers:
                try:
                    motor_controller.fault_flags = motor_controller.can_node.sdo['Qry_FLTFLAG'].raw
                except canopen.SdoCommunicationError:
                    rospy.logwarn(
                        f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError ' 
                        f'occurred while reading fault flags'
                    )
                    self._error_handle()
                yield motor_controller.fault_flags

    def query_runtime_stat_flag(self) -> Generator:
        with self._lock:
            for motor_controller in self._motor_controllers:
                for i, wheel in enumerate(self._wheels):
                    try:
                        motor_controller.runtime_stat_flag[i] = int.from_bytes(
                            motor_controller.can_node.sdo['Qry_MOTFLAGS'][wheel].data, 'little'
                        ) 
                    except canopen.SdoCommunicationError:
                        rospy.logwarn(
                            f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                            f'occurred while reading runtime status flag'
                        )
                        self._error_handle()
                    yield motor_controller.runtime_stat_flag[i]
        
    def _error_handle(self) -> None:
        self._err_times.append(time())
        self._err_times.pop(0)

    def _turn_on_roboteq_emergency_stop(self) -> None:
        with self._lock:
            for motor_controller in self._motor_controllers:
                motor_controller.can_node.sdo['Cmd_ESTOP'].raw = 1

    def _turn_off_roboteq_emergency_stop(self) -> None:
        with self._lock:
            for motor_controller in self._motor_controllers:
                motor_controller.can_node.sdo['Cmd_MGO'].raw = 1

        