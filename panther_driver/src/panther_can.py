#!/usr/bin/python3

import canopen
from dataclasses import dataclass
from threading import Lock
from typing import Generator, List

import rospy


@dataclass
class ControllerChannels:
    LEFT_WHEEL: int = 2
    RIGHT_WHEEL: int = 1
    VOLT_CHANNEL: int = 2

    def __setattr__(self, name, value):
        raise AttributeError(f'can\'t reassign constant {name}')


class MotorController:
    HEARTBEAT_TIMEOUT_S = 0.3

    def __init__(self, can_node_id: int, eds_file: str, lock: Lock) -> None:
        self._lock = lock
        self._can_node_id = can_node_id
        self._heartbeat_timestamp = rospy.Time.now()

        self._wheel_pos = [0.0, 0.0]
        self._wheel_vel = [0.0, 0.0]
        self._wheel_curr = [0.0, 0.0]
        self._battery_data = [0.0, 0.0]  # I, V
        self._driver_temperature = 0.0
        self._runtime_stat_flags = [0, 0]
        self._fault_flags = 0
        self._script_flags = 0

        self._wheels = [
            ControllerChannels.LEFT_WHEEL,
            ControllerChannels.RIGHT_WHEEL,
        ]

        self._can_node = canopen.RemoteNode(can_node_id, eds_file)

    @property
    def connection_error(self) -> bool:
        if self._can_node.nmt.state != 'OPERATIONAL':
            return True

        return rospy.Time.now() - self._heartbeat_timestamp > rospy.Duration(
            self.HEARTBEAT_TIMEOUT_S
        )

    @property
    def driver_temperature(self) -> float:
        with self._lock:
            try:
                self._driver_temperature = float(self._can_node.sdo['Qry_TEMP'][1].raw)
            except canopen.SdoCommunicationError:
                pass

        return self._driver_temperature

    @property
    def battery_data(self) -> List[float]:
        with self._lock:
            try:
                # division by 10 is needed according to documentation
                amps = sum(
                    [
                        float(self._can_node.sdo['Qry_BATAMPS'][wheel].raw) / 10.0
                        for wheel in self._wheels
                    ]
                )
                voltage = (
                    float(self._can_node.sdo['Qry_VOLTS'][ControllerChannels.VOLT_CHANNEL].raw)
                    / 10.0
                )

                self._battery_data = [amps, voltage]
            except canopen.SdoCommunicationError:
                pass

        return self._battery_data

    def setup(self, network: canopen.Network) -> None:
        network.add_node(self._can_node)
        self._can_node.nmt.add_hearbeat_callback(self._heartbeat_cb)

    def write_wheels_enc_velocity(self, vel: List[float]) -> None:
        with self._lock:
            for i, wheel in enumerate(self._wheels):
                try:
                    self._can_node.sdo['Cmd_CANGO'][wheel].raw = vel[i]
                except:
                    pass

    def _heartbeat_cb(self, nmt_state: int) -> None:
        self._heartbeat_timestamp = rospy.Time.now()


class MotorControllerPDO(MotorController):
    def __init__(self, can_node_id: int, eds_file: str, lock: Lock) -> None:
        super().__init__(can_node_id, eds_file, lock)

    @property
    def wheel_pos(self) -> List[float]:
        return self._wheel_pos

    @property
    def wheel_vel(self) -> List[float]:
        return self._wheel_vel

    @property
    def wheel_curr(self) -> List[float]:
        return self._wheel_curr

    @property
    def runtime_stat_flags(self) -> List[float]:
        return self._runtime_stat_flags

    @property
    def fault_flags(self) -> float:
        return self._fault_flags

    @property
    def script_flags(self) -> float:
        return self._script_flags

    def setup(self, network: canopen.Network) -> None:
        super().setup(network)

        number_of_retries = 0
        while not self._configure_tpdo() and not rospy.is_shutdown():
            if number_of_retries >= 5:
                rospy.logerr(
                    f'[{rospy.get_name()}] Failed to configure TPDO. \n'
                    f'[{rospy.get_name()}] Check CAN connection. Shutting down.'
                )
                rospy.signal_shutdown('Failed to configure TPDO')
                return

            rospy.loginfo(f'[{rospy.get_name()}] Retrying TPDO configuration.')
            number_of_retries += 1
            rospy.sleep(1.0)

        self.restart_script()
        rospy.sleep(1.0)

    def _configure_tpdo(self) -> bool:
        with self._lock:
            try:
                self._can_node.tpdo.read()
            except canopen.SdoCommunicationError:
                rospy.loginfo(
                    f"[{rospy.get_name()}] Failed to read {'front' if self._can_node_id == 1 else 'rear'} "
                    f"controller TPDO configuration."
                )
                return False

            self._can_node.tpdo[1].add_callback(self._update_wheels_pos_cb)
            self._can_node.tpdo[2].add_callback(self._update_wheels_vel_cb)
            self._can_node.tpdo[3].add_callback(self._update_wheels_current_cb)
            self._can_node.tpdo[4].add_callback(self._update_wheels_flags_cb)

            rospy.loginfo(
                f'[{rospy.get_name()}] {"Front" if self._can_node_id == 1 else "Rear"} '
                f'controller TPDO configured successfully.'
            )

        return True

    def restart_script(self) -> bool:
        with self._lock:
            try:
                self._can_node.sdo['Cmd_BRUN'].raw = 2
            except:
                rospy.logdebug(
                    f'[{rospy.get_name()}] PantherCAN: SdoCommunicationError '
                    f'occurred while restarting roboteq script'
                )
                return False

        return True

    def _update_wheels_pos_cb(self, message: canopen.pdo.Map) -> None:
        self._wheel_pos = [m.raw for m in reversed(message)]

    def _update_wheels_vel_cb(self, message: canopen.pdo.Map) -> None:
        self._wheel_vel = [m.raw for m in reversed(message)]

    def _update_wheels_current_cb(self, message: canopen.pdo.Map) -> None:
        # division by 10 is needed according to documentation
        self._wheel_curr = [m.raw / 10.0 for m in reversed(message)]

    def _update_wheels_flags_cb(self, message: canopen.pdo.Map) -> None:
        self._fault_flags = message[0].data[0]
        self._script_flags = message[0].data[2]
        self._runtime_stat_flags = [m for m in message[1].data[:2]]


class MotorControllerSDO(MotorController):
    def __init__(self, can_node_id: int, eds_file: str, lock: Lock) -> None:
        super().__init__(can_node_id, eds_file, lock)

    @property
    def wheel_pos(self) -> List[float]:
        with self._lock:
            try:
                self._wheel_pos = [
                    self._can_node.sdo['Qry_ABCNTR'][wheel].raw for wheel in self._wheels
                ]
            except canopen.SdoCommunicationError:
                pass

        return self._wheel_pos

    @property
    def wheel_vel(self) -> List[float]:
        with self._lock:
            try:
                self._wheel_vel = [
                    self._can_node.sdo['Qry_ABSPEED'][wheel].raw for wheel in self._wheels
                ]
            except canopen.SdoCommunicationError:
                pass

        return self._wheel_vel

    @property
    def wheel_curr(self) -> List[float]:
        with self._lock:
            try:
                # division by 10 is needed according to documentation
                self._wheel_curr = [
                    self._can_node.sdo['Qry_MOTAMPS'][wheel].raw / 10.0 for wheel in self._wheels
                ]
            except canopen.SdoCommunicationError:
                pass

        return self._wheel_curr

    @property
    def runtime_stat_flags(self) -> List[float]:
        with self._lock:
            try:
                self._runtime_stat_flags = [
                    int.from_bytes(self._can_node.sdo['Qry_MOTFLAGS'][wheel].data, 'little')
                    for wheel in self._wheels
                ]
            except canopen.SdoCommunicationError:
                pass

        return self._runtime_stat_flags

    @property
    def fault_flags(self) -> float:
        with self._lock:
            try:
                self._fault_flags = self._can_node.sdo['Qry_FLTFLAG'].raw
            except canopen.SdoCommunicationError:
                pass

        return self._fault_flags

    @property
    def script_flags(self) -> float:
        # mockup for compatybility with new driver version
        return self._script_flags


class PantherCAN:
    def __init__(self, eds_file: str, can_interface: str, use_pdo: bool) -> None:
        self._lock = Lock()

        motor_controller_factory = MotorControllerPDO if use_pdo else MotorControllerSDO

        self._motor_controllers = [
            motor_controller_factory(1, eds_file, self._lock),  # front
            motor_controller_factory(2, eds_file, self._lock),  # rear
        ]

        self._network = canopen.Network()
        self._network.connect(channel=can_interface, bustype='socketcan')

        for controller in self._motor_controllers:
            controller.setup(self._network)

        self._robot_driver_initialized = False
        self._can_init_time = rospy.Time.now()

        rospy.loginfo(f'[{rospy.get_name()}] Connected to the CAN bus.')

    def can_connection_error(self) -> Generator[bool, None, None]:
        for controller in self._motor_controllers:
            if self._robot_driver_initialized:
                yield controller.connection_error
            else:
                self._robot_driver_initialized = (
                    rospy.Time.now() - self._can_init_time > rospy.Duration(10.0)
                )
                yield False

    def restart_roboteq_script(self) -> bool:
        for controller in self._motor_controllers:
            if not isinstance(controller, MotorControllerPDO):
                raise TypeError("An SDO object does not require script reset.")

            if not controller.restart_script():
                return False

        return True

    def set_wheels_enc_velocity(self, velocity: List[float]) -> None:
        vel = [velocity[:2], velocity[2:]]
        for controller, v in zip(self._motor_controllers, vel):
            controller.write_wheels_enc_velocity(v)

    def query_wheels_enc_pose(self) -> Generator[float, None, None]:
        for motor_controller in self._motor_controllers:
            for wheel_pos in motor_controller.wheel_pos:
                yield wheel_pos

    def query_wheels_enc_velocity(self) -> Generator[float, None, None]:
        for motor_controller in self._motor_controllers:
            for wheel_vel in motor_controller.wheel_vel:
                yield wheel_vel

    def query_motor_current(self) -> Generator[float, None, None]:
        for motor_controller in self._motor_controllers:
            for wheel_curr in motor_controller.wheel_curr:
                yield wheel_curr

    def query_battery_data(self) -> Generator[float, None, None]:
        for motor_controller in self._motor_controllers:
            for battery_data in motor_controller.battery_data:
                yield battery_data

    def query_driver_temperature_data(self) -> Generator[float, None, None]:
        for motor_controller in self._motor_controllers:
            yield motor_controller.driver_temperature

    def query_runtime_stat_flag(self) -> Generator[int, None, None]:
        for motor_controller in self._motor_controllers:
            for runtime_flag in motor_controller.runtime_stat_flags:
                yield runtime_flag

    def query_fault_flags(self) -> Generator[int, None, None]:
        for motor_controller in self._motor_controllers:
            yield motor_controller.fault_flags

    def query_script_flags(self) -> Generator[int, None, None]:
        for motor_controller in self._motor_controllers:
            yield motor_controller.script_flags
