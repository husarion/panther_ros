#!/usr/bin/python3

import math
from typing import List, Tuple, TypeVar
from threading import Lock

import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros

from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from panther_msgs.msg import DriverState, FaultFlag, IOState, RuntimeError, ScriptFlag
from panther_can import PantherCANSDO, PantherCANPDO
from panther_kinematics import PantherDifferential, PantherMecanum
from velocity_smoother import VelocitySmoother


class DriverFlagLogger:
    MAX_LOG_INTERVAL: float = 2.0
    MsgType = TypeVar('MsgType', FaultFlag, RuntimeError, ScriptFlag)
    SUPPRESSED_FLAGS = {'safety_stop_active', 'amps_limit_active'}

    def __init__(self, flag_list: list, msg_type: MsgType) -> None:
        self._flag_list = flag_list
        self._msg_type = msg_type
        self._last_log_time = rospy.Time.now()
        self._last_logged_msg_state = None

    def __call__(self, flag_val: int) -> MsgType:
        faults, msg = self._decode_flag(flag_val)

        if faults and (
            (rospy.Time.now() - self._last_log_time).secs > self.MAX_LOG_INTERVAL
            or self._last_logged_msg_state != flag_val
        ):
            faults_str = ', '.join(faults)
            rospy.logwarn(f'[{rospy.get_name()}] Motor controller faults: {faults_str}')

            self._last_log_time = rospy.Time.now()
            self._last_logged_msg_state = flag_val

        return msg

    def _decode_flag(self, flag_val: int) -> Tuple[List[str], MsgType]:
        msg = self._msg_type()

        faults = [
            self._set_msg_field_and_return_name(field_name, msg)
            for i, field_name in enumerate(self._flag_list)
            if bool(flag_val & 0b00000001 << i)
        ]
        faults = list(set(faults) - self.SUPPRESSED_FLAGS)

        return faults, msg

    def _set_msg_field_and_return_name(self, field_name: str, msg: MsgType) -> str:
        setattr(msg, field_name, True)
        return field_name


class PantherDriverNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._lock = Lock()

        self._eds_file = rospy.get_param('~eds_file')
        self._use_pdo = rospy.get_param('~use_pdo', False)
        self._can_interface = rospy.get_param('~can_interface', 'panther_can')
        self._kinematics_type = rospy.get_param('~kinematics', 'differential')
        self._motor_torque_constant = rospy.get_param('~motor_torque_constant', 2.6149)
        self._gear_ratio = rospy.get_param('~gear_ratio', 30.08)
        self._encoder_resolution = rospy.get_param('~encoder_resolution', 400 * 4)
        self._v_x_var = float(rospy.get_param('~odom_stderr/vel_x', 3.2e-3)) ** 2
        self._v_y_var = float(rospy.get_param('~odom_stderr/vel_y', 3.2e-3)) ** 2
        self._v_yaw_var = float(rospy.get_param('~odom_stderr/vel_yaw', 8.5e-3)) ** 2

        self._publish_tf = rospy.get_param('~publish_tf', True)
        self._publish_odom = rospy.get_param('~publish_odometry', True)
        self._publish_pose = rospy.get_param('~publish_pose', True)
        self._publish_joints = rospy.get_param('~publish_joints', True)
        self._odom_frame = rospy.get_param('~odom_frame', 'odom')
        self._base_link_frame = rospy.get_param('~base_link_frame', 'base_link')

        robot_width = rospy.get_param('~wheel_separation', 0.697)
        robot_length = rospy.get_param('~robot_length', 0.44)
        wheel_radius = rospy.get_param('~wheel_radius', 0.1825)

        max_vel_x = rospy.get_param('~max_vel_x', 2.0)
        max_vel_y = rospy.get_param('~max_vel_y', 2.0)
        max_vel_theta = rospy.get_param('~max_vel_theta', 4.0)
        acc_lim_x = rospy.get_param('~acc_lim_x', 1.0)
        acc_lim_y = rospy.get_param('~acc_lim_y', 1.0)
        acc_lim_theta = rospy.get_param('~acc_lim_theta', 1.57)
        decel_lim_x = rospy.get_param('~decel_lim_x', 1.5)
        decel_lim_y = rospy.get_param('~decel_lim_y', 1.5)
        decel_lim_theta = rospy.get_param('~decel_lim_theta', 2.3)
        emergency_decel_lim_x = rospy.get_param('~emergency_decel_lim_x', 2.7)
        emergency_decel_lim_y = rospy.get_param('~emergency_decel_lim_y', 2.7)
        emergency_decel_lim_theta = rospy.get_param('~emergency_decel_lim_theta', 5.74)

        self._wheels_joints_names = [
            'fl_wheel_joint',
            'fr_wheel_joint',
            'rl_wheel_joint',
            'rr_wheel_joint',
        ]

        self._main_timer_period = 1.0 / 15.0  # freq. 15 Hz
        self._driver_state_timer_period = 1.0 / 10.0  # freq. 10 Hz
        self._safety_timer_period = 1.0 / 20.0  # freq. 20 Hz
        self._time_last = rospy.Time.now()
        self._motor_off_last_time = rospy.Time.now()
        self._cmd_vel_command_last_time = rospy.Time.now()
        self._cmd_vel_timeout = 0.2
        self._motor_power_on_timeout = 2.3
        self._motor_power_on_error_timeout = 2.5

        self._robot_pos = [0.0, 0.0, 0.0]  # x,  y,  yaw
        self._robot_vel = [0.0, 0.0, 0.0]  # lin_x, lin_y, ang_z
        self._robot_orientation_quat = [0.0, 0.0, 0.0, 0.0]  # qx, qy, qz, qw
        self._wheels_ang_pos = [0.0, 0.0, 0.0, 0.0]
        self._wheels_ang_vel = [0.0, 0.0, 0.0, 0.0]
        self._motors_effort = [0.0, 0.0, 0.0, 0.0]

        self._e_stop_cliented = False
        self._motor_power = False
        self._stop_motors = True

        self._velocity_smoother = VelocitySmoother(
            max_vel_x,
            max_vel_y,
            max_vel_theta,
            acc_lim_x,
            acc_lim_y,
            acc_lim_theta,
            decel_lim_x,
            decel_lim_y,
            decel_lim_theta,
            emergency_decel_lim_x,
            emergency_decel_lim_y,
            emergency_decel_lim_theta,
        )

        # -------------------------------
        #   Kinematic type
        # -------------------------------

        assert self._kinematics_type in [
            'differential',
            'mecanum',
        ], f'[{rospy.get_name()}] The kinematics type is incorrect: {self._kinematics_type}'

        if self._kinematics_type == 'differential':
            self._panther_kinematics = PantherDifferential(
                robot_width=robot_width,
                robot_length=robot_length,
                wheel_radius=wheel_radius,
                encoder_resolution=self._encoder_resolution,
                gear_ratio=self._gear_ratio,
            )
        else:
            self._panther_kinematics = PantherMecanum(
                robot_width=robot_width,
                robot_length=robot_length,
                wheel_radius=wheel_radius,
                encoder_resolution=self._encoder_resolution,
                gear_ratio=self._gear_ratio,
            )

        # -------------------------------
        #   CAN interface
        # -------------------------------

        if self._use_pdo:
            self._panther_can = PantherCANPDO(
                eds_file=self._eds_file, can_interface=self._can_interface
            )
        else:
            self._panther_can = PantherCANSDO(
                eds_file=self._eds_file, can_interface=self._can_interface
            )
        rospy.sleep(4.0)

        # -------------------------------
        #   Publishers
        # -------------------------------

        if self._publish_joints:
            self._joint_state_msg = JointState()
            self._joint_state_msg.header.frame_id = self._base_link_frame
            self._joint_state_msg.name = self._wheels_joints_names
            self._joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

        if self._publish_pose:
            self._pose_msg = Pose()
            self._pose_pub = rospy.Publisher('pose', Pose, queue_size=1)

        if self._publish_odom:
            self._odom_msg = Odometry()
            self._odom_msg.pose.covariance = [0.1 if (i % 7) == 0 else 0.0 for i in range(36)]

            not_used_var = 1e6
            self._odom_msg.twist.covariance = [self._v_x_var, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, self._v_y_var, 0.0, 0.0, 0.0, 0.0, 
                                               0.0, 0.0, not_used_var, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, not_used_var, 0.0, 0.0, 
                                               0.0, 0.0, 0.0, 0.0, not_used_var, 0.0, 
                                               0.0, 0.0, 0.0, 0.0, 0.0, self._v_yaw_var]
            self._odom_msg.header.frame_id = self._odom_frame
            self._odom_msg.child_frame_id = self._base_link_frame
            self._odom_pub = rospy.Publisher('odom/wheels', Odometry, queue_size=1)

        self._driver_state_msg = DriverState()
        self._driver_state_msg.front.left_motor.motor_joint_name = self._wheels_joints_names[0]
        self._driver_state_msg.front.right_motor.motor_joint_name = self._wheels_joints_names[1]
        self._driver_state_msg.rear.left_motor.motor_joint_name = self._wheels_joints_names[2]
        self._driver_state_msg.rear.right_motor.motor_joint_name = self._wheels_joints_names[3]
        self._driver_state_pub = rospy.Publisher(
            'driver/motor_controllers_state', DriverState, queue_size=1
        )

        driver_fault_flags = [
            'overheat',
            'overvoltage',
            'undervoltage',
            'short_circuit',
            'emergency_stop',
            'motor_or_sensor_setup_fault',
            'mosfet_failure',
            'default_config_loaded_at_startup',
        ]
        driver_runtime_errors = [
            'amps_limit_active',
            'motor_stall',
            'loop_error',
            'safety_stop_active',
            'forward_limit_triggered',
            'reverse_limit_triggered',
            'amps_trigger_activated',
        ]
        driver_script_flags = [
            'loop_error',
            'encoder_disconected',
            'amp_limiter',
        ]

        self._driver_flag_loggers = {
            'fault_flags': DriverFlagLogger(driver_fault_flags, FaultFlag),
            'runtime_errors': DriverFlagLogger(driver_runtime_errors, RuntimeError),
            'script_flags': DriverFlagLogger(driver_script_flags, ScriptFlag),
        }

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb, queue_size=1)
        self._e_stop_sub = rospy.Subscriber('hardware/e_stop', Bool, self._e_stop_cb, queue_size=1)
        self._io_state_sub = rospy.Subscriber(
            'hardware/io_state', IOState, self._io_state_cb, queue_size=1
        )

        # -------------------------------
        #   Service clients
        # -------------------------------

        self._e_stop_client = rospy.ServiceProxy('hardware/e_stop_trigger', Trigger)

        # -------------------------------
        #   Service servers
        # -------------------------------

        if self._use_pdo:
            self._reset_roboteq_script_server = rospy.Service(
                'driver/reset_roboteq_script', Trigger, self._reset_roboteq_script_cb
            )

        # -------------------------------
        #   Transforms
        # -------------------------------

        if self._publish_tf:
            self._tf_stamped = TransformStamped()
            self._tf_stamped.header.frame_id = self._odom_frame
            self._tf_stamped.child_frame_id = self._base_link_frame
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        # -------------------------------
        #   Timers
        # -------------------------------

        self._main_timer = rospy.Timer(rospy.Duration(self._main_timer_period), self._main_timer_cb)
        self._driver_state_timer = rospy.Timer(
            rospy.Duration(self._driver_state_timer_period), self._driver_state_timer_cb
        )
        self._safety_timer = rospy.Timer(
            rospy.Duration(self._safety_timer_period), self._safety_timer_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _main_timer_cb(self, *args) -> None:
        time_now = rospy.Time.now()
        dt = (time_now - self._time_last).to_sec()
        self._time_last = time_now

        if (time_now - self._cmd_vel_command_last_time) > rospy.Duration(self._cmd_vel_timeout):
            self._cmd_vel = Twist()
            emergency_breaking = True
        else:
            emergency_breaking = False

        if not self._stop_motors:
            cmd_vel = self._velocity_smoother.get_smooth_velocity(
                self._cmd_vel, self._main_timer_period, emergency_breaking
            )
            self._panther_kinematics.inverse_kinematics(cmd_vel)
            self._panther_can.write_wheels_enc_velocity(self._panther_kinematics.wheels_enc_speed)
        else:
            self._panther_can.write_wheels_enc_velocity([0.0, 0.0, 0.0, 0.0])

        wheel_enc_pos = self._panther_can.query_wheels_enc_pose()
        wheel_enc_vel = self._panther_can.query_wheels_enc_velocity()
        wheel_enc_curr = self._panther_can.query_motor_current()

        # convert tics to rad
        self._wheels_ang_pos = [
            (2.0 * math.pi) * (pos / (self._encoder_resolution * self._gear_ratio))
            for pos in wheel_enc_pos
        ]
        # convert RPM to rad/s
        self._wheels_ang_vel = [
            (2.0 * math.pi / 60.0) * (vel / self._gear_ratio) for vel in wheel_enc_vel
        ]
        # convert A to Nm
        self._motors_effort = [
            enc_curr * self._motor_torque_constant for enc_curr in wheel_enc_curr
        ]

        try:
            self._robot_pos, self._robot_vel = self._panther_kinematics.forward_kinematics(
                *self._wheels_ang_vel, dt=dt
            )
        except:
            rospy.logwarn(f'[{rospy.get_name()}] Could not get robot pose')

        self._robot_orientation_quat = quaternion_from_euler(0.0, 0.0, self._robot_pos[2])

        if self._publish_joints:
            self._publish_joint_state_cb()
        if self._publish_pose:
            self._publish_pose_cb()
        if self._publish_odom:
            self._publish_odom_cb()
        if self._publish_tf:
            self._publish_tf_cb()

    def _driver_state_timer_cb(self, *args) -> None:
        # wait for motor drivers to power on before publishing their state
        if rospy.Time.now() - self._motor_off_last_time < rospy.Duration(
            self._motor_power_on_timeout
        ):
            return

        if not self._motor_on:
            return

        [
            self._driver_state_msg.front.current,
            self._driver_state_msg.front.voltage,
            self._driver_state_msg.rear.current,
            self._driver_state_msg.rear.voltage,
        ] = self._panther_can.query_battery_data()

        [
            self._driver_state_msg.front.temperature,
            self._driver_state_msg.rear.temperature,
        ] = self._panther_can.query_driver_temperature_data()

        [self._driver_state_msg.front.fault_flag, self._driver_state_msg.rear.fault_flag] = [
            self._driver_flag_loggers['fault_flags'](flag_val)
            for flag_val in self._panther_can.query_fault_flags()
        ]

        (
            self._driver_state_msg.front.fault_flag.can_net_err,
            self._driver_state_msg.rear.fault_flag.can_net_err,
        ) = self._panther_can.can_connection_error()

        [self._driver_state_msg.front.script_flag, self._driver_state_msg.rear.script_flag] = [
            self._driver_flag_loggers['script_flags'](flag_val)
            for flag_val in self._panther_can.query_script_flags()
        ]

        [
            self._driver_state_msg.front.right_motor.runtime_error,
            self._driver_state_msg.front.left_motor.runtime_error,
            self._driver_state_msg.rear.right_motor.runtime_error,
            self._driver_state_msg.rear.left_motor.runtime_error,
        ] = [
            self._driver_flag_loggers['runtime_errors'](flag_val)
            for flag_val in self._panther_can.query_runtime_stat_flag()
        ]

        self._driver_state_pub.publish(self._driver_state_msg)

    def _safety_timer_cb(self, *args) -> None:
        with self._lock:
            e_stop_cliented = self._e_stop_cliented

        if any(self._panther_can.can_connection_error()):
            if not e_stop_cliented:
                self._trigger_panther_e_stop()
                self._stop_motors = True

            if not self._motor_power:
                rospy.logwarn_throttle(
                    60.0, f'[{rospy.get_name()}] Motor controllers are not powered on'
                )
                self._motor_off_last_time = rospy.Time.now()
            else:
                # wait for motor drivers to power on before logging an error
                if rospy.Time.now() - self._motor_off_last_time < rospy.Duration(
                    self._motor_power_on_error_timeout
                ):
                    return
                rospy.logerr_throttle(
                    10.0,
                    f'[{rospy.get_name()}] Unable to communicate with motor controllers (CAN interface connection failure)',
                )
        elif self._e_stop_cliented:
            self._stop_motors = True
        else:
            self._stop_motors = False

    def _e_stop_cb(self, data: Bool) -> None:
        with self._lock:
            if data.data:
                self._velocity_smoother.reset()
            self._e_stop_cliented = data.data

    def _cmd_vel_cb(self, data: Twist) -> None:
        self._cmd_vel = data
        self._cmd_vel_command_last_time = rospy.Time.now()

    def _io_state_cb(self, io_state: IOState) -> None:
        self._motor_power = io_state.motor_power

    def _reset_roboteq_script_cb(self, req: TriggerRequest) -> TriggerResponse:
        try:
            if self._panther_can.restart_roboteq_script():
                return TriggerResponse(True, 'Roboteq script reset successful.')
        except Exception as err:
            return TriggerResponse(False, f'Roboteq script reset failed: \n{err}')

        return TriggerResponse(False, f'Roboteq script reset failed')

    def _publish_joint_state_cb(self) -> None:
        self._joint_state_msg.header.stamp = rospy.Time.now()
        self._joint_state_msg.position = [
            math.atan2(math.sin(pos), math.cos(pos)) for pos in self._wheels_ang_pos
        ]
        self._joint_state_msg.velocity = self._wheels_ang_vel
        self._joint_state_msg.effort = self._motors_effort
        self._joint_state_pub.publish(self._joint_state_msg)

    def _publish_pose_cb(self) -> None:
        self._pose_msg.position.x = self._robot_pos[0]
        self._pose_msg.position.y = self._robot_pos[1]
        self._pose_msg.orientation.x = self._robot_orientation_quat[0]
        self._pose_msg.orientation.y = self._robot_orientation_quat[1]
        self._pose_msg.orientation.z = self._robot_orientation_quat[2]
        self._pose_msg.orientation.w = self._robot_orientation_quat[3]
        self._pose_pub.publish(self._pose_msg)

    def _publish_tf_cb(self) -> None:
        self._tf_stamped.header.stamp = rospy.Time.now()
        self._tf_stamped.transform.translation.x = self._robot_pos[0]
        self._tf_stamped.transform.translation.y = self._robot_pos[1]
        self._tf_stamped.transform.translation.z = 0.0
        self._tf_stamped.transform.rotation.x = self._robot_orientation_quat[0]
        self._tf_stamped.transform.rotation.y = self._robot_orientation_quat[1]
        self._tf_stamped.transform.rotation.z = self._robot_orientation_quat[2]
        self._tf_stamped.transform.rotation.w = self._robot_orientation_quat[3]
        self._tf_broadcaster.sendTransform(self._tf_stamped)

    def _publish_odom_cb(self) -> None:
        self._odom_msg.header.stamp = rospy.Time.now()
        self._odom_msg.pose.pose.position.x = self._robot_pos[0]
        self._odom_msg.pose.pose.position.y = self._robot_pos[1]
        self._odom_msg.pose.pose.orientation.x = self._robot_orientation_quat[0]
        self._odom_msg.pose.pose.orientation.y = self._robot_orientation_quat[1]
        self._odom_msg.pose.pose.orientation.z = self._robot_orientation_quat[2]
        self._odom_msg.pose.pose.orientation.w = self._robot_orientation_quat[3]
        self._odom_msg.twist.twist.linear.x = self._robot_vel[0]
        self._odom_msg.twist.twist.linear.y = self._robot_vel[1]
        self._odom_msg.twist.twist.angular.z = self._robot_vel[2]
        self._odom_pub.publish(self._odom_msg)

    def _trigger_panther_e_stop(self) -> bool:
        try:
            response = self._e_stop_client()
            rospy.logwarn_throttle(
                2.0,
                f'[{rospy.get_name()}] Trying to trigger Panther e-stop... Response: {response.success}',
            )
            return response.success

        except rospy.ServiceException as e:
            rospy.logerr(f'[{rospy.get_name()}] Can\'t trigger Panther e-stop... \n{e}')

        return True


def main():
    panther_driver_node = PantherDriverNode('panther_driver_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
