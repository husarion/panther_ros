#!/usr/bin/python3

import math
from time import sleep

import rospy
import tf2_ros

from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from panther_msgs.msg import DriverState, FaultFlag, RuntimeError

from panther_can import PantherCAN
from panther_kinematics import PantherDifferential, PantherMecanum


class PantherDriverNode:
    def __init__(self, name) -> None:
        rospy.init_node(name, anonymous=False)

        self._eds_file = rospy.get_param('~eds_file')
        self._can_interface = rospy.get_param('~can_interface', 'panther_can')
        self._kinematics_type = rospy.get_param('~kinematics', 'differential')
        self._motor_torque_constant = rospy.get_param('~motor_torque_constant', 2.6149)
        self._gear_ratio = rospy.get_param('~gear_ratio', 30.08)
        self._encoder_resolution = rospy.get_param('~encoder_resolution', 400 * 4)

        self._publish_tf = rospy.get_param('~publish_tf', True)
        self._publish_odom = rospy.get_param('~publish_odometry', True)
        self._publish_pose = rospy.get_param('~publish_pose', True)
        self._publish_joints = rospy.get_param('~publish_joints', True)
        self._odom_frame = rospy.get_param('~odom_frame', 'odom')
        self._base_link_frame = rospy.get_param('~base_link_frame', 'base_link')

        robot_width = rospy.get_param('~wheel_separation', 0.697)
        robot_length = rospy.get_param('~robot_length', 0.44)
        wheel_radius = rospy.get_param('~wheel_radius', 0.1825)
        power_factor = rospy.get_param('~power_factor', 0.04166667)

        self._wheels_joints_names = [
            'fl_joint',
            'fr_joint',
            'rl_joint',
            'rr_joint',
        ]

        self._main_timer_period = 1.0 / 15.0           # freq. 15 Hz
        self._driver_state_timer_period = 1.0 / 4.0    # freq.  4 Hz
        self._safety_timer_period = 1.0 / 4.0          # freq.  4 Hz
        self._time_last = rospy.Time.now()
        self._cmd_vel_command_last_time = rospy.Time.now()
        self._cmd_vel_timeout = 0.2
        

        self._robot_x_pos = 0.0
        self._robot_y_pos = 0.0
        self._robot_th_pos = 0.0
        self._wheels_ang_pos = [0.0, 0.0, 0.0, 0.0]
        self._wheels_ang_vel = [0.0, 0.0, 0.0, 0.0]
        self._motors_effort = [0.0, 0.0, 0.0, 0.0]
        self._roboteq_fault_flags = [0, 0]
        self._roboteq_runtime_flags = [0, 0, 0, 0]

        self._estop_triggered = False
        self._stop_cmd_vel_cb = True
        self._state_err_no = 0

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
                power_factor=power_factor
            )
        else:
            self._panther_kinematics = PantherMecanum(
                robot_width=robot_width, 
                robot_length=robot_length, 
                wheel_radius=wheel_radius, 
                encoder_resolution=self._encoder_resolution, 
                gear_ratio=self._gear_ratio, 
                power_factor=power_factor
            )

        # -------------------------------
        #   CAN interface
        # -------------------------------

        self._panther_can = PantherCAN(eds_file=self._eds_file, can_interface=self._can_interface)
        sleep(4)

        # -------------------------------
        #   Publishers & Subscribers
        # -------------------------------

        if self._publish_joints:
            self._joint_state_msg = JointState()
            self._joint_state_msg.header.frame_id = self._base_link_frame
            self._joint_state_msg.name = self._wheels_joints_names
            self._joint_publisher = rospy.Publisher('joint_states', JointState, queue_size=1)

        if self._publish_pose:
            self._pose_msg = Pose()
            self._pose_publisher = rospy.Publisher('pose', Pose, queue_size=1)
        
        if self._publish_odom:
            self._odom_msg = Odometry()
            self._odom_msg.header.frame_id = self._odom_frame
            self._odom_publisher = rospy.Publisher('odom/wheel', Odometry, queue_size=1)

        if self._publish_tf:
            self._tf_stamped = TransformStamped()
            self._tf_stamped.header.frame_id = self._odom_frame
            self._tf_stamped.child_frame_id = self._base_link_frame
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._driver_state_msg = DriverState()
        self._driver_state_msg.front.left_motor.motor_joint_name = self._wheels_joints_names[0]
        self._driver_state_msg.front.right_motor.motor_joint_name = self._wheels_joints_names[1]
        self._driver_state_msg.rear.left_motor.motor_joint_name = self._wheels_joints_names[2]
        self._driver_state_msg.rear.right_motor.motor_joint_name = self._wheels_joints_names[3]
        self._driver_state_publisher = rospy.Publisher('motor_controllers_state', DriverState, queue_size=1)

        rospy.Subscriber('/cmd_vel', Twist, self._cmd_vel_cb, queue_size=1)
        rospy.Subscriber('/panther_hardware/e_stop', Bool, self._estop_cb, queue_size=1)

        # -------------------------------
        #   Services
        # -------------------------------

        self._estop_trigger = rospy.ServiceProxy('/panther_hardware/e_stop_trigger', Trigger)

        # -------------------------------
        #   Timers
        # -------------------------------

        self._main_timer = rospy.Timer(
            rospy.Duration(self._main_timer_period), self._main_timer_cb
        )
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

        if (time_now - self._cmd_vel_command_last_time) < rospy.Duration(secs=self._cmd_vel_timeout):
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
            (2.0 * math.pi / 60.0) * (vel / self._gear_ratio)
            for vel in wheel_enc_vel
        ]
        # convert A to Nm
        self._motors_effort = [
            enc_curr * self._motor_torque_constant * math.copysign(1, enc_vel)
            for enc_vel, enc_curr in zip(wheel_enc_vel, wheel_enc_curr)
        ]

        try:
            self._robot_x_pos, self._robot_y_pos, self._robot_th_pos = \
                self._panther_kinematics.forward_kinematics(*self._wheels_ang_vel, dt=dt) 
        except:
            rospy.logwarn(f'[{rospy.get_name()}] Could not get robot pose')

        self._qx, self._qy, self._qz, self._qw = self.euler_to_quaternion(self._robot_th_pos, 0, 0)

        if self._publish_joints: 
            self._publish_joint_state_cb()
        if self._publish_pose:   
            self._publish_pose_cb()
        if self._publish_odom:   
            self._publish_odom_cb()
        if self._publish_tf:     
            self._publish_tf_cb()


    def _driver_state_timer_cb(self, *args) -> None:
        [
            self._driver_state_msg.front.voltage, 
            self._driver_state_msg.front.current, 
            self._driver_state_msg.rear.voltage, 
            self._driver_state_msg.rear.current,
        ] = self._panther_can.query_battery_data()

        self._roboteq_fault_flags = list(self._panther_can.query_fault_flags())
        self._roboteq_runtime_flags = list(self._panther_can.query_runtime_stat_flag())

        self._driver_state_msg.front.fault_flag = self._decode_fault_flag(self._roboteq_fault_flags[0])
        self._driver_state_msg.rear.fault_flag = self._decode_fault_flag(self._roboteq_fault_flags[1])

        self._driver_state_msg.front.right_motor.runtime_error = \
            self._decode_runtime_flag(self._roboteq_runtime_flags[0]) 
        self._driver_state_msg.front.left_motor.runtime_error = \
            self._decode_runtime_flag(self._roboteq_runtime_flags[1]) 
        self._driver_state_msg.rear.right_motor.runtime_error = \
            self._decode_runtime_flag(self._roboteq_runtime_flags[2]) 
        self._driver_state_msg.rear.left_motor.runtime_error = \
            self._decode_runtime_flag(self._roboteq_runtime_flags[3])         

        self._driver_state_publisher.publish(self._driver_state_msg)

    def _safety_timer_cb(self, *args) -> None:
        if not self._panther_can.can_connection_correct() and not self._estop_triggered:
            self._trigger_panther_estop()
            self._stop_cmd_vel_cb = True

        elif (
            not self._roboteq_state_is_correct([self._roboteq_fault_flags, self._roboteq_runtime_flags]) or
            self._estop_triggered):
            self._stop_cmd_vel_cb = True

        else:
            self._stop_cmd_vel_cb = False

    def _estop_cb(self, data) -> None:
        self._estop_triggered = data.data            

    def _cmd_vel_cb(self, data) -> None:
        # Block all motors if any Roboteq controller returns a fault flag or runtime error flag
        if not self._stop_cmd_vel_cb:
            self._panther_kinematics.inverse_kinematics(data)
        else:
            self._panther_kinematics.inverse_kinematics(Twist())

        self._cmd_vel_command_last_time = rospy.Time.now()

    def _roboteq_state_is_correct(self, flags: list) -> bool:
        error_detected = False
        
        for flag in flags:
            if flag.count(0.0) != len(flag):
                error_detected = True 

        if error_detected:
            self._state_err_no += 1  
        else:
            self._state_err_no = 0
    
        return (self._state_err_no <= 1 / self._driver_state_timer_period)
    
    def _decode_fault_flag(self, flag_val: int) -> FaultFlag:
        # For more info see 272-roboteq-controllers-user-manual-v21 p. 246
        #   https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/
        ref_flag_val = 0b00000001

        msg = FaultFlag()
        msg.can_net_err = self._panther_can.can_connection_correct()
        msg.overheat = bool(flag_val & ref_flag_val << 0)
        msg.overvoltage = bool(flag_val & ref_flag_val << 1)
        msg.undervoltage = bool(flag_val & ref_flag_val << 2)
        msg.short_circuit = bool(flag_val & ref_flag_val << 3)
        msg.emergency_stop = bool(flag_val & ref_flag_val << 4)
        msg.motor_or_sensor_setup_fault = bool(flag_val & ref_flag_val << 5)
        msg.mosfet_failure = bool(flag_val & ref_flag_val << 6)
        msg.default_config_loaded_at_startup = bool(flag_val & ref_flag_val << 7)

        return msg

    @staticmethod
    def _decode_runtime_flag(flag_val: int) -> RuntimeError:
        # For more info see 272-roboteq-controllers-user-manual-v21 p. 247
        #   https://www.roboteq.com/docman-list/motor-controllers-documents-and-files/
        ref_flag_val = 0b00000001

        msg = RuntimeError()
        msg.amps_limit_active = bool(flag_val & ref_flag_val << 0)
        msg.motor_stall = bool(flag_val & ref_flag_val << 1)
        msg.loop_error = bool(flag_val & ref_flag_val << 2)
        msg.safety_stop_active = bool(flag_val & ref_flag_val << 3)
        msg.forward_limit_triggered = bool(flag_val & ref_flag_val << 4)
        msg.reverse_limit_triggered = bool(flag_val & ref_flag_val << 5)
        msg.amps_trigger_activated = bool(flag_val & ref_flag_val << 6)

        return msg

    def _trigger_panther_estop(self) -> bool:
        try:
            response = self._estop_trigger()
            rospy.logwarn(f'[{rospy.get_name()}] Trying to trigger Panther e-stop... Response: {response.success}')

            if not response.success:
                return True

        except rospy.ServiceException as e:
            rospy.logerr(f'[{rospy.get_name()}] Can\'t trigger Panther e-stop... \n{e}')

        return False

    def _publish_joint_state_cb(self) -> None:
        self._joint_state_msg.header.stamp = rospy.Time.now()
        self._joint_state_msg.position = self._wheels_ang_pos
        self._joint_state_msg.velocity = self._wheels_ang_vel
        self._joint_state_msg.effort = self._motors_effort
        self._joint_publisher.publish(self._joint_state_msg)

    def _publish_pose_cb(self) -> None:
        self._pose_msg.position.x = self._robot_x_pos
        self._pose_msg.position.y = self._robot_y_pos
        self._pose_msg.orientation.x = self._qx
        self._pose_msg.orientation.y = self._qy
        self._pose_msg.orientation.z = self._qz
        self._pose_msg.orientation.w = self._qw
        self._pose_publisher.publish(self._pose_msg)

    def _publish_tf_cb(self) -> None:
        self._tf_stamped.header.stamp = rospy.Time.now()
        self._tf_stamped.transform.translation.x = self._robot_x_pos
        self._tf_stamped.transform.translation.y = self._robot_y_pos
        self._tf_stamped.transform.translation.z = 0.0
        self._tf_stamped.transform.rotation.x = self._qx
        self._tf_stamped.transform.rotation.y = self._qy
        self._tf_stamped.transform.rotation.z = self._qz
        self._tf_stamped.transform.rotation.w = self._qw
        self._tf_broadcaster.sendTransform(self._tf_stamped)

    def _publish_odom_cb(self) -> None:
        self._odom_msg.header.stamp = rospy.Time.now()
        self._odom_msg.pose.pose.position.x = self._robot_x_pos
        self._odom_msg.pose.pose.position.y = self._robot_y_pos
        self._odom_msg.pose.pose.orientation.x = self._qx
        self._odom_msg.pose.pose.orientation.y = self._qy
        self._odom_msg.pose.pose.orientation.z = self._qz
        self._odom_msg.pose.pose.orientation.w = self._qw
        self._odom_publisher.publish(self._odom_msg)
    
    @staticmethod
    def euler_to_quaternion(yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
            math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
            math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
            math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

        return [qx, qy, qz, qw]


def main():
    panther_driver_node = PantherDriverNode('panther_driver_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
