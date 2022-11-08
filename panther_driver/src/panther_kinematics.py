#!/usr/bin/python3

from abc import abstractmethod
import math
from numpy import clip

from geometry_msgs.msg import Twist


class PantherKinematics:
    def __init__(
            self, 
            robot_width, 
            robot_length,
            wheel_radius,
            encoder_resolution,
            gear_ratio,
            power_factor,
        ) -> None:

        self._robot_width = robot_width
        self._robot_length = robot_length
        self._wheel_radius = wheel_radius  # Distance of the wheel center, to the roller center
        self._const_factor = power_factor * float(encoder_resolution * gear_ratio) / (2.0 * math.pi)

        self._wheels_enc_speed = [0.0, 0.0, 0.0, 0.0]
        self._lin_x = 0.0
        self._lin_y = 0.0
        self._ang_z = 0.0
        self._robot_x_pos = 0.0
        self._robot_y_pos = 0.0
        self._robot_th_pos = 0.0
        self._max_speed = 950.0
        self._scale_factor_x = 0.25
        self._scale_factor_y = 0.25
        self._scale_factor_th = 0.125

    @property
    def wheels_enc_speed(self):
        return self._wheels_enc_speed

    @abstractmethod
    def inverse_kinematics(self, data: Twist) -> None:
        pass

    @abstractmethod
    def forward_kinematics(self, fl_ang_vel, fr_ang_vel, rl_ang_vel, rr_ang_vel, dt) -> float:
        pass

    def _get_motor_speed(self, wheel_ang_vel: list) -> list:
        return [
            clip(self._const_factor * ang_vel, -self._max_speed, self._max_speed) 
            for ang_vel in wheel_ang_vel
        ]


class PantherDifferential(PantherKinematics):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def inverse_kinematics(self, data: Twist) -> None:
        self._lin_x = data.linear.x * self._scale_factor_x      # [m/s]
        self._lin_y = data.linear.y * self._scale_factor_y      # [m/s]
        self._ang_z = data.angular.z * self._scale_factor_th    # [rad/s]

        wheel_front_right_ang_vel = wheel_rear_right_ang_vel = (1.0 / self._wheel_radius) * (
            self._lin_x + (self._robot_width + self._robot_length) * self._ang_z
        )
        wheel_front_left_ang_vel = wheel_rear_left_ang_vel = (1.0 / self._wheel_radius) * (
            self._lin_x - (self._robot_width + self._robot_length) * self._ang_z
        )

        self._wheels_enc_speed = self._get_motor_speed([
            wheel_front_left_ang_vel,
            wheel_front_right_ang_vel,
            wheel_rear_left_ang_vel,
            wheel_rear_right_ang_vel
        ])

    def forward_kinematics(self, fl_ang_vel, fr_ang_vel, rl_ang_vel, rr_ang_vel, dt) -> float:
        linear_velocity_x = (
            fl_ang_vel + fr_ang_vel + rl_ang_vel + rr_ang_vel
        ) * (self._wheel_radius / 4.0)

        linear_velocity_y = (
            -fl_ang_vel + fr_ang_vel + rl_ang_vel - rr_ang_vel
        ) * (self._wheel_radius / 4.0)

        angular_velocity_z = (
            -fl_ang_vel + fr_ang_vel - rl_ang_vel + rr_ang_vel
        ) * (self._wheel_radius / (4.0 * (self._robot_width / 2.0 + self._robot_length / 2.0)))

        delta_heading = angular_velocity_z * dt  # [radians]
        self._robot_th_pos = self._robot_th_pos + delta_heading

        delta_x = (
            linear_velocity_x * math.cos(self._robot_th_pos) - linear_velocity_y * math.sin(self._robot_th_pos)
        ) * dt  # [m]

        delta_y = (
            linear_velocity_x * math.sin(self._robot_th_pos) + linear_velocity_y * math.cos(self._robot_th_pos)
        ) * dt  # [m]

        self._robot_x_pos = self._robot_x_pos + delta_x
        self._robot_y_pos = self._robot_y_pos + delta_y

        return self._robot_x_pos, self._robot_y_pos, self._robot_th_pos


class PantherMecanum(PantherKinematics):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def inverse_kinematics(self, data: Twist) -> None:
        self._lin_x = data.linear.x * self._scale_factor_x  # [m/s]
        self._lin_y = data.linear.y * self._scale_factor_y  # [m/s]
        self._ang_z = data.angular.z * self._scale_factor_th  # [rad/s]
        
        wheel_front_right_ang_vel = (1.0 / self._wheel_radius) * (
            self._lin_x + self._lin_y + (self._robot_width + self._robot_length) * self._ang_z
        )
        wheel_front_left_ang_vel = (1.0 / self._wheel_radius) * (
            self._lin_x - self._lin_y - (self._robot_width + self._robot_length) * self._ang_z
        )
        wheel_rear_right_ang_vel = (1.0 / self._wheel_radius) * (
            self._lin_x - self._lin_y + (self._robot_width + self._robot_length) * self._ang_z
        )
        wheel_rear_left_ang_vel = (1.0 / self._wheel_radius) * (
            self._lin_x + self._lin_y - (self._robot_width + self._robot_length) * self._ang_z
        )

        self._wheels_enc_speed = self._get_motor_speed([
            wheel_front_left_ang_vel,
            wheel_front_right_ang_vel,
            wheel_rear_left_ang_vel,
            wheel_rear_right_ang_vel
        ])

    def forward_kinematics(self, fl_ang_vel, fr_ang_vel, rl_ang_vel, rr_ang_vel, dt) -> float:
        linear_velocity_x = (
            fl_ang_vel + fr_ang_vel + rl_ang_vel + rr_ang_vel
        ) * (self._wheel_radius / 4.0)

        linear_velocity_y = (
            -fl_ang_vel + fr_ang_vel + rl_ang_vel - rr_ang_vel
        ) * (self._wheel_radius / 4.0)

        angular_velocity_z = (
            -fl_ang_vel + fr_ang_vel - rl_ang_vel + rr_ang_vel
        ) * (self._wheel_radius / (4.0 * (self._robot_width / 2.0 + self._robot_length / 2.0)))

        delta_heading = angular_velocity_z * dt  # [radians]
        self._robot_th_pos = self._robot_th_pos + delta_heading

        delta_x = (
            linear_velocity_x * math.cos(self._robot_th_pos)
            - linear_velocity_y * math.sin(self._robot_th_pos)
        ) * dt  # [m]

        delta_y = (
            linear_velocity_x * math.sin(self._robot_th_pos)
            + linear_velocity_y * math.cos(self._robot_th_pos)
        ) * dt  # [m]

        self._robot_x_pos = self._robot_x_pos + delta_x
        self._robot_y_pos = self._robot_y_pos + delta_y

        return self._robot_x_pos, self._robot_y_pos, self._robot_th_pos
