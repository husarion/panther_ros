#!/usr/bin/python3

from geometry_msgs.msg import Twist


class VelocitySmoother:
    def __init__(
        self,
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
    ) -> None:

        self._max_vel_x = max_vel_x
        self._max_vel_y = max_vel_y
        self._max_vel_theta = max_vel_theta
        self._acc_lim_x = acc_lim_x
        self._acc_lim_y = acc_lim_y
        self._acc_lim_theta = acc_lim_theta
        self._decel_lim_x = decel_lim_x
        self._decel_lim_y = decel_lim_y
        self._decel_lim_theta = decel_lim_theta
        self._emergency_decel_lim_x = emergency_decel_lim_x
        self._emergency_decel_lim_y = emergency_decel_lim_y
        self._emergency_decel_lim_theta = emergency_decel_lim_theta

        self._robot_vel_x = 0.0
        self._robot_vel_y = 0.0
        self._robot_vel_theta = 0.0

    def get_smooth_velocity(
        self, cmd_vel: Twist, period: float, emergency_breaking: bool = False
    ) -> Twist:
        acc_x_factor = self._acc_lim_x * period
        acc_y_factor = self._acc_lim_y * period
        acc_theta_factor = self._acc_lim_theta * period
        if not emergency_breaking:
            decel_x_factor = self._decel_lim_x * period
            decel_y_factor = self._decel_lim_y * period
            decel_theta_factor = self._decel_lim_theta * period
        else:
            decel_x_factor = self._emergency_decel_lim_x * period
            decel_y_factor = self._emergency_decel_lim_y * period
            decel_theta_factor = self._emergency_decel_lim_theta * period

        output_vel = Twist()
        output_vel.linear.x = self._clamp(
            self._accel_limiter(cmd_vel.linear.x, self._robot_vel_x, acc_x_factor, decel_x_factor),
            -self._max_vel_x,
            self._max_vel_x,
        )
        output_vel.linear.y = self._clamp(
            self._accel_limiter(cmd_vel.linear.y, self._robot_vel_y, acc_y_factor, decel_y_factor),
            -self._max_vel_y,
            self._max_vel_y,
        )
        output_vel.angular.z = self._clamp(
            self._accel_limiter(
                cmd_vel.angular.z,
                self._robot_vel_theta,
                acc_theta_factor,
                decel_theta_factor,
            ),
            -self._max_vel_theta,
            self._max_vel_theta,
        )

        self._robot_vel_x = output_vel.linear.x
        self._robot_vel_y = output_vel.linear.y
        self._robot_vel_theta = output_vel.angular.z

        return output_vel

    def _accel_limiter(
        self, cmd_vel: float, robot_vel: float, acc_factor: float, decel_factor
    ) -> float:
        if robot_vel >= 0.0:
            return robot_vel + self._clamp(cmd_vel - robot_vel, -decel_factor, acc_factor)
        else:
            return robot_vel + self._clamp(cmd_vel - robot_vel, -acc_factor, decel_factor)

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)
