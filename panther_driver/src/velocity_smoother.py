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

        self._robot_velocity = [0.0, 0.0, 0.0]  # vel_x, vel_y, vel_theta

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
        output_vel.linear.x = self._velocity_limiter(
            self._accel_limiter(
                cmd_vel.linear.x, self._robot_velocity[0], acc_x_factor, decel_x_factor
            ),
            self._max_vel_x,
        )
        output_vel.linear.y = self._velocity_limiter(
            self._accel_limiter(
                cmd_vel.linear.y, self._robot_velocity[1], acc_y_factor, decel_y_factor
            ),
            self._max_vel_y,
        )
        output_vel.angular.z = self._velocity_limiter(
            self._accel_limiter(
                cmd_vel.angular.z,
                self._robot_velocity[2],
                acc_theta_factor,
                decel_theta_factor,
            ),
            self._max_vel_theta,
        )

        self._robot_velocity[0] = output_vel.linear.x
        self._robot_velocity[1] = output_vel.linear.y
        self._robot_velocity[2] = output_vel.angular.z

        return output_vel

    @staticmethod
    def _accel_limiter(cmd_vel: float, robot_vel: float, acc_factor: float, decel_factor) -> float:
        if cmd_vel > robot_vel:
            if robot_vel >= 0.0:
                return min(cmd_vel, robot_vel + acc_factor)
            else:
                return min(cmd_vel, robot_vel + decel_factor)
        elif cmd_vel < robot_vel:
            if robot_vel <= 0.0:
                return max(cmd_vel, robot_vel - acc_factor)
            else:
                return max(cmd_vel, robot_vel - decel_factor)
        else:
            return cmd_vel

    @staticmethod
    def _velocity_limiter(cmd_vel: float, max_vel) -> float:
        return min(cmd_vel, max_vel)
