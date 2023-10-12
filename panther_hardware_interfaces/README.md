## Lely CANopen installation
sudo apt-get update && \
sudo apt-get install -y software-properties-common && \
sudo add-apt-repository ppa:lely/ppa && \
sudo apt-get update && \
sudo apt-get install -y liblely-coapp-dev liblely-co-tools python3-dcf-tools

## RT
(information from ros2 control)

If you have a realtime kernel installed, the main thread of Controller Manager attempts to configure ``SCHED_FIFO`` with a priority of ``50``.
By default, the user does not have permission to set such a high priority.
To give the user such permissions, add a group named realtime and add the user controlling your robot to this group:

.. code-block:: console

    $ sudo addgroup realtime
    $ sudo usermod -a -G realtime $(whoami)

Afterwards, add the following limits to the realtime group in ``/etc/security/limits.conf``:

.. code-block:: console

    @realtime soft rtprio 99
    @realtime soft priority 99
    @realtime soft memlock 102400
    @realtime hard rtprio 99
    @realtime hard priority 99
    @realtime hard memlock 102400

The limits will be applied after you log out and in again.


## Conversions
  // Converts desired wheel torque in Nm to Roboteq motor command. Steps:
  // 1. Convert desired wheel Nm torque to motor Nm ideal torque (multiplication by (1.0/gear_ratio))
  // 2. Convert motor Nm ideal torque to motor Nm real torque (multiplication by (1.0/gearbox_efficiency))
  // 3. Convert motor Nm real torque to motor A current (multiplication by (1.0/motor_torque_constant))
  // 4. Convert motor A current to Roboteq GO command - permille of the Amps limit current
  //    set in the roboteq driver (ALIM parameter) - multiplication by 1000.0/max_amps_motor_current
  newton_meter_to_roboteq_cmd_ = (1.0 / drivetrain_settings.gear_ratio) *
                                 (1.0 / drivetrain_settings.gearbox_efficiency) *
                                 (1.0 / drivetrain_settings.motor_torque_constant) *
                                 (1000.0 / drivetrain_settings.max_amps_motor_current);