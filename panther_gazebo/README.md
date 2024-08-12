# panther_gazebo

The package contains a launch file and source files used to run the robot simulation in Gazebo. The simulator tries to reproduce the behavior of a real robot as much as possible, including the provision of an analogous ROS_API.

## Launch Files

- `spawn_robot.launch.py`: Responsible for spawning the robot in the simulator.
- `simulate_robot.launch.py`: Responsible for giving birth to the robot and simulating its physical behavior, such as driving, displaying data, etc.
- `simulate_multiple_robots.launch.py`: Similar to the above with logic allowing you to quickly add a swarm of robots.
- **`simulation.launch.py`**: A target file that runs the gazebo simulator that adds and simulates the robot's behavior in accordance with the given arguments.

## Configuration Files

- [`battery_plugin_config.yaml`](./config/battery_plugin_config.yaml): Simulated LinearBatteryPlugin configuration.
- [`gz_bridge.yaml`](./config/gz_bridge.yaml): Specify data to exchange between ROS and Gazebo simulation.
- [`led_strips.yaml`](./config/led_strips.yaml): Configure properties of led strips in simulation to animate lights.
- [`teleop_with_estop.config`](./config/teleop_with_estop.config): Gazebo layout configuration file, which adds E-Stop and Teleop widgets.

## ROS Nodes

### EStop

`EStop` is a Gazebo GUI plugin responsible for easy and convenient changing of the robot's E-stop state.

#### Service Clients

- `hardware/e_stop_reset` [*std_srvs/Trigger*]: Resets E-stop.
- `hardware/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.

### GzPantherSystem

Plugin based on `ign_system` is responsible for handling sensor interfaces (only IMU for now) and sending requests for joints compatible with `ros2_control`. Plugin also adds E-Stop support.

#### Publishers

- `gz_ros2_control/e_stop` [*std_msgs/Bool*]: Current E-stop state.

#### Service Servers

- `gz_ros2_control/e_stop_reset` [*std_srvs/Trigger*]: Resets E-stop.
- `gz_ros2_control/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.

> [!NOTE]
> Above topics and services should be remapped to match real robot

#### Parameters

Required parameters are defined when including the interface in the URDF (you can check out [panther_macro.urdf.xacro](../panther_description/urdf/panther_macro.urdf.xacro)).

- `e_stop_initial_state` [*bool*, default: **true**]: Initial state of E-stop.
