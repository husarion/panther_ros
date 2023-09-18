# panther_gazebo

A package containing the launch files and dependencies needed to run the simulation in Gazebo-classic.

## Usage

Basic example with Docker configuration can be found in this [compose.simulation.yaml](https://github.com/husarion/panther-docker/blob/ros1/demo/simulation/compose.simulation.yaml) file. This repository also contains [examples](https://github.com/husarion/panther-docker/tree/ros1/demo/simulation) of simulations with custom robot models, such as one equipped with LIDAR sensors, cameras, or manipulators.

The advised way of launching the simulation is by using [panther_simulation.launch](./launch/panther_simulation.launch) file. Below you can find launch arguments that allows simulation configuration.

### Launch Arguments

- `use_rviz` [*bool*, default: **true**]: whether to launch RViz alongside Gazebo simulation.
- `rviz_config` [*string*, default: **$(find panther_description)/rviz/panther.rviz**]: path to RViz configuration file. Valid when `use_rviz` is **true**.
- `use_ros_control` [*bool*, default: **false**]: whether to use `ros_control` for simulation.
- `wheel_type` [*string*, default: **WH01**]: type of wheel, possible are: **WH01** - offroad, **WH02** - mecanum, **WH04** - small pneumatic.
- `use_gpu` [*bool*, default: **false**]: turns on GPU acceleration for sensors.
- `publish_robot_state` [*bool*, default: **true**]: whether to publish robot state.
- `pos_x` [*float*, default: **0.0**]: spawn position **[m]** of the robot in the world in **X** direction.
- `pos_y` [*float*, default: **0.0**]: spawn position **[m]** of the robot in the world in **Y** direction.
- `pos_z` [*float*, default: **0.0**]: spawn position **[m]** of the robot in the world in **Z** direction.
- `rot_yaw` [*float*, default: **0.0**]: spawn yaw angle **[rad]** of the robot in the world.
- `world_file` [*string*, default: **worlds/empty.world**]: path to Gazebo world file used for simulation.