# panther_description

A package containing the description of the Husarion Panther robot.

## Usage

Basic Panther configuration can be found in file [panther.urdf.xacro](./urdf/panther.urdf.xacro). This is an example configuration showing how to use the model. This can be used to import within launch files as a baseline model. For more advanced use cases, [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro) is designed to be integrated into custom robot configurations.

## Parameters

Arguments passed to the [panther.urdf.xacro](./urdf/panther.urdf.xacro) are the same as parameters of [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro). Thus, this section covers both of them.

- `dual_bat` [*bool*, default: **false**]: Changes inertia and mass for robot body to match 2 batteries setup. Not implemented yet.
- `imu_pos_x` [*float*, default: **0.169**]: **X** coordinate of IMU sensor in relation to `body_link`.
- `imu_pos_y` [*float*, default: **0.025**]: **Y** coordinate of IMU sensor in relation to `body_link`.
- `imu_pos_z` [*float*,default: **0.092**]: **Z** coordinate of IMU sensor in relation to `body_link`.
- `imu_rot_r` [*float*, default: **0.0**]: roll rotation of IMU sensor in relation to `body_link`.
- `imu_rot_p` [*float*, default: **0.0**]: pitch rotation of IMU sensor in relation to `body_link`.
- `imu_rot_y`  [*float*, default: **0.0**]: yaw rotation of IMU sensor in relation to `body_link`.
- `use_sim` [*bool*, default: **false**]: Unused. Kept for ROS 2 compatibility.
- `wheel_config_path` [*float*, default: **$(find panther_description)/config/WH01.yaml**]: - absolute path to YAML file defining wheel properties.
- `simulation_engine` [*float*, default: **gazebo-classic**]: physics engine to select plugins for. Supported engines: `gazebo-classic`. Kept for compatibility with ROS2 model. Currently, no other engines are planned to be supported.
- `use_ros_control` [*bool*, default: **false**]: whether to use `ros_control`.

There is one additional [panther.urdf.xacro](./urdf/panther.urdf.xacro) argument:
- `use_gpu` [*float*, default: **false**]: Turns on GPU acceleration for sensors.
It is not present in the [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro) since base of a robot does not have any sensors that can be accelerated with GPU.

Parameter `wheel_config_path` allows using non-standard wheels with Panther robot without modifying URDF file. The syntax is following:
- `wheel_radius` - wheel radius in **[m]**.
- `wheel_separation` - separation of wheels alongside *y* axis in **[m]**.
- `mass` - wheel mass in **[Kg]**.
- `inertia` - diagonal of inertia tensor in **[Kg m^2]**. Required subfields:
  - `ixx` - inertia alongside axis **x**.
  - `iyy` - inertia alongside axis **y**.
  - `izz` - inertia alongside axis **z**.
- `inertia_y_offset` - Offset of center of mass in **y** direction in **[m]**.
- `mesh_package` - ROS package name to search for custom meshes. Used in evaluation **$(find my_amazing_package)/**.
- `folder_path` - path used to search for mesh files within the ROS package.
- `kinematics` - kinematics type. Possible options: `differential`, `mecanum`.

Wheels have to be named as follows:
- `wheel_collision.stl` - wheel collision mesh.
- `fl_wheel.dae` - front, left wheel visual mesh.
- `fr_wheel.dae` - front, right wheel visual mesh.
- `rl_wheel.dae` - rear, left wheel visual mesh.
- `rr_wheel.dae` - rear, right wheel visual mesh.

## Links

Evaluating [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro) following links are created:
- `base_link` laying on the ground in center of the robot.
- `body_link` right above the `base_link` on the level of rotation axis of wheels.
- `imu_link` location of IMU sensor.
- `fl_wheel_link` front left wheel link.
- `fr_wheel_link` front right wheel link.
- `rl_wheel_link` rear left wheel link.
- `rr_wheel_link` rear right wheel link.

There are also links created for user to attach own sensors and components. The purpose of those links is to simplify localization of mounting points. Those are mentioned links:
- `cover_link` at the level of the top surface of rails, located in the center of the robot. Positive `x` axis pointing in front of the robot, positive `z` axis pointing up from the robot, and positive `y` pointing to the left of a robot.
- `front_light_link` in front of the robot, at the height of v-slot of front lights. Exact height is in the middle of the v-slot. Position in `x` axis is the front surface of the v-slot, while in `y` axis it is the center of the robot. Rotation is same as `cover_link`.
- `rear_light_link` analogous to `front_light_link`, but mounted to the back of the robot. This reference frame has positive `x` facing to the back of a robot. Positive `z` is facing up and positive and `y` is facing to the right of a robot. 

## Panther specific components configuration

GPS antenna component is defined in [external_antenna.urdf.xacro](./urdf/components/external_antenna.urdf.xacro). Parameters of the macro follow the convention from the next section.

## Sensor configuration

Sensors are defined in [husarion/ros_components_description](https://github.com/husarion/ros_components_description) repository. Information on how to include those sensors can be found in that repository.
