# panther_description

A package containing the description of the Husarion Panther robot.

## Usage

Basic Panther configuration can be found in file [panther.urdf.xacro](./urdf/panther.urdf.xacro). This is an example configuration showing how to use the model. This can be used to import within launch files as a baseline model. For more advanced use cases, [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro) is designed to be integrated into custom robot configurations.

## Parameters

> **Note**
> Default location of the IMU sensor in the URDF is the same as in Panther 1.0. On a real robot, the location is set by [panther_bringup/bringup.launch](../panther_bringup/launch/bringup.launch) from environment variables. Environment variables store the IMU location of the given robot and are set on boot time.

Arguments passed to the [panther.urdf.xacro](./urdf/panther.urdf.xacro) are the same as parameters of [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro). Thus, this section covers both of them.

- `dual_bat` [*bool*, default: **false**]: changes inertia and mass for robot body to match 2 batteries setup. Not implemented yet.
- `imu_pos_x` [*float*, default: **0.169**]: **X** coordinate of IMU sensor in relation to `body_link`.
- `imu_pos_y` [*float*, default: **0.025**]: **Y** coordinate of IMU sensor in relation to `body_link`.
- `imu_pos_z` [*float*,default: **0.092**]: **Z** coordinate of IMU sensor in relation to `body_link`.
- `imu_rot_r` [*float*, default: **0.0**]: roll rotation of IMU sensor in relation to `body_link`.
- `imu_rot_p` [*float*, default: **0.0**]: pitch rotation of IMU sensor in relation to `body_link`.
- `imu_rot_y`  [*float*, default: **0.0**]: yaw rotation of IMU sensor in relation to `body_link`.
- `use_sim` [*bool*, default: **false**]: unused. Kept for ROS 2 compatibility.
- `wheel_config_path` [*float*, default: **$(find panther_description)/config/WH01.yaml**]: - absolute path to YAML file defining wheel properties.
- `simulation_engine` [*float*, default: **gazebo-classic**]: physics engine to select plugins for. Supported engines: **gazebo-classic**. Kept for compatibility with ROS2 model. Currently, no other engines are planned to be supported.
- `use_ros_control` [*bool*, default: **false**]: whether to use `ros_control`.

There is one additional [panther.urdf.xacro](./urdf/panther.urdf.xacro) argument:
- `use_gpu` [*float*, default: **false**]: Turns on GPU acceleration for sensors.
It is not present in the [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro) since the base of a robot does not have any sensors that can be accelerated with GPU.

Parameter `wheel_config_path` allows using non-standard wheels with Panther robot without modifying URDF file. The syntax is following:
- `wheel_radius` - wheel radius in **[m]**.
- `wheel_separation` - separation of wheels alongside **Y** axis in **[m]**.
- `mass` - wheel mass in **[kg]**.
- `inertia` - the diagonal of inertia tensor in **[kg m^2]**. Required subfields:
  - `ixx` - inertia alongside **X** axis.
  - `iyy` - inertia alongside **Y** axis.
  - `izz` - inertia alongside **Z** axis.
- `inertia_y_offset` - Offset of center of mass in **Y** direction in **[m]**.
- `mesh_package` - ROS package name to search for custom meshes. Used in evaluation **$(find my_amazing_package)/**.
- `folder_path` - path used to search for mesh files within the ROS package.
- `kinematics` - kinematics type. Possible options: **differential**, **mecanum**.
- `odom_stderr` - standard deviation used to populate odometry message:
  - `vel_x` - standard deviation for linear velocity in **X** direction.
  - `vel_y` - standard deviation for linear velocity in **Y** direction.
  - `vel_yaw` - standard deviation for angular velocity alongside **Z** axis.

Wheels have to be named as follows:
- `wheel_collision.stl` - wheel collision mesh.
- `fl_wheel.dae` - front, left wheel visual mesh.
- `fr_wheel.dae` - front, right wheel visual mesh.
- `rl_wheel.dae` - rear, left wheel visual mesh.
- `rr_wheel.dae` - rear, right wheel visual mesh.

## Links

Evaluating [panther_macro.urdf.xacro](./urdf/panther_macro.urdf.xacro) following links are created:
- `base_link` laying on the ground in the center of the robot.
- `body_link` right above the `base_link` on the level of the rotation axis of the wheels.
- `imu_link` location of IMU sensor.
- `fl_wheel_link` front left wheel link.
- `fr_wheel_link` front right wheel link.
- `rl_wheel_link` rear left wheel link.
- `rr_wheel_link` rear right wheel link.
- `front_light_link` Front Bumper Lights link. Reference for front animation image.
- `rear_light_link` Rear Bumper Lights link. Reference for rear animation image.

There are also links created for users to attach their sensors and components. The purpose of those links is to simplify the localization of mounting points. Those are mentioned links:
- `cover_link` at the level of the top surface of the Mounting rails, located in the center of the robot. The positive **X** axis points in front of the robot, the positive **Z** axis points up from the robot, and the positive **Y** points to the left of the robot.
- `front_bumper_link` in front of the robot, at the height of the v-slot of front bumpers. The exact height is in the middle of the v-slot. Position in the **X** axis is the front surface of the v-slot, while in the **Y** axis it is the center of the robot. Rotation is the same as `cover_link`.
- `rear_bumper_link` analogous to `front_bumper_link`, but mounted to the back of the robot. This reference frame has a positive **X** facing the back of a robot. Positive **Z** is facing up and positive and **Y** is facing to the right of the robot.

## Panther Specific Components Configuration

GPS antenna component is defined in [external_antenna.urdf.xacro](./urdf/components/external_antenna.urdf.xacro). The parameters of the macro follow the convention from the next section.

## Sensor Configuration

Sensors are defined in [husarion/ros_components_description](https://github.com/husarion/ros_components_description) repository. Information on how to include those sensors can be found in that repository.
