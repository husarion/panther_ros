# panther_description

## Installation

This package relates to other repositories that have to be built from source. In order to build the package run:
``` bash
vcs import < components.repos src
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
catkin build
```

<!-- ## Usage

Basic Panther configuration can be found in file [panther.urdf.xacro](./panther_description/urdf/panther.urdf.xacro). This is an example configuration showing how to use the model. This can be used to import in launch files as a baseline model. For more advanced use cases, [panther_macro.urdf.xacro](./panther_description/urdf/panther_macro.urdf.xacro) is designed to be integrated into custom robot configurations. -->

## Parameters

Arguments passed to the [panther.urdf.xacro](./panther_description/urdf/panther.urdf.xacro) are the same as parameters of [panther_macro.urdf.xacro](./panther_description/urdf/panther_macro.urdf.xacro). Thus, this section covers both of them.


- `use_sim` *(default: false)* - Unused. Kept for ROS 2 compatibility.
- `dual_bat` *(default: false)* - Changes inertia and mass for robot body to match 2 batteries setup.
- `imu_pos_x` *(default: 0.169)* - **x** coordinate of IMU sensor in relation to `body_link`.
- `imu_pos_y` *(default: 0.025)* - **y** coordinate of IMU sensor in relation to `body_link`.
- `imu_pos_z` *(default: 0.092)* - **z** coordinate of IMU sensor in relation to `body_link`.
- `imu_rot_r` *(default: 0.0)* - roll rotation of IMU sensor in relation to `body_link`.
- `imu_rot_p` *(default: 0.0)* - pitch rotation of IMU sensor in relation to `body_link`.
- `imu_rot_y`  *(default: 0.0)* - yaw rotation of IMU sensor in relation to `body_link`.
- `wheel_config_path` *(default: $(find panther_description)/config/WH01.yaml)* - absolute path to YAML file defining wheel properties.
- `simulation_engine` *(default: gazebo-classic)* - physics engine to select plugins for. Supported engines: `gazebo-classic`. Kept for comaptibility with ROS2 model. Currently no other engines are planned to be supported.

There is one additional [panther.urdf.xacro](./panther_description/urdf/panther.urdf.xacro) argument:
- `use_gpu` *(default: false)* - Turns on GPU acceleration for sensors.
It is not present in the [panther_macro.urdf.xacro](./panther_description/urdf/panther_macro.urdf.xacro) since base of a robot does not have any sensors that can be accelerated with GPU.

Parameter `wheel_config_path` allows using non-standard wheels with Panther robot without modifying URDF file. Syntax is following:
- `wheel_radius` - wheel radius in **[m]**.
- `wheel_separation` - separation of wheels alongside *y* axis in **[m]**.
- `mass` - wheel mass in **[Kg]**.
- `inertia` - diagonal of inertia tensor in **[Kg m^2]**. Required subfields:
  - `ixx` - inertia alongside axis **x**.
  - `iyy` - inertia alongside axis **y**.
  - `izz` - inertia alongside axis **z**.
- `inertia_y_offset` - Offset of center of mass in **y** direction in **[m]**.
- `mesh_package` - ROS package name to search for custom meshes. Used in evaluation **$(find my_amazing_package)/**.
- `folder_path` - path used to search for mesh files within ROS package.
- `kinematics` - kinematics type. Possible options: `differential`, `mecanum`.

Wheels have to be named as follows:
- `collision.stl` - wheel collision mesh.
- `fl_wheel.dae` - front, left wheel visual mesh.
- `fr_wheel.dae` - front, right wheel visual mesh.
- `rl_wheel.dae` - rear, left wheel visual mesh.
- `rr_wheel.dae` - rear, right wheel visual mesh.

## Panther specific components configuration

GPS antenna component is defined in [external_antenna.urdf.xacro](./panther_description/urdf/components/external_antenna.urdf.xacro). Parameters of the macro follow convention from the next section.

## Sensor configuration

Sensors are defined in [husarion/ros_components_description](https://github.com/husarion/ros_components_description) repository. Which is downloaded by VCS in the installation step. A guide on how to combine those sensors with the robot will be written soon.
