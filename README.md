# panther_ros

Packages composing the basic functionalities of the Husarion Panther robot.
API for each node can be found in each package folder.

> **Warning**
> Software stack for ROS 1 will no longer receive major features. Bugs will still be fixed.
> Main development will now be targeted towards ROS 2.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/day_with_lights.png">
  <img alt="Panther preview" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/day_no_lights.png">
</picture>

> **Note**
> Building the latest version of this code will likely not work if you didn't update the OS. Make sure you are running the newest OS image for Built-in Computer. You can find the link to download it and instructions on updating at [husarion.com](https://husarion.com/manuals/panther/operating-system-reinstallation/#built-in-computer-system-reinstallation).
>
> Current software stack uses some of the features in the kernel and underlying OS that were not introduced in older versions. You need system version **v1.1.0** or newer. You can check the currently installed OS version by examining this environment variable:
> ``` bash
> echo $SYSTEM_BUILD_VERSION
> ```
>
> Some of the safety features broke compatibility for older system images, hence full system update is required.

## Docker Images

> **Note**
>  Docker is the recommended approach for utilizing this software stack.

There are already prebuilt, and tested Docker images dedicated to Panther robot software and simulation. For more information about the latest stable versions, guidance on updating your software, or running the simulation, please refer to [panther-docker](https://github.com/husarion/panther-docker/tree/ros1) repository documentation.

## Build and Run on Hardware

To build hardware interface packages running on the Panther robot's Built-in Computer, use the following commands:
``` bash
export HUSARION_ROS_BUILD_TYPE=hardware

cd /home/husarion/husarion_ws
git clone https://github.com/husarion/panther_ros.git src/panther_ros
vcs import src < src/panther_ros/panther/panther.repos
rm -r src/panther_ros/panther_gazebo
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# To allow ROS commands to recognize panther_ros in this terminal session
source /home/husarion/husarion_ws/devel/setup.bash
```

After a successful build run:
``` bash
roslaunch panther_bringup bringup.launch
```

This will launch all nodes related to the Panther robot, selecting them to match your specific hardware revision. Please keep in mind, those packages require environment variables that are set during the boot procedure of Panther.

## Build and Run Simulation

To build your system run:
``` bash
export HUSARION_ROS_BUILD_TYPE=simulation

pip3 install rosdep vcstool

git clone https://github.com/husarion/panther_ros.git src/panther_ros
vcs import src < src/panther_ros/panther/panther.repos
mv panther_ros/panther_description/ src/panther_description
mv panther_ros/panther_gazebo/ src/panther_gazebo
rm -rf panther_ros
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release
```

Later to launch a simulation run:
``` bash
roslaunch panther_gazebo panther_simulation.launch
```

This will launch Gazebo and Rviz.

## Robot Configuration

Parameters stored in [**panther_default.yaml**](./panther_bringup/config/panther_default.yaml) are generic and are overwritten with your specific Panther configuration on start.

### Changing Wheel Type

Changing the wheels requires changing the `wheel_type` parameter. To do this, you can use the following command by modifying the value of the `wheel_type:=WH0X` parameter:
``` bash
roslaunch panther_bringup bringup.launch wheel_type:=WH02
```

Possible wheels names:
- **WH01**: default offroad wheels.
- **WH02**: mecanum.
- **WH04**: small pneumatic wheels.
- **custom**: custom wheels type.

If you want to use custom wheels, use the argument `wheel_config_file` where you provide a path to `wheel.yaml` file. The file has to be written in the same manner as the default panther wheel configurations found in [**WH01.yaml**](./panther_description/config/WH01.yaml).

### Providing a Custom Robot Description

By default, Panther will launch the default robot description - raw robot without any sensors. There are two ways of providing custom robot descriptions, which are described below.

**a) Disabling the Default Robot State Publisher**

If you want to use a custom robot description from a different location within your project, you can disable the default `robot_state_publisher` and run it separately. To do this, use the `publish_robot_state` launch argument with a value of **false**.

```bash
roslaunch panther_bringup bringup.launch publish_robot_state:=false
```

**b) Providing a Custom Robot Description**

Alternatively, you can provide a custom robot description directly using the `robot_description` launch argument.

```bash
roslaunch panther_bringup bringup.launch robot_description:="xacro $(rospack find my_awesome_package)/urdf/panther.urdf.xacro"
```

## Software Shutdown of User Computers

You can shutdown any computer within the internal Panther network.
You can do so by exchanging SSH public keys between the Built-in Computer and the one you want to shutdown. Later, you have to modify [**shutdown_hosts.yaml**](./panther_bringup/config/shutdown_hosts.yaml) adding your computer as follows:
``` yaml
hosts:
  - ip: 10.15.20.XX
    username: my-username
```
Refer to the [panther_manager documentation](./panther_manager/README.md) for more information.

## Managing Bumper Lights' Animations

You can easily customize Panther Bumper Lights by defining new animations based on images. They can be created with simple YAML syntax, such as the one shown below: 

```yaml
# user_animations.yaml
user_animations:
  # animation with default image and custom color
  - id: 21
    name: MY_AWESOME_ANIMATION
    priority: 2
    animation:
      both:
        type: image_animation
        image: $(find panther_lights)/animations/strip01_red.png
        duration: 2
```

You also have the option to create code-based animations. Detailed information on how to make use of these features, as well as a description of default behavior and customization options for Panther Bumper Lights, can be found in the [panther_lights](./panther_lights/README.md) documentation.

