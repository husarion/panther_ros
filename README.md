# panther_ros

Packages composing basic functionalities of the Husarion Panther robot.
API for each node can be found in each package folder.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/day_with_lights.png">
  <img alt="Panther preview" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/day_no_lights.png">
</picture>

> **Note**: 
> Building the latest version of this code might not always work with OS image you already have installed on your robot. Make sure you are running the newest OS image for Built-in Computer. You can find it in the downloads section at [husarion.com](https://husarion.com/downloads/#built-in-computer).
>
> We are making great effort to achieve backward compatibility, but can not test everything. Currently, fully tested images are `v1.0.0` and above. If the command below shows you the tag, you can be assured the newest code will work on your robot.
> ``` bash
> echo $SYSTEM_BUILD_VERSION
> ```
>
> If you are running an older OS image, the software should run within docker. We did our best to achieve backward compatibility, yet we do not advise using the newest software stack with older OS images.

## Docker image

There is already prebuild and tested docker image for Panther robot. This is the docker image robots are shipped with.
In order to download it, run:
``` bash
docker pull husarion/panther
```

Example **compose.yaml** can be found on [husarion/panther-docker](https://github.com/husarion/panther-docker/) in **demo** section.

## Build and run on hardware

To build hardware interface packages running on the Panther robot's internal computer, use the following commands:
``` bash
export HUSARION_ROS_BUILD_TYPE=hardware

git clone https://github.com/husarion/panther_ros.git src/panther_ros
vcs import src < src/panther_ros/panther/panther.repos
rm -r src/panther_ros/panther_gazebo
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release
```

After successful build run:
``` bash
roslaunch panther_bringup bringup.launch
```

This will launch all nodes related to Panther robot, selecting them to match your specific hardware revision. Please keep in mind, this packages require environment variables that are set during boot procedure of Panther.

## Build and run simulation

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

Later to launch simulation run:
``` bash
roslaunch panther_gazebo panther_simulation.launch
```

This will launch Gazebo and Rviz.

## Robot configuration

Parameters stored in [**panther_default.yaml**](./panther_bringup/config/panther_default.yaml) are generic and are overwritten with your specific Panther configuration on start.

### Changing wheels type

Changing the wheels requires changing the `wheel_type` parameter. To do this, you can use the following command by modifying the value of the wheel_type:=WH0X parameter:
``` bash
roslaunch panther_bringup bringup.launch wheel_type:=WH02
```

Possible wheels names:
- `WH01`: default offroad wheels.
- `WH02`: mecanum.
- `WH04`: small pneumatic wheels.
- `custom`: custom wheels type.

If you want to use custom wheels, use argument `wheel_config_file` where you provide a path to `wheel.yaml` file. The file has to be written in the same manner as default panther wheel configurations found in [**WH01.yaml**](./panther_description/config/WH01.yaml).

### Providing custom robot description

By default Panther will launch the default robot description - raw robot without any sensors. There are two ways of providing custom robot description, which are described below.

**a) Disabling the default robot state publisher**

If you want to use a custom robot description from a different location within your project, you can disable the default `robot_state_publisher` and run it separately. To do this, use the `publish_robot_state` launch argument with a value of **false**:

```bash
roslaunch panther_bringup bringup.launch publish_robot_state:=false
```

**b) Providing custom robot description**

Alternatively, you can provide a custom robot description directly using the `robot_description` launch argument. For example:

```bash
roslaunch panther_bringup bringup.launch robot_description:="xacro $(rospack find my_awesome_package)/urdf/panther.urdf.xacro"
```

## Shutting down User Computers

You can shut down any computer within internal Panther's network.
You can do so by exchanging SSH public keys between the Built-in Computer and the one you want to shutdown. Later you have to modify [**shutdown_hosts.yaml**](./panther_bringup/config/shutdown_hosts.yaml) adding your computer as follows:
``` yaml
hosts:
  - ip: 10.15.20.XX
    username: my-username
```
Refer to the [panther_manager documentation](./panther_manager/README.md) for more information.

## Managing LED animations

You can easily customize Panther LED panels by defining new animations based on images. They can be created with simple YAML syntax, such as the one shown below: 

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

You also have the option to create code-based animations. Detailed information on how to make use of these features, as well as a description of default behavior and customization options for Panther LED panels, can be found in [panther_lights documentation](./panther_lights/README.md).
