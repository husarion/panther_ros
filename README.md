# panther_ros

Packages composing basic functionalities of the Husarion Panther robot.
API for each node can be found in each package folder.

![Panther dark](https://husarion.com/assets/images/night_with_lights-f1e591289905c18c839b2142160e00ef.png#gh-dark-mode-only)
![Panther light](https://husarion.com/assets/images/day_no_light_crop-6072a2346aede8746029b888fa16b214.png#gh-light-mode-only)


:warning: **Warning**: 
Building the latest version of this code might not always work with OS image you already have installed on your robot. Make sure you are running the newest OS image for Build-in Computer. You can find it in the downloads section at [husarion.com](https://husarion.com/downloads/#internal-computer-raspberry-pi-4).

We are making great effort to achieve backward compatibility, but can not test everything. Currently, fully tested images are `v1.0.0` and above. If the command below shows you the tag, you can be assured the newest code will work on your robot.
``` bash
echo $SYSTEM_BUILD_VERSION
```

If you are running an older OS image, the software should run within docker. We did our best to achieve backward compatibility, yet we do not advise using the newest software stack with older OS images.

## Docker image

There is already prebuild and tested docker image for Panther robot. This is the docker image robots are shipped with.
In order to download it, run:
``` bash
docker pull husarion/panther
```

Example **compose.yaml** can be found on [husarion/panther-docker](https://github.com/husarion/panther-docker/) in **demo** section.

## Build and run on hardware

To build hardware interface packages running on for Panther robot's Build-in Computer, use the following commands:
``` bash
export HUSARION_ROS_BUILD_TYPE=hardware

pip3 install apa102-pi rosdep vcstool

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

Parameters stored in [**panther_default.yaml**](./panther_bringup/config/panther_default.yaml) are generic and are overwritten with your specific panther configuration on start.

In order to change wheels used with your robot, add `wheel_type:=WH0X` to launch command as follows:
``` bash
roslaunch panther_bringup bringup.launch wheel_type:=WH02
```

Possible wheels names:
- `WH01`: default offroad wheels.
- `WH02`: mecanum.
- `WH04`: small pneumatic wheels.

If you want to use custom wheels, use argument `wheel_config_file` where you provide a path to `wheel.yaml` file. The file has to be written in the same manner as default panther wheel configurations found in [**WH01.yaml**](./panther_description/config/WH01.yaml).

## Shutting down User Computers

You can shut down any computer within internal Panther's network.
You can do so by exchanging SSH public keys between the Build-in Computer and the one you want to shutdown. Later you have to modify [**shutdown_hosts.yaml**](./panther_bringup/config/shutdown_hosts.yaml) adding your computer as follows:
``` yaml
hosts:
  - ip: 10.15.20.XX
    username: my-username
```
Refer to the [panther_manager documentation](./panther_manager/README.md) for more information.