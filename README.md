# panther_ros

Packages composing basic functionalities of the Husarion Panther robot.
API for each node can be found in each package folder.

## Build and run on hardware

In order to build the Panther for hardware setup on internal computer, use following commands.
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

After successful build run
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

Parameters stored in [panther_default.yaml](./panther_bringup/config/panther_default.yaml) are generic and are overwritten with your specific panther configuration on start.

In order to change wheels used with your robot, add `wheel_type:=WH0X` to launch command as follows:
``` bash
roslaunch panther_bringup bringup.launch wheel_type:=WH02
```

Possible wheels:
- `WH01`: default offroad wheels.
- `WH02`: mecanum.
- `WH04`: small pneumatic wheels.

If you want to use custom wheels, use argument `wheel_config_file` where you provide a path to `wheel.yaml` file. The file has to be written in the same manner as [default panther wheel configurations](./panther_description/config/WH01.yaml).