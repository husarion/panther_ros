# panther_ros

Packages composing basic funcinalities of the Husarion Panther robot.

## Build and run on hardware

In order to bould the Panhter for hardware setup on Raspberry Pi 4 use folowing commands.
``` bash
export HUSARION_ROS_BUILD_TYPE=hardware

pip3 install apa102-pi rosdep psutil vcstool

git clone https://github.com/husarion/panther_ros.git src/panther_ros
vcs import src < src/panther_ros/panther/panther.repos
rm -r src/panther_ros/panther_gazebo
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release
```

After succesfull build run
``` bash
roslaunch panther_bringup bringup.launch
```

This will launch all nodes related to Panhter robot selecting them to match your specyfic hardware revision. Don't forget thos packages require environment variables that are set during boot procedure of Panther.

## Build and run simulation

In order to bould your system run:

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