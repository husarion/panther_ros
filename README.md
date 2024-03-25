# panther_ros

ROS 2 packages for Panther autonomous mobile robot

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

## Quick start

### Configure environment

The repository is used to run the code both on the real robot and in the simulation. Specify `HUSARION_ROS_BUILD_TYPE` the variable according to your needs.

Real robot:

``` bash
export HUSARION_ROS_BUILD_TYPE=hardware
```

Simulation:

```bash
export HUSARION_ROS_BUILD_TYPE=simulation
```

### Create workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone https://github.com/husarion/panther_ros.git src/panther_ros
```

### Build

``` bash
vcs import src < src/panther_ros/panther/panther_hardware.repos
if [ "$HUSARION_ROS_BUILD_TYPE" == "simulation" ]; then
  vcs import src < src/panther_ros/panther/panther_simulation.repos
fi

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to panther --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

>[!NOTE]
> To build code on a real robot you need to run above commands on the Panther Built-in Computer.

### Running

Real robot:

```bash
ros2 launch panther_bringup bringup.launch.py
```

Simulation:

```bash
ros2 launch panther_gazebo simulation.launch.py
```


## Developer Info

### Setup pre-commit

This project uses pre-commit to maintain high quality of the source code. Install precommit after downloading the repository to apply the changes.

```bash
pre-commit install
```
