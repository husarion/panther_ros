# panther_gazebo

The package contains a launch file and source files used to run the robot simulation in Gazebo. The simulator tries to reproduce the behavior of a real robot as much as possible, including the provision of an analogous ROS_API.

## Launch Files

- `spawn_robot.launch.py`: Responsible for spawning the robot in the simulator.
- `simulate_robot.launch.py`: Responsible for giving birth to the robot and simulating its physical behavior, such as driving, displaying data, etc.
- `simulate_multiple_robots.launch.py`: Similar to the above with logic allowing you to quickly add a swarm of robots.
- **`simulation.launch.py`**: A target file that runs the gazebo simulator that adds and simulates the robot's behavior in accordance with the given arguments.

## Configuration Files

- [`battery_plugin_config.yaml`](./config/battery_plugin_config.yaml): Simulated LinearBatteryPlugin configuration.
- [`gz_bridge.yaml`](./config/gz_bridge.yaml): Specify data to exchange between ROS and Gazebo simulation.
- [`led_strips.yaml`](./config/led_strips.yaml): Configure properties of led strips in simulation to animate lights.
