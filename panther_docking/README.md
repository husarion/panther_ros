ros2 action send_goal /panther/dock_robot opennav_docking_msgs/action/DockRobot "{dock_id: main_dock, navigate_to_staging_pose: false}"
ros2 launch panther_bringup bringup.launch.py namespace:=panther components_config_path:=$(pwd)/gazebo_components.yaml
PANTHER_ROBOT_VERSION=1.21 ros2 launch panther_docking docking.launch.py namespace:=panther
