^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-03-29)
------------------
* Ros2 namespace (`#255 <https://github.com/husarion/panther_ros/issues/255>`_)
  * Preparation for namespace
  * Simulation working
  * Hardware look ok
  * Update panther_controller/config/WH01_controller.yaml
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  * Apply Jakub suggestions
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  * Fix imu
  * Readme
  * Add imu namespace
  * Jakub suggestions
  * Add panther manager to xml
  * pre-commit
  * Fixed ekf
  * Additional remapping
  * fix imu
  * Pawel suggestions (collision with gamepad)
  * cmd_vel
  * Use namespace instead of PushRosNamespace
  ---------
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  Co-authored-by: Jakub Delicat <jakub.delicat@husarion.com>
* Merge pull request `#257 <https://github.com/husarion/panther_ros/issues/257>`_ from husarion/ros2-headers
  Divide Headers into std and local liblaries
* Headers + Copyright
* Merge pull request `#246 <https://github.com/husarion/panther_ros/issues/246>`_ from husarion/ros2-panther-manager
  ROS 2 panther_manager
* Add launch behavior
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge pull request `#240 <https://github.com/husarion/panther_ros/issues/240>`_ from husarion/ekf_optimalization
  Ekf optimalization
* fix
* Add initial ekf setting
* Update panther_bringup/config/ekf.yaml
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge pull request `#251 <https://github.com/husarion/panther_ros/issues/251>`_ from husarion/ros2-build-depend
  Hardware / Sim Dependencies
* Update panther_bringup/launch/bringup.launch.py
* FindPackageShare
* Update panther_bringup/package.xml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* HW/SIM Dependencies
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-manager-plugins
* Ros2 lights controller (`#241 <https://github.com/husarion/panther_ros/issues/241>`_)
  * ROS 2 lights animations (`#221 <https://github.com/husarion/panther_ros/issues/221>`_)
  * add animation and image_animation class
  * controller node and pluginlib
  * add tests and fix issues
  * add animation images
  * add alpha channel
  * add charging animation with tests
  * update dummy controller
  * fix missing includes
  * add missing dep
  * Update panther_lights/include/panther_lights/animation/animation.hpp
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  * Update panther_lights/include/panther_lights/animation/animation.hpp
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  * review changes
  * update tests
  ---------
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  * ROS 2 lights converter (`#223 <https://github.com/husarion/panther_ros/issues/223>`_)
  * add led_segment
  * WIP led_panel and segment converter
  * simplify converter
  * update segment conversion
  * add test for led panel, segment, and converter
  * review fixes
  * update copyright year
  * update controller so it somehow works
  * Update tests
  * Apply review fixes
  * fix gpio tests
  * parse controller configuration
  * add default animation
  * add yaml_utils to panther_utils
  * add led animation and queue
  * Fix queuing
  * fix bug
  * priority and timeout queue validation
  * move queue to separate file
  * add briefs
  * param and brightness handle
  * user animations, bugs, briefs
  * use yaml utils
  * fix tests
  * update tests
  * add led_animation test
  * test fixxes
  * add led animations queue tests
  * clean up code | clean up code
  * Update documentation | add launching controller node
  * make it work
  * update scheduler
  * Update panther_lights/LIGHTS_API.md
  Co-authored-by: Paweł Irzyk <108666440+pawelirh@users.noreply.github.com>
  * review fixes
  * update pre-commit and fix typos
  * Update panther_bringup/README.md
  Co-authored-by: rafal-gorecki <126687345+rafal-gorecki@users.noreply.github.com>
  * Update panther_hardware_interfaces/README.md
  Co-authored-by: rafal-gorecki <126687345+rafal-gorecki@users.noreply.github.com>
  * Update panther_lights/README.md
  Co-authored-by: rafal-gorecki <126687345+rafal-gorecki@users.noreply.github.com>
  * Update panther_lights/test/test_controller_node.cpp
  Co-authored-by: rafal-gorecki <126687345+rafal-gorecki@users.noreply.github.com>
  * review fixes
  * Update README.md
  ---------
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  Co-authored-by: Paweł Irzyk <108666440+pawelirh@users.noreply.github.com>
  Co-authored-by: rafal-gorecki <126687345+rafal-gorecki@users.noreply.github.com>
* EKF config
* Merge pull request `#235 <https://github.com/husarion/panther_ros/issues/235>`_ from husarion/ros2-dependencies
  Fix dependencies
* Ros2 diagnostics hardware interfaces (`#231 <https://github.com/husarion/panther_ros/issues/231>`_)
  * add GPIO controller
  * Basic integration of gpio controller and panther system
  * [WIP] Add panther version
  * add io state topic
  * Remove unnecessary parts from cmakelists
  * Cleanup gpio controller
  * Add estop to panther system
  * Add todo comment
  * Add ServiceWrapper
  * Add estop services
  * Add remaps to ros2 control
  * Add publishing estop state, change iostate to latched and fix publishing initial state
  * revise e-stop logic in initial stage
  * same, but in better way
  * small changes
  * remove clear_errors service
  * Fix test
  * Add resetting gpio controller
  * Change wheel separation multiplier to 1.0
  * fix pin names list
  * add robot version check before GPIO read
  * Change lock in gpio driver
  * Fix order in cmakelists
  * Change throws to exception in briefs
  * Remove unnecessary includes
  * Fix controller_manager topic remaps
  * Add checking if last commands were 0 before resetting estop
  * Change estop variable to atomic bool
  * Add motor controller mutex
  * Change order of operations when setting estop
  * Fix order of methods
  * Fixes in panther system - change methods order, use ReadDriverStatesUpdateFrequency, remove unnecessary logs
  * Remove max_safety_stop_attempts (no longer needed after adding gpio controller)
  * Refactor setting estop in write method
  * Fix estop naming convention
  * Remove old todos
  * Fix typo
  * Review fixes
  * fix formatting
  * Update panther_hardware_interfaces/include/panther_hardware_interfaces/gpio_controller.hpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * review fixes
  * rename some methods
  * draft of InitializeAndPublishIOStateMsg functionality
  * Initialize diagnostic updater
  * Update docs
  * fix io_state topic
  * fix service warappers
  * small fix
  * Add missing dependencies
  * Implement diagnostics tasks
  * Add header file to panther_system
  * Add get map methods
  * Add utilities and tests
  * Fix mistaken removal
  * Fix method order
  * Update panther_hardware_interfaces/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_hardware_interfaces/src/panther_system.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_hardware_interfaces/src/panther_system.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_hardware_interfaces/src/panther_system.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_hardware_interfaces/src/roboteq_data_converters.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_utils/include/panther_utils/common_utilities.hpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_utils/include/panther_utils/diagnostics.hpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_utils/test/test_common_utilities.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_utils/test/test_diagnostics.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Add additional test in test_diagnostics
  ---------
  Co-authored-by: Paweł Kowalski <kowalski.pawel.r@gmail.com>
  Co-authored-by: Maciej Stępień <maciej.stepien@husarion.com>
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Decrease bringup time
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#208 <https://github.com/husarion/panther_ros/issues/208>`_ from husarion/ros2-control
  Add ROS 2 control
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_gpiod/CMakeLists.txt
  panther_gpiod/package.xml
  panther_gpiod/src/gpio_driver.cpp
* Add GPIO controller (`#222 <https://github.com/husarion/panther_ros/issues/222>`_)
  * add GPIO controller
  * Basic integration of gpio controller and panther system
  * [WIP] Add panther version
  * add io state topic
  * Remove unnecessary parts from cmakelists
  * Cleanup gpio controller
  * Add estop to panther system
  * Add todo comment
  * Add ServiceWrapper
  * Add estop services
  * Add remaps to ros2 control
  * Add publishing estop state, change iostate to latched and fix publishing initial state
  * revise e-stop logic in initial stage
  * same, but in better way
  * small changes
  * remove clear_errors service
  * Fix test
  * Add resetting gpio controller
  * Change wheel separation multiplier to 1.0
  * fix pin names list
  * add robot version check before GPIO read
  * Change lock in gpio driver
  * Fix order in cmakelists
  * Change throws to exception in briefs
  * Remove unnecessary includes
  * Fix controller_manager topic remaps
  * Add checking if last commands were 0 before resetting estop
  * Change estop variable to atomic bool
  * Add motor controller mutex
  * Change order of operations when setting estop
  * Fix order of methods
  * Fixes in panther system - change methods order, use ReadDriverStatesUpdateFrequency, remove unnecessary logs
  * Remove max_safety_stop_attempts (no longer needed after adding gpio controller)
  * Refactor setting estop in write method
  * Fix estop naming convention
  * Remove old todos
  * Fix typo
  * Review fixes
  * fix formatting
  * Update panther_hardware_interfaces/include/panther_hardware_interfaces/gpio_controller.hpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * review fixes
  * rename some methods
  * draft of InitializeAndPublishIOStateMsg functionality
  * fix io_state topic
  * fix service warappers
  * small fix
  * rewiew fixes
  * add briefs in gpio_controler
  * review fixes
  * small fix
  ---------
  Co-authored-by: Paweł Kowalski <kowalski.pawel.r@gmail.com>
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Add IMU noise + basic EKF configuration (`#229 <https://github.com/husarion/panther_ros/issues/229>`_)
  * Fix collisions
  * remove parent dir
  * Add IMU noise
  * EKF working
  * Add controller
  * Update panther_bringup/config/ekf.yaml
  * Update panther_bringup/config/ekf.yaml
  * Format
* Merge branch 'ros2-devel' into ros2-control-pdo-commands
  Conflicts:
  panther_bringup/launch/bringup.launch.py
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_bringup/launch/bringup.launch.py
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
* Manuall merge of ros2-prealpha to ros2-dev (`#218 <https://github.com/husarion/panther_ros/issues/218>`_)
  * manually merge prealpha with ros2-dev
  * typo and formatting
  * change locks and simplify code
  * add missing library
  * fix build
* Remove todos
* Merge branch 'ros2-control' into ros2-control-pdo-commands
  Conflicts:
  panther_hardware_interfaces/README.md
  panther_hardware_interfaces/include/panther_hardware_interfaces/canopen_controller.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/panther_system.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_data_converters.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_driver.hpp
  panther_hardware_interfaces/src/motors_controller.cpp
  panther_hardware_interfaces/src/panther_system.cpp
  panther_hardware_interfaces/src/roboteq_driver.cpp
* ROS 2 ekf update (`#214 <https://github.com/husarion/panther_ros/issues/214>`_)
  * update ekf config
  * add ekf launch arguments
  * add README
  * Update panther_bringup/README.md
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
  * Update panther_bringup/README.md
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
  * Update panther_bringup/README.md
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
  ---------
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
* Fix imu launch
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_bringup/launch/bringup.launch.py
* Fix launches
* Merge pull request `#212 <https://github.com/husarion/panther_ros/issues/212>`_ from husarion/ros2-imu-node
  ROS 2 imu node
* review fixes
* fix launch
* update imu config
* add use_sim condition
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  .clang-format
  README.md
  panther_controller/CMakeLists.txt
  panther_controller/launch/controller.launch.py
  panther_controller/package.xml
  panther_description/CMakeLists.txt
  panther_description/config/WH01.yaml
  panther_description/config/WH02.yaml
  panther_description/config/WH04.yaml
  panther_description/meshes/WH01/fl_wheel.dae
  panther_description/meshes/WH01/fr_wheel.dae
  panther_description/meshes/WH01/rl_wheel.dae
  panther_description/meshes/WH01/rr_wheel.dae
  panther_description/meshes/WH02/fl_wheel.dae
  panther_description/meshes/WH02/fr_wheel.dae
  panther_description/meshes/WH02/rl_wheel.dae
  panther_description/meshes/WH02/rr_wheel.dae
  panther_description/meshes/WH04/fl_wheel.dae
  panther_description/meshes/WH04/fr_wheel.dae
  panther_description/meshes/WH04/rl_wheel.dae
  panther_description/meshes/WH04/rr_wheel.dae
  panther_description/meshes/body.dae
  panther_description/meshes/components/external_antenna.dae
  panther_description/package.xml
  panther_description/rviz/panther.rviz
  panther_description/urdf/body.urdf.xacro
  panther_description/urdf/components/external_antenna.urdf.xacro
  panther_description/urdf/panther.urdf.xacro
  panther_description/urdf/panther_macro.urdf.xacro
  panther_description/urdf/wheel.urdf.xacro
* add launching imu nodes
* Add pre-commit, clang-format and license to files (`#207 <https://github.com/husarion/panther_ros/issues/207>`_)
  Add pre-commit, clang-format and license to files
* Merge pull request `#201 <https://github.com/husarion/panther_ros/issues/201>`_ from husarion/ros2-gazebo
  Ros2 gazebo
* review fixes
* add puslish_robot_state param in all files
* rename ekf node
* review fixes
* Update panther_bringup/launch/bringup.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_bringup/launch/bringup.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_bringup/launch/bringup.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_bringup/config/ekf.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* fix controller and ekf covariance
* fix ekf
* grammar fixes
* grammar fixes
* add battery plugin
* add wheel params in launches
* add imu filter and ekf
* initial sim configuration draft
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Krzysztof Wojciechowski, Maciej Stępień, Paweł Irzyk, Paweł Kowalski, rafal-gorecki
