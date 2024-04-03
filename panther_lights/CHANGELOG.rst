^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_lights
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* few more
* Group and order improvement
* Rest of fils
* Headers + Copyright
* Merge pull request `#234 <https://github.com/husarion/panther_ros/issues/234>`_ from husarion/ros2-lights-tests
  ROS 2 lights tests
* Merge pull request `#246 <https://github.com/husarion/panther_ros/issues/246>`_ from husarion/ros2-panther-manager
  ROS 2 panther_manager
* Merge pull request `#232 <https://github.com/husarion/panther_ros/issues/232>`_ from husarion/ros2-manager-plugins
  ROS 2 manager plugins
* Clean up
* Add Dawid suggestions
* fix
* Comment
* Test Pass
* Add launch behavior
* Clean up
* Rename
* Almost passing
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Rename topics
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Changed tests' names to PascalCase | added testing::TempDir() | Starting services when there are wrong parameters
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
* GPIO release
* add getter
* Next tests
* Clean up
* Add first tests
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#208 <https://github.com/husarion/panther_ros/issues/208>`_ from husarion/ros2-control
  Add ROS 2 control
* Fix IsPinAvailable calls
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_gpiod/CMakeLists.txt
  panther_gpiod/package.xml
  panther_gpiod/src/gpio_driver.cpp
* Update readme in battery and lights after diagnostics changes (`#230 <https://github.com/husarion/panther_ros/issues/230>`_)
  * Update readme in battery and lights after diagnostics changes
  * Update panther_lights/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  ---------
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Ros2 code style fixes (`#215 <https://github.com/husarion/panther_ros/issues/215>`_)
  * Fix style of cstdint usage in battery
  * Fix style of cstdint usage in lights
  * Unify handling exceptions
  * Fix formatting
* Ros2 diagnostics (`#224 <https://github.com/husarion/panther_ros/issues/224>`_)
  * Implement diagnostics in panther_battery
  * Correct class diagnostic updater member name
  * Order panther battery dependencies
  * Add diagnostics to panther lights
  * Minor diagnostics changes
  * Improve messages and add broadcasting in lights
  * Add broadcasting in battery node
  * Add additional diagnostic in battery
  * Change pointers policy
  * Review changes
  * Restore LogError
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
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_bringup/launch/bringup.launch.py
* ROS 2 lights gpio handle (`#213 <https://github.com/husarion/panther_ros/issues/213>`_)
  * use gpio driver
  * fix build
  * review fixes
* ROS 2 panther lights (`#210 <https://github.com/husarion/panther_ros/issues/210>`_)
  * add panther_lights package
  * add ROS 2 lights_driver_node
  * add dummy controller node
  * fix driver
  * fix on shutdown cleanup
  * add ros synchronous client
  * update apa102 driver
  * use service to power up LEDs
  * add Copyright
  * revert add ros synchronous client
  * revert use service to power up LEDs
  * make it work
  * use bool value in SetPowerPin
  * libgpiod installation in CMake
  * rename nodes
  * update README
  * change naming
  * code fixes
  * add brief
  * update authors
  * Update panther_lights/src/apa102.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_lights/src/apa102.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_lights/src/apa102.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * fix lights
  * update methods briefs
  * simplify condition
  * Update panther_lights/src/apa102.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_lights/src/apa102.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * Update panther_lights/src/apa102.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  ---------
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Maciej Stępień, Paweł Irzyk, rafal-gorecki
