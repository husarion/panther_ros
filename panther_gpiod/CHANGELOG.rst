^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_gpiod
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-03-29)
------------------
* Merge pull request `#257 <https://github.com/husarion/panther_ros/issues/257>`_ from husarion/ros2-headers
  Divide Headers into std and local liblaries
* Rest of fils
* Headers + Copyright
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-manager-plugins
* undo test remove (`#244 <https://github.com/husarion/panther_ros/issues/244>`_)
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
* ROS 2 control liblely instalation (`#239 <https://github.com/husarion/panther_ros/issues/239>`_)
  * install liblely with cmake
  * simplify instalation
  * fix condition
  * add super build
  * libgpiod super build
  * remove obsolate info
  * add missing PKG_CONFIG_PATH
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#208 <https://github.com/husarion/panther_ros/issues/208>`_ from husarion/ros2-control
  Add ROS 2 control
* Fix IsPinAvailable calls
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
* Ros2 code style fixes (`#215 <https://github.com/husarion/panther_ros/issues/215>`_)
  * Fix style of cstdint usage in battery
  * Fix style of cstdint usage in lights
  * Unify handling exceptions
  * Fix formatting
* Merge branch 'ros2-devel' into ros2-add-mecanum-controller
* Ros2 - Add GPIO Tests (`#220 <https://github.com/husarion/panther_ros/issues/220>`_)
  * initial aproach
  * Update README.md
  * fix typo
  * Resolve GPIOMonitor thread blocking issue
  * add tests
  * fix typo
  * update readme
  * fix cmake
  * small fixes
  * Update panther_gpiod/test/test_gpiod.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * review fixes
  * add missing tests
  * review fixes
  * review fixes
  * fix package.xml
  ---------
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Merge branch 'ros2-control' into ros2-control-pdo-commands
  Conflicts:
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
  panther_description/urdf/panther_macro.urdf.xacro
  panther_hardware_interfaces/CMakeLists.txt
  panther_hardware_interfaces/CODE_STRUCTURE.md
  panther_hardware_interfaces/README.md
  panther_hardware_interfaces/include/panther_hardware_interfaces/canopen_controller.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/motors_controller.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/panther_system.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/panther_system_ros_interface.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_data_converters.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_driver.hpp
  panther_hardware_interfaces/src/canopen_controller.cpp
  panther_hardware_interfaces/src/motors_controller.cpp
  panther_hardware_interfaces/src/panther_system.cpp
  panther_hardware_interfaces/src/panther_system_ros_interface.cpp
  panther_hardware_interfaces/src/roboteq_driver.cpp
* Merge branch 'ros2-devel' into ros2-control-pdo-commands
  Conflicts:
  panther_bringup/launch/bringup.launch.py
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
* CR suggestions - move configureRT to panther_utils
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
* ROS 2 - Add GPIO driver package (`#211 <https://github.com/husarion/panther_ros/issues/211>`_)
  * driver draft
  * refactor and briefs
  * unique_lock small fixes
  * fix typo
  * Update panther_gpiod/include/panther_gpiod/gpio_driver.hpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_gpiod/src/gpio_driver.cpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * review fixes
  * remove bias from GPIOInfo
  * add rt config
  * change thread priority
  * add REDME
  * fix typo
  * small fixes
  * fix rt cofig handle || update pins release
  * review fixes
  * almost there, closer than ever before
  * fix typo
  * brief update
  * add enable_gpio_monitoring param
  * review fixes
  * small fix
  * add condition_variable
  * add GPIOMonitorEnable()
  * update brief
  * Update panther_gpiod/src/gpio_driver.cpp
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
  * Update panther_gpiod/src/gpio_driver.cpp
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
  * Update README.md
  * Very small update
  ---------
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  Co-authored-by: Maciej Stępień <maciek1284@outlook.com>
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Maciej Stępień, Paweł Irzyk, Paweł Kowalski, rafal-gorecki
