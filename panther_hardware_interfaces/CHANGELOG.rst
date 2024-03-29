^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_hardware_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-03-29)
------------------
* Merge pull request `#258 <https://github.com/husarion/panther_ros/issues/258>`_ from husarion/ros2-control-fix-err-flag-reset
  ROS 2- Fix Error Clearing Mechanism for Roboteq Controllers
* fixes for pth 1.06
* update brief
* move PDO operations to wrapper function
* fix clearing errors
* Merge pull request `#257 <https://github.com/husarion/panther_ros/issues/257>`_ from husarion/ros2-headers
  Divide Headers into std and local liblaries
* Group and order improvement
* ROS 2 - Add GPIOController Tests (`#247 <https://github.com/husarion/panther_ros/issues/247>`_)
  * undo test remove
  * move GPIOController to another branch :p
  * add GPIOController tests
  * review fixes
  * small fixes
  * fix formatting
* Ros2 control add tests (`#253 <https://github.com/husarion/panther_ros/issues/253>`_)
  * undo test remove
  * move GPIOController to another branch :p
  * fix build and typos
  * Fix test naming
  * fix tests
  * clean up code
  * fix includes
  ---------
  Co-authored-by: Paweł Kowalski <kowalski.pawel.r@gmail.com>
* Rest of fils
* Headers + Copyright
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge pull request `#254 <https://github.com/husarion/panther_ros/issues/254>`_ from husarion/ros2-liblely-disable-debug-logs
  ROS 2 disable liblely debug logs
* Define `NDEBUG` macro
* ROS 2 - Fix estop threads (`#249 <https://github.com/husarion/panther_ros/issues/249>`_)
  * fix header stamp and qos
  * add MultiThreadedExecutor
  * Add compatibility with PTH 1.06
  * small fixes
  * review fixes
  * Disable CAN usage for EStop trigger on PTH >= 1.2"
  * fix typo
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* fix qos (`#250 <https://github.com/husarion/panther_ros/issues/250>`_)
* Merge pull request `#240 <https://github.com/husarion/panther_ros/issues/240>`_ from husarion/ekf_optimalization
  Ekf optimalization
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Change relative to abs speed frame
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-manager-plugins
* update  params (`#243 <https://github.com/husarion/panther_ros/issues/243>`_)
* Merge pull request `#245 <https://github.com/husarion/panther_ros/issues/245>`_ from husarion/ros2-fix-roboteq-battery
  ROS 2 - Fix Roboteq Battery
* fix header stamp and qos
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
* Merge pull request `#242 <https://github.com/husarion/panther_ros/issues/242>`_ from husarion/ros2-fix-pointers-loop
  ROS2 - Fix Pointer Cyclic Dependencies
* fix pointer cyclic dependencies
* ROS2 - Fix Power Motor Service (`#238 <https://github.com/husarion/panther_ros/issues/238>`_)
  * add new functionality
  * review fixes
* ROS 2 control liblely instalation (`#239 <https://github.com/husarion/panther_ros/issues/239>`_)
  * install liblely with cmake
  * simplify instalation
  * fix condition
  * add super build
  * libgpiod super build
  * remove obsolate info
  * add missing PKG_CONFIG_PATH
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
* Merge pull request `#233 <https://github.com/husarion/panther_ros/issues/233>`_ from husarion/ros2-update-service-wrapper
  ROS 2 - Update Service Wrapper
* review fixes
* update service wrapper
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#208 <https://github.com/husarion/panther_ros/issues/208>`_ from husarion/ros2-control
  Add ROS 2 control
* Temporarily remove tests
  tests were moved to ros2-control-add-tests branch and should be merged after additional review process
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
* Merge pull request `#219 <https://github.com/husarion/panther_ros/issues/219>`_ from husarion/ros2-control-pdo-commands
  ros2_control PDO commands
* Refactor tests
* CR suggestions - use future in roboteq driver boot
* CR suggestions - change to lock guard and fix locking range
* CR suggestions - move roboteq mock methods implementation
* CR suggestions - move flags reading to a separate variable
* CR suggestions - update readme
* CR suggestions - readme fixes
* Refactor panther system
* CR suggestions
* Remove old todo comment
* Update coment
* Add std to int types
* Update tests
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
* CR suggestions - add default FlagError destructor
* CR suggestions - update error msg and refactor checksafetystop method
* CR suggestions - fix consts
* CR suggestions - add exception msg in service
* CR suggestions - fix includes in motor controller
* CR suggestions - update roboteq driver briefs
* CR suggestions - move configureRT to panther_utils
* CR suggestion - create roboteq error filter cpp file for implementations
* CR suggestion - move longer methods to cpp file
* CR suggestions - add package links in readme
* CR suggestions - add tags to readme
* CR suggestions - add more thorough checking of joint names
* CR suggestions - add node name and options parameters
* Move initialization and activation of ros interface to constructor (and destructor)
* Add checking initialization state in canopen and motor controllers
* CR suggestions - make RoboteqCANObjects static
* CR suggestions
* CR suggestion - fix CAN, PDO, SDO, CANopen names
* CR suggestions - add ms to timeouts
* CR suggestions
* Add checking if joint name doesn't contain any reserved sequences (fl fr rr rl)
* Remove unnecessary string literals
* Update roboteq error filter
* CR suggestions
* Move ids and subids of canopen objects into seperate struct
* Update communication parameters
* Update readme
* CR suggestions - fix includes
* Remove todos
* Remove comment and todos
* Remove additional timeout in sdo operations
* Use sdo operation timeout parameter
* Fix naming and update documentation
* Update documentation
* Update ignored runtime errors
* Update volts amps and battery names
* Update log messages
* Change return failure to error (in this cases on_error method should be triggered)
* Fix destroying canopen controller
* Update PDO driver state timeout log
* Switch to loop driver (better performance)
* Fix destroying objects
* Fix spinning in panther system ros interface
* Add configurable driver states update frequency
* Rename eds file
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
* Remove old gpio driver and temporarily comment out tests
* Update whole system to use new pdo communication and add proper timeouts
* Add heatsink temperature
* New pdo configuration
* CR suggestions - use bitset in flag errors
* Change setting init value of flags to just 0
* Fix constant name
* CR suggestions - getbyte as template
* CR suggestions - consts in overridden methods
* CR suggestions - add const to submit write
* CR suggestions - variable name change
* CR suggestions - any of and auto
* CR suggestions - std array
* CR suggestions - change constructor parameter types
* Move additional wait to constant member
* Move can interface name to parameter
* CR suggestions - rename canopen configuration file
* CR suggestions - cstdint types
* Change reading driver state to pdo and update pdo remapping
* Add missing dependencies
* CR suggestions
* Change commands to pdos, update sdo operations and update reading pdos
* Update eds to fw21a and change it to match new sent data
* Update todo comments
* Add warning about safety critical parameters
* Refactor panther system test utils
* Move code structure to separate file
* Refactor setting surpressed flags
* Add set bit utility function
* Decrease wait timeout
* Refactor error filter ids
* Add comment about sdo operation deadlock
* Add comment about can loop error
* Refactor update error msg
* Update todos
* Move service name to constants
* Refactor test_update_system_pdo_feedback_timeout
* Move topic name to constants
* Use WaitForMsg from panther utils
* Add boot timeout test
* Update msgs in boot exceptions
* Add first channel check in safety stop test
* Refactor motor controller state msg - remove joint name and move runtime error
* Update todos
* Refactor updatemsgerrors method
* Refactor locks usage
* Move can error flag to MotorControllerState
* Use wait for msg function from panther utils
* Add plugin name constant
* Refactor test constants naming
* Move settings to constants
* Remove comment
* Fix lock naming
* Add timeout when waiting for boot
* Refactor error filters
* Move setting safetystop, so that it can be set faster in the write function
* Refactor tests
* Fix types and casting in tests
* Remove comments
* Change types from double to float
* Refactor panther system logging and fix throttling
* Grammar fixes
* Refactor tests - add namespace and fix roboteq mock file name
* Refactor test utils
* Refactor - rename panther wheels controller to motors controller and fix order of methods/variables
* Remove comments from roboteq driver
* Refactor - fix include guards
* Refactor - rename variables
* Remove unused variable
* Refactor - change panther system node name to ros interface
* Refactor - fix naming
* Fix roboteq naming
* Make handling exceptions unified
* Update readme
* Refactor tests
* Use typename in templates
* Refactor roboteq driver
* Refactor roboteq driver - separate channel operations
* Precommit changes
* Remove wait in initialization
* Move OperationWithAttempts and add tests
* Fix panther system onerror test
* Add panther system onerror test
* Move setup/teardown to constructor/destructor
* Add wrong order urdf test
* Update comments
* Fix roboteq driver tests
* Add pdo and read sdo timeout tests
* Use atomic_bool type alias
* [WIP] Refactor panther system tests
* Add sdo timeout test and refactor tests
* Refactor and add tests for utils
* [WIP] Update roboteq driver comments
* Add briefs to data converters
* Refactor panther wheels controller
* [WIP] Refactor panther system
* Refactor system node and add tests
* Refactor panther system node and add documentation
* Refactor canopen controller
* Refactor panther system node
* Move panther system node to new files
* [WIP] Refactor panther system
* [WIP] Refactor panther system - move node functionalities to separate class
* [WIP] Refactor panther system
* Refactor error filter
* Refactor can controller
* Add flags and timestamps tests to roboteq driver tests
* [WIP] Add wheels controller tests
* [WIP] Add roboteq driver tests
* Add can controller test
* Update setting wait in roboteq mock
* Refactor - create can_controller class
* Seperate boot errors handling
* Remove old todos
* Remove unnecessary headers
* Update data converters
  refactor
  fix voltage calculation
* Add data converters test
* Add parameter description to readme
* [WIP] Update readme
* Fix edge case when multiple sdo operations are queued
* [WIP] Fix system error
* Add operation attempts method
* Remove turn off estop in activation (no longer needed)
* Fix clearing errors
* Add comment
* Change unnecessary uint8_t types
* Make clearing errors multi thread safe
* Fix turn on safety stop
* Add safety stop attempts, fix counter types, fix updating pdo error
* Add safety stop
* Add clear errors service
* Use ptrs from rclcpp
* Fix urdf in tests
* Fix initialization and activation attempts
* Move timeouts and attempts to parameters
* Add old data info to state msg
* Add error log about roboteq errors and refactor flag errors
* Move feedback timeout to parameter
* Fix unique/shared ptrs
* Change default c++ version to 17
* Add more roboteq intialization/activation attempts
* Refactor error handler and add tests
* Add todos
* Increase can thread priority
* Rename methods
* Separate SDO and PDO errors
* Add test urdf with changed order
* Comment out timeout test
* Add todos
* Refactor panther system
  add error handler and move code to separate functions
* Add hardware interface readme
* Add sdo write timeout test
* Fix setting error and add error to msg
* Add more error handling in initialization
* Update timeouts
* Add sdo read and write error counts (allow some failures before escalating)
* Update feedback timeout
* Fix channel order in tests
* Fix motor order
* Increase sdo timeout
* Update hardware interfaces test readme
* Fix temperature type
* Update initial procedure test
* Update test slave bin (change heartbeat and eds)
* Fix eds version
  some other version than for firmware 2.1 was used
* Remove comment
* Rename variable
* Remove not used stuff
* Rename data converters
* Read single sdo value every read cycle
* Refactor data conversion
* Remove visibility control (windows is not supported)
* Update encoder disconnected test
* Refactor - move feedback converters, proper error handling
* Rename tests file
* Refactor tests
* Refactor tests - add setup and teardown
* Add waiting for mock start in tests
* [WIP] Refactor tests
* Add initial procedure test
* Add reading roboteq feedback test
* Fix calculating current
* Add encoder disconnected test
* Change function name
* Add reading test
* Add writing test
* Add deactivate unconfigure test
* Remove using namespace lely
* Change io guard to local variable
* Fix deinitialization of panther wheels controller
* Fix memory problems in roboteq mock
* [WIP] Add roboteq mock tests
* Refactor roboteq mock
* Fix temp sdo data type
* Add mock slave configuration
* Add roboteq mock for tests
* Add checking state in test
* Fix handling executor in hardware system
* Add can executor thread join on deinitialize
* Remove unnecessary thread include
* Add panther load system test
* Remove comments
* Rename joint size variable and check if hardware parameters were defined
* Add information when RT can thread is used
* Add safety stop
* Rename and adjust feedback timeout
* Add handling error when reseting roboteq script
* Add turning off estop on activation
* Add triggering estop to on_error
* Add wait timeout
* Fix building
* Update sched priority of can node
* Add reading other roboteq driver feedback
* Refactor checking flags
* Refactor types
* Update sdo communication
* Add comments
* Refactor handling commands and states
* Remove torque control code
* Add todo comments
* Fix handling error flags
* Move roboteq cmd and feedback conversion to roboteq driver
* Add default value
* Add wait for boot and fix handling can exceptions
* Add comments
* Update error handling
* Fix hardware interface
* Add error handling
* [WIP] Refactor
* Refactor
* Fix build
* Add eds config
* Add ros2 control
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Maciej Stępień, Paweł Irzyk, Paweł Kowalski, pawelirh, rafal-gorecki
