^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_battery
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2024-05-01)
------------------
* Merge pull request `#261 <https://github.com/husarion/panther_ros/issues/261>`_ from husarion/ros2-readme
* Pawel sugestions
* Merge branch 'ros2-devel' into ros2-readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
* Add controller readme
* Contributors: Jakub Delicat, Paweł Irzyk, rafal-gorecki

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
* Group and order improvement
* Rest of fils
* Headers + Copyright
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* fix qos (`#250 <https://github.com/husarion/panther_ros/issues/250>`_)
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-manager-plugins
* Merge pull request `#245 <https://github.com/husarion/panther_ros/issues/245>`_ from husarion/ros2-fix-roboteq-battery
  ROS 2 - Fix Roboteq Battery
* fix header stamp and qos
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#208 <https://github.com/husarion/panther_ros/issues/208>`_ from husarion/ros2-control
  Add ROS 2 control
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
* Merge branch 'ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#228 <https://github.com/husarion/panther_ros/issues/228>`_ from husarion/ros2-update-utils
  Move ros test utils to separate file and add ExpectThrowWithDescription
* move ros test utils to separate file and add ExpectThrowWithDescription
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
* Update roboteq battery after changes in msg
* Merge pull request `#209 <https://github.com/husarion/panther_ros/issues/209>`_ from husarion/ros2-battery-fix-merge
  fix branch merge error
* Make handling exceptions unified
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
* fix branch merge error
* ROS 2 add Roboteq Battery (`#206 <https://github.com/husarion/panther_ros/issues/206>`_)
  * add adc data reader
  * add adc node and tests
  * update nodes and tests
  * add readings timeout
  * update tests
  * data reader fixes
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * add adc to battery converter
  * battery class
  * battery params
  * fix republisher node
  * update adc node
  * update tests
  * update adc_node test
  * small fixes and formating
  * update headers and create battery.cpp
  * use shared ptr for adc readers
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_node.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * fix
  * update adc node
  * update tests
  * update tests
  * reorganize tests
  * update adc_data_reader
  * add battery_publisher class
  * clean up battery class
  * separate headers for bat publishers
  * add adc_battery subclass
  * update battery publisher
  * fix ADCBattery
  * fix error logging
  * add battery tests
  * add battery_publisher test
  * add single and dual battery publisher tests
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * formating and small fixes
  * update readme
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * add MergeBatteryPowerSupplyHealth method
  * code formatting
  * fix bat params order
  * review fixes
  * update test_utils and add test for it
  * update tests
  * remove unnecessary try catch
  * add roboteq battery and battery node
  * add roboteq battery tests
  * validate driver state in roboteq_battery
  * update includes
  * ROS 2 adc node refactor (`#202 <https://github.com/husarion/panther_ros/issues/202>`_)
  * update adc_data_reader
  * add battery_publisher class
  * clean up battery class
  * separate headers for bat publishers
  * add adc_battery subclass
  * update battery publisher
  * fix ADCBattery
  * fix error logging
  * add battery tests
  * add battery_publisher test
  * add single and dual battery publisher tests
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * formating and small fixes
  * update readme
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * add MergeBatteryPowerSupplyHealth method
  * code formatting
  * fix bat params order
  * review fixes
  * update test_utils and add test for it
  * update tests
  * remove unnecessary try catch
  * update battery virtual methods
  * review fixes
  * small fix
  * fixes again
  * change MergeBatteryPowerSupplyStatus logic
  ---------
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * update CmakeLists
  * update README
  * remove unnecessary files
  * formatting
  * add copyright
  * update authors
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * add unused param comments
  * review fixes
  * fix formatting
  ---------
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
* Add pre-commit, clang-format and license to files (`#207 <https://github.com/husarion/panther_ros/issues/207>`_)
  Add pre-commit, clang-format and license to files
* ROS 2 adc node (`#135 <https://github.com/husarion/panther_ros/issues/135>`_)
  * add adc data reader
  * add adc node and tests
  * update nodes and tests
  * add readings timeout
  * update tests
  * data reader fixes
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/README.md
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * add adc to battery converter
  * battery class
  * battery params
  * fix republisher node
  * update adc node
  * update tests
  * update adc_node test
  * small fixes and formating
  * update headers and create battery.cpp
  * use shared ptr for adc readers
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/battery.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_node.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * fix
  * update adc node
  * update tests
  * update tests
  * reorganize tests
  * ROS 2 adc node refactor (`#202 <https://github.com/husarion/panther_ros/issues/202>`_)
  * update adc_data_reader
  * add battery_publisher class
  * clean up battery class
  * separate headers for bat publishers
  * add adc_battery subclass
  * update battery publisher
  * fix ADCBattery
  * fix error logging
  * add battery tests
  * add battery_publisher test
  * add single and dual battery publisher tests
  * Update panther_battery/src/adc_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * formating and small fixes
  * update readme
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/src/battery_publisher.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/include/panther_battery/adc_data_reader.hpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * add MergeBatteryPowerSupplyHealth method
  * code formatting
  * fix bat params order
  * review fixes
  * update test_utils and add test for it
  * update tests
  * remove unnecessary try catch
  * update battery virtual methods
  * review fixes
  * small fix
  * fixes again
  * change MergeBatteryPowerSupplyStatus logic
  ---------
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  ---------
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
* review fixes
* review fixes
* Fix battery type from li-pol to li-ion (`#138 <https://github.com/husarion/panther_ros/issues/138>`_)
* Baterry capacity unmeasured to nan (`#136 <https://github.com/husarion/panther_ros/issues/136>`_)
  * Baterry capacity unmeasured to nan
  * Update tests
  * Remove bat_capacity\_ param
* fix current for roboteq republisher (`#133 <https://github.com/husarion/panther_ros/issues/133>`_)
* ROS 2 panther battery package (`#128 <https://github.com/husarion/panther_ros/issues/128>`_)
  * add panther_battery package
  * update roboteq_republisher_node
  * small fixes
  * add moving average
  * small fixes
  * update launch
  * add republisher test
  * update tests
  * add test utils
  * small fixes
  * Update panther_battery/test/test_roboteq_republisher_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/test/test_roboteq_republisher_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/test/test_roboteq_republisher_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/test/test_roboteq_republisher_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * Update panther_battery/test/test_roboteq_republisher_node.cpp
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
  * review fixes
  * update tests
  * another test update
  * review fixes
  * add README
  * small fixes
  ---------
  Co-authored-by: Krzysztof Wojciechowski <49921081+Kotochleb@users.noreply.github.com>
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Krzysztof Wojciechowski, Maciej Stępień, Paweł Irzyk, Paweł Kowalski, rafal-gorecki
