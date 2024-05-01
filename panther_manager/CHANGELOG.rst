^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2024-05-01)
------------------
* Merge pull request `#261 <https://github.com/husarion/panther_ros/issues/261>`_ from husarion/ros2-readme
* Pawel sugestions
* Merge branch 'ros2-devel' into ros2-readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
* Add controller readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
* Contributors: Jakub Delicat, Paweł Irzyk, rafal-gorecki

2.0.0 (2024-03-29)
------------------
* Merge pull request `#258 <https://github.com/husarion/panther_ros/issues/258>`_ from husarion/ros2-control-fix-err-flag-reset
  ROS 2- Fix Error Clearing Mechanism for Roboteq Controllers
* fixes for pth 1.06
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
* Merge pull request `#246 <https://github.com/husarion/panther_ros/issues/246>`_ from husarion/ros2-panther-manager
  ROS 2 panther_manager
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge pull request `#232 <https://github.com/husarion/panther_ros/issues/232>`_ from husarion/ros2-manager-plugins
  ROS 2 manager plugins
* fix tests
* add missing params and fix default launch
* Fixed typo | cleaned up the test_shutdown_hosts_node
* fix
* Added shutdonw hosts node
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* remove MultiThreadedExecutor
* Add suggestions
* Add README
* remove bad file
* Added all suggestions
* fixed user to username | added test_shutdown_host
* fixed tick after timeout node
* add MultiThreadedExecutor
* review fixes
* fix merge and move files
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* fixed tests
* Changed tests' names to PascalCase | added testing::TempDir() | Starting services when there are wrong parameters
* Added RegisterNode template function
* Made  fixed commit for behaviortreee | templated create service function | removed unused warnings
* applied panther_utils
* Changed utils names and moved start stop to constructor and destructor
* Added package suggestions without tests
* fix bb constant name
* add missing dependencies
* add manager_bt_node tests
* add log if tree fails
* clean up code
* port manager to ROS 2
* added tests for tick_after_timeout
* Added TickAfterTimeout
* typo
* tested on the robot
* Fixed tesT
* Revert "added test for single host plugin"
  This reverts commit a4f9051c8dfcf03cefa4f827904126fb50c0b316.
* added test for single host plugin
* added test single plugin
* Added tests for signal shutdown plugin
* Fixed building trigger
* Removed different types of building behavio tree
* Moved pluigns to actions | added shutdown plugins
* Added explicite casts
* Added tests for set animation plugin
* Added trigger tests
* Added full call_set_bool plugin test
* Added trigger service call plugin
* added set bool service
* added set bool service
* added behaviortree_ros2 to the repository because it is not in rosped
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Paweł Irzyk, Paweł Kowalski, rafal-gorecki
