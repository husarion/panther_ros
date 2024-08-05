^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2024-08-02)
------------------
* Fixed gazebo lights tfs (`#377 <https://github.com/husarion/panther_ros/issues/377>`_)
* Merge pull request `#362 <https://github.com/husarion/panther_ros/issues/362>`_ from husarion/ros2-api-reorganization
* Fix battery topic in simulation
* Enhance ROS API names in the stack
* Contributors: Dawid Kmak, Jakub Delicat, pawelirh

2.0.4 (2024-06-28)
------------------
* Add EKF GPS configuration (`#351 <https://github.com/husarion/panther_ros/issues/351>`_)
* Merge pull request `#337 <https://github.com/husarion/panther_ros/issues/337>`_ from husarion/ros2-gz-lights
* Fix indexing
* Suggestions
* Add image width check
* Some more suggestions
* Suggestions and pre-commit update
* Move antenna to ros-components-description (`#340 <https://github.com/husarion/panther_ros/issues/340>`_)
* Improve readme
* Add img encoding check
* Clean up
* Simplify lights to one class
* Fix
* Code rabbit suggestions
* Add manager
* Merge branch 'ros2-devel' into ros2-gz-lights
* Save work
* Fixes
* Add next suggestions
* Apply some Dawid suggestions
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-gpio-controller-revision
* Merge branch 'ros2' into ros2-build-in-animation
* update package xml
* Add markers
* First marker script
* Simplify and add cmd_vel
* Add lights to panther_gazebo
* Add lights launch to panther_gazebo
* Merge branch 'ros2' into ros2-gz-lights
* decrease intensity
* Steering lights from channel topics
* Working light changing from ros
* Add light macro
* Contributors: Dawid Kmak, pawelirh, rafal-gorecki

2.0.3 (2024-06-06)
------------------
* Merge pull request `#320 <https://github.com/husarion/panther_ros/issues/320>`_ from husarion/ros2-clear-logs
* Emulate tty in simulation
* Contributors: Dawid Kmak, pawelirh

2.0.2 (2024-06-05)
------------------
* Launch refactor (`#307 <https://github.com/husarion/panther_ros/issues/307>`_)
* Merge branch 'ros2' of https://github.com/husarion/panther_ros into ros2-manager-refactor
* Ros2 add components (`#277 <https://github.com/husarion/panther_ros/issues/277>`_)
* Merge pull request `#303 <https://github.com/husarion/panther_ros/issues/303>`_ from husarion/ros2-controler-patch
* Patch
* Remove const name
* Merge branch 'ros2' of https://github.com/husarion/panther_ros into ros2-manager-refactor
* Multi robot spawn working (`#256 <https://github.com/husarion/panther_ros/issues/256>`_)
* Merge pull request `#300 <https://github.com/husarion/panther_ros/issues/300>`_ from husarion/rename_package
* use new launch
* Package rename
* Contributors: Dawid, Jakub Delicat, Paweł Irzyk, rafal-gorecki

2.0.1 (2024-05-01)
------------------
* Merge pull request `#273 <https://github.com/husarion/panther_ros/issues/273>`_ from husarion/ros2-sim-battery
* Fix battery discharge
* Merge pull request `#264 <https://github.com/husarion/panther_ros/issues/264>`_ from husarion/ros2-use-absolute-url
* Absolute URL
* Merge pull request `#261 <https://github.com/husarion/panther_ros/issues/261>`_ from husarion/ros2-readme
* Pawel sugestions
* Merge branch 'ros2-devel' into ros2-readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
* Add controller readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
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
* Headers + Copyright
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge pull request `#252 <https://github.com/husarion/panther_ros/issues/252>`_ from husarion/ros2-depend-patch
  Dependency simulation fix
* Dependency simulation fix
* Merge pull request `#251 <https://github.com/husarion/panther_ros/issues/251>`_ from husarion/ros2-build-depend
  Hardware / Sim Dependencies
* Use FindPackageShare
* reverted panther_gazebo
* Added TickAfterTimeout
* Revert "added test for single host plugin"
  This reverts commit a4f9051c8dfcf03cefa4f827904126fb50c0b316.
* added test for single host plugin
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_gpiod/CMakeLists.txt
  panther_gpiod/package.xml
  panther_gpiod/src/gpio_driver.cpp
* Gazebo - fix collisions (`#225 <https://github.com/husarion/panther_ros/issues/225>`_)
  * Fix collisions
  * remove parent dir
  * Clean up suggestions
  * Clean up suggestions
  * Undo changes
  * Undo commit
* added behaviortree_ros2 to the repository because it is not in rosped
* Fix collisions
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
* Add pre-commit, clang-format and license to files (`#207 <https://github.com/husarion/panther_ros/issues/207>`_)
  Add pre-commit, clang-format and license to files
* Add panther_gazebo launch params docs (`#204 <https://github.com/husarion/panther_ros/issues/204>`_)
  * add params docs
  * small fixes
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * Update README.md
  * Update README.md
  * Update panther_gazebo/README.md
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  ---------
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Merge pull request `#201 <https://github.com/husarion/panther_ros/issues/201>`_ from husarion/ros2-gazebo
  Ros2 gazebo
* review fixes
* add puslish_robot_state param in all files
* add new launch params
* review fixes
* Update panther_gazebo/launch/simulation.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_gazebo/package.xml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_gazebo/config/battery_plugin.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_gazebo/launch/simulation.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_gazebo/launch/simulation.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_gazebo/launch/simulation.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* fix ekf
* fix deps
* grammar fixes
* add battery plugin
* add wheel params in launches
* initial sim configuration draft
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Krzysztof Wojciechowski, Maciej Stępień, Paweł Irzyk, Paweł Kowalski, rafal-gorecki
