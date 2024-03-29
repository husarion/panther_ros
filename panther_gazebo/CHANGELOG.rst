^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package panther_gazebo
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
