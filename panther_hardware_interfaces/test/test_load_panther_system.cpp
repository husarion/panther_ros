#include <string>

#include <gtest/gtest.h>

#include <hardware_interface/resource_manager.hpp>

TEST(TestPantherSystem, load_panther_system)
{
  std::string panther_system_urdf =
    R"(
<?xml version="1.0" encoding="utf-8"?>
<robot name="Panther">
  <ros2_control name="wheels" type="system">
    <hardware>
      <plugin>panther_hardware_interfaces/PantherSystem</plugin>
      <param name="encoder_resolution">1600</param>
      <param name="gear_ratio">30.08</param>
      <param name="gearbox_efficiency">0.75</param>
      <param name="motor_torque_constant">0.11</param>
      <param name="max_rpm_motor_speed">3600.0</param>
      <param name="master_can_id">3</param>
      <param name="front_driver_can_id">1</param>
      <param name="rear_driver_can_id">2</param>
      <param name="roboteq_state_period">1.0</param>
    </hardware>

    <joint name="fl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="fr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="rl_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
    <joint name="rr_wheel_joint">
      <command_interface name="velocity" />
      <state_interface name="position" />
      <state_interface name="velocity" />
      <state_interface name="effort" />
    </joint>
  </ros2_control>
</robot>
)";

  // Use try-catch instead of ASSERT_NO_THROW to get and print exception message
  try {
    hardware_interface::ResourceManager rm(panther_system_urdf);
    SUCCEED();
  } catch (std::exception & err) {
    FAIL() << "Exception caught when trying to create resource manager: " << err.what();
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}