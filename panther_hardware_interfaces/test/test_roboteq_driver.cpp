#include <string>

#include <gtest/gtest.h>

#include <mock_roboteq.hpp>
#include <panther_hardware_interfaces/can_controller.hpp>
#include <panther_hardware_interfaces/roboteq_driver.hpp>

#include <iostream>

class TestRoboteqDriver : public ::testing::Test
{
public:
  std::unique_ptr<RoboteqMock> roboteq_mock_;
  panther_hardware_interfaces::CanSettings can_settings_;

  std::unique_ptr<panther_hardware_interfaces::CanController> can_controller_;

  TestRoboteqDriver()
  {
    can_settings_.master_can_id = 3;
    can_settings_.front_driver_can_id = 1;
    can_settings_.rear_driver_can_id = 2;
    can_settings_.feedback_timeout = std::chrono::milliseconds(15);
    can_settings_.sdo_operation_timeout = std::chrono::milliseconds(4);

    can_controller_ = std::make_unique<panther_hardware_interfaces::CanController>(can_settings_);

    roboteq_mock_ = std::make_unique<RoboteqMock>();
    roboteq_mock_->Start();
    can_controller_->Initialize();
  }

  ~TestRoboteqDriver()
  {
    can_controller_->Deinitialize();
    roboteq_mock_->Stop();
    roboteq_mock_.reset();
  }
};

// These tests are related to can_controller tests, were boot should be already tested

TEST_F(TestRoboteqDriver, test_read_temperature)
{
  const int16_t f_temp = 30;
  const int16_t r_temp = 32;

  roboteq_mock_->front_driver_->SetTemperature(f_temp);
  roboteq_mock_->rear_driver_->SetTemperature(r_temp);

  ASSERT_EQ(can_controller_->GetFrontDriver()->ReadTemperature(), f_temp);
  ASSERT_EQ(can_controller_->GetRearDriver()->ReadTemperature(), r_temp);
}

TEST_F(TestRoboteqDriver, test_read_voltage)
{
  const uint16_t f_volt = 400;
  const uint16_t r_volt = 430;

  roboteq_mock_->front_driver_->SetVoltage(f_volt);
  roboteq_mock_->rear_driver_->SetVoltage(r_volt);

  ASSERT_EQ(can_controller_->GetFrontDriver()->ReadVoltage(), f_volt);
  ASSERT_EQ(can_controller_->GetRearDriver()->ReadVoltage(), r_volt);
}

TEST_F(TestRoboteqDriver, test_read_bat_amps1)
{
  const int16_t f_bat_amps_1 = 10;
  const int16_t r_bat_amps_1 = 30;

  roboteq_mock_->front_driver_->SetBatAmps1(f_bat_amps_1);
  roboteq_mock_->rear_driver_->SetBatAmps1(r_bat_amps_1);

  ASSERT_EQ(can_controller_->GetFrontDriver()->ReadBatAmps1(), f_bat_amps_1);
  ASSERT_EQ(can_controller_->GetRearDriver()->ReadBatAmps1(), r_bat_amps_1);
}

TEST_F(TestRoboteqDriver, test_read_bat_amps2)
{
  const int16_t f_bat_amps_2 = 30;
  const int16_t r_bat_amps_2 = 40;

  roboteq_mock_->front_driver_->SetBatAmps2(f_bat_amps_2);
  roboteq_mock_->rear_driver_->SetBatAmps2(r_bat_amps_2);

  ASSERT_EQ(can_controller_->GetFrontDriver()->ReadBatAmps2(), f_bat_amps_2);
  ASSERT_EQ(can_controller_->GetRearDriver()->ReadBatAmps2(), r_bat_amps_2);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_driver_feedback_values)
{
  const int32_t fl_pos = 101;
  const int32_t fl_vel = 102;
  const int32_t fl_current = 103;
  const int32_t fr_pos = 201;
  const int32_t fr_vel = 202;
  const int32_t fr_current = 203;
  const int32_t rl_pos = 301;
  const int32_t rl_vel = 302;
  const int32_t rl_current = 303;
  const int32_t rr_pos = 401;
  const int32_t rr_vel = 402;
  const int32_t rr_current = 403;

  roboteq_mock_->front_driver_->SetPosition(2, fl_pos);
  roboteq_mock_->front_driver_->SetPosition(1, fr_pos);
  roboteq_mock_->rear_driver_->SetPosition(2, rl_pos);
  roboteq_mock_->rear_driver_->SetPosition(1, rr_pos);

  roboteq_mock_->front_driver_->SetVelocity(2, fl_vel);
  roboteq_mock_->front_driver_->SetVelocity(1, fr_vel);
  roboteq_mock_->rear_driver_->SetVelocity(2, rl_vel);
  roboteq_mock_->rear_driver_->SetVelocity(1, rr_vel);

  roboteq_mock_->front_driver_->SetCurrent(2, fl_current);
  roboteq_mock_->front_driver_->SetCurrent(1, fr_current);
  roboteq_mock_->rear_driver_->SetCurrent(2, rl_current);
  roboteq_mock_->rear_driver_->SetCurrent(1, rr_current);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  panther_hardware_interfaces::RoboteqDriverFeedback f_fb =
    can_controller_->GetFrontDriver()->ReadRoboteqDriverFeedback();
  panther_hardware_interfaces::RoboteqDriverFeedback r_fb =
    can_controller_->GetRearDriver()->ReadRoboteqDriverFeedback();

  ASSERT_EQ(f_fb.motor_2.pos, fl_pos);
  ASSERT_EQ(f_fb.motor_2.vel, fl_vel);
  ASSERT_EQ(f_fb.motor_2.current, fl_current);

  ASSERT_EQ(f_fb.motor_1.pos, fr_pos);
  ASSERT_EQ(f_fb.motor_1.vel, fr_vel);
  ASSERT_EQ(f_fb.motor_1.current, fr_current);

  ASSERT_EQ(r_fb.motor_2.pos, rl_pos);
  ASSERT_EQ(r_fb.motor_2.vel, rl_vel);
  ASSERT_EQ(r_fb.motor_2.current, rl_current);

  ASSERT_EQ(r_fb.motor_1.pos, rr_pos);
  ASSERT_EQ(r_fb.motor_1.vel, rr_vel);
  ASSERT_EQ(r_fb.motor_1.current, rr_current);
}

// TODO
TEST_F(TestRoboteqDriver, test_read_roboteq_driver_feedback_flags) {}

// TODO
TEST_F(TestRoboteqDriver, test_read_roboteq_driver_feedback_timestamp) {}

TEST_F(TestRoboteqDriver, test_send_roboteq_cmd)
{
  const int32_t fl_v = 10;
  const int32_t fr_v = 20;
  const int32_t rl_v = 30;
  const int32_t rr_v = 40;

  can_controller_->GetFrontDriver()->SendRoboteqCmd(fr_v, fl_v);
  can_controller_->GetRearDriver()->SendRoboteqCmd(rr_v, rl_v);

  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(2), fl_v);
  ASSERT_EQ(roboteq_mock_->front_driver_->GetRoboteqCmd(1), fr_v);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(2), rl_v);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetRoboteqCmd(1), rr_v);
}

TEST_F(TestRoboteqDriver, test_reset_roboteq_script)
{
  roboteq_mock_->front_driver_->SetResetRoboteqScript(65);
  roboteq_mock_->rear_driver_->SetResetRoboteqScript(23);

  can_controller_->GetFrontDriver()->ResetRoboteqScript();
  can_controller_->GetRearDriver()->ResetRoboteqScript();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetResetRoboteqScript(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetResetRoboteqScript(), 2);
}

TEST_F(TestRoboteqDriver, test_read_roboteq_turn_on_estop)
{
  roboteq_mock_->front_driver_->SetTurnOnEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOnEstop(23);

  can_controller_->GetFrontDriver()->TurnOnEstop();
  can_controller_->GetRearDriver()->TurnOnEstop();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnEstop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_off_estop)
{
  roboteq_mock_->front_driver_->SetTurnOffEstop(65);
  roboteq_mock_->rear_driver_->SetTurnOffEstop(23);

  can_controller_->GetFrontDriver()->TurnOffEstop();
  can_controller_->GetRearDriver()->TurnOffEstop();

  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOffEstop(), 1);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOffEstop(), 1);
}

TEST_F(TestRoboteqDriver, test_turn_on_safety_stop)
{
  roboteq_mock_->front_driver_->SetTurnOnSafetyStop(65);
  roboteq_mock_->rear_driver_->SetTurnOnSafetyStop(23);

  can_controller_->GetFrontDriver()->TurnOnSafetyStop();
  can_controller_->GetRearDriver()->TurnOnSafetyStop();

  // TODO: somehow check is first channel was also set
  ASSERT_EQ(roboteq_mock_->front_driver_->GetTurnOnSafetyStop(), 2);
  ASSERT_EQ(roboteq_mock_->rear_driver_->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestRoboteqDriver, test_write_timeout)
{
  roboteq_mock_->front_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 100000);
  ASSERT_THROW(can_controller_->GetFrontDriver()->SendRoboteqCmd(0, 0), std::runtime_error);
}

TEST_F(TestRoboteqDriver, test_read_timeout)
{
  roboteq_mock_->front_driver_->SetOnReadWait<int8_t>(0x210F, 1, 100000);
  ASSERT_THROW(can_controller_->GetFrontDriver()->ReadTemperature(), std::runtime_error);
}

// TODO
TEST_F(TestRoboteqDriver, test_can_error)
{
  // roboteq_mock_->front_driver_->SetOnWriteWait<int32_t>(0x2000, 1, 100000);
  // ASSERT_THROW(can_controller_->GetFrontDriver()->SendRoboteqCmd(0, 0), std::runtime_error);
  // ASSERT_TRUE(can_controller_->GetFrontDriver()->get_can_error());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}