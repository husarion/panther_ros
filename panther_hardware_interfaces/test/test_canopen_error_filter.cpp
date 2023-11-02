#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/canopen_error_filter.hpp>

TEST(TestCanOpenErrorFilter, test_initial_state)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_filter_write_sdo_error)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateWriteSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateWriteSDOError(false);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateWriteSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_write_sdo_error)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateWriteSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateWriteSDOError(true);

  ASSERT_TRUE(canopen_error_filter.IsError());
  ASSERT_TRUE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_filter_read_sdo_error)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateReadSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateReadSDOError(false);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateReadSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_read_sdo_error)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateReadSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateReadSDOError(true);

  ASSERT_TRUE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_TRUE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_filter_read_pdo_error)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 2);

  canopen_error_filter.UpdateReadPDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateReadPDOError(false);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateReadPDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_read_pdo_error)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 2);

  canopen_error_filter.UpdateReadPDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.UpdateReadPDOError(true);

  ASSERT_TRUE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_TRUE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_read_pdo_error_single)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateReadPDOError(true);

  ASSERT_TRUE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_TRUE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_clear_errors)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateWriteSDOError(true);
  canopen_error_filter.UpdateWriteSDOError(true);

  canopen_error_filter.UpdateReadSDOError(true);
  canopen_error_filter.UpdateReadSDOError(true);

  canopen_error_filter.UpdateReadPDOError(true);

  ASSERT_TRUE(canopen_error_filter.IsError());
  ASSERT_TRUE(canopen_error_filter.IsWriteSDOError());
  ASSERT_TRUE(canopen_error_filter.IsReadSDOError());
  ASSERT_TRUE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.SetClearErrorsFlag();
  // Has to trigger at least one update to clear errors
  canopen_error_filter.UpdateWriteSDOError(true);

  // TODO add test if flag was set back to false

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

TEST(TestCanOpenErrorFilter, test_clear_errors_counters)
{
  panther_hardware_interfaces::CanOpenErrorFilter canopen_error_filter(2, 2, 1);

  canopen_error_filter.UpdateWriteSDOError(true);
  canopen_error_filter.UpdateReadSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());

  canopen_error_filter.SetClearErrorsFlag();

  canopen_error_filter.UpdateWriteSDOError(true);
  canopen_error_filter.UpdateReadSDOError(true);

  ASSERT_FALSE(canopen_error_filter.IsError());
  ASSERT_FALSE(canopen_error_filter.IsWriteSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadSDOError());
  ASSERT_FALSE(canopen_error_filter.IsReadPDOError());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}