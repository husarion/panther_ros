#include <string>

#include <gtest/gtest.h>

#include <panther_hardware_interfaces/panther_system_error_handler.hpp>

TEST(TestPantherSystemErrorHandler, test_initial_state)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_filter_write_sdo_error)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateWriteSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateWriteSDOErrors(false);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateWriteSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_write_sdo_error)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateWriteSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateWriteSDOErrors(true);

  ASSERT_TRUE(error_handler.IsError());
  ASSERT_TRUE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_filter_read_sdo_error)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateReadSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateReadSDOErrors(false);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateReadSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_read_sdo_error)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateReadSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateReadSDOErrors(true);

  ASSERT_TRUE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_TRUE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_filter_read_pdo_error)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 2);

  error_handler.UpdateReadPDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateReadPDOErrors(false);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateReadPDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_read_pdo_error)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 2);

  error_handler.UpdateReadPDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.UpdateReadPDOErrors(true);

  ASSERT_TRUE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_TRUE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_read_pdo_error_single)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateReadPDOErrors(true);

  ASSERT_TRUE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_TRUE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_clear_errors)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateWriteSDOErrors(true);
  error_handler.UpdateWriteSDOErrors(true);

  error_handler.UpdateReadSDOErrors(true);
  error_handler.UpdateReadSDOErrors(true);

  error_handler.UpdateReadPDOErrors(true);

  ASSERT_TRUE(error_handler.IsError());
  ASSERT_TRUE(error_handler.IsWriteSDOError());
  ASSERT_TRUE(error_handler.IsReadSDOError());
  ASSERT_TRUE(error_handler.IsReadPDOError());

  error_handler.SetClearErrorFlag();
  // Has to trigger at least one update to clear errors
  error_handler.UpdateWriteSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

TEST(TestPantherSystemErrorHandler, test_clear_errors_counters)
{
  panther_hardware_interfaces::PantherSystemErrorHandler error_handler(2, 2, 1);

  error_handler.UpdateWriteSDOErrors(true);
  error_handler.UpdateReadSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());

  error_handler.SetClearErrorFlag();

  error_handler.UpdateWriteSDOErrors(true);
  error_handler.UpdateReadSDOErrors(true);

  ASSERT_FALSE(error_handler.IsError());
  ASSERT_FALSE(error_handler.IsWriteSDOError());
  ASSERT_FALSE(error_handler.IsReadSDOError());
  ASSERT_FALSE(error_handler.IsReadPDOError());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}