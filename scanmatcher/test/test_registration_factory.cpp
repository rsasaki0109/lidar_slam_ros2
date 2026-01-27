#include <gtest/gtest.h>
#include "scanmatcher/registration_factory.hpp"

using namespace graphslam;

class RegistrationFactoryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Default valid params
    valid_params_.method = "NDT";
    valid_params_.ndt_resolution = 5.0;
    valid_params_.ndt_num_threads = 2;
    valid_params_.gicp_corr_dist_threshold = 5.0;
  }

  RegistrationParams valid_params_;
};

// Test: Create NDT registration successfully
TEST_F(RegistrationFactoryTest, CreateNDTSuccess)
{
  valid_params_.method = "NDT";
  auto registration = RegistrationFactory::create(valid_params_);
  ASSERT_NE(registration, nullptr);
}

// Test: Create GICP registration successfully
TEST_F(RegistrationFactoryTest, CreateGICPSuccess)
{
  valid_params_.method = "GICP";
  auto registration = RegistrationFactory::create(valid_params_);
  ASSERT_NE(registration, nullptr);
}

// Test: Invalid registration method throws exception
TEST_F(RegistrationFactoryTest, InvalidMethodThrows)
{
  valid_params_.method = "INVALID";
  EXPECT_THROW(RegistrationFactory::create(valid_params_), std::invalid_argument);
}

// Test: Negative NDT resolution throws exception
TEST_F(RegistrationFactoryTest, NegativeNdtResolutionThrows)
{
  valid_params_.ndt_resolution = -1.0;
  EXPECT_THROW(RegistrationFactory::create(valid_params_), std::invalid_argument);
}

// Test: Zero NDT resolution throws exception
TEST_F(RegistrationFactoryTest, ZeroNdtResolutionThrows)
{
  valid_params_.ndt_resolution = 0.0;
  EXPECT_THROW(RegistrationFactory::create(valid_params_), std::invalid_argument);
}

// Test: Negative GICP correspondence distance throws exception
TEST_F(RegistrationFactoryTest, NegativeGicpCorrDistThrows)
{
  valid_params_.gicp_corr_dist_threshold = -1.0;
  EXPECT_THROW(RegistrationFactory::create(valid_params_), std::invalid_argument);
}

// Test: Zero GICP correspondence distance throws exception
TEST_F(RegistrationFactoryTest, ZeroGicpCorrDistThrows)
{
  valid_params_.gicp_corr_dist_threshold = 0.0;
  EXPECT_THROW(RegistrationFactory::create(valid_params_), std::invalid_argument);
}

// Test: Default params are valid
TEST_F(RegistrationFactoryTest, DefaultParamsAreValid)
{
  RegistrationParams default_params;  // Use default values
  EXPECT_NO_THROW(RegistrationFactory::create(default_params));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
