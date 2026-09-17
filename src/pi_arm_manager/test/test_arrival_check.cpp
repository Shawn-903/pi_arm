#include <gtest/gtest.h>
#include <limits>
#include "pi_arm_manager/arrival_check.hpp"

using pi_arm_manager::check_arrival;
TEST(Arrival, StationaryButOffTargetIsNotSuccess)
{
  const auto r = check_arrival({"j"}, {0.0}, {"j"}, {0.02}, {0.0}, true, 0.00349, 0.000175);
  ASSERT_TRUE(r.valid);
  EXPECT_FALSE(r.reached);
  EXPECT_DOUBLE_EQ(r.errors[0], -0.02);
}
TEST(Arrival, PositionAndVelocityMustBothSettle)
{
  EXPECT_TRUE(check_arrival({"j"}, {0.0}, {"j"}, {0.001}, {0.0}, true, 0.00349, 0.000175).reached);
  EXPECT_FALSE(check_arrival({"j"}, {0.0}, {"j"}, {0.001}, {0.001}, true, 0.00349, 0.000175).reached);
}
TEST(Arrival, InvalidFeedbackNeverQualifies)
{
  EXPECT_FALSE(check_arrival({"j"}, {0.0}, {"j"}, {0.0}, {0.0}, false, 0.01, 0.01).valid);
  EXPECT_FALSE(check_arrival({"j"}, {0.0}, {"x"}, {0.0}, {0.0}, true, 0.01, 0.01).valid);
  EXPECT_FALSE(check_arrival({"j"}, {0.0}, {"j"}, {}, {0.0}, true, 0.01, 0.01).valid);
  EXPECT_FALSE(check_arrival({"j"}, {0.0}, {"j"}, {std::numeric_limits<double>::quiet_NaN()}, {0.0}, true, 0.01, 0.01).valid);
  EXPECT_FALSE(check_arrival({"j", "j"}, {0.0, 0.0}, {"j", "j"}, {0.0, 0.0}, {0.0, 0.0}, true, 0.01, 0.01).valid);
}
TEST(Arrival, ContinuousFreshSamplesRequired)
{
  pi_arm_manager::ArrivalWindow window(0.5);
  EXPECT_FALSE(window.update(true, 0.0, 1));
  EXPECT_FALSE(window.update(true, 1.0, 1));  // Cached data cannot advance the window.
  EXPECT_FALSE(window.update(false, 1.1, 2));
  EXPECT_FALSE(window.update(true, 1.2, 3));
  EXPECT_FALSE(window.update(true, 1.6, 4));
  EXPECT_TRUE(window.update(true, 1.8, 5));
  EXPECT_FALSE(window.update(false, 1.9, 6));  // Stop-induced position drift invalidates arrival.
}
