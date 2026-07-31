#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "mavros_xyz_position_offboard/navigation/mission_config.hpp"
#include "mavros_xyz_position_offboard/navigation/target_tracker.hpp"

namespace
{
using mavros_xyz_position_offboard::navigation::MissionConfig;
using mavros_xyz_position_offboard::navigation::TargetTracker;

TEST(MissionConfigTest, RejectsInvalidIndependentXyLimits)
{
  for (const double value : {
      0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
    MissionConfig invalid_speed;
    invalid_speed.car_tracking_max_speed_m_s = value;
    EXPECT_THROW(invalid_speed.validate(), std::invalid_argument);

    MissionConfig invalid_accel;
    invalid_accel.car_tracking_max_accel_m_s2 = value;
    EXPECT_THROW(invalid_accel.validate(), std::invalid_argument);

    MissionConfig invalid_return_speed;
    invalid_return_speed.return_max_speed_m_s = value;
    EXPECT_THROW(invalid_return_speed.validate(), std::invalid_argument);

    MissionConfig invalid_return_accel;
    invalid_return_accel.return_max_accel_m_s2 = value;
    EXPECT_THROW(invalid_return_accel.validate(), std::invalid_argument);
  }
}

TEST(TargetTrackerTest, FiltersNoisyConstantVelocityAndFindsFirstIntercept)
{
  MissionConfig mission;
  TargetTracker tracker(mission);
  for (int sample = 0; sample < 20; ++sample) {
    const double time = 0.1 * sample;
    const double noise = sample % 2 == 0 ? 0.015 : -0.015;
    ASSERT_TRUE(tracker.update(2.0 - 0.40 * time + noise, 1.0 + 0.20 * time - noise, time));
  }
  const auto estimate = tracker.estimate(2.0);
  ASSERT_TRUE(tracker.ready());
  EXPECT_NEAR(estimate.x_m, 1.20, 0.08);
  EXPECT_NEAR(estimate.y_m, 1.40, 0.08);
  EXPECT_NEAR(estimate.vx_m_s, -0.40, 0.10);
  EXPECT_NEAR(estimate.vy_m_s, 0.20, 0.10);

  TargetTracker crossing(mission);
  ASSERT_TRUE(crossing.update(1.0, 0.0, 0.0));
  ASSERT_TRUE(crossing.update(0.9, 0.0, 0.1));
  ASSERT_TRUE(crossing.update(0.8, 0.0, 0.2));
  EXPECT_FALSE(crossing.update(10.0, 10.0, 0.3));
  const auto intercept = crossing.time_to_distance(0.0, 0.0, 0.0, 0.0, 0.20, 0.2, 2.0);
  ASSERT_TRUE(intercept);
  EXPECT_NEAR(*intercept, 0.6, 0.08);
}

TEST(TargetTrackerTest, RejectsUnreachableOrInvalidInterceptQueries)
{
  MissionConfig mission;
  TargetTracker tracker(mission);
  EXPECT_FALSE(tracker.time_to_distance(0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 2.0));
  ASSERT_TRUE(tracker.update(3.0, 0.0, 0.0));
  ASSERT_TRUE(tracker.update(3.0, 0.0, 0.1));
  ASSERT_TRUE(tracker.update(3.0, 0.0, 0.2));
  EXPECT_FALSE(tracker.time_to_distance(0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 2.0));
  EXPECT_FALSE(tracker.time_to_distance(0.0, 0.0, 0.0, 0.0, -0.2, 0.2, 2.0));
}

}  // namespace
