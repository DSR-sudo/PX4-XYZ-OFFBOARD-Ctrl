#include <cmath>

#include <gtest/gtest.h>

#include "mavros_xyz_position_offboard/common/types.hpp"
#include "mavros_xyz_position_offboard/navigation/trajectory_planner.hpp"

namespace
{
using mavros_xyz_position_offboard::common::SafetyConfig;
using mavros_xyz_position_offboard::navigation::TrajectoryPlanner;

TEST(TrajectoryPlannerTest, QuinticSetpointsObserveBoundsAndReplanContinuously)
{
  SafetyConfig config;
  config.target_xy_max_speed_m_s = 0.25;
  config.target_xy_max_accel_m_s2 = 0.50;
  config.max_z_setpoint_rate_m_s = 0.20;
  config.max_z_setpoint_accel_m_s2 = 0.40;
  TrajectoryPlanner planner(config);
  planner.latch(0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 1.0});
  planner.set_target(1.0, 1.0, 0.8);

  constexpr double kDt = 0.01;
  auto previous = planner.current();
  double previous_vx = 0.0;
  double previous_vy = 0.0;
  for (int i = 0; i < 80; ++i) {
    const auto current = planner.update(kDt);
    const double vx = (current.x_m - previous.x_m) / kDt;
    const double vy = (current.y_m - previous.y_m) / kDt;
    EXPECT_LE(std::hypot(vx, vy), config.target_xy_max_speed_m_s * 1.01);
    EXPECT_LE(std::hypot(vx - previous_vx, vy - previous_vy) / kDt,
      config.target_xy_max_accel_m_s2 * 1.04);
    EXPECT_LE(std::abs(current.vertical_rate_m_s), config.max_z_setpoint_rate_m_s * 1.002);
    previous = current;
    previous_vx = vx;
    previous_vy = vy;
  }

  const auto before_replan = planner.current();
  planner.set_xy_target(-0.5, 0.25);
  const auto after_replan = planner.update(kDt);
  EXPECT_LT(std::hypot(after_replan.x_m - before_replan.x_m, after_replan.y_m - before_replan.y_m),
    config.target_xy_max_speed_m_s * kDt * 1.02);
  EXPECT_NEAR(planner.target_x_m(), -0.5, 1e-12);
  EXPECT_NEAR(planner.target_y_m(), 0.25, 1e-12);
}

TEST(TrajectoryPlannerTest, TimedXyTargetMeetsFeasibleDeadlineAndExtendsUnsafeDeadline)
{
  SafetyConfig feasible;
  feasible.target_xy_max_speed_m_s = 10.0;
  feasible.target_xy_max_accel_m_s2 = 100.0;
  TrajectoryPlanner planner(feasible);
  planner.latch(0.0, 0.0, 1.5, {0.0, 0.0, 0.0, 1.0});
  EXPECT_TRUE(planner.set_xy_target_with_arrival_time(1.0, 0.0, 1.0));
  EXPECT_TRUE(planner.xy_arrival_time_met());
  EXPECT_NEAR(planner.xy_trajectory_duration_s(), 1.0, 1e-9);

  SafetyConfig constrained;
  constrained.target_xy_max_speed_m_s = 0.25;
  constrained.target_xy_max_accel_m_s2 = 0.50;
  TrajectoryPlanner limited(constrained);
  limited.latch(0.0, 0.0, 1.5, {0.0, 0.0, 0.0, 1.0});
  EXPECT_FALSE(limited.set_xy_target_with_arrival_time(1.0, 0.0, 1.0));
  EXPECT_FALSE(limited.xy_arrival_time_met());
  EXPECT_GT(limited.xy_trajectory_duration_s(), 1.0);
}

TEST(TrajectoryPlannerTest, CustomXyLimitsConstrainResultantMotionWithoutChangingGlobalApi)
{
  SafetyConfig config;
  config.target_xy_max_speed_m_s = 10.0;
  config.target_xy_max_accel_m_s2 = 10.0;
  constexpr double kCustomSpeed = 0.35;
  constexpr double kCustomAccel = 0.65;

  TrajectoryPlanner global(config);
  global.latch(0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 1.0});
  global.set_xy_target(1.2, 0.9);

  TrajectoryPlanner custom(config);
  custom.latch(0.0, 0.0, 0.0, {0.0, 0.0, 0.0, 1.0});
  custom.set_xy_target_with_limits(1.2, 0.9, kCustomSpeed, kCustomAccel);
  EXPECT_GT(custom.xy_trajectory_duration_s(), global.xy_trajectory_duration_s());

  constexpr double kDt = 0.01;
  auto previous = custom.current();
  double previous_vx = 0.0;
  double previous_vy = 0.0;
  for (int i = 0; i < 2000 && !custom.xy_target_reached(); ++i) {
    const auto current = custom.update(kDt);
    const double vx = (current.x_m - previous.x_m) / kDt;
    const double vy = (current.y_m - previous.y_m) / kDt;
    EXPECT_LE(std::hypot(vx, vy), kCustomSpeed * 1.02);
    EXPECT_LE(std::hypot(vx - previous_vx, vy - previous_vy) / kDt, kCustomAccel * 1.10);
    previous = current;
    previous_vx = vx;
    previous_vy = vy;
  }
  EXPECT_TRUE(custom.xy_target_reached());
}

}  // namespace
