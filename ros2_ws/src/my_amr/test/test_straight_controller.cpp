// Copyright 2026 LJH5
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "my_amr/straight_controller.hpp"

TEST(StraightController, CorrectsSignedHeadingAndLateralErrorInBodyFrame) {
  my_amr::StraightController control({});
  const auto out = control.Compute({}, {0.2, 0.1, 0.2});
  EXPECT_LT(out.wz, 0);
  // Rotate the body velocity back into the reference line frame.
  EXPECT_NEAR(std::cos(0.2) * out.vx - std::sin(0.2) * out.vy, 0.1, 1e-12);
  EXPECT_NEAR(std::sin(0.2) * out.vx + std::cos(0.2) * out.vy, -0.05, 1e-12);
  const auto other = control.Compute({}, {0.2, -0.1, -0.2});
  EXPECT_GT(other.wz, 0);
  EXPECT_GT(other.vy, 0);
}

TEST(StraightController, RotatedStartWrapLimitsAndGoal) {
  my_amr::StraightController control({});
  const double pi = std::acos(-1.0);
  const auto rotated = control.Compute({2, 3, pi / 2}, {1.9, 3.2, pi / 2});
  EXPECT_NEAR(rotated.remaining, 0.8, 1e-12);
  EXPECT_NEAR(rotated.lateral_error, 0.1, 1e-12);
  const auto wrapped = control.Compute({0, 0, pi - 0.01}, {0, 0, -pi + 0.01});
  EXPECT_NEAR(wrapped.heading_error, 0.02, 1e-12);
  const auto limited = control.Compute({}, {0, 10, 1});
  EXPECT_NEAR(limited.wz, -0.3, 1e-12);
  const auto goal = control.Compute({}, {1, 0, 0});
  EXPECT_TRUE(goal.at_goal);
  EXPECT_EQ(goal.vx, 0);
  EXPECT_EQ(goal.vy, 0);
  EXPECT_EQ(goal.wz, 0);
  EXPECT_FALSE(control.Compute({}, {1.2, 0, 0}).at_goal);
}

TEST(StraightController, RejectsInvalidSettingsAndPose) {
  my_amr::StraightConfig config;
  config.speed = -1;
  EXPECT_THROW(my_amr::StraightController{config}, std::invalid_argument);
  my_amr::StraightController control({});
  EXPECT_THROW(
      control.Compute({}, {0, 0, std::numeric_limits<double>::quiet_NaN()}),
      std::invalid_argument);
}

TEST(StraightController, ClosedLoopReducesSyntheticDriftAndConverges) {
  my_amr::StraightController control({});
  my_amr::PlanarPose pose{0, 0.15, 0.2};
  for (int i = 0; i < 3000; ++i) {
    const auto out = control.Compute({}, pose);
    pose.x +=
        (std::cos(pose.yaw) * out.vx - std::sin(pose.yaw) * out.vy) * 0.02;
    pose.y +=
        (std::sin(pose.yaw) * out.vx + std::cos(pose.yaw) * out.vy) * 0.02;
    pose.yaw += out.wz * 0.02;
    if (out.at_goal) {
      break;
    }
  }
  EXPECT_TRUE(control.Compute({}, pose).at_goal);
  EXPECT_LT(std::abs(pose.y), 0.01);
  EXPECT_LT(std::abs(pose.yaw), 0.03);
}
