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

#include "gtest/gtest.h"
#include "my_amr/mecanum_odometry.hpp"

TEST(MecanumOdometry, KnownAxisMotionAndSignedWheelOrder)
{
  my_amr::MecanumOdometry odom(0.075, 0.30, 0.36, {-1, 1, 1, -1});
  const auto forward = odom.integrate({1, 1, 1, 1});
  EXPECT_NEAR(forward.x, 0.075, 1e-12);
  EXPECT_NEAR(forward.y, 0, 1e-12);
  odom.reset();
  const auto left = odom.integrate({-1, 1, 1, -1});
  EXPECT_NEAR(left.x, 0, 1e-12);
  EXPECT_NEAR(left.y, 0.075, 1e-12);
  odom.reset();
  const auto turn = odom.integrate({-1, 1, -1, 1});
  EXPECT_NEAR(turn.yaw, 0.075 / 0.33, 1e-12);
  EXPECT_NEAR(turn.x, 0, 1e-12);
}

TEST(MecanumOdometry, FiniteArcAndInitialHeadingAreIntegratedExactly)
{
  my_amr::MecanumOdometry odom(1, 1, 1, {-1, 1, 1, -1});
  const double angle = std::acos(-1.0) / 2;
  // Unit forward distance and pi/2 yaw over one constant-twist interval.
  const auto pose = odom.integrate({1 - angle, 1 + angle, 1 - angle, 1 + angle});
  EXPECT_NEAR(pose.x, 1 / angle, 1e-12);
  EXPECT_NEAR(pose.y, 1 / angle, 1e-12);
  const auto next = odom.integrate({1, 1, 1, 1});
  EXPECT_NEAR(next.x, pose.x, 1e-12);
  EXPECT_NEAR(next.y, pose.y + 1, 1e-12);
  EXPECT_NEAR(next.yaw, angle, 1e-12);
}

TEST(MecanumOdometry, InvalidGeometryAndMeasurementsAreRejected)
{
  EXPECT_THROW((my_amr::MecanumOdometry(0, 1, 1, {-1, 1, 1, -1})), std::invalid_argument);
  EXPECT_THROW((my_amr::MecanumOdometry(1, 1, 1, {1, 1, 1, 1})), std::invalid_argument);
  my_amr::MecanumOdometry odom(1, 1, 1, {-1, 1, 1, -1});
  EXPECT_THROW(odom.integrate({0, 0, std::numeric_limits<double>::quiet_NaN(), 0}),
    std::invalid_argument);
}
