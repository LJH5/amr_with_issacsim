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
#include "my_amr/mecanum_drive.hpp"

namespace {
my_amr::DriveConfig Config() {
  return {0.075, 0.30, 0.36, 1.5, 1.2, 0.5, 1.0, 35.0, 0.25, {-1, 1, 1, -1}};
}

TEST(MecanumDrive, ForwardStrafeAndYaw) {
  for (const auto command :
       {std::array<double, 3>{0.03, 0, 0}, {0, 0.03, 0}, {0, 0, 0.05}}) {
    my_amr::MecanumDrive drive(Config());
    drive.Receive(command, 0);
    const auto actual = drive.Update(0.1, 0.1);
    const std::array<double, 4> expected =
        command[0] != 0   ? std::array<double, 4>{0.4, 0.4, 0.4, 0.4}
        : command[1] != 0 ? std::array<double, 4>{-0.4, 0.4, 0.4, -0.4}
                          : std::array<double, 4>{-0.22, 0.22, -0.22, 0.22};
    for (size_t i = 0; i < 4; ++i) {
      EXPECT_NEAR(actual[i], expected[i], 1e-9);
    }
  }
}

TEST(MecanumDrive, StopTimeoutInvalidAndRepeatedCommands) {
  my_amr::MecanumDrive drive(Config());
  EXPECT_EQ(drive.Update(0, 0.1), (std::array<double, 4>{}));
  drive.Receive({0.3, 0, 0}, 0);
  EXPECT_GT(drive.Update(0.1, 0.1)[0], 0);
  drive.Receive({0.3, 0, 0}, 0.2);
  EXPECT_GT(drive.Update(0.3, 0.1)[0], 0);
  EXPECT_EQ(drive.Update(0.46, 0.1), (std::array<double, 4>{}));
  drive.Receive({0.3, 0, 0}, 0.5);
  drive.Update(0.6, 0.1);
  drive.Receive({0, 0, 0}, 0.6);
  EXPECT_EQ(drive.Update(0.61, 0.01), (std::array<double, 4>{}));
  drive.Receive({std::numeric_limits<double>::quiet_NaN(), 0, 0}, 0.7);
  EXPECT_EQ(drive.Update(0.71, 0.01), (std::array<double, 4>{}));
}

TEST(MecanumDrive, AccelerationAndWheelLimits) {
  my_amr::MecanumDrive drive(Config());
  drive.Receive({1.5, 1.5, 0}, 0);
  const auto first = drive.Update(0.1, 0.1);
  const double vx = (first[0] + first[1]) * 0.075 / 2;
  const double vy = (first[1] - first[0]) * 0.075 / 2;
  EXPECT_NEAR(std::hypot(vx, vy), 0.05, 1e-9);
  for (int i = 1; i < 100; ++i) {
    drive.Receive({100, 100, 100}, i * 0.1);
    for (double wheel : drive.Update(i * 0.1, 0.1)) {
      EXPECT_LE(std::abs(wheel), 35.0);
    }
  }
  auto invalid = Config();
  invalid.radius = 0;
  EXPECT_THROW(my_amr::MecanumDrive{invalid}, std::invalid_argument);
}
}  // namespace
