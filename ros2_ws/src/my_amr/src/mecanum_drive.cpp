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

#include "my_amr/mecanum_drive.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_amr
{
  MecanumDrive::MecanumDrive(DriveConfig config)
  : config_(config)
  {
    for (const auto value : {config.radius, config.wheelbase, config.track, config.max_speed,
        config.max_yaw, config.acceleration, config.yaw_acceleration, config.max_wheel_speed,
        config.timeout})
    {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Drive parameters must be positive and finite");
      }
    }

    for (const auto hand : config.handedness) {
      if (hand != -1 && hand != 1) {
        throw std::invalid_argument("Roller handedness must be -1 or 1");
      }
    }
  }

  void MecanumDrive::receive(const std::array<double, 3> & command, double steady_time)
  {
    if (!std::all_of(command.begin(), command.end(), [](double v) {return std::isfinite(v);})) {
      target_ = {};
      received_ = false;
      return;
    }

    target_ = command;
    const double speed = std::hypot(target_[0], target_[1]);

    if (speed > config_.max_speed) {
      target_[0] *= config_.max_speed / speed;
      target_[1] *= config_.max_speed / speed;
    }

    target_[2] = std::clamp(target_[2], -config_.max_yaw, config_.max_yaw);
    received_at_ = steady_time;
    received_ = true;
  }

  std::array<double, 4> MecanumDrive::update(double steady_time, double dt)
  {
    if (!received_ || steady_time - received_at_ >= config_.timeout ||
      target_ == std::array<double, 3>{})
    {
      current_ = {};
      return {};
    }

    dt = std::clamp(dt, 0.0, 0.1);
    const double dx = target_[0] - current_[0], dy = target_[1] - current_[1];
    const double distance = std::hypot(dx, dy);
    const double scale = distance > 0 ? std::min(1.0, config_.acceleration * dt / distance) : 1.0;
    current_[0] += dx * scale;
    current_[1] += dy * scale;
    current_[2] += std::clamp(
    target_[2] - current_[2], -config_.yaw_acceleration * dt, config_.yaw_acceleration * dt);
    std::array<double, 4> wheels{};
    constexpr std::array<int, 4> x_sign{1, 1, -1, -1}, y_sign{1, -1, 1, -1};
    double peak = 0.0;

    for (size_t i = 0; i < wheels.size(); ++i) {
      const double x = x_sign[i] * config_.wheelbase / 2;
      const double y = y_sign[i] * config_.track / 2;
      wheels[i] = (current_[0] - current_[2] * y +
        config_.handedness[i] * (current_[1] + current_[2] * x)) / config_.radius;
      peak = std::max(peak, std::abs(wheels[i]));
    }

    if (peak > config_.max_wheel_speed) {
      for (auto & value : wheels) {
        value *= config_.max_wheel_speed / peak;
      }
    }

    return wheels;
  }
}  // namespace my_amr
