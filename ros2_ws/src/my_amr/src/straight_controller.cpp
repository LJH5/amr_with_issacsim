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

#include "my_amr/straight_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_amr {
StraightController::StraightController(StraightConfig config)
    : config_(config) {
  for (double value :
       {config.distance, config.speed, config.lateral_gain, config.heading_gain,
        config.max_lateral, config.max_yaw, config.distance_gain,
        config.position_tolerance, config.yaw_tolerance}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(
          "Straight drive settings must be positive and finite");
    }
  }
  if (config.speed > 0.3 || config.max_lateral > 0.3 || config.max_yaw > 0.5 ||
      config.position_tolerance >= config.distance) {
    throw std::invalid_argument(
        "Invalid low-speed straight drive limits/tolerance");
  }
}

StraightOutput StraightController::Compute(const PlanarPose& start,
                                           const PlanarPose& pose) const {
  for (double value : {start.x, start.y, start.yaw, pose.x, pose.y, pose.yaw}) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("Straight drive pose must be finite");
    }
  }
  const double dx = pose.x - start.x, dy = pose.y - start.y;
  StraightOutput out;
  out.remaining =
      config_.distance - std::cos(start.yaw) * dx - std::sin(start.yaw) * dy;
  out.lateral_error = -std::sin(start.yaw) * dx + std::cos(start.yaw) * dy;
  out.heading_error = std::atan2(std::sin(pose.yaw - start.yaw),
                                 std::cos(pose.yaw - start.yaw));
  out.at_goal = std::abs(out.remaining) <= config_.position_tolerance &&
                std::abs(out.lateral_error) <= config_.position_tolerance &&
                std::abs(out.heading_error) <= config_.yaw_tolerance;
  const double along =
      out.remaining > config_.position_tolerance
          ? std::min(config_.speed, config_.distance_gain * out.remaining)
          : 0.0;
  const double lateral = std::clamp(-config_.lateral_gain * out.lateral_error,
                                    -config_.max_lateral, config_.max_lateral);
  // Convert velocities in the starting line frame into the current body frame.
  out.vx = std::cos(out.heading_error) * along +
           std::sin(out.heading_error) * lateral;
  out.vy = -std::sin(out.heading_error) * along +
           std::cos(out.heading_error) * lateral;
  out.wz = std::clamp(-config_.heading_gain * out.heading_error,
                      -config_.max_yaw, config_.max_yaw);
  if (out.at_goal) {
    out.vx = out.vy = out.wz = 0;
  }
  return out;
}
}  // namespace my_amr
