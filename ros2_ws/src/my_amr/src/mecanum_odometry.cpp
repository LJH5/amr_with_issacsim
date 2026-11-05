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

#include "my_amr/mecanum_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_amr
{
  MecanumOdometry::MecanumOdometry(
    double radius, double wheelbase, double track, std::array<int, 4> hands)
  {
    for (double v : {radius, wheelbase, track}) {
      if (!std::isfinite(v) || v <= 0) {
        throw std::invalid_argument("Wheel geometry must be positive and finite");
      }
    }

    constexpr std::array<int, 4> xs{1, 1, -1, -1}, ys{1, -1, 1, -1};
    std::array<std::array<double, 3>, 4> a{};

    for (size_t i = 0; i < 4; ++i) {
      if (hands[i] != -1 && hands[i] != 1) {
        throw std::invalid_argument("Invalid roller handedness");
      }

      a[i] = {1 / radius, hands[i] / radius,
        (-ys[i] * track / 2 + hands[i] * xs[i] * wheelbase / 2) / radius};
    }

    // Invert the 3x3 normal matrix to support the shared handedness definition.
    double matrix[3][6]{};

    for (size_t r = 0; r < 3; ++r) {
      matrix[r][r + 3] = 1;

      for (size_t c = 0; c < 3; ++c) {
        for (size_t i = 0; i < 4; ++i) {
          matrix[r][c] += a[i][r] * a[i][c];
        }
      }
    }

    for (size_t c = 0; c < 3; ++c) {
      size_t pivot = c;

      for (size_t r = c + 1; r < 3; ++r) {
        if (std::abs(matrix[r][c]) > std::abs(matrix[pivot][c])) {
          pivot = r;
        }
      }

      if (std::abs(matrix[pivot][c]) < 1e-12) {
        throw std::invalid_argument("Mecanum geometry has insufficient planar degrees of freedom");
      }

      for (size_t k = 0; k < 6; ++k) {
        std::swap(matrix[c][k], matrix[pivot][k]);
      }

      const double divisor = matrix[c][c];

      for (double & v : matrix[c]) {
        v /= divisor;
      }

      for (size_t r = 0; r < 3; ++r) {
        if (r == c) {
          continue;
        }

        const double factor = matrix[r][c];

        for (size_t k = 0; k < 6; ++k) {
          matrix[r][k] -= factor * matrix[c][k];
        }
      }
    }

    for (size_t r = 0; r < 3; ++r) {
      for (size_t i = 0; i < 4; ++i) {
        for (size_t k = 0; k < 3; ++k) {
          forward_[r][i] += matrix[r][k + 3] * a[i][k];
        }
      }
    }
  }

  void MecanumOdometry::reset()
  {
    pose_ = {};
  }

  std::array<double,
    3> MecanumOdometry::forward_kinematics(const std::array<double, 4> & wheels) const
  {
    for (double v : wheels) {
      if (!std::isfinite(v)) {
        throw std::invalid_argument("Wheel data must be finite");
      }
    }

    std::array<double, 3> result{};

    for (size_t r = 0; r < 3; ++r) {
      for (size_t i = 0; i < 4; ++i) {
        result[r] += forward_[r][i] * wheels[i];
      }
    }

    return result;
  }

  PlanarPose MecanumOdometry::integrate(const std::array<double, 4> & delta)
  {
    const auto body = forward_kinematics(delta);
    const double angle = body[2];
    const double s = std::abs(angle) < 1e-8 ? 1 - angle * angle / 6 : std::sin(angle) / angle;
    const double c = std::abs(angle) < 1e-8 ? angle / 2 : (1 - std::cos(angle)) / angle;
    const double x = s * body[0] - c * body[1], y = c * body[0] + s * body[1];
    pose_.x += std::cos(pose_.yaw) * x - std::sin(pose_.yaw) * y;
    pose_.y += std::sin(pose_.yaw) * x + std::cos(pose_.yaw) * y;
    pose_.yaw += angle;
    return pose_;
  }
}  // namespace my_amr
