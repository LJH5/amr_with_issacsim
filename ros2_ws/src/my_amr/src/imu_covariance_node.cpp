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

#include "my_amr/imu_covariance_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace my_amr {
ImuCovarianceNode::ImuCovarianceNode() : Node("imu_covariance") {
  orientation_variance_ = declare_parameter("orientation_variance_rad2", 0.01);
  angular_variance_ =
      declare_parameter("angular_velocity_variance_rad2_s2", 0.0025);
  for (double value : {orientation_variance_, angular_variance_}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument("IMU variances must be positive and finite");
    }
  }
  publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("output_topic", "/imu/filtered_input"),
      rclcpp::SensorDataQoS());
  subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("input_topic", "/imu/data"),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { Receive(m); });
}

void ImuCovarianceNode::Receive(sensor_msgs::msg::Imu::ConstSharedPtr message) {
  const auto& q = message->orientation;
  const auto& w = message->angular_velocity;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (message->header.frame_id != "imu_link" || message->header.stamp.sec < 0 ||
      message->header.stamp.nanosec >= 1000000000u ||
      (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0) ||
      !std::isfinite(norm) || std::abs(norm - 1) > 1e-3 ||
      !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z) ||
      message->orientation_covariance[0] < 0 ||
      message->angular_velocity_covariance[0] < 0) {
    return;
  }
  auto output = *message;
  for (auto pair :
       {std::make_pair(&output.orientation_covariance, orientation_variance_),
        std::make_pair(&output.angular_velocity_covariance,
                       angular_variance_)}) {
    auto& covariance = *pair.first;
    if (!std::all_of(covariance.begin(), covariance.end(),
                     [](double v) { return std::isfinite(v); })) {
      return;
    }
    if (std::all_of(covariance.begin(), covariance.end(),
                    [](double v) { return v == 0; })) {
      covariance[0] = covariance[4] = covariance[8] = pair.second;
    }
  }
  publisher_->publish(output);
}
}  // namespace my_amr
