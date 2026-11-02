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

#include "my_amr/drive_node.hpp"

#include <cmath>
#include <set>
#include <stdexcept>

namespace my_amr
{
DriveConfig DriveNode::load_config()
{
  DriveConfig config{};
  config.radius = declare_parameter("wheel_radius_m", 0.0);
  config.wheelbase = declare_parameter("wheelbase_m", 0.0);
  config.track = declare_parameter("track_m", 0.0);
  config.max_speed = declare_parameter("max_planar_speed_m_s", 0.0);
  config.max_yaw = declare_parameter("max_yaw_rate_rad_s", 0.0);
  config.acceleration = declare_parameter("max_planar_acceleration_m_s2", 0.0);
  config.yaw_acceleration = declare_parameter("max_yaw_acceleration_rad_s2", 0.0);
  config.max_wheel_speed = declare_parameter("joint_velocity_limit_rad_s", 0.0);
  config.timeout = declare_parameter("command_timeout_s", 0.0);
  const auto hands = declare_parameter<std::vector<int64_t>>(
    "roller_handedness", std::vector<int64_t>{});
  if (hands.size() != 4) {
    throw std::invalid_argument("Four roller handedness values required; use drive.launch.py");
  }
  for (size_t i = 0; i < 4; ++i) {
    if (hands[i] != -1 && hands[i] != 1) {
      throw std::invalid_argument("Roller handedness must be -1 or 1");
    }
    config.handedness[i] = static_cast<int>(hands[i]);
  }
  return config;
}

double DriveNode::steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

DriveNode::DriveNode()
: Node("my_amr_node"), drive_(load_config()), previous_time_(steady_seconds())
{
  names_ = declare_parameter<std::vector<std::string>>(
    "wheel_joint_names", std::vector<std::string>{});
  if (names_.size() != 4 || std::set<std::string>(names_.begin(), names_.end()).size() != 4) {
    throw std::invalid_argument("Four unique wheel joint names required");
  }
  const auto input = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const auto output = declare_parameter<std::string>("wheel_command_topic", "/wheel_commands");
  const double rate = declare_parameter("update_rate_hz", 50.0);
  if (!std::isfinite(rate) || rate <= 0 || rate > 1000) {
    throw std::invalid_argument("update_rate_hz must be in (0, 1000]");
  }
  publisher_ = create_publisher<sensor_msgs::msg::JointState>(output, rclcpp::QoS(1));
  subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    input, rclcpp::QoS(1), [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
      drive_.receive({message->linear.x, message->linear.y, message->angular.z}, steady_seconds());
    });
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate), [this]() {publish_command();});
  RCLCPP_INFO(get_logger(), "Mecanum control: %s -> %s (rad/s)", input.c_str(), output.c_str());
}

void DriveNode::publish_command()
{
  const double current_time = steady_seconds();
  const auto wheels = drive_.update(current_time, current_time - previous_time_);
  previous_time_ = current_time;
  sensor_msgs::msg::JointState message;
  message.header.stamp = now();
  message.name = names_;
  message.velocity.assign(wheels.begin(), wheels.end());
  publisher_->publish(message);
}
}  // namespace my_amr
