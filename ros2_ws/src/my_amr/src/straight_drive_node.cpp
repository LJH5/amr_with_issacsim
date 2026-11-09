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

#include "my_amr/straight_drive_node.hpp"

#include <cmath>
#include <stdexcept>

namespace my_amr {
StraightConfig StraightDriveNode::LoadConfig() {
  StraightConfig config;
  config.distance = declare_parameter("distance_m", config.distance);
  config.speed = declare_parameter("speed_m_s", config.speed);
  config.lateral_gain =
      declare_parameter("lateral_gain_s_inv", config.lateral_gain);
  config.heading_gain =
      declare_parameter("heading_gain_s_inv", config.heading_gain);
  config.max_lateral =
      declare_parameter("max_lateral_speed_m_s", config.max_lateral);
  config.max_yaw = declare_parameter("max_yaw_rate_rad_s", config.max_yaw);
  config.distance_gain =
      declare_parameter("distance_gain_s_inv", config.distance_gain);
  config.position_tolerance =
      declare_parameter("position_tolerance_m", config.position_tolerance);
  config.yaw_tolerance =
      declare_parameter("yaw_tolerance_rad", config.yaw_tolerance);
  return config;
}

StraightDriveNode::StraightDriveNode()
    : Node("straight_drive"), controller_(LoadConfig()) {
  timeout_ = declare_parameter("feedback_timeout_s", 0.3);
  max_duration_ = declare_parameter("max_duration_s", 60.0);
  max_lateral_error_ = declare_parameter("max_lateral_error_m", 0.5);
  max_heading_error_ = declare_parameter("max_heading_error_rad", 0.7);
  for (double value :
       {timeout_, max_duration_, max_lateral_error_, max_heading_error_}) {
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(
          "Straight drive safety limits must be positive and finite");
    }
  }
  command_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_,
                                                           rclcpp::QoS(1));
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odom_topic", "/odom"), rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        ReceiveOdometry(m, true);
      });
  wheels_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("wheel_odom_topic", "/odometry/wheel"),
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        ReceiveOdometry(m, false);
      });
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter<std::string>("imu_topic", "/imu/filtered_input"),
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        const auto& q = m->orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (m->header.frame_id == "imu_link" && std::isfinite(norm) &&
            std::abs(norm - 1) < 1e-3 && std::isfinite(m->angular_velocity.z) &&
            m->orientation_covariance[0] >= 0 &&
            m->angular_velocity_covariance[0] >= 0) {
          Record(m->header.stamp, imu_);
        }
      });
  start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = !active_ && Healthy() &&
                            std::hypot(measured_velocity_.linear.x,
                                       measured_velocity_.linear.y) < 0.02 &&
                            std::abs(measured_velocity_.angular.z) < 0.03;
        response->message =
            response->success
                ? "Straight drive started"
                : "Start refused: already active, stale "
                  "sensors/clock, moving base, or another cmd_vel publisher";
        if (response->success) {
          start_ = pose_;
          started_ = now();
          settled_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          active_ = true;
          RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        }
      });
  stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        Stop("requested");
        response->success = true;
        response->message = "Stopped";
      });
  timer_ =
      create_wall_timer(std::chrono::milliseconds(20), [this]() { Tick(); });
  RCLCPP_INFO(
      get_logger(),
      "Idle; call /straight_drive/start after confirming a clear test area");
}

bool StraightDriveNode::Record(const builtin_interfaces::msg::Time& stamp,
                               Freshness& stream) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
      (stamp.sec == 0 && stamp.nanosec == 0)) {
    return false;
  }
  const rclcpp::Time time(stamp, RCL_ROS_TIME);
  if (stream.valid && time < stream.stamp) {
    Stop("measurement time reset; explicit restart required");
    fused_.valid = wheels_.valid = imu_.valid = false;
  }
  if (stream.valid && time == stream.stamp) {
    return false;
  }
  const double age = (now() - time).seconds();
  if (age < -0.05 || age > timeout_) {
    return false;
  }
  stream = {time, std::chrono::steady_clock::now(), true};
  return true;
}

bool StraightDriveNode::Fresh(const Freshness& stream) const {
  if (!stream.valid) {
    return false;
  }
  const double age = (now() - stream.stamp).seconds();
  return age >= -0.05 && age <= timeout_ &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       stream.wall)
                 .count() <= timeout_;
}

bool StraightDriveNode::Healthy() const {
  return Fresh(fused_) && Fresh(wheels_) && Fresh(imu_) &&
         count_publishers(command_topic_) == 1;
}

void StraightDriveNode::ReceiveOdometry(
    nav_msgs::msg::Odometry::ConstSharedPtr m, bool filtered) {
  const auto& p = m->pose.pose.position;
  const auto& q = m->pose.pose.orientation;
  const auto& v = m->twist.twist;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (m->header.frame_id != "odom" || m->child_frame_id != "base_footprint" ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) ||
      std::abs(norm - 1) > 1e-3 || !std::isfinite(v.linear.x) ||
      !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z)) {
    return;
  }
  auto& stream = filtered ? fused_ : wheels_;
  if (!Record(m->header.stamp, stream) || !filtered) {
    return;
  }
  pose_ = {
      p.x, p.y,
      std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
  measured_velocity_ = v;
}

void StraightDriveNode::Tick() {
  if (!active_) {
    return;
  }
  if (!Healthy() || (now() - started_).seconds() > max_duration_) {
    Stop(
        "feedback/clock timeout, conflicting publisher, or trial duration "
        "limit");
    return;
  }
  const auto command = controller_.Compute(start_, pose_);
  if (std::abs(command.lateral_error) > max_lateral_error_ ||
      std::abs(command.heading_error) > max_heading_error_ ||
      command.remaining < -0.15) {
    Stop("path deviation/overshoot limit");
    return;
  }
  const bool settled = command.at_goal &&
                       std::hypot(measured_velocity_.linear.x,
                                  measured_velocity_.linear.y) < 0.02 &&
                       std::abs(measured_velocity_.angular.z) < 0.03;
  if (settled) {
    if (settled_.nanoseconds() == 0) {
      settled_ = now();
    }
    if ((now() - settled_).seconds() >= 0.3) {
      Stop("completed");
      return;
    }
  } else {
    settled_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  geometry_msgs::msg::Twist message;
  message.linear.x = command.vx;
  message.linear.y = command.vy;
  message.angular.z = command.wz;
  publisher_->publish(message);
}

void StraightDriveNode::Stop(const std::string& reason) {
  publisher_->publish(geometry_msgs::msg::Twist());
  if (active_) {
    RCLCPP_INFO(get_logger(), "Straight drive stopped: %s", reason.c_str());
  }
  active_ = false;
}
}  // namespace my_amr
