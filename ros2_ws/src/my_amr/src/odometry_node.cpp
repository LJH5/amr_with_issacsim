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

#include "my_amr/odometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace my_amr
{
MecanumOdometry OdometryNode::load_geometry()
{
  const double radius = declare_parameter("wheel_radius_m", 0.0);
  const double wheelbase = declare_parameter("wheelbase_m", 0.0);
  const double track = declare_parameter("track_m", 0.0);
  const auto hands = declare_parameter<std::vector<int64_t>>(
    "roller_handedness", std::vector<int64_t>{});
  std::array<int, 4> values{};
  if (hands.size() != 4) {
    throw std::invalid_argument("Four handedness values required; use odometry.launch.py");
  }
  for (size_t i = 0; i < values.size(); ++i) {
    if (hands[i] != -1 && hands[i] != 1) {
      throw std::invalid_argument("Handedness must be -1 or 1");
    }
    values[i] = static_cast<int>(hands[i]);
  }
  return MecanumOdometry(radius, wheelbase, track, values);
}

OdometryNode::OdometryNode()
: Node("wheel_odometry"), odometry_(load_geometry())
{
  names_ = declare_parameter<std::vector<std::string>>(
    "wheel_joint_names", std::vector<std::string>{});
  if (names_.size() != 4 || std::set<std::string>(names_.begin(), names_.end()).size() != 4 ||
    std::any_of(names_.begin(), names_.end(), [](const auto & s) {return s.empty();}))
  {
    throw std::invalid_argument("Four unique nonempty wheel names required");
  }
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_ ||
    odom_frame_.front() == '/' || base_frame_.front() == '/')
  {
    throw std::invalid_argument("Distinct nonempty TF frames without leading slash required");
  }
  max_gap_ = declare_parameter("max_sample_gap_s", 0.5);
  max_wheel_speed_ = declare_parameter("joint_velocity_limit_rad_s", 0.0);
  wrap_period_ = declare_parameter("position_wrap_period_rad", 0.0);
  if (!std::isfinite(max_gap_) || max_gap_ <= 0 ||
    !std::isfinite(max_wheel_speed_) || max_wheel_speed_ <= 0 ||
    !std::isfinite(wrap_period_) || wrap_period_ < 0)
  {
    throw std::invalid_argument("Invalid sample gap, wheel speed limit or angle wrap period");
  }
  for (const auto & name : {"pose_covariance_diagonal", "twist_covariance_diagonal"}) {
    const auto diagonal = declare_parameter<std::vector<double>>(
      name, {0.01, 0.01, 1e6, 1e6, 1e6, 0.04});
    if (diagonal.size() != 6 || std::any_of(
        diagonal.begin(), diagonal.end(), [](double v) {return !std::isfinite(v) || v <= 0;}))
    {
      throw std::invalid_argument("Six positive finite covariance diagonal values required");
    }
    auto & covariance = std::string(name) == "pose_covariance_diagonal" ?
      pose_covariance_ : twist_covariance_;
    for (size_t i = 0; i < 6; ++i) {
      covariance[i * 7] = diagonal[i];
    }
  }
  publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    declare_parameter<std::string>("odom_topic", "/odom"), rclcpp::QoS(10));
  if (declare_parameter("publish_tf", true)) {
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
    declare_parameter<std::string>("joint_state_topic", "/joint_states"),
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {receive(message);});
  RCLCPP_INFO(get_logger(), "Measured wheel odometry: %s -> %s; TF %s",
    odom_frame_.c_str(), base_frame_.c_str(), broadcaster_ ? "enabled" : "disabled");
}

void OdometryNode::receive(sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  if (message->position.size() != message->name.size() || message->header.stamp.sec < 0 ||
    message->header.stamp.nanosec >= 1000000000u)
  {
    RCLCPP_WARN(get_logger(), "Ignoring invalid JointState positions or stamp");
    return;
  }
  std::array<double, 4> current{};
  for (size_t i = 0; i < 4; ++i) {
    if (std::count(message->name.begin(), message->name.end(), names_[i]) != 1) {
      RCLCPP_WARN(get_logger(), "Ignoring JointState with missing/duplicate wheel name");
      return;
    }
    const auto index = std::find(message->name.begin(), message->name.end(), names_[i]) -
      message->name.begin();
    current[i] = message->position[index];
    if (!std::isfinite(current[i]) ||
      (wrap_period_ > 0 && std::abs(current[i]) > wrap_period_ / 2 + 0.01))
    {
      RCLCPP_WARN(get_logger(), "Ignoring nonfinite or out-of-range wheel position");
      return;
    }
  }
  const rclcpp::Time stamp(message->header.stamp, RCL_ROS_TIME);
  if (initialized_ && stamp == previous_stamp_) {return;}
  std::array<double, 3> velocity{};
  if (initialized_ && stamp < previous_stamp_) {
    odometry_.reset();
    pose_ = {};
    RCLCPP_WARN(get_logger(), "Measurement time reset: restarting wheel odometry at origin");
  } else if (initialized_) {
    const double dt = (stamp - previous_stamp_).seconds();
    std::array<double, 4> delta{};
    for (size_t i = 0; i < 4; ++i) {
      delta[i] = current[i] - previous_[i];
      if (wrap_period_ > 0) {delta[i] = std::remainder(delta[i], wrap_period_);}
    }
    // Tolerate 0.01 rad of measurement quantization, not encoder reset jumps.
    const bool unambiguous = wrap_period_ == 0 || max_wheel_speed_ * dt + 0.01 < wrap_period_ / 2;
    if (dt <= max_gap_ && unambiguous && std::all_of(delta.begin(), delta.end(), [&](double v) {
        return std::isfinite(v) && std::abs(v) <= max_wheel_speed_ * dt + 0.01;
      }))
    {
      pose_ = odometry_.integrate(delta);
      velocity = odometry_.forward_kinematics(delta);
      for (double & v : velocity) {
        v /= dt;
      }
    } else {
      RCLCPP_WARN(get_logger(), "Feedback gap or wheel position jump: rebasing without motion");
      previous_ = current;
      previous_stamp_ = stamp;
      return;
    }
  }
  previous_ = current;
  previous_stamp_ = stamp;
  initialized_ = true;
  publish(stamp, velocity);
}

void OdometryNode::publish(
  const rclcpp::Time & stamp, const std::array<double, 3> & velocity)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = pose_.x;
  message.pose.pose.position.y = pose_.y;
  message.pose.pose.orientation.z = std::sin(pose_.yaw / 2);
  message.pose.pose.orientation.w = std::cos(pose_.yaw / 2);
  message.pose.covariance = pose_covariance_;
  message.twist.twist.linear.x = velocity[0];
  message.twist.twist.linear.y = velocity[1];
  message.twist.twist.angular.z = velocity[2];
  message.twist.covariance = twist_covariance_;
  publisher_->publish(message);
  if (broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pose_.x;
    transform.transform.translation.y = pose_.y;
    transform.transform.rotation = message.pose.pose.orientation;
    broadcaster_->sendTransform(transform);
  }
}
}  // namespace my_amr
