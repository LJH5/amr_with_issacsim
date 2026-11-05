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

#include "my_amr/drive_validation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <set>
#include <stdexcept>

namespace my_amr
{
namespace
{
constexpr std::array<const char *, 6> kTests{"forward", "backward", "left", "right", "ccw", "cw"};
/** @brief yaw 차이를 [-pi, pi]로 정규화한다. @param angle 차이 [rad]. @return 정규화한 각 [rad]. */
double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
}

double DriveValidation::wall_time()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double DriveValidation::positive(const std::string & name, double fallback)
{
  const double value = declare_parameter(name, fallback);
  if (!std::isfinite(value) || value <= 0) {
    throw std::invalid_argument(name + " must be positive and finite");
  }
  return value;
}

DriveValidation::DriveValidation()
: Node("drive_validation")
{
  const double radius = positive("wheel_radius_m", 0.0);
  const double wheelbase = positive("wheelbase_m", 0.0), track = positive("track_m", 0.0);
  const auto hands = declare_parameter<std::vector<int64_t>>("roller_handedness",
      std::vector<int64_t>{});
  if (hands.size() != 4) {throw std::invalid_argument("Four roller handedness values required");}
  std::array<int, 4> handedness{};
  for (size_t i = 0; i < 4; ++i) {
    if (hands[i] != -1 && hands[i] != 1) {throw std::invalid_argument("Invalid roller handedness");}
    handedness[i] = static_cast<int>(hands[i]);
  }
  odometry_ = std::make_unique<MecanumOdometry>(radius, wheelbase, track, handedness);
  names_ = declare_parameter<std::vector<std::string>>("wheel_joint_names",
      std::vector<std::string>{});
  if (names_.size() != 4 || std::set<std::string>(names_.begin(), names_.end()).size() != 4) {
    throw std::invalid_argument("Four unique wheel names required");
  }
  speed_ = positive("speed_m_s", 0.2);
  yaw_rate_ = positive("yaw_rate_rad_s", 0.3);
  drive_seconds_ = positive("drive_seconds", 5.0);
  settle_seconds_ = positive("settle_seconds", 2.0);
  travel_limit_ = positive("travel_limit_m", 1.5);
  stop_linear_ = positive("stop_linear_tolerance_m_s", 0.02);
  stop_yaw_ = positive("stop_yaw_tolerance_rad_s", 0.03);
  stop_wheel_ = positive("stop_wheel_tolerance_rad_s", 0.2);
  settled_window_ = positive("settled_window_s", 0.5);
  if (settled_window_ > settle_seconds_) {
    throw std::invalid_argument("settled_window_s cannot exceed settle_seconds");
  }
  repetitions_ = declare_parameter("repetitions", 3);
  if (speed_ > 0.3 || yaw_rate_ > 0.5 || repetitions_ < 1 || repetitions_ > 100) {
    throw std::invalid_argument(
        "Low-speed validation requires speed <= 0.3, yaw <= 0.5, repetitions 1..100");
  }
  cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  target_topic_ = declare_parameter<std::string>("wheel_command_topic", "/wheel_commands");
  prefix_ = declare_parameter<std::string>("output_prefix", "drive_validation_" + std::to_string(
      std::chrono::system_clock::now().time_since_epoch().count()));
  if (prefix_.empty()) {throw std::invalid_argument("output_prefix cannot be empty");}
  const std::filesystem::path prefix(prefix_);
  for (const auto suffix : {"_samples.csv", "_summary.csv", "_status.txt"}) {
    if (std::filesystem::exists(prefix_ + suffix)) {
      throw std::invalid_argument("Report already exists; choose a new output_prefix");
    }
  }
  if (prefix.has_parent_path()) {std::filesystem::create_directories(prefix.parent_path());}
  samples_.exceptions(std::ios::badbit | std::ios::failbit);
  summary_.exceptions(std::ios::badbit | std::ios::failbit);
  samples_.open(prefix_ + "_samples.csv");
  summary_.open(prefix_ + "_summary.csv");
  samples_ << std::setprecision(12)
           << "trial,repetition,phase,sim_s,elapsed_s,cmd_vx,cmd_vy,cmd_wz,"
           << "wheel_x_m,wheel_y_m,wheel_yaw_rad,truth_x_m,truth_y_m,truth_yaw_rad,"
           << "world_x_m,world_y_m,world_yaw_rad";
  for (const auto label : {"position", "velocity", "target"}) {
    for (const auto & name : names_) {
      samples_ << ',' << label << '_' << name;
    }
  }
  samples_ << '\n';
  summary_ << std::setprecision(12)
           << "trial,repetition,sim_start_s,sim_end_s,wheel_x_m,wheel_y_m,wheel_yaw_rad,"
           << "truth_x_m,truth_y_m,truth_yaw_rad,position_error_m,yaw_error_rad,"
           << "cross_track_m,rotation_center_drift_m,wheel_tracking_rmse_rad_s,"
           << "stop_path_m,stop_time_s,settled,samples\n";
  status("waiting");
  publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, rclcpp::QoS(1));
  joints_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS().keep_last(100),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      try {
        const auto key = stamp_key(m->header.stamp);
        if (key) {pending_[key].wheels = read_wheels(*m, true); match(key);}
      } catch (const std::exception & e) {
        stop(e.what());
      }
    });
  truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ground_truth/pose", rclcpp::SensorDataQoS().keep_last(100),
    [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
      try {
        const auto key = stamp_key(m->header.stamp);
        if (!key) {return;}
        const auto & q = m->pose.orientation;
        const auto & p = m->pose.position;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (m->header.frame_id != "world" || !std::isfinite(norm) || std::abs(norm - 1) > 1e-3 ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        {throw std::runtime_error("Invalid world ground truth pose");}
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        pending_[key].truth = PlanarPose{p.x, p.y, yaw};
        match(key);
      } catch (const std::exception & e) {
        stop(e.what());
      }
    });
  targets_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    target_topic_, rclcpp::QoS(1), [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
      try {
        target_ = read_wheels(*m, false).velocity; target_wall_ = wall_time();
      } catch (const std::exception & e) {
        stop(e.what());
      }
    });
  started_wall_ = wall_time();
  timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
  RCLCPP_INFO(get_logger(), "Validation will move the robot; reports: %s_*", prefix_.c_str());
}

DriveValidation::Wheels DriveValidation::read_wheels(
  const sensor_msgs::msg::JointState & message, bool positions) const
{
  Wheels result;
  for (size_t i = 0; i < 4; ++i) {
    const auto found = std::find(message.name.begin(), message.name.end(), names_[i]);
    if (found == message.name.end() || std::count(message.name.begin(), message.name.end(),
        names_[i]) != 1)
    {
      throw std::invalid_argument("Missing or duplicate wheel name");
    }
    const size_t index = std::distance(message.name.begin(), found);
    if (index >= message.velocity.size() || (positions && index >= message.position.size())) {
      throw std::invalid_argument("Missing wheel position/velocity");
    }
    result.velocity[i] = message.velocity[index];
    result.position[i] = positions ? message.position[index] : 0;
    if (!std::isfinite(result.velocity[i]) || !std::isfinite(result.position[i])) {
      throw std::invalid_argument("Non-finite wheel feedback");
    }
  }
  return result;
}

int64_t DriveValidation::stamp_key(const builtin_interfaces::msg::Time & stamp) const
{
  const int64_t key = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  if (ready_ && key * 1e-9 < sample_time_ - 0.5) {
    throw std::runtime_error("Simulation time reset; restart validation");
  }
  return key > 0 && key * 1e-9 > sample_time_ ? key : 0;
}

void DriveValidation::match(int64_t key)
{
  if (finished_) {return;}
  const auto & entry = pending_.at(key);
  if (entry.truth && entry.wheels) {
    const auto truth = *entry.truth;
    const auto wheels = *entry.wheels;
    sample(key * 1e-9, truth, wheels);
    pending_.erase(pending_.begin(), pending_.upper_bound(key));
  }
  while (pending_.size() > 100) {pending_.erase(pending_.begin());}
}

void DriveValidation::begin(double time, const PlanarPose & truth, const Wheels & wheels)
{
  start_ = previous_truth_ = truth;
  previous_angles_ = wheels.position;
  previous_yaw_ = truth.yaw;
  trial_start_ = time;
  yaw_ = stop_distance_ = tracking_squared_ = 0;
  tracking_count_ = rows_ = 0;
  stopping_ = false;
  odometry_->reset();
  wheel_pose_ = {};
  command_ = geometry_msgs::msg::Twist();
  switch (trial_ % 6) {
    case 0: command_.linear.x = speed_; break;
    case 1: command_.linear.x = -speed_; break;
    case 2: command_.linear.y = speed_; break;
    case 3: command_.linear.y = -speed_; break;
    case 4: command_.angular.z = yaw_rate_; break;
    case 5: command_.angular.z = -yaw_rate_; break;
  }
  active_ = true;
  status("running: " + std::string(kTests[trial_ % 6]) +
    " repetition " + std::to_string(trial_ / 6 + 1));
  RCLCPP_INFO(get_logger(), "Trial %s repetition %d", kTests[trial_ % 6], trial_ / 6 + 1);
}

void DriveValidation::sample(double time, const PlanarPose & truth, const Wheels & wheels)
{
  received_wall_ = wall_time();
  if (!ready_) {
    ready_ = true;
    origin_ = truth;
    warmup_at_ = sample_time_ = time;
  }
  const double dt = time - sample_time_;
  if (dt > 0.25) {
    throw std::runtime_error("Synchronized data gap exceeds 0.25 simulation seconds");
  }
  sample_time_ = time;
  if (std::hypot(truth.x - origin_.x, truth.y - origin_.y) > travel_limit_) {
    throw std::runtime_error("Travel limit exceeded; check floor and slip");
  }
  if (!active_) {
    if (time - warmup_at_ >= settle_seconds_ && target_wall_ > 0) {begin(time, truth, wheels);}
    return;
  }
  std::array<double, 4> delta{};
  for (size_t i = 0; i < 4; ++i) {
    delta[i] = wheels.position[i] - previous_angles_[i];
  }
  wheel_pose_ = odometry_->integrate(delta);
  previous_angles_ = wheels.position;
  yaw_ += wrap(truth.yaw - previous_yaw_);
  previous_yaw_ = truth.yaw;
  const double dx = truth.x - start_.x, dy = truth.y - start_.y;
  const PlanarPose actual{std::cos(start_.yaw) * dx + std::sin(start_.yaw) * dy,
    -std::sin(start_.yaw) * dx + std::cos(start_.yaw) * dy, yaw_};
  if (!stopping_ && time - trial_start_ >= drive_seconds_) {
    command_ = geometry_msgs::msg::Twist();
    publisher_->publish(command_);
    stopping_ = true;
    stop_at_ = last_moving_ = time;
  } else if (stopping_ && dt > 0) {
    const double distance = std::hypot(truth.x - previous_truth_.x, truth.y - previous_truth_.y);
    stop_distance_ += distance;
    const bool wheels_moving = std::any_of(
      wheels.velocity.begin(), wheels.velocity.end(),
      [this](double v) {return std::abs(v) > stop_wheel_;});
    const bool moving = distance / dt > stop_linear_ ||
      std::abs(wrap(truth.yaw - previous_truth_.yaw)) / dt > stop_yaw_ || wheels_moving;
    if (moving) {last_moving_ = time;}
  }
  previous_truth_ = truth;
  if (!stopping_) {
    for (size_t i = 0; i < 4; ++i) {
      tracking_squared_ += std::pow(wheels.velocity[i] - target_[i], 2);
      ++tracking_count_;
    }
  }
  samples_ << kTests[trial_ % 6] << ',' << trial_ / 6 + 1 << ',' << (stopping_ ? "stop" : "drive")
           << ',' << time << ',' << time - trial_start_ << ',' << command_.linear.x << ','
           << command_.linear.y << ',' << command_.angular.z << ',' << wheel_pose_.x << ','
           << wheel_pose_.y << ',' << wheel_pose_.yaw << ',' << actual.x << ',' << actual.y <<
    ',' << actual.yaw << ',' << truth.x << ',' << truth.y << ',' << truth.yaw;
  for (const auto & values : {wheels.position, wheels.velocity, target_}) {
    for (double v : values) {
      samples_ << ',' << v;
    }
  }
  samples_ << '\n';
  ++rows_;
  if (stopping_ && time - stop_at_ >= settle_seconds_) {
    summarize(time, actual);
    if (time - last_moving_ < settled_window_) {
      throw std::runtime_error("Robot did not settle; increase settle_seconds or inspect contact");
    }
    if (++trial_ == repetitions_ * 6) {
      status("completed: measurements collected; no friction acceptance threshold applied");
      finished_ = true;
      RCLCPP_INFO(get_logger(), "Validation completed: %s_summary.csv", prefix_.c_str());
    } else {
      begin(time, truth, wheels);
    }
  }
}

void DriveValidation::summarize(double time, const PlanarPose & actual)
{
  const double error = std::hypot(wheel_pose_.x - actual.x, wheel_pose_.y - actual.y);
  const double cross = trial_ % 6 < 2 ? actual.y : (trial_ % 6 < 4 ? actual.x : 0);
  summary_ << kTests[trial_ % 6] << ',' << trial_ / 6 + 1 << ',' << trial_start_ << ',' << time
           << ',' << wheel_pose_.x << ',' << wheel_pose_.y << ',' << wheel_pose_.yaw
           << ',' << actual.x << ',' << actual.y << ',' << actual.yaw << ',' << error
           << ',' << wheel_pose_.yaw - actual.yaw << ',' << cross
           << ',' << (trial_ % 6 >= 4 ? std::hypot(actual.x, actual.y) : 0)
           << ',' << (tracking_count_ ? std::sqrt(tracking_squared_ / tracking_count_) : 0)
           << ',' << stop_distance_ << ',' << last_moving_ - stop_at_
           << ',' << (time - last_moving_ >= settled_window_) << ',' << rows_ << '\n';
  samples_.flush();
  summary_.flush();
}

void DriveValidation::tick()
{
  if (finished_) {return;}
  try {
    const double time = wall_time();
    if (count_publishers(cmd_topic_) > 1 || count_publishers(target_topic_) > 1) {
      throw std::runtime_error(
          "Multiple command publishers; stop keyboard/other controller before testing");
    }
    const bool all_streams_ready = ready_ && target_wall_ > 0;
    if ((!all_streams_ready && time - started_wall_ > 30) ||
      (all_streams_ready && (time - received_wall_ > 1 || time - target_wall_ > 1)))
    {throw std::runtime_error("Feedback/target timeout; simulator may be paused or disconnected");}
    publisher_->publish(command_);
  } catch (const std::exception & e) {
    stop(e.what());
  }
}

void DriveValidation::status(const std::string & state)
{
  std::ofstream file(prefix_ + "_status.txt");
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file << state << '\n';
}

void DriveValidation::stop(const std::string & reason)
{
  command_ = geometry_msgs::msg::Twist();
  if (publisher_) {publisher_->publish(command_);}
  if (!finished_) {
    failed_ = finished_ = true;
    RCLCPP_ERROR(get_logger(), "Validation aborted: %s", reason.c_str());
    try {
      status("aborted: " + reason); samples_.flush(); summary_.flush();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Report write failed: %s", e.what());
    }
  }
}
}  // namespace my_amr
