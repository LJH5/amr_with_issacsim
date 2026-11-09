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

#ifndef MY_AMR__ODOMETRY_NODE_HPP_
#define MY_AMR__ODOMETRY_NODE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "my_amr/mecanum_odometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace my_amr {
/** @brief 실제 휠 위치를 적분해 평면 Odometry와 같은 시각의 TF를 발행한다. */
class OdometryNode : public rclcpp::Node {
 public:
  /**
   * @brief 공통 형상·공분산·프레임 설정과 실제 JointState 구독을 생성한다.
   * @throws std::invalid_argument 형상, 이름, 프레임 또는 설정값이 유효하지
   * 않을 때.
   * @details odometry.launch.py를 사용한다. 명령·차체 참값·IMU는 적분하지
   * 않는다.
   */
  OdometryNode();

 private:
  /**
   * @brief 공통 JSON에서 launch가 전달한 형상으로 기존 계산기를 생성한다.
   * @return FL/FR/RL/RR 순서의 메카넘 위치 적분기.
   * @throws std::invalid_argument 형상이나 handedness가 유효하지 않을 때.
   */
  MecanumOdometry LoadGeometry();

  /**
   * @brief JointState의 네 휠을 이름으로 찾아 입력 시각의 위치 증분을 적분한다.
   * @param message 위치 [rad]와 측정 시각을 포함한 실제 관절 상태.
   * @details 첫 표본은 원점 기준이다. 중복 시각은 무시하며 시각 역행은 pose를
   * 초기화한다. 긴 측정 공백·불가능한 위치 점프는 pose를 유지하고 기준만
   * 갱신한다. 설정된 주기의 각도 래핑을 보정하며 가능한 회전량이 반주기 이상인
   * 표본은 모호하므로 재기준화한다. 입력 중단 시 Odometry/TF도 중단한다.
   */
  void Receive(sensor_msgs::msg::JointState::ConstSharedPtr message);

  /**
   * @brief 측정 시각과 같은 pose를 Odometry 및 선택적 TF에 기록한다.
   * @param stamp 실제 JointState 시각, ROS time.
   * @param velocity 차체 좌표계 [vx, vy, wz], [m/s, m/s, rad/s].
   */
  void Publish(const rclcpp::Time& stamp,
               const std::array<double, 3>& velocity);

  MecanumOdometry odometry_;
  PlanarPose pose_{};
  std::vector<std::string> names_;
  std::string odom_frame_, base_frame_;
  std::array<double, 36> pose_covariance_{}, twist_covariance_{};
  std::array<double, 4> previous_{};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  bool initialized_{false};
  double max_gap_, max_wheel_speed_, wrap_period_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};
}  // namespace my_amr
#endif  // MY_AMR__ODOMETRY_NODE_HPP_
