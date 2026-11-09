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

#ifndef MY_AMR__STRAIGHT_DRIVE_NODE_HPP_
#define MY_AMR__STRAIGHT_DRIVE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "my_amr/straight_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace my_amr {
/** @brief 명시적인 start 요청으로만 시작하는 저속 직선 시험 노드. */
class StraightDriveNode : public rclcpp::Node {
 public:
  /** @brief 제어 설정·센서 health 구독·start/stop 서비스를 준비한다. */
  StraightDriveNode();

  /** @brief 0 명령을 발행하고 시험을 종료한다. 자동 재시작하지 않는다.
   * @param reason 로그에 기록할 정지 이유.
   */
  void Stop(const std::string& reason);

 private:
  struct Freshness {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point wall{};
    bool valid{false};
  };

  /** @brief 시작 ROS 파라미터를 읽어 제어기를 생성한다. @return 검증한 설정. */
  StraightConfig LoadConfig();

  /** @brief 유효하고 증가하는 측정 시각만 생존 기록에 반영한다.
   * @param stamp 원본 측정 시각. @param stream 센서별 기록.
   * @return 새 시각을 기록했으면 true. 시각 역행 시 주행을 중단한다.
   */
  bool Record(const builtin_interfaces::msg::Time& stamp, Freshness& stream);

  /** @brief ROS 시각과 steady clock 양쪽에서 센서 공백을 확인한다.
   * @param stream 센서별 기록. @return 두 시계 모두 timeout 이내이면 true.
   */
  bool Fresh(const Freshness& stream) const;

  /** @brief 세 측정 스트림과 명령 발행자 수를 확인한다. @return 시작/유지 가능
   * 여부. */
  bool Healthy() const;

  /** @brief EKF 위치와 실제 휠 측정의 유효성·시각을 확인한다.
   * @param message odom/base_footprint 측정.
   * @param filtered true이면 EKF 제어 입력, false이면 원본 휠 생존 입력.
   */
  void ReceiveOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message,
                       bool filtered);

  /** @brief 제어 주기마다 보정 속도·종료·단절·이탈 한계를 처리한다. */
  void Tick();

  StraightController controller_;
  PlanarPose pose_{}, start_{};
  Freshness fused_, wheels_, imu_;
  geometry_msgs::msg::Twist measured_velocity_;
  std::string command_topic_;
  double timeout_, max_duration_, max_lateral_error_, max_heading_error_;
  rclcpp::Time started_{0, 0, RCL_ROS_TIME}, settled_{0, 0, RCL_ROS_TIME};
  bool active_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_,
      wheels_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_,
      stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace my_amr
#endif  // MY_AMR__STRAIGHT_DRIVE_NODE_HPP_
