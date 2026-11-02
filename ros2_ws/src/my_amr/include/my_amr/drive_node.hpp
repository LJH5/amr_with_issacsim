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

#ifndef MY_AMR__DRIVE_NODE_HPP_
#define MY_AMR__DRIVE_NODE_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "my_amr/mecanum_drive.hpp"

namespace my_amr
{
/** @brief Twist를 받아 이름이 지정된 네 휠의 JointState 속도 명령을 발행하는 ROS 노드. */
class DriveNode : public rclcpp::Node
{
public:
  /**
   * @brief 시작 파라미터를 읽고 구독자·발행자·주기 타이머를 생성한다.
   * @throws std::invalid_argument 제원, 휠 이름 또는 발행 주기가 유효하지 않을 때.
   * @details drive.launch.py가 amr_description의 공통 JSON 제원을 전달한다.
   */
  DriveNode();

private:
  /**
   * @brief 주행 제원에 해당하는 ROS 시작 파라미터를 선언하고 읽는다.
   * @return 메카넘 계산기에 전달할 형상·속도·가속도·timeout 설정.
   * @throws std::invalid_argument 네 롤러 방향이 -1/+1로 지정되지 않았을 때.
   */
  DriveConfig load_config();
  /**
   * @brief ROS /clock 변경에 영향을 받지 않는 현재 시각을 구한다.
   * @return steady clock epoch 이후 경과 시간 [s].
   */
  static double steady_seconds();
  /**
   * @brief 계산기를 갱신하고 네 조인트 이름과 목표 각속도 [rad/s]를 발행한다.
   * @details wall timer에서 호출하며 timeout 시에도 0 속도 명령 발행을 유지한다.
   */
  void publish_command();
  MecanumDrive drive_;
  std::vector<std::string> names_;
  double previous_time_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace my_amr
#endif  // MY_AMR__DRIVE_NODE_HPP_
