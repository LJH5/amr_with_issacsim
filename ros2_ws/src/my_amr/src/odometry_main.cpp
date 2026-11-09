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

#include <exception>
#include <memory>

#include "my_amr/odometry_node.hpp"

/** @brief ROS 오도메트리 노드를 실행하고 시작 설정 오류를 기록한다.
 * @param argc ROS 명령행 인자 수.
 * @param argv ROS 명령행 인자 배열.
 * @return 정상 종료 0, 시작 또는 실행 오류 1.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<my_amr::OdometryNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("wheel_odometry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
