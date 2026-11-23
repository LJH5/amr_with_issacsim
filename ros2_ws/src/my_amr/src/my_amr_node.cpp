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

#include <memory>

#include "my_amr/drive_node.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief ROS를 초기화하고 AMR 주행 제어 노드의 이벤트 루프를 실행한다.
 * @param argc ROS 실행 인자를 포함한 명령행 인자 개수.
 * @param argv ROS 파라미터·리매핑을 포함한 명령행 인자 배열.
 * @return 정상 종료이면 0, 노드 생성 또는 실행 오류이면 1.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<my_amr::DriveNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("my_amr_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
