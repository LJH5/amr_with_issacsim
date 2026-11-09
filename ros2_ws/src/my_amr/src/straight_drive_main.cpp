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

#include <csignal>
#include <exception>
#include <memory>
#include <thread>

#include "my_amr/straight_drive_node.hpp"

namespace {
volatile std::sig_atomic_t interrupted = 0;
/** @brief 종료 플래그만 갱신한다. @param signal 수신한 POSIX signal. */
void StopSignal(int signal) { interrupted = signal; }
}  // namespace

/** @brief 직선 시험을 실행하고 종료 전 0 명령을 발행한다.
 * @param argc ROS 인자 수. @param argv ROS 인자 배열.
 * @return 정상 종료 0, 설정·실행 오류 1.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, StopSignal);
  std::signal(SIGTERM, StopSignal);
  try {
    auto node = std::make_shared<my_amr::StraightDriveNode>();
    while (rclcpp::ok() && !interrupted) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
      node->Stop("shutdown");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("straight_drive"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
