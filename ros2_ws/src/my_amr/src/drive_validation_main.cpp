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
#include <thread>

#include "my_amr/drive_validation.hpp"

namespace
{
  volatile std::sig_atomic_t interrupted = 0;
  /** @brief 신호 핸들러에서 종료 요청만 저장한다. @param signal POSIX 종료 신호. */
  void stop_signal(int signal)
  {
    interrupted = signal;
  }
}

/** @brief 자동 주행 검증을 실행하고 처리 가능한 종료 시 0 명령을 보낸다.
 * @param argc 인자 수.
 * @param argv ROS 인자 배열.
 * @return 유효한 측정 완료 0, 설정 오류·중단·측정 무효 1.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);
  int result = 1;
  try {
    auto node = std::make_shared<my_amr::DriveValidation>();

    while (rclcpp::ok() && !interrupted && !node->finished()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
      node->stop("interrupted");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    result = node->failed() ? 1 : 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("drive_validation"), "%s", e.what());
  }
  rclcpp::shutdown();
  return result;
}
