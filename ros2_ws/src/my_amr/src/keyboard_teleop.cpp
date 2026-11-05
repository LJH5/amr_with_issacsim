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

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  volatile std::sig_atomic_t interrupted = 0;

  /**
   * @brief 시그널 핸들러에서 안전하게 키보드 루프의 종료를 요청한다.
   * @param signal_number 수신된 SIGINT 또는 SIGTERM 번호.
   */
  void stop_signal(int signal_number)
  {
    (void)signal_number;
    interrupted = 1;
  }

  /** @brief 터미널의 한 글자 입력 모드를 관리하고 종료 시 원래 설정을 복원한다. */
  class TerminalMode
  {
  public:
    /**
     * @brief 표준 입력 터미널을 저장하고 Enter 없는 키 입력을 활성화한다.
     * @throws std::runtime_error 대화형 터미널이 없거나 설정에 실패했을 때.
     */
    TerminalMode()
    {
      if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) {
        throw std::runtime_error("Run keyboard_teleop in an interactive terminal");
      }

      auto mode = saved_;
      mode.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
      mode.c_cc[VMIN] = 0;
      mode.c_cc[VTIME] = 0;

      if (tcsetattr(STDIN_FILENO, TCSANOW, &mode) != 0) {
        throw std::runtime_error("Cannot configure terminal");
      }
    }

    /** @brief 정상 종료 또는 예외 처리 시 저장한 터미널 설정을 복원한다. */
    ~TerminalMode()
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }

  private:
    termios saved_{};
  };
}  // namespace

/**
 * @brief WASD 주행, Q/E 제자리 회전, J 정지를 /cmd_vel에 발행한다.
 * @param argc ROS 실행 인자를 포함한 명령행 인자 개수.
 * @param argv ROS 파라미터·리매핑을 포함한 명령행 인자 배열.
 * @return 정상 종료이면 0, 터미널 또는 설정 오류이면 1.
 * @details 방향은 다음 키까지 유지한다. J는 종료하지 않고 정지만 하며,
 * Ctrl+C·입력 단절·처리 가능한 오류 시 정지 명령을 발행하고 종료한다.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);
  auto node = std::make_shared<rclcpp::Node>("keyboard_teleop");
  const double speed = node->declare_parameter("speed_m_s", 0.3);
  const double yaw_rate = node->declare_parameter("yaw_rate_rad_s", 0.3);
  const auto topic = node->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(topic, rclcpp::QoS(1));
  int result = 0;
  try {
    if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(yaw_rate) || yaw_rate <= 0) {
      throw std::invalid_argument("speed_m_s and yaw_rate_rad_s must be positive and finite");
    }

    TerminalMode terminal;
    geometry_msgs::msg::Twist command;
    std::cout << "W/S: forward/backward, A/D: strafe, Q/E: rotate left/right, J: stop\n"
              << "Direction stays active until J or another key. Ctrl+C: exit\n"
              << "Speed: " << speed << " m/s, yaw: " << yaw_rate << " rad/s\n";

    while (rclcpp::ok() && !interrupted) {
      pollfd input{STDIN_FILENO, POLLIN, 0};
      const int ready = poll(&input, 1, 50);

      if (ready < 0 && !interrupted) {
        throw std::runtime_error("Keyboard poll failed");
      }

      if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        break;
      }

      if (input.revents & POLLIN) {
        char key;

        if (read(STDIN_FILENO, &key, 1) != 1) {
          break;
        }

        command = geometry_msgs::msg::Twist();

        switch (key) {
          case 'w': case 'W': command.linear.x = speed; break;
          case 's': case 'S': command.linear.x = -speed; break;
          case 'a': case 'A': command.linear.y = speed; break;
          case 'd': case 'D': command.linear.y = -speed; break;
          case 'q': case 'Q': command.angular.z = yaw_rate; break;
          case 'e': case 'E': command.angular.z = -yaw_rate; break;
          default: break;  // J and unknown keys stop; J keeps the program running.
        }
      }

      publisher->publish(command);
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }

  // Keep ROS alive briefly so the explicit stop can leave the process.
  for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
    publisher->publish(geometry_msgs::msg::Twist());
    usleep(20000);
  }

  rclcpp::shutdown();
  return result;
}
