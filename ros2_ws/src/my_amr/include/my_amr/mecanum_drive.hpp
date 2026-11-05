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

#ifndef MY_AMR__MECANUM_DRIVE_HPP_
#define MY_AMR__MECANUM_DRIVE_HPP_

#include <array>

namespace my_amr
{
  /** @brief ROS 노드가 공통 로봇 제원에서 읽어 전달하는 주행 제한과 휠 형상. */
  struct DriveConfig
  {
    double radius;  ///< 휠 반지름 [m].

    double wheelbase;  ///< 앞뒤 휠 중심 간격 [m].

    double track;  ///< 좌우 휠 중심 간격 [m].

    double max_speed;  ///< 병진 속도 벡터의 최대 크기 [m/s].

    double max_yaw;  ///< 최대 yaw 각속도 [rad/s].

    double acceleration;  ///< 병진 가속도 벡터의 최대 크기 [m/s^2].

    double yaw_acceleration;  ///< 최대 yaw 각가속도 [rad/s^2].

    double max_wheel_speed;  ///< 각 휠의 최대 각속도 크기 [rad/s].

    double timeout;  ///< 명령 수신 중단 허용 시간 [s], steady clock 기준.

    std::array<int, 4> handedness;  ///< FL, FR, RL, RR 롤러 방향, 각각 -1 또는 +1.
  };

  /** @brief ROS 및 시뮬레이터에 의존하지 않는 메카넘 속도 목표 계산기. */
  class MecanumDrive
  {
  public:
    /**
     * @brief 제원을 검증하고 정지 상태로 계산기를 초기화한다.
     * @param config 휠 형상, 롤러 방향 및 양수·유한값인 주행 제한.
     * @throws std::invalid_argument 제원이나 롤러 방향이 유효하지 않을 때.
     */
    explicit MecanumDrive(DriveConfig config);

    /**
     * @brief 새 차체 속도 명령과 수신 시각을 저장한다.
     * @param command 로봇 좌표계의 [vx (m/s), vy (m/s), wz (rad/s)].
     * @param steady_time 단조 증가 시계로 측정한 수신 시각 [s].
     * @details 속도 제한을 적용하며 NaN/Inf 입력은 기존 명령을 무효화한다.
     */
    void receive(const std::array<double, 3> & command, double steady_time);

    /**
     * @brief 가속도 제한과 역기구학을 적용해 네 바퀴 속도 목표를 계산한다.
     * @param steady_time receive()와 같은 steady clock의 현재 시각 [s].
     * @param dt 이전 제어 갱신 이후의 경과 시간 [s], 내부에서 0~0.1로 제한.
     * @return [FL, FR, RL, RR] 순서의 목표 각속도 [rad/s].
     * @details 최초 명령 이전, timeout 또는 명시적 정지 명령이면 모두 0을 반환한다.
     * 물리적인 정지 거리나 실제 관절 속도가 즉시 0이 됨을 보장하지 않는다.
     */
    std::array<double, 4> update(double steady_time, double dt);

  private:
    DriveConfig config_;

    std::array<double, 3> target_{}, current_{};

    double received_at_{0.0};

    bool received_{false};
  };
}  // namespace my_amr
#endif  // MY_AMR__MECANUM_DRIVE_HPP_
