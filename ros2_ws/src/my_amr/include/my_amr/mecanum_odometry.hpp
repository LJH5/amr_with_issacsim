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

#ifndef MY_AMR__MECANUM_ODOMETRY_HPP_
#define MY_AMR__MECANUM_ODOMETRY_HPP_

#include <array>

namespace my_amr
{
/** @brief 평면 위치 [m]와 누적 yaw [rad]. */
struct PlanarPose
{
  double x{0.0}, y{0.0}, yaw{0.0};
};

/** @brief 실제 휠 회전 증분을 적분하는 ROS 독립 메카넘 오도메트리. */
class MecanumOdometry
{
public:
  /**
   * @brief 공통 형상의 역기구학 행렬로부터 최소제곱 전방 기구학을 구성한다.
   * @param radius 휠 반지름 [m].
   * @param wheelbase 앞뒤 휠 중심 거리 [m].
   * @param track 좌우 휠 중심 거리 [m].
   * @param hands FL, FR, RL, RR 롤러 방향, 각각 -1 또는 +1.
   * @throws std::invalid_argument 양수 유한 형상이 아니거나 행렬이 특이할 때.
   */
  MecanumOdometry(double radius, double wheelbase, double track, std::array<int, 4> hands);
  /** @brief 시작 좌표계 원점으로 누적 위치와 yaw를 초기화한다. */
  void reset();
  /**
   * @brief 휠 회전 증분을 차체 변위로 변환한다.
   * @param wheels FL, FR, RL, RR 회전 증분 [rad] 또는 속도 [rad/s].
   * @return 차체 [dx, dy, dyaw] 또는 [vx, vy, wz], 입력의 시간 단위를 따른다.
   * @throws std::invalid_argument 입력에 NaN/Inf가 있을 때.
   */
  std::array<double, 3> forward_kinematics(const std::array<double, 4> & wheels) const;
  /**
   * @brief 일정 차체 twist를 가정한 SE(2) 지수맵으로 회전 증분을 적분한다.
   * @param delta 실제 관절 위치 차이 [rad], FL, FR, RL, RR 순서.
   * @return 시험 시작 차체 좌표계에서 누적한 pose [m, m, rad].
   * @throws std::invalid_argument 입력에 NaN/Inf가 있을 때.
   * @details 미끄러짐을 추정·보정하지 않으며 yaw는 wrap하지 않는다.
   */
  PlanarPose integrate(const std::array<double, 4> & delta);

private:
  std::array<std::array<double, 4>, 3> forward_{};
  PlanarPose pose_{};
};
}  // namespace my_amr
#endif  // MY_AMR__MECANUM_ODOMETRY_HPP_
