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

#ifndef MY_AMR__STRAIGHT_CONTROLLER_HPP_
#define MY_AMR__STRAIGHT_CONTROLLER_HPP_

#include "my_amr/mecanum_odometry.hpp"

namespace my_amr {
/** @brief 저속 직선 시험의 거리·이득·속도·종료 허용값. */
struct StraightConfig {
  double distance{1.0}, speed{0.1}, lateral_gain{0.5}, heading_gain{1.0};
  double max_lateral{0.1}, max_yaw{0.3}, distance_gain{0.5};
  double position_tolerance{0.01}, yaw_tolerance{0.03};
};

/** @brief 차체 속도와 시작 직선에 대한 오차, 단위 m/s·rad/s·m·rad. */
struct StraightOutput {
  double vx{}, vy{}, wz{}, remaining{}, lateral_error{}, heading_error{};
  bool at_goal{};
};

/** @brief 시작 pose가 정의한 유한 직선을 추종하는 ROS 독립 P 제어기. */
class StraightController {
 public:
  /** @brief 양수 유한 설정을 검증한다. @param config 저속 시험 설정.
   * @throws std::invalid_argument 설정이 유효하지 않을 때.
   */
  explicit StraightController(StraightConfig config);

  /** @brief 시작 직선 기준의 오차를 차체 속도로 변환한다.
   * @param start odom 기준 시작 pose [m, m, rad].
   * @param pose 같은 odom 기준의 현재 추정 pose.
   * @return 제한된 차체 속도와 경로 오차. 목표 통과 후 전진하지 않는다.
   * @throws std::invalid_argument pose에 NaN/Inf가 있을 때.
   */
  StraightOutput Compute(const PlanarPose& start, const PlanarPose& pose) const;

 private:
  StraightConfig config_;
};
}  // namespace my_amr
#endif  // MY_AMR__STRAIGHT_CONTROLLER_HPP_
