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

#ifndef MY_AMR__IMU_COVARIANCE_NODE_HPP_
#define MY_AMR__IMU_COVARIANCE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace my_amr {
/** @brief Bridge IMU의 미지정 공분산에만 설정된 초기 분산을 부여한다. */
class ImuCovarianceNode : public rclcpp::Node {
 public:
  /** @brief 토픽과 분산을 검증하고 relay를 생성한다.
   * @throws std::invalid_argument 분산이 양수 유한값이 아닐 때.
   */
  ImuCovarianceNode();

 private:
  /** @brief 유효한 IMU를 복사하고 0 행렬인 공분산만 채운다.
   * @param message imu_link 기준 ENU 물리 측정. 시각·측정값을 유지한다.
   */
  void Receive(sensor_msgs::msg::Imu::ConstSharedPtr message);
  double orientation_variance_, angular_variance_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
};
}  // namespace my_amr
#endif  // MY_AMR__IMU_COVARIANCE_NODE_HPP_
