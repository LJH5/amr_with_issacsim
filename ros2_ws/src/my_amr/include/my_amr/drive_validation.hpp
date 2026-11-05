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

#ifndef MY_AMR__DRIVE_VALIDATION_HPP_
#define MY_AMR__DRIVE_VALIDATION_HPP_

#include <array>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_amr/mecanum_odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace my_amr
{
  /** @brief 동시각 관절·차체 참값을 비교하는 유한 시간 자동 주행 시험 노드. */
  class DriveValidation : public rclcpp::Node
  {
  public:
    /** @brief 시작 파라미터를 검증하고 CSV·ROS 입출력을 준비한다.
     * @throws std::exception 제원·출력 경로·ROS 설정이 유효하지 않을 때.
     */
    DriveValidation();

    /** @brief 종료 또는 중단으로 더 이상 시험 명령을 생성하지 않는지 반환한다.
     * @return 시험 종료 상태.
     */
    bool finished() const
    {
      return finished_;
    }

    /** @brief 측정 실패 여부를 반환한다.
     * @return 데이터 단절·중복 발행자·중단·미정지 등으로 무효이면 true.
     */
    bool failed() const
    {
      return failed_;
    }

    /** @brief 0 명령을 발행하고 미완료 실행을 중단 상태로 기록한다.
     * @param reason 미완료일 때 상태 파일에 남길 종료 이유.
     */
    void stop(const std::string & reason);

  private:
    struct Wheels {std::array<double, 4> position{}, velocity{};};

    struct Sample {std::optional<PlanarPose> truth; std::optional<Wheels> wheels;};

    /** @brief 양수·유한 실수 파라미터를 읽는다.
     * @param name 파라미터 이름.
     * @param fallback 시작 기본값.
     * @return 검증한 값.
     * @throws std::invalid_argument 값이 양수 유한값이 아닐 때.
     */
    double positive(const std::string & name, double fallback);

    /** @brief monotonic wall clock 시각 [s]을 반환한다. @return 현재 단조 시각 [s]. */
    static double wall_time();

    /** @brief 조인트 이름으로 실제 또는 목표 휠 값을 매핑한다.
     * @param message 원본 관절 메시지.
     * @param positions true이면 위치와 속도, false이면 목표 속도만 요구한다.
     * @return FL, FR, RL, RR 순서의 유한 휠 값 [rad, rad/s].
     * @throws std::invalid_argument 필드가 없거나 중복·NaN/Inf가 있을 때.
     */
    Wheels read_wheels(const sensor_msgs::msg::JointState & message, bool positions) const;

    /** @brief 관절·참값 메시지의 시뮬레이션 timestamp를 검증한다.
     * @param stamp 메시지 시각.
     * @return 양수 nanoseconds, 초기 시각 0 또는 이전 표본은 0.
     * @throws std::runtime_error 실행 중 큰 시각 역행이 발생할 때.
     */
    int64_t stamp_key(const builtin_interfaces::msg::Time & stamp) const;

    /** @brief 같은 timestamp의 관절·참값만 처리하고 버퍼 크기를 제한한다.
     * @param key 새 표본의 nanoseconds.
     */
    void match(int64_t key);

    /** @brief 동기 표본으로 회전 적분·시험 상태·CSV를 갱신한다.
     * @param time 시뮬레이션 시각 [s].
     * @param truth world 평면 pose [m, m, rad].
     * @param wheels 실제 휠 값.
     */
    void sample(double time, const PlanarPose & truth, const Wheels & wheels);

    /** @brief 한 시험의 시작 좌표계와 휠 기준값을 초기화한다.
     * @param time 시작 시뮬레이션 시각 [s].
     * @param truth 시작 월드 pose.
     * @param wheels 시작 실제 관절 값.
     */
    void begin(double time, const PlanarPose & truth, const Wheels & wheels);

    /** @brief 현재 시험의 한 요약 행을 기록한다.
     * @param time 최종 시뮬레이션 시각 [s].
     * @param actual 시험 시작 차체 좌표계의 실제 변위.
     */
    void summarize(double time, const PlanarPose & actual);

    /** @brief wall timer에서 명령 발행·중복 제어·통신 단절을 확인한다. */
    void tick();

    /** @brief 실행 상태 파일을 갱신한다.
     * @param state completed 또는 aborted와 진단 설명.
     */
    void status(const std::string & state);

    std::unique_ptr<MecanumOdometry> odometry_;

    std::vector<std::string> names_;

    std::string prefix_, cmd_topic_, target_topic_;

    std::ofstream samples_, summary_;

    std::map<int64_t, Sample> pending_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr truth_sub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_, targets_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist command_;

    std::array<double, 4> target_{}, previous_angles_{};

    PlanarPose origin_{}, start_{}, previous_truth_{}, wheel_pose_{};

    double speed_{}, yaw_rate_{}, drive_seconds_{}, settle_seconds_{}, travel_limit_{};

    double stop_linear_{}, stop_yaw_{}, stop_wheel_{}, settled_window_{};

    double started_wall_{}, received_wall_{}, target_wall_{}, sample_time_{}, trial_start_{};

    double stop_at_{}, last_moving_{}, stop_distance_{}, yaw_{}, tracking_squared_{};

    double previous_yaw_{}, warmup_at_{};

    size_t tracking_count_{0}, rows_{0};

    int repetitions_{}, trial_{0};

    bool ready_{false}, active_{false}, stopping_{false}, finished_{false}, failed_{false};
  };
}  // namespace my_amr
#endif  // MY_AMR__DRIVE_VALIDATION_HPP_
