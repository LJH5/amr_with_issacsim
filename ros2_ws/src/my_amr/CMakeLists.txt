# Copyright 2026 LJH5
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(my_amr LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mecanum_drive src/mecanum_drive.cpp src/mecanum_odometry.cpp)
target_compile_features(mecanum_drive PUBLIC cxx_std_17)
target_include_directories(mecanum_drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(my_amr_node src/my_amr_node.cpp src/drive_node.cpp)
target_link_libraries(my_amr_node mecanum_drive)
set_target_properties(my_amr_node PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
ament_target_dependencies(
  my_amr_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_compile_features(keyboard_teleop PRIVATE cxx_std_17)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs)

add_executable(drive_validation src/drive_validation_main.cpp src/drive_validation.cpp)
target_link_libraries(drive_validation mecanum_drive)
ament_target_dependencies(drive_validation rclcpp geometry_msgs sensor_msgs)

add_executable(wheel_odometry src/odometry_main.cpp src/odometry_node.cpp)
target_link_libraries(wheel_odometry mecanum_drive)
ament_target_dependencies(wheel_odometry rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

install(TARGETS my_amr_node keyboard_teleop drive_validation wheel_odometry mecanum_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/uncrustify.cfg")
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mecanum_drive test/test_mecanum_drive.cpp)
  target_link_libraries(test_mecanum_drive mecanum_drive)
  ament_add_gtest(test_mecanum_odometry test/test_mecanum_odometry.cpp)
  target_link_libraries(test_mecanum_odometry mecanum_drive)
endif()

ament_package()
