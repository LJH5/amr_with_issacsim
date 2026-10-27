cmake_minimum_required(VERSION 3.8)
project(amr_description)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config urdf launch rviz DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME compact_amr_model
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_SOURCE_DIR}/test/check_model.py)
endif()
ament_package()
