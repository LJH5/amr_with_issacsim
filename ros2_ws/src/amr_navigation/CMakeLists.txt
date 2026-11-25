cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/imu_covariance.py scripts/wait_for_navigation_tf.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME navigation_integration
    COMMAND ${Python3_EXECUTABLE} -u ${CMAKE_CURRENT_SOURCE_DIR}/test/navigation_integration.py)
  set_tests_properties(navigation_integration PROPERTIES TIMEOUT 150)
endif()

ament_package()
