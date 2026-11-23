cmake_minimum_required(VERSION 3.8)
project(amr_mapping)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch rviz map DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME mapping_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/mapping_integration.py)
  set_tests_properties(mapping_integration PROPERTIES TIMEOUT 120)
endif()

ament_package()
