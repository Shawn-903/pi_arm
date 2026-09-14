cmake_minimum_required(VERSION 3.16)
project(pi_arm_output_visualizer LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(pi_arm_transmission REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(output_shaft_mapper
  src/output_shaft_mapper.cpp
)
target_include_directories(output_shaft_mapper
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(output_shaft_mapper
  pi_arm_transmission
  sensor_msgs
)
target_compile_options(output_shaft_mapper PRIVATE -Wall -Wextra -Wpedantic)

add_executable(output_shaft_visualizer_node
  src/output_shaft_visualizer_node.cpp
)
target_link_libraries(output_shaft_visualizer_node output_shaft_mapper)
ament_target_dependencies(output_shaft_visualizer_node
  pi_arm_transmission
  rclcpp
  sensor_msgs
)
target_compile_options(output_shaft_visualizer_node PRIVATE -Wall -Wextra -Wpedantic)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS output_shaft_mapper output_shaft_visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_output_shaft_mapper
    test/test_output_shaft_mapper.cpp
  )
  if(TARGET test_output_shaft_mapper)
    target_link_libraries(test_output_shaft_mapper output_shaft_mapper)
    ament_target_dependencies(test_output_shaft_mapper
      pi_arm_transmission
      sensor_msgs
    )
    target_compile_options(test_output_shaft_mapper PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  add_test(
    NAME test_visualizer_assets
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_visualizer_assets.py"
  )
endif()

ament_package()
