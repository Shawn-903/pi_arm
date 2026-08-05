cmake_minimum_required(VERSION 3.16)
project(pi_arm_validation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pi_arm_transmission REQUIRED)

add_executable(kinematics_validation
  src/kinematics_validation.cpp
)
ament_target_dependencies(kinematics_validation
  ament_index_cpp
  pi_arm_transmission
)
target_compile_options(kinematics_validation PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS kinematics_validation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/plot_kinematics_validation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_kinematics_validation
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  set(validation_test_output "${CMAKE_CURRENT_BINARY_DIR}/validation_smoke")
  add_test(
    NAME validation_generate_smoke
    COMMAND
      $<TARGET_FILE:kinematics_validation>
      --output-dir "${validation_test_output}"
      --samples 32
      --sweep-points 9
      --seed 20260730
  )
  add_test(
    NAME validation_plot_smoke
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/scripts/plot_kinematics_validation.py"
      --input-dir "${validation_test_output}"
  )
  set_tests_properties(
    validation_plot_smoke
    PROPERTIES DEPENDS validation_generate_smoke
  )
endif()

ament_package()
