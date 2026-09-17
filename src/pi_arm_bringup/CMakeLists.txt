cmake_minimum_required(VERSION 3.16)
project(pi_arm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_bringup_launch_safety
    test/test_bringup_launch_safety.py
  )
endif()

ament_package()
