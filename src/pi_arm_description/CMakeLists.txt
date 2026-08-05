cmake_minimum_required(VERSION 3.16)
project(pi_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_description_configuration
    test/test_description_configuration.py
  )
endif()

ament_package()
