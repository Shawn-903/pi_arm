cmake_minimum_required(VERSION 3.16)
project(pi_arm_transmission LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pi_arm_transmission SHARED
  src/cable_kinematics.cpp
  src/config_loader.cpp
)
add_library(pi_arm_transmission::pi_arm_transmission ALIAS pi_arm_transmission)
target_include_directories(pi_arm_transmission
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(pi_arm_transmission PRIVATE yaml-cpp)
target_compile_options(pi_arm_transmission PRIVATE -Wall -Wextra -Wpedantic)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pi_arm_transmission
  EXPORT export_pi_arm_transmission
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cable_kinematics
    test/test_cable_kinematics.cpp
  )
  if(TARGET test_cable_kinematics)
    target_link_libraries(test_cable_kinematics pi_arm_transmission)
    target_compile_options(test_cable_kinematics PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  ament_add_gtest(test_config_loader
    test/test_config_loader.cpp
  )
  if(TARGET test_config_loader)
    target_link_libraries(test_config_loader pi_arm_transmission)
    target_compile_options(test_config_loader PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_export_targets(export_pi_arm_transmission HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(pi_arm_transmission)
ament_export_dependencies(yaml_cpp_vendor)
ament_package()
