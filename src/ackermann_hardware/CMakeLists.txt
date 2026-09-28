cmake_minimum_required(VERSION 3.8)
project(ackermann_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# Phoenix 6 is only needed to drive the real motor; without it the plugin still
# builds and runs with use_mock_hardware=true (handy on a laptop or in CI).
find_package(phoenix6 QUIET)

add_library(${PROJECT_NAME} SHARED src/ackermann_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(phoenix6_FOUND)
  message(STATUS "Phoenix 6 found: building ackermann_hardware with the TalonFX drive motor")
  target_compile_definitions(${PROJECT_NAME} PRIVATE ACKERMANN_HARDWARE_WITH_PHOENIX6=1)
  ament_target_dependencies(${PROJECT_NAME} phoenix6)
  # The phoenix6 CMake config does not export link targets, so link the libraries directly.
  target_link_libraries(${PROJECT_NAME}
    /usr/lib/phoenix6/libCTRE_Phoenix6.so
    /usr/lib/phoenix6/libCTRE_PhoenixTools.so)
else()
  message(WARNING "Phoenix 6 not found: ackermann_hardware will only run with use_mock_hardware=true")
  target_compile_definitions(${PROJECT_NAME} PRIVATE ACKERMANN_HARDWARE_WITH_PHOENIX6=0)
endif()

pluginlib_export_plugin_description_file(hardware_interface ackermann_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  # Pure kinematics/config tests: no ROS runtime needed.
  ament_add_gtest(test_ackermann_config test/test_ackermann_config.cpp)
  target_include_directories(test_ackermann_config PRIVATE include)

  # Loads the plugin through pluginlib and drives it in mock mode.
  ament_add_gtest(test_ackermann_system test/test_ackermann_system.cpp)
  target_include_directories(test_ackermann_system PRIVATE include)
  ament_target_dependencies(test_ackermann_system
    hardware_interface
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
