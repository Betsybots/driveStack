cmake_minimum_required(VERSION 3.8)
project(ackermann-drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(phoenix6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(ackermann-drive src/ackermann-drive-main.cpp)
target_link_libraries(${PROJECT_NAME} /usr/lib/phoenix6/libCTRE_Phoenix6.so /usr/lib/phoenix6/libCTRE_PhoenixTools.so)
target_include_directories(ackermann-drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ackermann-drive PUBLIC c_std_99 cxx_std_17)

set(dependencies
  rclcpp
  phoenix6
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)
ament_target_dependencies(ackermann-drive ${dependencies})

install(TARGETS ackermann-drive
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
