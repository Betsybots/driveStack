cmake_minimum_required(VERSION 3.10)
project(wall_follower LANGUAGES C CXX CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# PCL's VTK config exports a target that links MPI::MPI_C; find it first or the export fails.
find_package(MPI REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)

# MPPI-Generic is a header-only CUDA template library; pull it in as a subproject
# so wall_follower_node.cpp can include its .cuh headers.
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/MPPI-Generic
  ${CMAKE_CURRENT_BINARY_DIR}/mppi-generic)

add_executable(wall_follower_node src/wall_follower_node.cpp)
# The MPPI headers use CUDA-only syntax, so this translation unit needs nvcc, not g++.
set_source_files_properties(src/wall_follower_node.cpp PROPERTIES LANGUAGE CUDA)
# add_subdirectory() sets CMAKE_CUDA_ARCHITECTURES only within its own scope, so set it here too.
set_target_properties(wall_follower_node PROPERTIES CUDA_ARCHITECTURES native)
ament_target_dependencies(wall_follower_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs pcl_conversions PCL)
target_link_libraries(wall_follower_node MPPI::MPPI)
target_include_directories(wall_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS wall_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
