cmake_minimum_required(VERSION 3.16)
project(track_fusion LANGUAGES CXX)

# Default to an optimised build (a bare `colcon build` would be -O0).
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(coop_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Real-time instrumentation (TimingStats, alloc_probe) shared with the ESKF.
find_package(agent_estimation REQUIRED)

# ROS-free core: Kalman filter, associators, tracker, reorder buffer.
add_library(fusion_core src/kalman.cpp src/associator.cpp src/tracker.cpp)
target_include_directories(fusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(fusion_core PUBLIC Eigen3::Eigen)

add_executable(fusion_node src/fusion_node.cpp)
target_link_libraries(fusion_node fusion_core agent_estimation::alloc_probe)
ament_target_dependencies(fusion_node rclcpp coop_msgs diagnostic_msgs visualization_msgs
  agent_estimation)

install(TARGETS fusion_core EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Linked with alloc_probe, so the test can prove process() never allocates.
  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker fusion_core agent_estimation::alloc_probe)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
