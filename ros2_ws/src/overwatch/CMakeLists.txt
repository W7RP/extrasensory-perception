cmake_minimum_required(VERSION 3.16)
project(overwatch LANGUAGES CXX)

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
find_package(std_msgs REQUIRED)
# Occluders and the visibility model, the same ones the detectors use.
find_package(synthetic_detector REQUIRED)

add_library(overwatch_planner src/planner.cpp src/zoom.cpp)
target_include_directories(overwatch_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(overwatch_planner PUBLIC Eigen3::Eigen synthetic_detector::detector_model)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node overwatch_planner)
ament_target_dependencies(planner_node rclcpp coop_msgs std_msgs)

install(TARGETS overwatch_planner EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner overwatch_planner)
  ament_add_gtest(test_zoom test/test_zoom.cpp)
  target_link_libraries(test_zoom overwatch_planner)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
