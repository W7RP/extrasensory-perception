cmake_minimum_required(VERSION 3.16)
project(synthetic_detector LANGUAGES CXX)

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
find_package(sdformat14 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(coop_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ROS-free core: geometry, the sensor model, and occluders from the world file.
add_library(detector_model src/geometry.cpp src/sensor_model.cpp src/sdf_occluders.cpp src/imaging.cpp)
target_include_directories(detector_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(detector_model PUBLIC Eigen3::Eigen sdformat14::sdformat14)

add_executable(synthetic_detector src/detector_node.cpp)
target_link_libraries(synthetic_detector detector_model)
ament_target_dependencies(synthetic_detector rclcpp coop_msgs nav_msgs px4_msgs tf2_msgs)

# Simulation-only: Gazebo poses into ROS with their names (see the file header).
add_executable(ground_truth_bridge src/ground_truth_bridge.cpp)
target_link_libraries(ground_truth_bridge detector_model gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
ament_target_dependencies(ground_truth_bridge rclcpp tf2_msgs visualization_msgs)

# Simulation-only: moves the rendered zoom camera where the gimbal points.
add_executable(gimbal_sim src/gimbal_sim.cpp)
target_link_libraries(gimbal_sim detector_model gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
ament_target_dependencies(gimbal_sim rclcpp coop_msgs tf2_msgs)

install(TARGETS detector_model EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS synthetic_detector ground_truth_bridge gimbal_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imaging test/test_imaging.cpp)
  target_link_libraries(test_imaging detector_model)
  ament_add_gtest(test_sensor_model test/test_sensor_model.cpp)
  target_link_libraries(test_sensor_model detector_model)
  # The project world, read back by the same loader the node uses.
  get_filename_component(_repo "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  target_compile_definitions(test_sensor_model PRIVATE
    COOP_WORLD_FILE="${_repo}/sim/worlds/coop_field.sdf"
    COOP_MODEL_DIR="${_repo}/sim/models")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 sdformat14)
ament_package()
