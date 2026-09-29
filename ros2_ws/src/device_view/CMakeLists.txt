cmake_minimum_required(VERSION 3.16)
project(device_view LANGUAGES CXX)

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
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs videoio)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(coop_msgs REQUIRED)
# Occluders from the world file and the segment test, shared with the detector.
find_package(synthetic_detector REQUIRED)

# ROS-free core: projection, boxes, silhouettes, ellipses, presentation rules.
add_library(overlay_geometry src/overlay_geometry.cpp)
target_include_directories(overlay_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(overlay_geometry PUBLIC Eigen3::Eigen)

add_executable(overlay_node src/overlay_node.cpp)
# OpenCV's headers are not -Wconversion clean: include them as system headers.
target_include_directories(overlay_node SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(overlay_node overlay_geometry synthetic_detector::detector_model
  ${OpenCV_LIBS})
ament_target_dependencies(overlay_node rclcpp cv_bridge sensor_msgs tf2_msgs coop_msgs)

install(TARGETS overlay_geometry EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS overlay_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_overlay_geometry test/test_overlay_geometry.cpp)
  target_link_libraries(test_overlay_geometry overlay_geometry)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
