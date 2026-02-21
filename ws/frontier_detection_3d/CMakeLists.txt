cmake_minimum_required(VERSION 3.8)
project(frontier_detection_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  octomap_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Frontier Detector Node
add_executable(frontier_detector_node
  src/frontier_detector.cpp
)

ament_target_dependencies(frontier_detector_node ${dependencies})
target_link_libraries(frontier_detector_node
  ${OCTOMAP_LIBRARIES}
)

# Install
install(TARGETS
  frontier_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()