cmake_minimum_required(VERSION 3.8)
project(octomap_3d_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find PCL first (before pcl_ros)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

# Find FLANN (required by PCL)
find_package(FLANN REQUIRED)

# Find OctoMap
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  octomap_msgs
)

# OctoMap Builder Node
add_executable(octomap_builder_node
  src/octomap_builder.cpp
)

ament_target_dependencies(octomap_builder_node ${dependencies})

target_link_libraries(octomap_builder_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${FLANN_LIBRARIES}
)

# Install
install(TARGETS
  octomap_builder_node
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