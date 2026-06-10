cmake_minimum_required(VERSION 3.8)
project(hector_pointcloud_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(point_cloud_transport REQUIRED)

add_library(hector_pointcloud_processing SHARED
    include/hector_pointcloud_processing/pointcloud_accumulator.hpp
    include/hector_pointcloud_processing/pointcloud_decimator.hpp
    src/nodes/pointcloud_accumulator_node.hpp
    src/nodes/pointcloud_accumulator_node.cpp
    src/pointcloud_accumulator.cpp
    src/pointcloud_decimator.cpp)

target_include_directories(
  hector_pointcloud_processing
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  hector_pointcloud_processing
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  tf2_eigen
  point_cloud_transport)

rclcpp_components_register_node(
  hector_pointcloud_processing PLUGIN
  "hector_pointcloud_processing::PointcloudAccumulatorNode" EXECUTABLE
  pointcloud_accumulator_node)

rclcpp_components_register_node(
  hector_pointcloud_processing PLUGIN
  "hector_pointcloud_processing::PointcloudDecimator" EXECUTABLE
  pointcloud_decimator)

ament_export_targets(export_hector_pointcloud_processing)
install(
  TARGETS hector_pointcloud_processing
  EXPORT export_hector_pointcloud_processing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
