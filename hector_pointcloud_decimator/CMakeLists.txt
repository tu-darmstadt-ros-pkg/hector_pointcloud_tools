cmake_minimum_required(VERSION 3.8)
project(hector_pointcloud_decimator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hector_ros2_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(point_cloud_transport REQUIRED)


add_executable(pointcloud_decimator src/pointcloud_decimator.cpp)

target_include_directories(pointcloud_decimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pointcloud_decimator
  hector_ros2_utils
  rclcpp
  std_msgs
  sensor_msgs
  point_cloud_transport
)

install(TARGETS pointcloud_decimator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
