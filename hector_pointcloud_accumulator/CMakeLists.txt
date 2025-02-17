cmake_minimum_required(VERSION 3.8)
project(hector_pointcloud_accumulator)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(SOURCES
  include/hector_pointcloud_accumulator/pointcloud_accumulator.hpp
  src/nodes/pointcloud_accumulator_node.hpp
  src/nodes/pointcloud_accumulator_node.cpp
  src/pointcloud_accumulator.cpp
)

# Create the library
add_library(hector_pointcloud_accumulator SHARED ${SOURCES})
target_include_directories(hector_pointcloud_accumulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hector_pointcloud_accumulator
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  geometry_msgs
  tf2_eigen
)

# Create the node executable
add_executable(pointcloud_accumulator_node src/pointcloud_accumulator_standalone.cpp)
target_link_libraries(pointcloud_accumulator_node hector_pointcloud_accumulator)

# Install
install(TARGETS hector_pointcloud_accumulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pointcloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
