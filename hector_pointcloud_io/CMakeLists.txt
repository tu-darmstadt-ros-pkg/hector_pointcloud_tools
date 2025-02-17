cmake_minimum_required(VERSION 3.8)
project(hector_pointcloud_io)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)

add_library(hector_pointcloud_io SHARED src/pointcloud_saver_node.cpp src/pointcloud_io.cpp)

rclcpp_components_register_node(hector_pointcloud_io
  PLUGIN "hector_pointcloud_io::PointcloudSaver"
  EXECUTABLE pointcloud_saver
)

target_include_directories(hector_pointcloud_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(hector_pointcloud_io PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(hector_pointcloud_io
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  pcl_conversions
)

add_executable(save_pointcloud src/save_pointcloud.cpp)
target_link_libraries(save_pointcloud hector_pointcloud_io)

add_executable(load_pointcloud src/load_pointcloud.cpp)
target_link_libraries(load_pointcloud hector_pointcloud_io)

install(TARGETS hector_pointcloud_io save_pointcloud load_pointcloud
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
