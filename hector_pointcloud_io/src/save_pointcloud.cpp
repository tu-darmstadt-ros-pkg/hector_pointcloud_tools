// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_io/pointcloud_io.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  std::vector<std::string> arguments = rclcpp::remove_ros_arguments( argc, argv );
  auto node = rclcpp::Node::make_shared( "save_pointcloud" );
  if ( arguments.size() != 3 ) {
    RCLCPP_ERROR( node->get_logger(), "Usage: save_pointcloud <topic> <path>" );
    return 1;
  }
  std::string topic = arguments[1];
  std::string path = arguments[2];
  std::string extension = path.substr( path.find_last_of( '.' ) + 1 );
  if ( extension != "pcd" && extension != "ifs" && extension != "ply" && extension != "vtk" ) {
    RCLCPP_ERROR( node->get_logger(), "Invalid output format '%s'", extension.c_str() );
    return 1;
  }
  sensor_msgs::msg::PointCloud2 pointcloud;
  RCLCPP_INFO( node->get_logger(), "Waiting for pointcloud message on topic '%s'.", topic.c_str() );
  if ( !rclcpp::wait_for_message<sensor_msgs::msg::PointCloud2>( pointcloud, node, topic ) ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to receive pointcloud message." );
    return 1;
  }
  RCLCPP_INFO( node->get_logger(), "Received pointcloud message. Saving to '%s'.", path.c_str() );
  hector_pointcloud_io::save_pointcloud( path, pointcloud );
  RCLCPP_INFO( node->get_logger(), "Saved pointcloud to '%s'.", path.c_str() );
  return 0;
}
