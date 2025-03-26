// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_io/pointcloud_saver_node.hpp"
#include "hector_pointcloud_io/pointcloud_io.hpp"

#include <functional>
#include <rclcpp/wait_for_message.hpp>

namespace hector_pointcloud_io
{

PointcloudSaver::PointcloudSaver( const rclcpp::NodeOptions &options )
    : Node( "pointcloud_saver", options )
{
  declare_reconfigurable_parameter( "topic", std::ref( topic_ ),
                                    "Topic to listen for pointclouds on" );
  declare_reconfigurable_parameter( "output_folder", std::ref( output_folder_ ),
                                    "Folder to save pointclouds to" );
  declare_reconfigurable_parameter(
      "output_format", std::ref( output_format_ ),
      "Output format of pointclouds. Supported: pcd, ifs, ply, vtk",
      hector::ParameterOptions<std::string>().onValidate( []( const std::string &value ) {
        return std::set<std::string>{ "pcd", "ifs", "ply", "vtk" }.count( value ) > 0;
      } ) );
  declare_reconfigurable_parameter(
      "output_filename_prefix", std::ref( output_filename_prefix_ ),
      "Prefix for output filenames. Name will be <prefix>.<timestamp>.<output_format>" );
  declare_reconfigurable_parameter( "timeout_ms", std::ref( timeout_ms_ ),
                                    "Timeout for waiting for pointclouds in ms.",
                                    hector::ParameterOptions<int>().setRange( 0, 30000, 50 ) );

  // service server for handling service calls
  service_server_ = this->create_service<std_srvs::srv::Trigger>(
      "~/save_pointcloud", std::bind( &PointcloudSaver::serviceCallback, this,
                                      std::placeholders::_1, std::placeholders::_2 ) );
}

void PointcloudSaver::serviceCallback( const std_srvs::srv::Trigger::Request::SharedPtr &,
                                       const std_srvs::srv::Trigger::Response::SharedPtr &response )
{
  RCLCPP_INFO( this->get_logger(), "Received request to save pointcloud." );
  sensor_msgs::msg::PointCloud2 pointcloud;
  if ( !rclcpp::wait_for_message( pointcloud, this->shared_from_this(), topic_,
                                  std::chrono::milliseconds( timeout_ms_ ) ) ) {
    RCLCPP_ERROR( this->get_logger(), "Timeout while waiting for pointcloud message." );
    response->success = false;
    return;
  }
  std::string timestamp =
      std::to_string( rclcpp::Time( pointcloud.header.stamp ).nanoseconds() / 1000 );
  std::string path =
      output_folder_ + "/" + output_filename_prefix_ + "." + timestamp + "." + output_format_;
  if ( !save_pointcloud( path, pointcloud ) ) {
    RCLCPP_ERROR( this->get_logger(), "Failed to write pointcloud to file." );
    response->success = false;
    return;
  }
  RCLCPP_INFO( this->get_logger(), "Saved pointcloud." );
  response->success = true;
}
} // namespace hector_pointcloud_io

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_io::PointcloudSaver )
