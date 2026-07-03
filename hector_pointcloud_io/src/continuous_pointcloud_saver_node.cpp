// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_io/continuous_pointcloud_saver_node.hpp"
#include "hector_pointcloud_io/pointcloud_io.hpp"

#include <functional>

namespace hector_pointcloud_io
{

ContinuousPointcloudSaver::ContinuousPointcloudSaver( const rclcpp::NodeOptions &options )
    : Node( "continuous_pointcloud_saver", options )
{
  last_cloud_stamp_ = get_clock()->now();
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
  declare_reconfigurable_parameter( "save_interval", std::ref( save_interval_ ),
                                    "Save interval for saving pointclouds.",
                                    hector::ParameterOptions<double>().setRange( 0.0, 300.0, 0.1 ) );

  // service server for handling service calls
  service_server_ = this->create_service<std_srvs::srv::Trigger>(
      "~/save_next_pointcloud", std::bind( &ContinuousPointcloudSaver::serviceCallback, this,
                                           std::placeholders::_1, std::placeholders::_2 ) );
  auto qos = rclcpp::QoS( 1 );
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, qos,
      std::bind( &ContinuousPointcloudSaver::cloudCallback, this, std::placeholders::_1 ) );
}

void ContinuousPointcloudSaver::serviceCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr &,
    const std_srvs::srv::Trigger::Response::SharedPtr &response )
{
  RCLCPP_INFO( this->get_logger(), "Received request to save next pointcloud." );
  save_next_ = true;

  response->success = true;
  return;
}

void ContinuousPointcloudSaver::cloudCallback( const sensor_msgs::msg::PointCloud2::ConstPtr &pointcloud )
{
  auto time = rclcpp::Time( pointcloud->header.stamp );
  if ( ( time - last_cloud_stamp_ ).seconds() < save_interval_ && !save_next_ ) {
    return;
  }
  save_next_ = false;
  last_cloud_stamp_ = time;

  std::string timestamp = std::to_string( time.nanoseconds() / 1000 );
  std::string path =
      output_folder_ + "/" + output_filename_prefix_ + "." + timestamp + "." + output_format_;
  if ( !save_pointcloud( path, *pointcloud ) ) {
    RCLCPP_ERROR( this->get_logger(), "Failed to write pointcloud to file." );
    return;
  }
  RCLCPP_INFO( this->get_logger(), "Saved pointcloud." );
}
} // namespace hector_pointcloud_io

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_io::ContinuousPointcloudSaver )
