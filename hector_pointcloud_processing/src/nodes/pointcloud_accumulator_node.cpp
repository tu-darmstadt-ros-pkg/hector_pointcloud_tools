// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "./pointcloud_accumulator_node.hpp"

namespace hector_pointcloud_processing
{

PointcloudAccumulatorNode::PointcloudAccumulatorNode( const rclcpp::NodeOptions &options )
    : Node( "hector_pointcloud_accumulator_node", options )
{
  if ( bool use_sim_time = get_parameter_or( "use_sim_time", false ); use_sim_time ) {
    RCLCPP_INFO( get_logger(), "Using simulation time." );
  }
  declare_parameter<double>( "resolution", 0.1 );
  declare_parameter<std::string>( "frame", "map" );
  declare_parameter<int>( "queue_size", 10 );
  declare_parameter<double>( "publish_rate", 1.0 );
  declare_parameter<std::string>( "aggregation_mode", "average" );
  double resolution = get_parameter_or( "resolution", 0.1 );
  int queue_size = get_parameter_or( "queue_size", 10 );
  if ( queue_size < 0 ) {
    RCLCPP_ERROR( get_logger(), "Queue size was smaller than 0 which is not valid. I'm using 10." );
    queue_size = 10;
  }
  auto frame = get_parameter_or<std::string>( "frame", "map" );
  double rate = get_parameter_or( "publish_rate", 1.0 );
  auto aggregation_mode = get_parameter_or<std::string>( "aggregation_mode", "average" );

  declare_parameter<std::vector<std::string>>( "topics" );
  std::vector<std::string> topics =
      get_parameter_or( "topics", std::vector<std::string>{ "pointcloud" } );

  using hector_pointcloud_processing::PointcloudAccumulator;
  if ( aggregation_mode == "average" ) {
    accumulator_ = std::make_shared<PointcloudAccumulator<AggregationMode::AVERAGE>>(
        *this, resolution, frame, rclcpp::Rate( rate ), topics, queue_size );
  } else if ( aggregation_mode == "highest_z" ) {
    accumulator_ = std::make_shared<PointcloudAccumulator<AggregationMode::HIGHEST_Z>>(
        *this, resolution, frame, rclcpp::Rate( rate ), topics, queue_size );
  } else if ( aggregation_mode == "closest_to_center" ) {
    accumulator_ = std::make_shared<PointcloudAccumulator<AggregationMode::CLOSEST_TO_CENTER>>(
        *this, resolution, frame, rclcpp::Rate( rate ), topics, queue_size );
  } else {
    RCLCPP_ERROR( get_logger(), "Unknown aggregation mode '%s'", aggregation_mode.c_str() );
  }
}
} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::PointcloudAccumulatorNode );
