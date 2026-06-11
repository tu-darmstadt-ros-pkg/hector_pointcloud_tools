#include "hector_pointcloud_processing/voxel_filter.hpp"

#include <functional>
#include <sstream>

namespace hector_pointcloud_processing
{

VoxelFilter::VoxelFilter( const rclcpp::NodeOptions &options )
    : Node( "voxel_filter", options ), input_( "pointcloud" ), output_( "pointcloud_filtered" ),
      voxel_size_( 0.1 ), max_distance_( 30.0 ), tf_prefix_( "" )
{
  declare_reconfigurable_parameter( "voxel_size", std::ref( voxel_size_ ),
                                    "Edge length (m) of each voxel",
                                    hector::ParameterOptions<double>().onValidate(
                                        []( const double &value ) { return value > 0.0; } ) );
  declare_reconfigurable_parameter(
      "max_distance", std::ref( max_distance_ ),
      "Points farther than this (m) from the cloud origin are dropped",
      hector::ParameterOptions<double>().onValidate(
          []( const double &value ) { return value > 0.0; } ) );
  declare_reconfigurable_parameter(
      "tf_prefix", std::ref( tf_prefix_ ),
      "Prefix prepended to frame ids before publishing filtered cloud" );
  declare_reconfigurable_parameter(
      "keep_fields", std::ref( keep_fields_ ),
      "Whitelist of fields to copy to the output; empty keeps all fields" );

  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(
      std::shared_ptr<VoxelFilter>( this, []( VoxelFilter * ) { /* no-op deleter */ } ) );
  node_topics_interface_ = get_node_topics_interface();

  printNodeStatus();

  setup();
}

void VoxelFilter::setup()
{
  pointcloud_publisher_ = pct_->advertise( output_, 10 );

  check_subscribers_timer_ =
      create_wall_timer( std::chrono::milliseconds( 100 ),
                         std::bind( &VoxelFilter::publisherSubscriptionCallback, this ) );
}

void VoxelFilter::pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg )
{
  const auto xyz = voxel_common::findXyzOffsets( msg );
  if ( !xyz ) {
    RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 2000,
                           "Input cloud has no FLOAT32 x/y/z fields, cannot filter." );
    return;
  }

  const auto layout = voxel_common::planOutputLayout( msg, keep_fields_ );
  if ( !layout ) {
    RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 2000,
                           "keep_fields matches none of the input fields." );
    return;
  }

  const double inv_voxel_size = 1.0 / voxel_size_;
  const double max_distance_sq = max_distance_ * max_distance_;

  sensor_msgs::msg::PointCloud2 output = voxel_common::filterCloud(
      msg, *xyz, *layout, max_distance_sq, occupied_,
      [inv_voxel_size]( double ) { return voxel_common::VoxelChoice{ inv_voxel_size, 0 }; } );

  output.header.frame_id = voxel_common::applyTfPrefix( tf_prefix_, output.header.frame_id );

  pointcloud_publisher_.publish( output );
}

void VoxelFilter::publisherSubscriptionCallback()
{
  const size_t subscribers = pointcloud_publisher_.getNumSubscribers();

  if ( subscribers == 0 && has_subscribers_ ) {
    has_subscribers_ = false;
    stopSubscribers();
  }
  if ( subscribers > 0 && !has_subscribers_ ) {
    has_subscribers_ = true;
    startSubscribers();
  }
}

void VoxelFilter::startSubscribers()
{
  RCLCPP_INFO( get_logger(), "Starting subscriber" );
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_, 10, std::bind( &VoxelFilter::pointcloudCallback, this, std::placeholders::_1 ) );
}

void VoxelFilter::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

void VoxelFilter::printNodeStatus() const
{
  const std::string input_remapped = node_topics_interface_->resolve_topic_name( input_ );
  const std::string output_remapped = node_topics_interface_->resolve_topic_name( output_ );

  std::stringstream info;
  info << "The node has the following attributes:" << std::endl;
  if ( input_ == input_remapped ) {
    info << "  input:        " << input_ << std::endl;
  } else {
    info << "  input remapped to: " << input_remapped << std::endl;
  }
  if ( output_ == output_remapped ) {
    info << "  output:       " << output_ << std::endl;
  } else {
    info << "  output remapped to: " << output_remapped << std::endl;
  }
  info << "  voxel_size:   " << voxel_size_ << std::endl;
  info << "  max_distance: " << max_distance_ << std::endl;
  info << "  tf_prefix:    " << ( tf_prefix_.empty() ? "<none>" : tf_prefix_ ) << std::endl;
  info << "  keep_fields:  "
       << ( keep_fields_.empty() ? "<all>" : std::to_string( keep_fields_.size() ) + " field(s)" );

  RCLCPP_INFO_STREAM( get_logger(), info.str() );
}

} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::VoxelFilter )
