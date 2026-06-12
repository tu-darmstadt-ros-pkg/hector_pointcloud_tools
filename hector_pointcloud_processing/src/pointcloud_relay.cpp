#include "hector_pointcloud_processing/pointcloud_relay.hpp"

#include "hector_pointcloud_processing/voxel_common.hpp"

#include <functional>
#include <sstream>

namespace hector_pointcloud_processing
{

PointcloudRelay::PointcloudRelay( const rclcpp::NodeOptions &options )
    : Node( "pointcloud_relay", options ), input_( "pointcloud" ), output_( "pointcloud_relayed" ),
      tf_prefix_( "" )
{
  declare_reconfigurable_parameter(
      "tf_prefix",
      std::ref( tf_prefix_ ), "Prefix prepended to the header frame id before publishing; empty relays the frame unchanged" );

  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(
      std::shared_ptr<PointcloudRelay>( this, []( PointcloudRelay * ) { /* no-op deleter */ } ) );
  node_topics_interface_ = get_node_topics_interface();

  printNodeStatus();

  setup();
}

void PointcloudRelay::setup()
{
  pointcloud_publisher_ =
      pct_->advertise( output_, rclcpp::QoS( 1 ).reliable().get_rmw_qos_profile() );

  check_subscribers_timer_ =
      create_wall_timer( std::chrono::milliseconds( 100 ),
                         std::bind( &PointcloudRelay::publisherSubscriptionCallback, this ) );
}

void PointcloudRelay::pointcloudCallback( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg )
{
  if ( tf_prefix_.empty() ) {
    // No header change: forward the shared message as-is.
    pointcloud_publisher_.publish( msg );
    return;
  }

  // The prefix changes the header, so we need an owned copy to modify before publishing.
  auto output = std::make_unique<sensor_msgs::msg::PointCloud2>( *msg );
  output->header.frame_id = voxel_common::applyTfPrefix( tf_prefix_, output->header.frame_id );
  pointcloud_publisher_.publish( std::move( output ) );
}

void PointcloudRelay::publisherSubscriptionCallback()
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

void PointcloudRelay::startSubscribers()
{
  RCLCPP_INFO( get_logger(), "Starting subscriber" );
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_, 10, std::bind( &PointcloudRelay::pointcloudCallback, this, std::placeholders::_1 ) );
}

void PointcloudRelay::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

void PointcloudRelay::printNodeStatus() const
{
  const std::string input_remapped = node_topics_interface_->resolve_topic_name( input_ );
  const std::string output_remapped = node_topics_interface_->resolve_topic_name( output_ );

  std::stringstream info;
  info << "The node has the following attributes:" << std::endl;
  if ( input_ == input_remapped ) {
    info << "  input:     " << input_ << std::endl;
  } else {
    info << "  input remapped to: " << input_remapped << std::endl;
  }
  if ( output_ == output_remapped ) {
    info << "  output:    " << output_ << std::endl;
  } else {
    info << "  output remapped to: " << output_remapped << std::endl;
  }
  info << "  tf_prefix: " << ( tf_prefix_.empty() ? "<none>" : tf_prefix_ );

  RCLCPP_INFO_STREAM( get_logger(), info.str() );
}

} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::PointcloudRelay )
