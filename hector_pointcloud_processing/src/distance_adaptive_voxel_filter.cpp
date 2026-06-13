#include "hector_pointcloud_processing/distance_adaptive_voxel_filter.hpp"

#include <functional>
#include <sstream>

#include <Eigen/Geometry>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace hector_pointcloud_processing
{
namespace
{

//! Validates band_distances: non-empty, all > 0, strictly increasing.
bool validBandDistances( const std::vector<double> &v )
{
  if ( v.empty() )
    return false;
  for ( size_t i = 0; i < v.size(); ++i ) {
    if ( v[i] <= 0.0 )
      return false;
    if ( i > 0 && v[i] <= v[i - 1] )
      return false;
  }
  return true;
}

//! Validates band_voxel_sizes: non-empty, all > 0.
bool validBandVoxelSizes( const std::vector<double> &v )
{
  if ( v.empty() )
    return false;
  for ( const double value : v )
    if ( value <= 0.0 )
      return false;
  return true;
}

} // namespace

DistanceAdaptiveVoxelFilter::DistanceAdaptiveVoxelFilter( const rclcpp::NodeOptions &options )
    : Node( "distance_adaptive_voxel_filter", options ), input_( "pointcloud" ),
      output_( "pointcloud_filtered" ), band_distances_( { 5.0, 15.0, 30.0 } ),
      band_voxel_sizes_( { 0.05, 0.15, 0.4 } ), target_frame_( "" ), tf_prefix_( "" )
{
  const auto rebuild_schedule = [this]( const std::vector<double> & ) {
    band_schedule_.rebuild( band_distances_, band_voxel_sizes_ );
  };
  declare_reconfigurable_parameter(
      "band_distances", std::ref( band_distances_ ),
      "Ascending upper distance bounds (m); index i pairs with band_voxel_sizes[i]",
      hector::ParameterOptions<std::vector<double>>()
          .setAdditionalConstraints(
              "Strictly increasing, all > 0, same length as band_voxel_sizes" )
          .onValidate( validBandDistances )
          .onUpdate( rebuild_schedule ) );
  declare_reconfigurable_parameter(
      "band_voxel_sizes", std::ref( band_voxel_sizes_ ),
      "Voxel edge length (m) for each distance band; index i pairs with band_distances[i]",
      hector::ParameterOptions<std::vector<double>>()
          .setAdditionalConstraints( "All > 0, same length as band_distances" )
          .onValidate( validBandVoxelSizes )
          .onUpdate( rebuild_schedule ) );
  declare_reconfigurable_parameter(
      "target_frame", std::ref( target_frame_ ),
      "Frame the point position is expressed in for binning; empty uses the raw point xyz" );
  declare_reconfigurable_parameter(
      "tf_prefix", std::ref( tf_prefix_ ),
      "Prefix prepended to frame ids before publishing filtered cloud" );
  declare_reconfigurable_parameter(
      "keep_fields", std::ref( keep_fields_ ),
      "Whitelist of fields to copy to the output; empty keeps all fields" );

  band_schedule_.rebuild( band_distances_, band_voxel_sizes_ );

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>( get_clock() );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, this );

  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(
      std::shared_ptr<DistanceAdaptiveVoxelFilter>(
          this, []( DistanceAdaptiveVoxelFilter * ) { /* no-op deleter */ } ) );
  node_topics_interface_ = get_node_topics_interface();

  printNodeStatus();

  setup();
}

void DistanceAdaptiveVoxelFilter::setup()
{
  pointcloud_publisher_ =
      pct_->advertise( output_, rclcpp::QoS( 1 ).reliable().get_rmw_qos_profile() );

  check_subscribers_timer_ = create_wall_timer(
      std::chrono::milliseconds( 100 ),
      std::bind( &DistanceAdaptiveVoxelFilter::publisherSubscriptionCallback, this ) );
}

void DistanceAdaptiveVoxelFilter::pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg )
{
  // The two band lists are validated independently; their lengths can only be checked together.
  if ( band_distances_.size() != band_voxel_sizes_.size() ) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "band_distances (%zu) and band_voxel_sizes (%zu) must have the same length.",
        band_distances_.size(), band_voxel_sizes_.size() );
    return;
  }

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

  // Optional transform into the reference frame used for the distance and binning.
  const bool transform_points = !target_frame_.empty();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if ( transform_points ) {
    try {
      const geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
          target_frame_, msg.header.frame_id, rclcpp::Time( msg.header.stamp ),
          rclcpp::Duration::from_seconds( 0.1 ) );
      transform = tf2::transformToEigen( tf );
    } catch ( const tf2::TransformException &ex ) {
      RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 2000, "Could not transform '%s' to '%s': %s",
                             msg.header.frame_id.c_str(), target_frame_.c_str(), ex.what() );
      return;
    }
  }

  // Points beyond the last band are dropped; its bound is the effective max distance.
  const double max_distance_sq = band_schedule_.bands.back().distance_sq;

  sensor_msgs::msg::PointCloud2 output = voxel_common::filterCloud(
      msg, *xyz, *layout, max_distance_sq, occupied_,
      [this]( double dist_sq ) { return band_schedule_.choose( dist_sq ); },
      transform_points ? &transform : nullptr );

  output.header.frame_id = voxel_common::applyTfPrefix( tf_prefix_, output.header.frame_id );

  pointcloud_publisher_.publish( output );
}

void DistanceAdaptiveVoxelFilter::publisherSubscriptionCallback()
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

void DistanceAdaptiveVoxelFilter::startSubscribers()
{
  RCLCPP_INFO( get_logger(), "Starting subscriber" );
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_, 10,
      std::bind( &DistanceAdaptiveVoxelFilter::pointcloudCallback, this, std::placeholders::_1 ) );
}

void DistanceAdaptiveVoxelFilter::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

void DistanceAdaptiveVoxelFilter::printNodeStatus() const
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
  info << "  bands:        ";
  const size_t band_count = std::min( band_distances_.size(), band_voxel_sizes_.size() );
  for ( size_t i = 0; i < band_count; ++i )
    info << "(<=" << band_distances_[i] << "m: " << band_voxel_sizes_[i] << "m) ";
  info << std::endl;
  info << "  target_frame: " << ( target_frame_.empty() ? "<point frame>" : target_frame_ )
       << std::endl;
  info << "  tf_prefix:    " << ( tf_prefix_.empty() ? "<none>" : tf_prefix_ ) << std::endl;
  info << "  keep_fields:  "
       << ( keep_fields_.empty() ? "<all>" : std::to_string( keep_fields_.size() ) + " field(s)" );

  RCLCPP_INFO_STREAM( get_logger(), info.str() );
}

} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::DistanceAdaptiveVoxelFilter )
