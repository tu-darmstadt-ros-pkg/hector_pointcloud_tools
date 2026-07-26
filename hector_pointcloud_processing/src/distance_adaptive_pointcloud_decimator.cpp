#include "hector_pointcloud_processing/distance_adaptive_pointcloud_decimator.hpp"

#include <functional>
#include <rclcpp/publisher_options.hpp>
#include <sstream>

#include <Eigen/Geometry>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hector_pointcloud_processing/voxel_common.hpp"

namespace hector_pointcloud_processing
{
namespace
{

//! Validates distances: non-empty, all >= 0, strictly increasing.
bool validDistances( const std::vector<double> &v )
{
  if ( v.empty() )
    return false;
  for ( size_t i = 0; i < v.size(); ++i ) {
    if ( v[i] < 0.0 )
      return false;
    if ( i > 0 && v[i] <= v[i - 1] )
      return false;
  }
  return true;
}

//! Validates percentages: non-empty, all in [0, 1].
bool validPercentages( const std::vector<double> &v )
{
  if ( v.empty() )
    return false;
  for ( const double value : v )
    if ( value < 0.0 || value > 1.0 )
      return false;
  return true;
}

} // namespace

DistanceAdaptivePointcloudDecimator::DistanceAdaptivePointcloudDecimator(
    const rclcpp::NodeOptions &options )
    : Node( "distance_adaptive_pointcloud_decimator", options ), input_( "pointcloud" ),
      output_( "pointcloud_decimated" ), schedule_{ { 2.0, 10.0, 30.0 }, { 1.0, 0.5, 0.1 } },
      target_frame_( "" ), tf_prefix_( "" )
{
  declare_readonly_parameter(
      "distances", schedule_.distances,
      "Ascending control-point ranges (m); strictly increasing, all >= 0, same length as "
      "percentages. Index i pairs with percentages[i]" );
  declare_readonly_parameter(
      "percentages", schedule_.percentages,
      "Keep fraction in [0, 1] for each control-point range; same length as distances. Index i "
      "pairs with distances[i]" );
  declare_readonly_parameter(
      "target_frame", target_frame_,
      "Frame the point position is expressed in for the distance; empty uses the raw point xyz" );
  declare_readonly_parameter( "tf_prefix", tf_prefix_,
                              "Prefix prepended to frame ids before publishing decimated cloud" );

  if ( !validDistances( schedule_.distances ) )
    throw rclcpp::exceptions::InvalidParametersException(
        "distances must be non-empty, all >= 0 and strictly increasing." );
  if ( !validPercentages( schedule_.percentages ) )
    throw rclcpp::exceptions::InvalidParametersException(
        "percentages must be non-empty and all in [0, 1]." );
  if ( schedule_.distances.size() != schedule_.percentages.size() )
    throw rclcpp::exceptions::InvalidParametersException(
        "distances and percentages must have the same length." );

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>( get_clock() );
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, this );

  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(
      std::shared_ptr<DistanceAdaptivePointcloudDecimator>(
          this, []( DistanceAdaptivePointcloudDecimator * ) { /* no-op deleter */ } ) );
  node_topics_interface_ = get_node_topics_interface();

  printNodeStatus();

  setup();
}

void DistanceAdaptivePointcloudDecimator::setup()
{
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pointcloud_publisher_ =
      pct_->advertise( output_, rclcpp::SensorDataQoS().get_rmw_qos_profile(), publisher_options );

  check_subscribers_timer_ = create_wall_timer(
      std::chrono::milliseconds( 100 ),
      std::bind( &DistanceAdaptivePointcloudDecimator::publisherSubscriptionCallback, this ) );
}

void DistanceAdaptivePointcloudDecimator::pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg )
{
  const auto xyz = voxel_common::findXyzOffsets( msg );
  if ( !xyz ) {
    RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 2000,
                           "Input cloud has no FLOAT32 x/y/z fields, cannot decimate." );
    return;
  }

  // Optional transform into the reference frame used for the distance.
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

  const size_t point_step = msg.point_step;
  const size_t input_size = static_cast<size_t>( msg.height ) * msg.width;

  // Copy shared fields. Published as unique_ptr to allow zero-copy intra-process delivery (once
  // pointcloud transport supports it).
  auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
  output->header = msg.header;
  output->height = 1;
  output->fields = msg.fields;
  output->is_bigendian = msg.is_bigendian;
  output->point_step = msg.point_step;
  output->is_dense = true;
  output->data.reserve(
      input_size * point_step *
      *std::max_element( schedule_.percentages.begin(), schedule_.percentages.end() ) );

  const uint8_t *src = msg.data.data();
  std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
  for ( size_t point = 0; point < input_size; ++point ) {
    const uint8_t *sp = src + point * point_step;
    const float x = voxel_common::readFloat( sp, xyz->x );
    const float y = voxel_common::readFloat( sp, xyz->y );
    const float z = voxel_common::readFloat( sp, xyz->z );
    if ( !std::isfinite( x ) || !std::isfinite( y ) || !std::isfinite( z ) )
      continue;

    Eigen::Vector3d position( x, y, z );
    if ( transform_points )
      position = transform * position;

    const double keep_probability = schedule_.keepProbability( position.norm() );
    if ( distribution( random_generator_ ) > keep_probability )
      continue;

    output->data.insert( output->data.end(), sp, sp + point_step );
  }

  output->row_step = output->data.size();
  output->width = point_step == 0 ? 0 : output->row_step / point_step;

  output->header.frame_id = voxel_common::applyTfPrefix( tf_prefix_, output->header.frame_id );

  pointcloud_publisher_.publish( std::move( output ) );
}

void DistanceAdaptivePointcloudDecimator::publisherSubscriptionCallback()
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

void DistanceAdaptivePointcloudDecimator::startSubscribers()
{
  RCLCPP_INFO( get_logger(), "Starting subscriber" );
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_, rclcpp::SensorDataQoS(),
      [this]( const sensor_msgs::msg::PointCloud2 &msg ) { pointcloudCallback( msg ); },
      subscription_options );
}

void DistanceAdaptivePointcloudDecimator::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

void DistanceAdaptivePointcloudDecimator::printNodeStatus() const
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
  for ( size_t i = 0; i < schedule_.distances.size(); ++i )
    info << "(" << schedule_.distances[i] << "m: " << schedule_.percentages[i] << ") ";
  info << std::endl;
  info << "  target_frame: " << ( target_frame_.empty() ? "<point frame>" : target_frame_ )
       << std::endl;
  info << "  tf_prefix:    " << ( tf_prefix_.empty() ? "<none>" : tf_prefix_ );

  RCLCPP_INFO_STREAM( get_logger(), info.str() );
}

} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::DistanceAdaptivePointcloudDecimator )
