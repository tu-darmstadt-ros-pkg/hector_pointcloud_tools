#include "hector_pointcloud_decimator/pointcloud_decimator.hpp"
#include <functional>
#include <hector_ros2_utils/node.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>

namespace hector_pointcloud_decimator
{

PointcloudDecimator::PointcloudDecimator( const rclcpp::NodeOptions &options )
    : Node( "pointcloud_decimator", options ), input_( "/pointcloud" ),
      output_( "/pointcloud_decimated" ), elimination_method_( "count" ),
      elimination_quantifier_( "fraction" ), point_fraction_( 0.1 ), point_count_( 1000 )
{
  // Parameters
  // input:                  Input topic
  // output:                 Output topic
  // elimination_method:     How to decimate the pointcloud
  // elimination_quantifier: How the amount of points to be eliminated is specified
  // point_fraction:         Fraction of points to keep
  // point_count:            Number of points to keep

  declare_reconfigurable_parameter( "input", std::ref( input_ ), "The input topic for pointclouds",
                                    hector::ParameterOptions<std::string>().onUpdate(
                                        [this]( const std::string & ) { this->setup(); } ) );
  // Base the default output topic on the chosen input topic
  output_ = input_ + "_decimated";
  declare_reconfigurable_parameter(
      "output", std::ref( output_ ), "The output topic for pointcloud_transport",
      hector::ParameterOptions<std::string>().onUpdate( [this]( const auto & ) { this->setup(); } ) );
  declare_reconfigurable_parameter(
      "elimination_method", std::ref( elimination_method_ ),
      "The method used to decimate the pointcloud",
      hector::ParameterOptions<std::string>()
          .setAdditionalConstraints( "Allowed values: 'random', 'count'" )
          .onValidate(
              []( const std::string &value ) { return value == "random" || value == "count"; } ) );
  declare_reconfigurable_parameter(
      "elimination_quantifier", std::ref( elimination_quantifier_ ),
      "How the amount of points to be eliminated is specified",
      hector::ParameterOptions<std::string>()
          .setAdditionalConstraints( "Allowed values: 'fraction', 'count'" )
          .onValidate( []( const std::string &value ) {
            return value == "fraction" || value == "count";
          } ) );
  declare_reconfigurable_parameter(
      "point_fraction", std::ref( point_fraction_ ), "The fraction of points to keep",
      hector::ParameterOptions<double>()
          .setAdditionalConstraints( "Mutually exclusive with point_total" )
          .onValidate( []( const double &value ) { return value >= 0.0 && value <= 1.0; } ) );
  declare_reconfigurable_parameter(
      "point_count", std::ref( point_count_ ), "The total number of points to keep",
      hector::ParameterOptions<int>().onValidate( []( const int &value ) { return value >= 0; } ) );

  // pct_node_ = std::make_shared<rclcpp::Node>( static_cast<std::string>( get_name() ) + "_pct" );
  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>( shared_from_this() );
  setup();
}

void PointcloudDecimator::setup()
{
  pointcloud_subscriber_ = {};

  // publisher for publishing outgoing messages
  pointcloud_publisher_ = pct_->advertise( output_, 10 );
  RCLCPP_INFO( get_logger(), "Publishing to '%s'", pointcloud_publisher_.getTopic().c_str() );

  check_subscribers_timer_ =
      create_wall_timer( std::chrono::milliseconds( 100 ),
                         std::bind( &PointcloudDecimator::publisherSubscriptionCallback, this ) );

  std::srand( std::time( {} ) );
}

void PointcloudDecimator::pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg )
{
  const size_t point_step = msg.point_step;
  const size_t input_size = msg.height * msg.width;

  size_t point_count;
  double point_fraction;
  if ( elimination_quantifier_ == "count" ) {
    point_count = std::min( static_cast<size_t>( point_count_ ), input_size );
    point_fraction =
        std::min( 1.0, static_cast<double>( point_count_ ) / static_cast<double>( input_size ) );
  } else {
    point_count = static_cast<size_t>( static_cast<double>( input_size ) * point_fraction_ );
    point_fraction = point_fraction_;
  }

  RCLCPP_DEBUG( get_logger(), "Reducing %lu points to %lu", input_size, point_count );

  // Copy shared fields
  sensor_msgs::msg::PointCloud2 output;
  output.header = msg.header;
  output.height = 1;
  output.fields = msg.fields;
  output.is_bigendian = msg.is_bigendian;
  output.point_step = msg.point_step;
  output.is_dense = false;

  output.data.clear();
  output.data.reserve( point_count * point_step );

  if ( elimination_method_ == "random" ) {
    // Choose randomly which points to include. This will likely not match the chosen fraction/count exactly
    for ( size_t point = 0; point < input_size; ++point ) {
      const size_t source_index = point * point_step;

      if ( static_cast<double>( std::rand() ) / RAND_MAX > point_fraction )
        continue;

      for ( size_t e = 0; e < point_step; ++e ) {
        output.data.push_back( msg.data[source_index + e] );
      }
    }
  } else {
    for ( size_t point = 0; point < point_count; ++point ) {
      const size_t source_index = static_cast<size_t>( point / point_fraction ) * point_step;

      for ( size_t e = 0; e < point_step; ++e ) {
        output.data.push_back( msg.data[source_index + e] );
      }
    }
  }

  output.row_step = output.data.size();
  output.width = output.row_step / point_step;

  pointcloud_publisher_.publish( output );
}

void PointcloudDecimator::publisherSubscriptionCallback()
{
  const size_t subscribers = pointcloud_publisher_.getNumSubscribers();

  // Changed to no subscribers
  if ( subscribers == 0 && has_subscribers_ ) {
    has_subscribers_ = false;
    stopSubscribers();
  }
  // Changed from no subscribers
  if ( subscribers > 0 && !has_subscribers_ ) {
    has_subscribers_ = true;
    startSubscribers();
  }
}

void PointcloudDecimator::startSubscribers()
{
  RCLCPP_INFO( get_logger(), "Starting subscriber" );
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_, 10, std::bind( &PointcloudDecimator::pointcloudCallback, this, std::placeholders::_1 ) );
}

void PointcloudDecimator::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

} // namespace hector_pointcloud_decimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_decimator::PointcloudDecimator )
