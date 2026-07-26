#include "hector_pointcloud_processing/pointcloud_decimator.hpp"
#include <cstring>
#include <functional>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hector_pointcloud_processing
{

PointcloudDecimator::PointcloudDecimator( const rclcpp::NodeOptions &options )
    : Node( "pointcloud_decimator", options ), input_( "pointcloud" ),
      output_( "pointcloud_decimated" ), elimination_method_( "count" ),
      elimination_quantifier_( "fraction" ), point_fraction_( 0.1 ), point_count_( 1000 )
{
  // Parameters
  // elimination_method:     How to decimate the pointcloud
  // elimination_quantifier: How the amount of points to be eliminated is specified
  // point_fraction:         Fraction of points to keep
  // point_count:            Number of points to keep

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

  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(
      std::shared_ptr<PointcloudDecimator>( this,
                                            []( PointcloudDecimator * ) { /* no-op deleter */ } ) );
  node_topics_interface_ = get_node_topics_interface();

  printNodeStatus();

  setup();
}

void PointcloudDecimator::setup()
{
  // publisher for publishing outgoing messages
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pointcloud_publisher_ =
      pct_->advertise( output_, rclcpp::SensorDataQoS().get_rmw_qos_profile(), publisher_options );

  check_subscribers_timer_ = create_wall_timer( std::chrono::milliseconds( 100 ),
                                                [this] { publisherSubscriptionCallback(); } );
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

  RCLCPP_DEBUG( get_logger(), "Reducing %zu points to %zu", input_size, point_count );

  // Copy shared fields. Published as unique_ptr to allow zero-copy intra-process delivery (once
  // pointcloud transport supports it).
  auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
  output->header = msg.header;
  output->height = 1;
  output->fields = msg.fields;
  output->is_bigendian = msg.is_bigendian;
  output->point_step = msg.point_step;
  output->is_dense = msg.is_dense;

  if ( elimination_method_ == "random" ) {
    // Choose points randomly. This will usually not match the chosen fraction/count exactly
    output->data.reserve( point_count * point_step );
    std::uniform_real_distribution<double> distribution( 0.0, 1.0 );
    for ( size_t point = 0; point < input_size; ++point ) {
      if ( distribution( random_generator_ ) > point_fraction )
        continue;

      const unsigned char *source = msg.data.data() + point * point_step;
      output->data.insert( output->data.end(), source, source + point_step );
    }
  } else {
    // Choose points with roughly equal distance. This could theoretically cause artifacts
    output->data.resize( point_count * point_step );
    unsigned char *destination = output->data.data();
    const double stride = 1.0 / point_fraction;
    for ( size_t point = 0; point < point_count; ++point ) {
      const size_t source_index = static_cast<size_t>( point * stride ) * point_step;

      std::memcpy( destination, msg.data.data() + source_index, point_step );
      destination += point_step;
    }
  }

  output->row_step = output->data.size();
  output->width = output->row_step / point_step;

  pointcloud_publisher_.publish( std::move( output ) );
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
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_, rclcpp::SensorDataQoS(),
      [this]( const sensor_msgs::msg::PointCloud2 &msg ) { pointcloudCallback( msg ); },
      subscription_options );
}

void PointcloudDecimator::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

void PointcloudDecimator::printNodeStatus() const
{
  const std::string input_remapped = node_topics_interface_->resolve_topic_name( input_ );
  const std::string output_remapped = node_topics_interface_->resolve_topic_name( output_ );

  std::stringstream info;
  info << "The node has the following attributes:" << std::endl;
  if ( input_ == input_remapped ) {
    info << "  input:                  " << input_ << std::endl;
  } else {
    info << "  input remapped to:      " << input_remapped << std::endl;
  }
  if ( output_ == output_remapped ) {
    info << "  output:                 " << output_ << std::endl;
  } else {
    info << "  output remapped to:     " << output_remapped << std::endl;
  }
  info << "  elimination_method:     " << elimination_method_ << std::endl;
  info << "  elimination_quantifier: " << elimination_quantifier_ << std::endl;
  info << "  point_fraction:         " << point_fraction_ << std::endl;
  info << "  point_count:            " << point_count_ << std::endl;

  RCLCPP_INFO_STREAM( get_logger(), info.str() );
}

} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::PointcloudDecimator )
