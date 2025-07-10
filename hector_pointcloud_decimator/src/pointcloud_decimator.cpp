#include "hector_pointcloud_decimator/pointcloud_decimator.hpp"
#include <functional>
#include <hector_ros2_utils/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>



namespace hector_pointcloud_decimator
{

PointcloudDecimator::PointcloudDecimator() : Node("pointcloud_decimator"),
  input_("/pointcloud"), output_("/pointcloud_decimated"),
  elimination_method_("count"), elimination_quantifier_("fraction"),
  point_fraction_(0.1), point_count_(1000)
{
  // Parameters
  // input:                  Input topic
  // output:                 Output topic
  // elimination_method:     How to decimate the pointcloud
  // elimination_quantifier: How the amount of points to be eliminated is specified
  // point_fraction:         Fraction of points to keep
  // point_count:            Number of points to keep
  // enabled:                Whether the node should function

  declare_reconfigurable_parameter(
    "input", std::ref(input_), "The input topic for pointclouds",
    hector::ParameterOptions<std::string>().onUpdate([this]( const std::string& ) {
      this->setup();
    })
  );
  // Base the default output topic on the chosen input topic
  output_ = input_ + "_decimated";
  declare_reconfigurable_parameter(
    "output", std::ref(output_), "The output topic for pointcloud_transport",
    hector::ParameterOptions<std::string>().onUpdate([this]( const auto& ) {
      this->setup();
    })
  );
  declare_reconfigurable_parameter(
    "elimination_method", std::ref(elimination_method_), "The method used to decimate the pointcloud",
    hector::ParameterOptions<std::string>()
      .setAdditionalConstraints( "Allowed values: 'random', 'count'" )
      .onValidate([]( const std::string& value ) {
        return value == "random" || value == "count";
      })
  );
  declare_reconfigurable_parameter(
    "elimination_quantifier", std::ref(elimination_quantifier_),
    "How the amount of points to be eliminated is specified",
    hector::ParameterOptions<std::string>()
      .setAdditionalConstraints( "Allowed values: 'fraction', 'count'" )
      .onValidate( []( const std::string& value ) {
        return value == "fraction" || value == "count";
      })
  );
  declare_reconfigurable_parameter(
    "point_fraction", std::ref(point_fraction_), "The fraction of points to keep",
    hector::ParameterOptions<double>()
      .setAdditionalConstraints( "Mutually exclusive with point_total" )
      .onValidate([]( const double& value ) {
        return value >= 0.0 && value <= 1.0;
      })
  );
  declare_reconfigurable_parameter(
    "point_count", std::ref(point_count_), "The total number of points to keep",
    hector::ParameterOptions<int>().onValidate([]( const int& value ) {
      return value >= 0;
    })
  );
  declare_reconfigurable_parameter(
    "enabled", std::ref(enabled_), "Whether the node should be active",
    hector::ParameterOptions<bool>().onUpdate( [this]( const bool& value ) {
      this->enabledCallback( value );
    })
  );

  pct_node_ = std::make_shared<rclcpp::Node>(static_cast<std::string>(get_name()) + "_pct");
  pct_ = std::make_unique<point_cloud_transport::PointCloudTransport>(pct_node_);
  setup();
}

void PointcloudDecimator::setup()
{
  // subscriber for handling incoming messages
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_, 10, std::bind(&PointcloudDecimator::pointcloudCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Subscribed to '%s'", pointcloud_subscriber_->get_topic_name());

  // publisher for publishing outgoing messages
  pointcloud_publisher_ = pct_->advertise(output_, 10);
  RCLCPP_INFO(get_logger(), "Publishing results to '%s'", pointcloud_publisher_.getTopic().c_str());

  // Node enable subscription / publisher
  enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    static_cast<std::string>(get_name()) + "/enabled", 10,
    std::bind(&PointcloudDecimator::msgEnabledCallback, this, std::placeholders::_1));
  enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
    static_cast<std::string>(get_name()) + "/enabled_status", 10);

  check_subscribers_timer_ =
      create_wall_timer( std::chrono::seconds( 1 ),
                                std::bind( &PointcloudDecimator::publisherSubscriptionCallback, this ) );

  std::srand(std::time({}));
}


void PointcloudDecimator::pointcloudCallback(const sensor_msgs::msg::PointCloud2& msg) {
  const size_t point_step = msg.point_step;
  const size_t input_size = msg.height * msg.width;

  const size_t point_count = elimination_quantifier_ == "count"
    ? std::min(static_cast<size_t>(point_count_), input_size)
    : static_cast<size_t>(static_cast<double>(input_size) * point_fraction_);
  const double point_fraction = elimination_quantifier_ == "count"
    ? std::min(1.0, static_cast<double>(point_count_) / static_cast<double>(input_size))
    : point_fraction_;

  RCLCPP_INFO(get_logger(), "Reducing %lu points to %lu", input_size, point_count);

  // Copy shared fields
  sensor_msgs::msg::PointCloud2 output;
  output.header = msg.header;
  output.height = 1;
  output.fields = msg.fields;
  output.is_bigendian = msg.is_bigendian;
  output.point_step = msg.point_step;
  output.is_dense = msg.is_dense;

  output.data.clear();
  output.data.reserve(output.row_step);

  if (elimination_method_ == "random") {
    // Choose randomly which points to include. This will likely not match the chosen fraction/count exactly
    for (size_t point = 0; point < input_size; ++point) {
      const size_t source_index = point * point_step;

      if (static_cast<double>(std::rand()) / RAND_MAX > point_fraction)
        continue;

      for (size_t e = 0; e < point_step; ++e) {
        output.data.push_back( msg.data[source_index + e] );
      }
    }
  }
  else {
    for (size_t point = 0; point < point_count; ++point) {
      const size_t source_index = static_cast<size_t>(point / point_fraction) * point_step;

      for (size_t e = 0; e < point_step; ++e) {
        output.data.push_back( msg.data[source_index + e] );
      }
    }
  }

  output.row_step = output.data.size();
  output.width = output.row_step / point_step;

  printf("output length: %d\n", output.width);

  pointcloud_publisher_.publish(output);
}

void PointcloudDecimator::enabledCallback( const bool &enabled )
{
  // Changed to disabled
  if ( !enabled && enabled_ ) {
    enabled_ = false;
    if ( has_subscribers_ )
      stopSubscribers();
  }
  // Changed to enabled
  if ( enabled && !enabled_ ) {
    enabled_ = true;
    if ( has_subscribers_ )
      startSubscribers();
  }

  publishEnableStatus();
}

void PointcloudDecimator::msgEnabledCallback( const std_msgs::msg::Bool::ConstSharedPtr &enabled )
{
  const rclcpp::Parameter parameter( "enabled", rclcpp::ParameterValue( enabled->data ) );
  set_parameter( parameter );
}

void PointcloudDecimator::publisherSubscriptionCallback()
{

  const size_t subscribers = pointcloud_publisher_.getNumSubscribers();

  RCLCPP_DEBUG_STREAM( get_logger(),
                       "Subscribers: " << subscribers << " has_subscribers_: " << has_subscribers_
                                       << " enabled_: " << enabled_ );

  // Changed to no subscribers
  if ( subscribers == 0 && has_subscribers_ ) {
    has_subscribers_ = false;
    if ( enabled_ )
      stopSubscribers();
  }
  // Changed from no subscribers
  if ( subscribers > 0 && !has_subscribers_ ) {
    has_subscribers_ = true;
    if ( enabled_ )
      startSubscribers();
  }
}

void PointcloudDecimator::publishEnableStatus() const
{
  std_msgs::msg::Bool bool_msg;
  bool_msg.data = enabled_;
  enabled_pub_->publish( bool_msg );

  RCLCPP_INFO_STREAM( get_logger(), "Node " << ( enabled_ ? "Enabled" : "Disabled" ));
}

void PointcloudDecimator::startSubscribers()
{
  RCLCPP_INFO( get_logger(), "Starting subscribers" );
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_, 10, std::bind(&PointcloudDecimator::pointcloudCallback, this, std::placeholders::_1));
}

void PointcloudDecimator::stopSubscribers()
{
  RCLCPP_INFO( get_logger(), "Stopping subscribers" );
  pointcloud_subscriber_.reset();
}

}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hector_pointcloud_decimator::PointcloudDecimator>());
  rclcpp::shutdown();

  return 0;
}
