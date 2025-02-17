//
// Created by stefan on 15.02.25.
//

#include "hector_pointcloud_io/pointcloud_io.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  std::vector<std::string> arguments = rclcpp::remove_ros_arguments( argc, argv );
  auto node = rclcpp::Node::make_shared( "load_pointcloud" );
  if ( arguments.size() != 3 && arguments.size() != 4 ) {
    RCLCPP_ERROR( node->get_logger(), "Usage: load_pointcloud <topic> <path> [frame]" );
    return 1;
  }
  std::string topic = arguments[1];
  std::string path = arguments[2];
  bool frame_was_provided = arguments.size() == 4;
  std::string frame = frame_was_provided ? arguments[3] : "map";
  sensor_msgs::msg::PointCloud2::SharedPtr pointcloud = hector_pointcloud_io::load_pointcloud( path );
  if ( !pointcloud ) {
    RCLCPP_ERROR( node->get_logger(), "Failed to load pointcloud from '%s'.", path.c_str() );
  }
  RCLCPP_INFO( node->get_logger(), "Loaded pointcloud with %d points.",
               pointcloud->width * pointcloud->height );
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>( topic, rclcpp::QoS( 1 ) );
  if ( pointcloud->header.frame_id.empty() || frame_was_provided ) {
    pointcloud->header.frame_id = frame;
  }
  if ( pointcloud->header.stamp.sec == 0 && pointcloud->header.stamp.nanosec == 0 ) {
    pointcloud->header.stamp = node->now();
  }
  RCLCPP_INFO( node->get_logger(), "Publishing pointcloud to topic '%s'.",
               publisher->get_topic_name() );
  size_t subscribers = publisher->get_subscription_count();
  publisher->publish( *pointcloud );
  while ( rclcpp::ok() ) {
    rclcpp::spin_some( node );
    if ( subscribers != publisher->get_subscription_count() ) {
      if ( publisher->get_subscription_count() > subscribers ) {
        RCLCPP_INFO( node->get_logger(), "New subscriber detected. Republishing pointcloud." );
        publisher->publish( *pointcloud );
      }
      subscribers = publisher->get_subscription_count();
    }
  }
  rclcpp::shutdown();
  return 0;
}
