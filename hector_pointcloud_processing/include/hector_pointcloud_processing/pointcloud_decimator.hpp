#ifndef HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_DECIMATOR_HPP
#define HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_DECIMATOR_HPP

#include <memory>
#include <random>
#include <string>

#include "hector_pointcloud_processing/_internal/hector_ros2_utils.hpp"
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hector_pointcloud_processing
{

class PointcloudDecimator : public hector::Node
{
public:
  PointcloudDecimator( const rclcpp::NodeOptions &options );

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg );

  void publisherSubscriptionCallback();

  void startSubscribers();

  void stopSubscribers();

  void printNodeStatus() const;

private:
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  point_cloud_transport::Publisher pointcloud_publisher_;

  rclcpp::TimerBase::SharedPtr check_subscribers_timer_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;

  std::mt19937 random_generator_{ std::random_device{}() };

  // Parameters
  std::string input_;
  std::string output_;
  std::string elimination_method_;
  std::string elimination_quantifier_;
  double point_fraction_;
  int point_count_;

  bool has_subscribers_ = false;
};

} // namespace hector_pointcloud_processing

#endif // HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_DECIMATOR_HPP
