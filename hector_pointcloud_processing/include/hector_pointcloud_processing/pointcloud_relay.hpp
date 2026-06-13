#ifndef HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_RELAY_HPP
#define HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_RELAY_HPP

#include <memory>
#include <string>

#include "hector_pointcloud_processing/_internal/hector_ros2_utils.hpp"
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hector_pointcloud_processing
{

/*!
 * Relays a PointCloud2 from input to output, republishing through point_cloud_transport.
 *
 * The subscription callback takes a ConstSharedPtr so that, inside a component container with
 * intra-process comms enabled, the cloud is handed over without a copy. When tf_prefix is empty
 * the same shared message is forwarded untouched (zero copy on the raw transport); setting
 * tf_prefix prepends a tf prefix ("prefix/frame") to the header frame id, which requires an owned
 * copy of the message to modify.
 *
 * Like the other nodes in this package, input/output topics are set via remapping rather than
 * parameters, and the subscriber is only created while the output has subscribers.
 */
class PointcloudRelay : public hector::Node
{
public:
  PointcloudRelay( const rclcpp::NodeOptions &options );

private:
  void setup();

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg );

  void publisherSubscriptionCallback();

  void startSubscribers();

  void stopSubscribers();

  void printNodeStatus() const;

  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  point_cloud_transport::Publisher pointcloud_publisher_;

  rclcpp::TimerBase::SharedPtr check_subscribers_timer_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;

  // Parameters
  std::string input_;
  std::string output_;
  std::string tf_prefix_;

  bool has_subscribers_ = false;
};

} // namespace hector_pointcloud_processing

#endif // HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_RELAY_HPP
