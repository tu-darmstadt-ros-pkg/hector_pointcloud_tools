#ifndef HECTOR_POINTCLOUD_DECIMATOR_POINTCLOUD_DECIMATOR_HPP
#define HECTOR_POINTCLOUD_DECIMATOR_POINTCLOUD_DECIMATOR_HPP

#include <memory>
#include <string>
#include <vector>

#include <hector_ros2_utils/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>


namespace hector_pointcloud_decimator
{

class PointcloudDecimator : public hector::Node
{
public:
  PointcloudDecimator();

private:
  //! @brief Sets up subscribers, publishers, etc. to configure the node
  void setup();

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2& msg );


  void enabledCallback( const bool& enabled );

  void msgEnabledCallback( const std_msgs::msg::Bool::ConstSharedPtr& enabled );

  void publisherSubscriptionCallback();

  void publishEnableStatus() const;

  void startSubscribers();

  void stopSubscribers();

private:
  rclcpp::Node::SharedPtr pct_node_;
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  point_cloud_transport::Publisher pointcloud_publisher_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enabled_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
  rclcpp::TimerBase::SharedPtr check_subscribers_timer_;

  // Parameters
  std::string input_;
  std::string output_;
  std::string elimination_method_;
  std::string elimination_quantifier_;
  double point_fraction_;
  int point_count_;

  bool has_subscribers_ = true;
  bool enabled_ = true;
};

}

#endif // HECTOR_POINTCLOUD_DECIMATOR_POINTCLOUD_DECIMATOR_HPP
