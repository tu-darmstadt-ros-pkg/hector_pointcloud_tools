#ifndef HECTOR_POINTCLOUD_DECIMATOR_POINTCLOUD_DECIMATOR_HPP
#define HECTOR_POINTCLOUD_DECIMATOR_POINTCLOUD_DECIMATOR_HPP

#include <memory>
#include <string>
#include <vector>

#include <hector_ros2_utils/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
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

  void topicCallback( const std_msgs::msg::Int32& msg );

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2& msg );


private:

  rclcpp::Node::SharedPtr pct_node_;
  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  point_cloud_transport::Publisher pointcloud_publisher_;

  // Parameters
  std::string input_ = "/pointcloud";
  std::string output_ = "/pointcloud_decimated";
  std::string elimination_method_ = "random";
  std::string elimination_quantifier_ = "fraction";
  double point_fraction_ = 0.1;
  int point_count_ = 1000;
};

}

#endif // HECTOR_POINTCLOUD_DECIMATOR_POINTCLOUD_DECIMATOR_HPP
