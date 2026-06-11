#ifndef HECTOR_POINTCLOUD_PROCESSING_VOXEL_FILTER_HPP
#define HECTOR_POINTCLOUD_PROCESSING_VOXEL_FILTER_HPP

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "hector_pointcloud_processing/_internal/hector_ros2_utils.hpp"
#include "hector_pointcloud_processing/voxel_common.hpp"
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hector_pointcloud_processing
{

/*!
 * Uniform voxel grid filter.
 *
 * Overlays a fixed-size voxel grid on the cloud and keeps the first input point falling into each
 * voxel, copying it verbatim so all fields are preserved. Points farther than max_distance from the
 * cloud origin (the sensor/robot) are dropped first.
 *
 * For a voxel size that grows with distance, use the DistanceAdaptiveVoxelFilter instead.
 */
class VoxelFilter : public hector::Node
{
public:
  VoxelFilter( const rclcpp::NodeOptions &options );

private:
  void setup();

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg );

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
  double voxel_size_;
  double max_distance_;
  std::string tf_prefix_;
  std::vector<std::string> keep_fields_;

  //! Voxel occupancy set, reused across clouds to keep its allocated capacity.
  std::unordered_set<voxel_common::VoxelKey, voxel_common::VoxelKeyHash> occupied_;

  bool has_subscribers_ = false;
};

} // namespace hector_pointcloud_processing

#endif // HECTOR_POINTCLOUD_PROCESSING_VOXEL_FILTER_HPP
