// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_ACCUMULATOR_H
#define HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_ACCUMULATOR_H

#include <Eigen/Geometry>
#include <mutex>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hector_pointcloud_processing
{

enum class AggregationMode { AVERAGE, HIGHEST_Z, CLOSEST_TO_CENTER };

class PointcloudAccumulatorBase
{
public:
  PointcloudAccumulatorBase( rclcpp::Node &node, double resolution, const std::string &frame,
                             const rclcpp::Rate &publish_rate,
                             const std::vector<std::string> &topics, size_t queue_size = 10 );

  virtual ~PointcloudAccumulatorBase();

  void publishPointcloud();

  void setEnabled( bool enabled );

  bool isEnabled() const { return enabled_; }

  virtual void reset();

protected:
  void onNewPointcloud( const sensor_msgs::msg::PointCloud2::SharedPtr &msg );

  void processQueue();

  void addField( const sensor_msgs::msg::PointField &field );

  virtual void processPointcloud( const sensor_msgs::msg::PointCloud2::SharedPtr &msg ) = 0;

  struct Index {
    int x;
    int y;
    int z;

    bool operator==( const Index &other ) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct index_hash {
    long cantor( long a, long b ) const noexcept { return ( a + b + 1 ) * ( a + b ) / 2 + b; }

    std::size_t operator()( const Index &index ) const noexcept
    {
      return cantor( index.x, cantor( index.y, index.z ) );
    }
  };

  bool shutting_down_ = false;
  bool enabled_ = true;
  bool updated_ = false;
  std::string frame_;
  double resolution_;
  size_t max_queue_size_;
  unsigned int count_last_processed_ = 0;
  unsigned long count_total_processed_ = 0;
  unsigned int count_last_published_ = 0;
  unsigned int dropped_pointclouds_ = 0;

  sensor_msgs::msg::PointCloud2 accumulated_cloud_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  rclcpp::Logger logger_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_publisher_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subscriptions_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> pointcloud_queue_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

  std::thread processing_thread_;
  std::mutex accumulated_pointcloud_mutex_;
  std::mutex pointcloud_queue_mutex_;
  std::condition_variable data_available;
};

template<AggregationMode AGGREGATION_MODE = AggregationMode::AVERAGE>
class PointcloudAccumulator : public PointcloudAccumulatorBase
{
  struct PointInfo;

public:
  using PointcloudAccumulatorBase::PointcloudAccumulatorBase;

  void reset() override;

  void processPointcloud( const sensor_msgs::msg::PointCloud2::SharedPtr &msg ) override;

private:
  template<typename T>
  void processPointCloudData( const sensor_msgs::msg::PointCloud2::SharedPtr &msg );

  PointInfo addNewPoint( const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                         const uint8_t *point_data, const Index &index,
                         const Eigen::Vector3f &point );

  void updatePoint( const Index &index, const Eigen::Vector3f &point, PointInfo &info );

  std::unordered_map<Index, PointInfo, index_hash> voxel_map_;
};

template<>
struct PointcloudAccumulator<AggregationMode::AVERAGE>::PointInfo {
  size_t index;
  size_t count;
};

template<>
struct PointcloudAccumulator<AggregationMode::HIGHEST_Z>::PointInfo {
  size_t index;
  float z;
};

template<>
struct PointcloudAccumulator<AggregationMode::CLOSEST_TO_CENTER>::PointInfo {
  size_t index;
  float distance;
};

} // namespace hector_pointcloud_processing

#endif
