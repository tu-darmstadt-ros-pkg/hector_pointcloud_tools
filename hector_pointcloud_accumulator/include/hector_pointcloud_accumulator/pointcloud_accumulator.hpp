// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POINTCLOUD_ACCUMULATOR_H
#define HECTOR_POINTCLOUD_ACCUMULATOR_H

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hector_pointcloud_accumulator
{

enum class AggregationMode { AVERAGE, HIGHEST_Z, CLOSEST_TO_CENTER };

class PointcloudAccumulatorBase
{
public:
  PointcloudAccumulatorBase( rclcpp::Node &node, double resolution, const std::string &frame,
                             const rclcpp::Rate &publish_rate );

  virtual ~PointcloudAccumulatorBase() = default;

  void publishPointcloud();

  void setEnabled( bool enabled );

  bool isEnabled() const { return enabled_; }

  virtual void reset();

  virtual void processPointcloud( const sensor_msgs::msg::PointCloud2::SharedPtr &msg ) = 0;

protected:
  void addField( const sensor_msgs::msg::PointField &field );

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

  bool enabled_ = true;
  bool updated_ = false;
  std::string frame_;
  double resolution_;
  unsigned int count_last_processed_ = 0;
  unsigned long count_total_processed_ = 0;
  unsigned int count_last_published_ = 0;

  sensor_msgs::msg::PointCloud2 accumulated_cloud_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr accumulated_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  std::mutex pointcloud_mutex_;
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

} // namespace hector_pointcloud_accumulator

#endif