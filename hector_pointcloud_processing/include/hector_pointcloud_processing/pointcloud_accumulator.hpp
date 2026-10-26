// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_ACCUMULATOR_HPP
#define HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_ACCUMULATOR_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <unordered_map>

namespace hector_pointcloud_processing
{

enum class AggregationMode { AVERAGE, HIGHEST_Z, CLOSEST_TO_CENTER };

class PointcloudAccumulatorBase
{
public:
  //! max_range: points farther from the origin of their cloud's frame are skipped, 0 keeps all.
  PointcloudAccumulatorBase( rclcpp::Node &node, double resolution, const std::string &frame,
                             const rclcpp::Rate &publish_rate, const std::vector<std::string> &topics,
                             size_t queue_size = 10, double max_range = 0.0 );

  virtual ~PointcloudAccumulatorBase();

  void publishPointcloud();

  void setEnabled( bool enabled );

  bool isEnabled() const { return enabled_; }

  //! Clouds stamped before the reset are dropped afterwards.
  void reset();

protected:
  //! Called by the most derived destructor, before the members the thread uses are destroyed.
  void stop();

  //! Called with both mutexes held.
  virtual void clearVoxels() = 0;

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

  //! 21 bits per axis, unique within 2^20 voxels of the origin in each direction.
  struct index_hash {
    std::size_t operator()( const Index &index ) const noexcept
    {
      constexpr uint64_t mask = ( uint64_t{ 1 } << 21 ) - 1;
      const auto bits = [mask]( int value ) {
        return ( static_cast<uint64_t>( value ) + ( uint64_t{ 1 } << 20 ) ) & mask;
      };
      return static_cast<std::size_t>( bits( index.x ) << 42 | bits( index.y ) << 21 |
                                       bits( index.z ) );
    }
  };

  bool shutting_down_ = false; // guarded by pointcloud_queue_mutex_
  std::atomic<bool> enabled_ = true;
  bool updated_ = false;
  std::string frame_;
  double resolution_;
  size_t max_queue_size_;
  double max_range_;
  unsigned int count_last_processed_ = 0;
  unsigned long count_total_processed_ = 0;
  unsigned int count_last_published_ = 0;
  std::atomic<unsigned int> dropped_pointclouds_ = 0;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time reset_stamp_;
  rclcpp::Time latest_stamp_;

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

  ~PointcloudAccumulator() override { stop(); }

  void processPointcloud( const sensor_msgs::msg::PointCloud2::SharedPtr &msg ) override;

protected:
  void clearVoxels() override { voxel_map_.clear(); }

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
