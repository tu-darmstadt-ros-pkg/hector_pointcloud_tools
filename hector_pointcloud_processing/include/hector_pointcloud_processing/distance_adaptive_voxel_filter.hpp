#ifndef HECTOR_POINTCLOUD_PROCESSING_DISTANCE_ADAPTIVE_VOXEL_FILTER_HPP
#define HECTOR_POINTCLOUD_PROCESSING_DISTANCE_ADAPTIVE_VOXEL_FILTER_HPP

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "hector_pointcloud_processing/_internal/hector_ros2_utils.hpp"
#include "hector_pointcloud_processing/voxel_common.hpp"
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hector_pointcloud_processing
{

/*!
 * Voxel grid filter whose voxel size grows with distance from the cloud origin.
 *
 * The size schedule is given as distance bands: two equal-length lists `band_distances` (ascending
 * upper bounds, m) and `band_voxel_sizes` (edge length, m). Index i pairs band_distances[i] with
 * band_voxel_sizes[i]. A point at range r uses the voxel size of the first band whose distance is
 * >= r, so points near the robot are filtered with a small voxel and distant points with a large
 * one. Points farther than the last band's distance are dropped entirely.
 *
 * The distance and binning can be evaluated in a different frame (`target_frame`, via tf), e.g.
 * the robot body frame when the cloud is published in a sensor or odom frame; the published points
 * stay in the input frame. With `tf_prefix`, the prefix is prepended to the published frame id.
 *
 * As in VoxelFilter, the first input point landing in each voxel is kept and copied verbatim.
 */
class DistanceAdaptiveVoxelFilter : public hector::Node
{
public:
  DistanceAdaptiveVoxelFilter( const rclcpp::NodeOptions &options );

private:
  void setup();

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg );

  void publisherSubscriptionCallback();

  void startSubscribers();

  void stopSubscribers();

  void printNodeStatus() const;

  //! Distance-band voxel size schedule, precomputed as squared bounds (compared against squared
  //! distances) and inverse voxel sizes (to multiply). Rebuilt only when the band parameters
  //! change, so the per-cloud path does no allocation or division.
  struct BandSchedule {
    struct Band {
      double distance_sq;
      double inv_voxel_size;
    };

    std::vector<Band> bands;

    //! Recomputes from the raw band parameters. No-op (keeps the previous schedule) when the two
    //! lists differ in length, which can happen transiently while the two params are updated one
    //! after the other; the callback's length check guards clouds during that window.
    void rebuild( const std::vector<double> &distances, const std::vector<double> &voxel_sizes )
    {
      if ( distances.size() != voxel_sizes.size() )
        return;
      bands.resize( distances.size() );
      for ( size_t i = 0; i < distances.size(); ++i ) {
        bands[i].distance_sq = distances[i] * distances[i];
        bands[i].inv_voxel_size = 1.0 / voxel_sizes[i];
      }
    }

    //! Bands are ascending: picks the first band whose bound covers this distance, else the last.
    voxel_common::VoxelChoice choose( double dist_sq ) const
    {
      const size_t count = bands.size();
      for ( size_t i = 0; i < count; ++i ) {
        if ( dist_sq <= bands[i].distance_sq )
          return { bands[i].inv_voxel_size, static_cast<int32_t>( i ) };
      }
      return { bands[count - 1].inv_voxel_size, static_cast<int32_t>( count - 1 ) };
    }
  };

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  point_cloud_transport::Publisher pointcloud_publisher_;

  rclcpp::TimerBase::SharedPtr check_subscribers_timer_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;

  // Parameters
  std::string input_;
  std::string output_;
  std::vector<double> band_distances_;
  std::vector<double> band_voxel_sizes_;
  std::string target_frame_;
  std::string tf_prefix_;
  std::vector<std::string> keep_fields_;

  //! Cached band schedule; rebuilt from the band params on construction and whenever they change.
  BandSchedule band_schedule_;

  //! Voxel occupancy set, reused across clouds to keep its allocated capacity.
  std::unordered_set<voxel_common::VoxelKey, voxel_common::VoxelKeyHash> occupied_;

  bool has_subscribers_ = false;
};

} // namespace hector_pointcloud_processing

#endif // HECTOR_POINTCLOUD_PROCESSING_DISTANCE_ADAPTIVE_VOXEL_FILTER_HPP
