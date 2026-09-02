#ifndef HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_MERGER_HPP
#define HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_MERGER_HPP

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "hector_pointcloud_processing/_internal/hector_ros2_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hector_pointcloud_processing
{

/*!
 * Concatenates time-synchronised clouds from several rigidly mounted LiDARs into one cloud
 * expressed in a common frame, preserving the per-point layout.
 *
 * This exists for consumers that take a single cloud but can use the coverage of several sensors,
 * such as LiDAR-inertial odometry. Unlike the accumulator, nothing is voxelised or aggregated:
 * every surviving point keeps its original field values, which is what makes the result usable by
 * a consumer that deskews using per-point timestamps.
 *
 * With exactly one input topic there is nothing to synchronise or concatenate, so this degenerates
 * into a pure per-point reframe into target_frame -- useful for feeding a cloud expressed in one
 * frame to a consumer (e.g. a self-filter) that treats target_frame's origin as its own.
 *
 * Two details make the output correct rather than merely concatenated:
 *
 * - Points are transformed into target_frame using the static sensor extrinsics *before* the
 *   consumer deskews them. Because the sensors are rigidly attached, applying the fixed
 *   sensor-to-sensor transform first and the motion correction afterwards yields the same world
 *   point as correcting in the observing sensor's own frame.
 * - Per-point times are relative to their own cloud's header stamp, so a cloud that starts a
 *   fraction of a scan later than the reference would otherwise have its points misplaced in time.
 *   All times are rebased onto the merged header stamp (the earliest stamp in the group).
 *
 * Inputs must share an identical field layout (same names, offsets, datatypes and point_step);
 * this is checked once and the node refuses to merge mismatched clouds rather than emit a cloud
 * whose fields mean different things in different halves.
 *
 * Input topics are parameters (there is a variable number of them); the output topic is set by
 * remapping "points_merged", like the other nodes in this package.
 */
class PointcloudMerger : public hector::Node
{
public:
  explicit PointcloudMerger( const rclcpp::NodeOptions &options );

private:
  //! One buffered cloud per input, oldest first.
  using CloudPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  void pointcloudCallback( size_t input_index, const CloudPtr &msg );

  /*!
   * Emits every group that can be completed, oldest first. A group is the oldest unmatched cloud
   * of input 0 plus the closest cloud of every other input within sync_tolerance.
   */
  void processBuffers();

  /*!
   * Returns the index of the cloud in \p buffer closest in time to \p stamp, or nullopt if none
   * lies within sync_tolerance.
   */
  std::optional<size_t> findMatch( const std::deque<CloudPtr> &buffer, double stamp ) const;

  //! Concatenates \p group into a single cloud. Returns nullptr if it could not be built.
  sensor_msgs::msg::PointCloud2::UniquePtr merge( const std::vector<CloudPtr> &group );

  /*!
   * Verifies that \p msg has the layout the merger requires and matches the layout seen first.
   * Logs once per problem and returns false if the cloud must not be merged.
   */
  bool validateLayout( const sensor_msgs::msg::PointCloud2 &msg );

  //! Looks up (and caches, if the transforms are static) target_frame <- \p source_frame.
  std::optional<Eigen::Isometry3f> lookupTransform( const std::string &source_frame,
                                                    const rclcpp::Time &stamp );

  void printNodeStatus() const;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
  std::vector<std::deque<CloudPtr>> buffers_;
  std::mutex mutex_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unordered_map<std::string, Eigen::Isometry3f> transform_cache_;

  //! Field layout of the first accepted cloud; all later clouds must match it.
  std::optional<std::vector<sensor_msgs::msg::PointField>> reference_fields_;
  uint32_t reference_point_step_ = 0;
  uint32_t x_offset_ = 0;
  std::optional<uint32_t> time_offset_;
  std::optional<uint32_t> line_offset_;

  // Parameters
  std::vector<std::string> input_topics_;
  std::string target_frame_;
  std::string time_field_;
  std::string line_field_;
  double sync_tolerance_ = 0.02;
  double min_range_ = 0.1;
  int64_t line_offset_step_ = 0;
  int64_t buffer_size_ = 10;
  bool cache_transforms_ = true;
};

} // namespace hector_pointcloud_processing

#endif // HECTOR_POINTCLOUD_PROCESSING_POINTCLOUD_MERGER_HPP
