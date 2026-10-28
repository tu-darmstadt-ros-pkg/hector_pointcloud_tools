#ifndef HECTOR_POINTCLOUD_PROCESSING_DISTANCE_ADAPTIVE_POINTCLOUD_DECIMATOR_HPP
#define HECTOR_POINTCLOUD_PROCESSING_DISTANCE_ADAPTIVE_POINTCLOUD_DECIMATOR_HPP

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "hector_pointcloud_processing/_internal/hector_ros2_utils.hpp"
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace hector_pointcloud_processing
{

/*!
 * Randomly decimates a cloud with a keep-probability that varies with each point's distance from
 * the origin.
 *
 * The probability is defined by two equal-length lists: `distances` (ascending ranges, m) and
 * `percentages` (keep fraction in [0, 1]). Index i pairs distances[i] with percentages[i]. For a point at
 * range r the keep probability is the linear interpolation of the surrounding control points; beyond
 * the first/last distance it is clamped to the first/last percentage (no extrapolation). Each point is
 * then kept by an independent uniform random draw, so the output size only matches the requested
 * fractions in expectation. Only `distances` has to be ascending; `percentages` may rise as well as
 * fall, so distant points can also be kept with a higher probability than close ones.
 *
 * The distance can be evaluated in a different frame (`target_frame`, via tf), e.g. the robot body
 * frame when the cloud is published in a sensor or odom frame; the published points stay in the
 * input frame. With `tf_prefix`, the prefix is prepended to the published frame id.
 */
class DistanceAdaptivePointcloudDecimator : public hector::Node
{
public:
  DistanceAdaptivePointcloudDecimator( const rclcpp::NodeOptions &options );

private:
  void setup();

  void pointcloudCallback( const sensor_msgs::msg::PointCloud2 &msg );

  void publisherSubscriptionCallback();

  void startSubscribers();

  void stopSubscribers();

  void printNodeStatus() const;

  //! Distance-to-keep-probability schedule. Read from the two parameter lists at startup and
  //! validated to be of equal length, so the per-point lookup needs no further checks.
  struct ProbabilitySchedule {
    std::vector<double> distances;   //!< Ascending control-point ranges (m).
    std::vector<double> percentages; //!< Keep fraction [0, 1] paired by index with distances.

    //! Linear interpolation of the keep probability at the given distance, clamped to the
    //! first/last percentage outside the control-point range.
    double keepProbability( double distance ) const
    {
      if ( distance <= distances.front() )
        return percentages.front();
      if ( distance >= distances.back() )
        return percentages.back();
      const auto upper = std::upper_bound( distances.begin(), distances.end(), distance );
      const size_t hi = static_cast<size_t>( upper - distances.begin() );
      const size_t lo = hi - 1;
      const double t = ( distance - distances[lo] ) / ( distances[hi] - distances[lo] );
      return percentages[lo] + t * ( percentages[hi] - percentages[lo] );
    }
  };

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
  point_cloud_transport::Publisher pointcloud_publisher_;

  rclcpp::TimerBase::SharedPtr check_subscribers_timer_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface_;

  std::mt19937 random_generator_{ std::random_device{}() };

  // Parameters
  std::string input_;
  std::string output_;
  //! Holds the distances and percentages parameters.
  ProbabilitySchedule schedule_;
  std::string target_frame_;
  std::string tf_prefix_;

  bool has_subscribers_ = false;
};

} // namespace hector_pointcloud_processing

#endif // HECTOR_POINTCLOUD_PROCESSING_DISTANCE_ADAPTIVE_POINTCLOUD_DECIMATOR_HPP
