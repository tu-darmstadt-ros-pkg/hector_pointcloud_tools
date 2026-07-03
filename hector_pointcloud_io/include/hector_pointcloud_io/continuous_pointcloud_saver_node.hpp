// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POINTCLOUD_IO_CONTINUOUS_POINTCLOUD_SAVER_NODE_HPP
#define HECTOR_POINTCLOUD_IO_CONTINUOUS_POINTCLOUD_SAVER_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include "hector_pointcloud_io/_internal/hector_ros2_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hector_pointcloud_io
{

class ContinuousPointcloudSaver : public hector::Node
{

public:
  explicit ContinuousPointcloudSaver( const rclcpp::NodeOptions &options );

  void cloudCallback( const sensor_msgs::msg::PointCloud2::ConstPtr &pointcloud );

private:
  void serviceCallback( const std_srvs::srv::Trigger::Request::SharedPtr &request,
                        const std_srvs::srv::Trigger::Response::SharedPtr &response );

  std::vector<std::tuple<std::string, std::function<void( const rclcpp::Parameter & )>>>
      auto_reconfigurable_params_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::string topic_ = "pointcloud";
  std::string output_folder_;
  std::string output_format_ = "pcd";
  std::string output_filename_prefix_ = "pointcloud";
  double save_interval_ = 10.0;
  rclcpp::Time last_cloud_stamp_;
  bool save_next_ = false;
};

} // namespace hector_pointcloud_io

#endif // HECTOR_POINTCLOUD_IO_CONTINUOUS_POINTCLOUD_SAVER_NODE_HPP
