// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POINTCLOUD_IO_POINTCLOUD_SAVER_NODE_HPP
#define HECTOR_POINTCLOUD_IO_POINTCLOUD_SAVER_NODE_HPP

#include <memory>
#include <string>
#include <vector>

#include <hector_ros2_utils/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace hector_pointcloud_io
{

class PointcloudSaver : public hector::Node
{

public:
  explicit PointcloudSaver( const rclcpp::NodeOptions &options );

private:
  void serviceCallback( const std_srvs::srv::Trigger::Request::SharedPtr &request,
                        const std_srvs::srv::Trigger::Response::SharedPtr &response );

  std::vector<std::tuple<std::string, std::function<void( const rclcpp::Parameter & )>>>
      auto_reconfigurable_params_;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server_;

  std::string topic_ = "pointcloud";
  std::string output_folder_;
  std::string output_format_ = "pcd";
  std::string output_filename_prefix_ = "pointcloud";
  int timeout_ms_ = 10000;
};

} // namespace hector_pointcloud_io

#endif // HECTOR_POINTCLOUD_IO_POINTCLOUD_SAVER_NODE_HPP
