// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef POINTCLOUD_ACCUMULATOR_NODE_HPP
#define POINTCLOUD_ACCUMULATOR_NODE_HPP

#include "hector_pointcloud_processing/pointcloud_accumulator.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hector_pointcloud_processing
{
class PointcloudAccumulatorNode : public rclcpp::Node
{
public:
  PointcloudAccumulatorNode( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );

private:
  std::shared_ptr<hector_pointcloud_processing::PointcloudAccumulatorBase> accumulator_;
};
} // namespace hector_pointcloud_processing

#endif // POINTCLOUD_ACCUMULATOR_NODE_HPP
