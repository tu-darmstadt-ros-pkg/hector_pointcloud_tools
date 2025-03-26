// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef POINTCLOUD_ACCUMULATOR_NODE_HPP
#define POINTCLOUD_ACCUMULATOR_NODE_HPP

#include "hector_pointcloud_accumulator/pointcloud_accumulator.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hector_pointcloud_accumulator
{
class PointcloudAccumulatorNode : public rclcpp::Node
{
public:
  PointcloudAccumulatorNode( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );

private:
  std::shared_ptr<hector_pointcloud_accumulator::PointcloudAccumulatorBase> hector_pointcloud_accumulator_;
};
} // namespace hector_pointcloud_accumulator

#endif // POINTCLOUD_ACCUMULATOR_NODE_HPP
