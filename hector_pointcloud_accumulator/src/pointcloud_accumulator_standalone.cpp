// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "./nodes/pointcloud_accumulator_node.hpp"

using namespace hector_pointcloud_accumulator;

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );

  auto node = std::make_shared<PointcloudAccumulatorNode>();
  rclcpp::spin( node );
  rclcpp::shutdown();
  return 0;
}