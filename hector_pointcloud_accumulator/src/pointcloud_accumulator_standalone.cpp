//
// Created by stefan on 13.02.25.
//
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