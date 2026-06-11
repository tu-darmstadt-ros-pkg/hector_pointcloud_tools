// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_processing/voxel_filter.hpp"

#include <array>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using namespace std::chrono_literals;
using hector_pointcloud_processing::VoxelFilter;

namespace
{

// Builds a width-N, height-1 FLOAT32 cloud from the given xyz points. With intensity, a
// fourth FLOAT32 field is appended (set to the point index) to exercise field stripping.
sensor_msgs::msg::PointCloud2 makeCloud( const std::vector<std::array<float, 3>> &points,
                                         bool with_intensity = false )
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "map";
  msg.height = 1;
  msg.width = points.size();
  msg.is_bigendian = false;
  msg.is_dense = true;

  std::vector<std::string> names = { "x", "y", "z" };
  if ( with_intensity )
    names.push_back( "intensity" );
  msg.fields.resize( names.size() );
  for ( size_t i = 0; i < names.size(); ++i ) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<uint32_t>( i * sizeof( float ) );
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }
  msg.point_step = static_cast<uint32_t>( names.size() * sizeof( float ) );
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize( static_cast<size_t>( msg.row_step ) * msg.height );
  for ( size_t p = 0; p < points.size(); ++p ) {
    auto *out = reinterpret_cast<float *>( msg.data.data() + p * msg.point_step );
    out[0] = points[p][0];
    out[1] = points[p][1];
    out[2] = points[p][2];
    if ( with_intensity )
      out[3] = static_cast<float>( p );
  }
  return msg;
}

std::vector<float> xValues( const sensor_msgs::msg::PointCloud2 &cloud )
{
  std::vector<float> xs;
  for ( size_t p = 0; p < cloud.width; ++p )
    xs.push_back( *reinterpret_cast<const float *>( cloud.data.data() + p * cloud.point_step ) );
  return xs;
}

// Drives one input cloud through the filter and returns the output, or nullptr on timeout.
sensor_msgs::msg::PointCloud2::SharedPtr runFilter( const rclcpp::NodeOptions &options,
                                                    const sensor_msgs::msg::PointCloud2 &input )
{
  auto filter = std::make_shared<VoxelFilter>( options );
  auto helper = rclcpp::Node::make_shared( "test_voxel_filter_helper" );

  sensor_msgs::msg::PointCloud2::SharedPtr received;
  auto sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pointcloud_filtered", rclcpp::QoS( 10 ).best_effort(),
      [&received]( sensor_msgs::msg::PointCloud2::SharedPtr msg ) { received = msg; } );
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>( "/pointcloud", 10 );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node( filter );
  executor.add_node( helper );

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while ( !received && std::chrono::steady_clock::now() < deadline ) {
    pub->publish( input );
    executor.spin_some();
    std::this_thread::sleep_for( 20ms );
  }
  return received;
}

TEST( VoxelFilterTest, KeepsFirstPointPerVoxel )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "voxel_size", 0.1 ), rclcpp::Parameter( "max_distance", 30.0 ) } );

  // Both points fall in the same 0.1 m voxel on x, so only the first survives.
  auto output = runFilter( options, makeCloud( { { 0.01f, 0, 0 }, { 0.05f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 1u );
  EXPECT_FLOAT_EQ( xValues( *output ).front(), 0.01f );
}

TEST( VoxelFilterTest, KeepsPointsInDifferentVoxels )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "voxel_size", 0.1 ), rclcpp::Parameter( "max_distance", 30.0 ) } );

  // x=0.01 -> voxel 0, x=0.5 -> voxel 5: different voxels, both kept.
  auto output = runFilter( options, makeCloud( { { 0.01f, 0, 0 }, { 0.5f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 2u );
}

TEST( VoxelFilterTest, DropsPointsBeyondMaxDistance )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "voxel_size", 0.1 ), rclcpp::Parameter( "max_distance", 1.0 ) } );

  auto output = runFilter( options, makeCloud( { { 0.5f, 0, 0 }, { 2.0f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 1u );
  EXPECT_FLOAT_EQ( xValues( *output ).front(), 0.5f );
}

TEST( VoxelFilterTest, StripsNonWhitelistedFields )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "voxel_size", 0.1 ), rclcpp::Parameter( "max_distance", 30.0 ),
        rclcpp::Parameter( "keep_fields", std::vector<std::string>{ "x", "y", "z" } ) } );

  auto output = runFilter( options, makeCloud( { { 5.0f, 0, 0 } }, /*with_intensity=*/true ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->fields.size(), 3u );
  EXPECT_EQ( output->point_step, 3u * sizeof( float ) );
  for ( const auto &field : output->fields ) EXPECT_NE( field.name, "intensity" );
}

TEST( VoxelFilterTest, IgnoresBlankKeepFieldNames )
{
  // Launch files pass [""] to express "keep all fields" since rcl cannot represent an empty
  // string array override; blank names must not be treated as a (non-matching) whitelist.
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "voxel_size", 0.1 ), rclcpp::Parameter( "max_distance", 30.0 ),
        rclcpp::Parameter( "keep_fields", std::vector<std::string>{ "" } ) } );

  auto output = runFilter( options, makeCloud( { { 5.0f, 0, 0 } }, /*with_intensity=*/true ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->fields.size(), 4u );
}

} // namespace

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
