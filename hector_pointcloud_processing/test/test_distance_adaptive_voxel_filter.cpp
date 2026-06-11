// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_processing/distance_adaptive_voxel_filter.hpp"

#include <array>
#include <chrono>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace std::chrono_literals;
using hector_pointcloud_processing::DistanceAdaptiveVoxelFilter;

namespace
{

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

// Drives one input cloud through the filter and returns the output, or nullptr on timeout.
// With publish_base_tf, a static transform "map" -> "base" at x=5 is broadcast for
// target_frame tests.
sensor_msgs::msg::PointCloud2::SharedPtr runFilter( const rclcpp::NodeOptions &options,
                                                    const sensor_msgs::msg::PointCloud2 &input,
                                                    bool publish_base_tf = false )
{
  auto filter = std::make_shared<DistanceAdaptiveVoxelFilter>( options );
  auto helper = rclcpp::Node::make_shared( "test_adaptive_voxel_filter_helper" );

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster;
  if ( publish_base_tf ) {
    tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>( helper );
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = helper->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "base";
    tf.transform.translation.x = 5.0;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster->sendTransform( tf );
  }

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

// Bands: range <= 2 m uses a 0.1 m voxel, range <= 30 m uses a 1.0 m voxel. The last band (30 m)
// is also the effective max distance, beyond which points are dropped.
rclcpp::NodeOptions defaultBands()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "band_distances", std::vector<double>{ 2.0, 30.0 } ),
        rclcpp::Parameter( "band_voxel_sizes", std::vector<double>{ 0.1, 1.0 } ) } );
  return options;
}

TEST( DistanceAdaptiveVoxelFilterTest, NearPointsKeptInFineVoxels )
{
  // 0.1 m apart near the origin: with the 0.1 m near voxel they land in voxels 0 and 1.
  auto output = runFilter( defaultBands(), makeCloud( { { 0.05f, 0, 0 }, { 0.15f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 2u );
}

TEST( DistanceAdaptiveVoxelFilterTest, FarPointsMergedInCoarseVoxels )
{
  // Same 0.5 m separation but at ~5 m range, where the 1.0 m voxel merges them into one.
  auto output = runFilter( defaultBands(), makeCloud( { { 5.0f, 0, 0 }, { 5.5f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 1u );
}

TEST( DistanceAdaptiveVoxelFilterTest, DropsPointsBeyondLastBand )
{
  // The last band ends at 30 m, so the 35 m point is dropped and only the near point survives.
  auto output = runFilter( defaultBands(), makeCloud( { { 1.0f, 0, 0 }, { 35.0f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 1u );
}

TEST( DistanceAdaptiveVoxelFilterTest, DifferentBandsDoNotAlias )
{
  // x=1.0 (near band, 0.1 m voxel -> index 10) and x=10.0 (far band, 1.0 m voxel -> index 10)
  // share the integer voxel index but live in different bands, so both must be kept.
  auto output = runFilter( defaultBands(), makeCloud( { { 1.0f, 0, 0 }, { 10.0f, 0, 0 } } ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 2u );
}

TEST( DistanceAdaptiveVoxelFilterTest, BinsInTargetFrame )
{
  // In the cloud frame both points sit ~5 m out and would merge in the 1 m far voxel. The
  // "base" frame sits at x=5, so relative to it they are 0.25 and 0.85 m away and land in
  // distinct 0.1 m near voxels.
  const auto cloud = makeCloud( { { 5.25f, 0, 0 }, { 5.85f, 0, 0 } } );

  auto merged = runFilter( defaultBands(), cloud );
  ASSERT_NE( merged, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( merged->width, 1u );

  rclcpp::NodeOptions options = defaultBands();
  options.append_parameter_override( "target_frame", "base" );
  auto output = runFilter( options, cloud, /*publish_base_tf=*/true );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->width, 2u );
}

TEST( DistanceAdaptiveVoxelFilterTest, StripsNonWhitelistedFields )
{
  rclcpp::NodeOptions options = defaultBands();
  options.parameter_overrides(
      { rclcpp::Parameter( "band_distances", std::vector<double>{ 2.0, 30.0 } ),
        rclcpp::Parameter( "band_voxel_sizes", std::vector<double>{ 0.1, 1.0 } ),
        rclcpp::Parameter( "keep_fields", std::vector<std::string>{ "x", "y", "z" } ) } );

  auto output = runFilter( options, makeCloud( { { 5.0f, 0, 0 } }, /*with_intensity=*/true ) );
  ASSERT_NE( output, nullptr ) << "Did not receive filtered cloud within timeout";
  EXPECT_EQ( output->fields.size(), 3u );
  EXPECT_EQ( output->point_step, 3u * sizeof( float ) );
  for ( const auto &field : output->fields ) EXPECT_NE( field.name, "intensity" );
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
