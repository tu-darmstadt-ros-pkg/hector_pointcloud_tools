// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_processing/distance_adaptive_pointcloud_decimator.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;
using hector_pointcloud_processing::DistanceAdaptivePointcloudDecimator;

namespace
{

// Builds a width-N, height-1 FLOAT32 xyz cloud where point i is (i, 0, 0), so the x coordinate
// both identifies the source point and equals its distance from the origin.
sensor_msgs::msg::PointCloud2 makeIndexedCloud( size_t n )
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "map";
  msg.height = 1;
  msg.width = n;
  msg.is_bigendian = false;
  msg.is_dense = true;
  const char *names[] = { "x", "y", "z" };
  msg.fields.resize( 3 );
  for ( size_t i = 0; i < 3; ++i ) {
    msg.fields[i].name = names[i];
    msg.fields[i].offset = static_cast<uint32_t>( i * sizeof( float ) );
    msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[i].count = 1;
  }
  msg.point_step = 3 * sizeof( float );
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize( static_cast<size_t>( msg.row_step ) * msg.height );
  for ( size_t p = 0; p < n; ++p ) {
    auto *out = reinterpret_cast<float *>( msg.data.data() + p * msg.point_step );
    out[0] = static_cast<float>( p );
    out[1] = 0.0f;
    out[2] = 0.0f;
  }
  return msg;
}

std::vector<float> xValues( const sensor_msgs::msg::PointCloud2 &cloud )
{
  std::vector<float> xs;
  for ( size_t p = 0; p < cloud.width; ++p ) {
    xs.push_back( *reinterpret_cast<const float *>( cloud.data.data() + p * cloud.point_step ) );
  }
  return xs;
}

// Drives one input cloud through the decimator and returns the decimated output, or nullptr on
// timeout. Handles the lazy-subscriber logic: the decimator only subscribes to its input once its
// output has a subscriber, so we keep publishing while spinning until output arrives.
sensor_msgs::msg::PointCloud2::SharedPtr runDecimator( const rclcpp::NodeOptions &options,
                                                       const sensor_msgs::msg::PointCloud2 &input )
{
  auto decimator = std::make_shared<DistanceAdaptivePointcloudDecimator>( options );
  auto helper = rclcpp::Node::make_shared( "test_adaptive_decimator_helper" );

  sensor_msgs::msg::PointCloud2::SharedPtr received;
  auto sub = helper->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_decimated", rclcpp::QoS( 10 ).best_effort(),
      [&received]( sensor_msgs::msg::PointCloud2::SharedPtr msg ) { received = msg; } );
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>( "pointcloud", 10 );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node( decimator );
  executor.add_node( helper );

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while ( !received && std::chrono::steady_clock::now() < deadline ) {
    pub->publish( input );
    executor.spin_some();
    std::this_thread::sleep_for( 20ms );
  }
  return received;
}

TEST( AdaptiveDecimatorTest, percentageOneKeepsEveryPoint )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "distances", std::vector<double>{ 1.0, 20.0 } ),
        rclcpp::Parameter( "percentages", std::vector<double>{ 1.0, 1.0 } ) } );

  auto output = runDecimator( options, makeIndexedCloud( 15 ) );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";
  EXPECT_EQ( output->width, 15u );
}

TEST( AdaptiveDecimatorTest, EndpointClampKeepsNearDropsFar )
{
  // percentages [1, 0] over distances [5, 10]: points at r <= 5 always kept (clamped to 1), points
  // at r >= 10 always dropped (clamped to 0). Points in (5, 10) are random and not asserted.
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "distances", std::vector<double>{ 5.0, 10.0 } ),
        rclcpp::Parameter( "percentages", std::vector<double>{ 1.0, 0.0 } ) } );

  auto output = runDecimator( options, makeIndexedCloud( 15 ) );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";

  const std::vector<float> xs = xValues( *output );
  // Every near point (x <= 5) survives.
  for ( float expected = 0.0f; expected <= 5.0f; expected += 1.0f )
    EXPECT_NE( std::find( xs.begin(), xs.end(), expected ), xs.end() )
        << "Near point x=" << expected << " should be kept";
  // No far point (x >= 10) survives.
  for ( float x : xs ) EXPECT_LT( x, 10.0f ) << "Far point x=" << x << " should be dropped";
}

TEST( AdaptiveDecimatorTest, IncreasingPercentagesKeepFarDropsNear )
{
  // Percentages may increase with distance: [0, 1] over distances [5, 10] inverts the schedule, so
  // points at r <= 5 are always dropped (clamped to 0) and points at r >= 10 always kept (clamped
  // to 1). Points in (5, 10) are random and not asserted.
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "distances", std::vector<double>{ 5.0, 10.0 } ),
        rclcpp::Parameter( "percentages", std::vector<double>{ 0.0, 1.0 } ) } );

  auto output = runDecimator( options, makeIndexedCloud( 15 ) );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";

  const std::vector<float> xs = xValues( *output );
  // Every far point (x >= 10) survives.
  for ( float expected = 10.0f; expected < 15.0f; expected += 1.0f )
    EXPECT_NE( std::find( xs.begin(), xs.end(), expected ), xs.end() )
        << "Far point x=" << expected << " should be kept";
  // No near point (x <= 5) survives.
  for ( float x : xs ) EXPECT_GT( x, 5.0f ) << "Near point x=" << x << " should be dropped";
}

TEST( AdaptiveDecimatorTest, OutputPreservesFieldLayout )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      { rclcpp::Parameter( "distances", std::vector<double>{ 1.0, 20.0 } ),
        rclcpp::Parameter( "percentages", std::vector<double>{ 1.0, 1.0 } ) } );

  const auto input = makeIndexedCloud( 10 );
  auto output = runDecimator( options, input );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";
  EXPECT_EQ( output->point_step, input.point_step );
  EXPECT_EQ( output->fields.size(), input.fields.size() );
  EXPECT_EQ( output->row_step, output->point_step * output->width );
}

// Overrides bypass the parameter validators, which rclcpp only registers after the declaration,
// so an invalid schedule has to be rejected by the constructor.
TEST( AdaptiveDecimatorTest, RejectsInvalidStartupParameters )
{
  const auto make = []( const std::vector<double> &distances,
                        const std::vector<double> &percentages ) {
    rclcpp::NodeOptions options;
    options.parameter_overrides( { rclcpp::Parameter( "distances", distances ),
                                   rclcpp::Parameter( "percentages", percentages ) } );
    return std::make_shared<DistanceAdaptivePointcloudDecimator>( options );
  };

  using rclcpp::exceptions::InvalidParametersException;
  EXPECT_THROW( make( { 10.0, 1.0 }, { 1.0, 0.5 } ), InvalidParametersException )
      << "Distances must be strictly increasing";
  EXPECT_THROW( make( { 1.0, 10.0 }, { 1.0, 1.5 } ), InvalidParametersException )
      << "Percentages must be in [0, 1]";
  EXPECT_THROW( make( { 1.0, 10.0, 20.0 }, { 1.0, 0.5 } ), InvalidParametersException )
      << "Lists must have the same length";
  EXPECT_NO_THROW( make( { 1.0, 10.0 }, { 1.0, 0.5 } ) );
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
