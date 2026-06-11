// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_processing/pointcloud_decimator.hpp"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;
using hector_pointcloud_processing::PointcloudDecimator;

namespace
{

// Builds a width-N, height-1 FLOAT32 xyz cloud where point i is (i, 0, 0), so the
// x coordinate identifies which source point ended up in the decimated output.
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

// Drives one input cloud through the decimator and returns the decimated output, or nullptr
// on timeout. Handles the lazy-subscriber logic: the decimator only subscribes to its input
// once its output has a subscriber, so we keep publishing while spinning until output arrives.
sensor_msgs::msg::PointCloud2::SharedPtr runDecimator( const rclcpp::NodeOptions &options,
                                                       const sensor_msgs::msg::PointCloud2 &input )
{
  auto decimator = std::make_shared<PointcloudDecimator>( options );
  auto helper = rclcpp::Node::make_shared( "test_decimator_helper" );

  sensor_msgs::msg::PointCloud2::SharedPtr received;
  // The point_cloud_transport raw publisher advertises with BEST_EFFORT reliability, so the
  // subscription must match it - otherwise it is never counted and the lazy subscriber that
  // gates the decimator's input never starts.
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

TEST( DecimatorTest, FractionQuantifierSelectsEvenlySpacedPoints )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides( { rclcpp::Parameter( "elimination_method", "count" ),
                                 rclcpp::Parameter( "elimination_quantifier", "fraction" ),
                                 rclcpp::Parameter( "point_fraction", 0.5 ) } );

  auto output = runDecimator( options, makeIndexedCloud( 10 ) );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";
  EXPECT_EQ( output->width, 5u );
  EXPECT_EQ( xValues( *output ), ( std::vector<float>{ 0, 2, 4, 6, 8 } ) );
}

TEST( DecimatorTest, CountQuantifierKeepsRequestedNumberOfPoints )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides( { rclcpp::Parameter( "elimination_method", "count" ),
                                 rclcpp::Parameter( "elimination_quantifier", "count" ),
                                 rclcpp::Parameter( "point_count", 5 ) } );

  auto output = runDecimator( options, makeIndexedCloud( 10 ) );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";
  EXPECT_EQ( output->width, 5u );
  EXPECT_EQ( xValues( *output ), ( std::vector<float>{ 0, 2, 4, 6, 8 } ) );
}

TEST( DecimatorTest, OutputPreservesFieldLayout )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides( { rclcpp::Parameter( "elimination_quantifier", "fraction" ),
                                 rclcpp::Parameter( "point_fraction", 0.5 ) } );

  const auto input = makeIndexedCloud( 10 );
  auto output = runDecimator( options, input );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";
  EXPECT_EQ( output->point_step, input.point_step );
  EXPECT_EQ( output->fields.size(), input.fields.size() );
  EXPECT_EQ( output->row_step, output->point_step * output->width );
}

TEST( DecimatorTest, RandomMethodKeepsValidSubset )
{
  rclcpp::NodeOptions options;
  options.parameter_overrides( { rclcpp::Parameter( "elimination_method", "random" ),
                                 rclcpp::Parameter( "elimination_quantifier", "fraction" ),
                                 rclcpp::Parameter( "point_fraction", 1.0 ) } );

  auto output = runDecimator( options, makeIndexedCloud( 10 ) );
  ASSERT_NE( output, nullptr ) << "Did not receive decimated cloud within timeout";
  // fraction 1.0 keeps every point.
  EXPECT_EQ( output->width, 10u );
  for ( float x : xValues( *output ) ) {
    EXPECT_GE( x, 0.0f );
    EXPECT_LT( x, 10.0f );
  }
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
