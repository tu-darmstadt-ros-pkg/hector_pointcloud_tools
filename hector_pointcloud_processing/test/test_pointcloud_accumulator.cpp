// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_processing/pointcloud_accumulator.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using hector_pointcloud_processing::AggregationMode;
using hector_pointcloud_processing::PointcloudAccumulator;

namespace
{

// Exposes the protected accumulated cloud so tests can inspect the result of
// processPointcloud() directly without spinning the node / publish timer.
template<AggregationMode MODE>
class TestableAccumulator : public PointcloudAccumulator<MODE>
{
public:
  using PointcloudAccumulator<MODE>::PointcloudAccumulator;

  const sensor_msgs::msg::PointCloud2 &accumulatedCloud() const { return this->accumulated_cloud_; }
};

struct Field {
  std::string name;
  float value;
};

// Builds a width-N, height-1 FLOAT32 cloud with x, y, z followed by the given
// extra per-point fields. The same extra-field values are written to every point.
sensor_msgs::msg::PointCloud2::SharedPtr makeCloud( const std::vector<Eigen::Vector3f> &points,
                                                    const std::vector<Field> &extra_fields = {},
                                                    const std::string &frame = "map" )
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = frame;
  msg->height = 1;
  msg->width = points.size();
  msg->is_bigendian = false;
  msg->is_dense = true;

  const std::vector<std::string> names = [&] {
    std::vector<std::string> n = { "x", "y", "z" };
    for ( const auto &f : extra_fields ) n.push_back( f.name );
    return n;
  }();
  msg->fields.resize( names.size() );
  for ( size_t i = 0; i < names.size(); ++i ) {
    msg->fields[i].name = names[i];
    msg->fields[i].offset = static_cast<uint32_t>( i * sizeof( float ) );
    msg->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg->fields[i].count = 1;
  }
  msg->point_step = static_cast<uint32_t>( names.size() * sizeof( float ) );
  msg->row_step = msg->point_step * msg->width;
  msg->data.resize( static_cast<size_t>( msg->row_step ) * msg->height );

  for ( size_t p = 0; p < points.size(); ++p ) {
    auto *out = reinterpret_cast<float *>( msg->data.data() + p * msg->point_step );
    out[0] = points[p].x();
    out[1] = points[p].y();
    out[2] = points[p].z();
    for ( size_t f = 0; f < extra_fields.size(); ++f ) out[3 + f] = extra_fields[f].value;
  }
  return msg;
}

// Reads point i (x, y, z) from an accumulated cloud whose first three fields are FLOAT32 x, y, z.
Eigen::Vector3f readPoint( const sensor_msgs::msg::PointCloud2 &cloud, size_t i )
{
  const auto *data = reinterpret_cast<const float *>( cloud.data.data() + i * cloud.point_step );
  return { data[0], data[1], data[2] };
}

template<AggregationMode MODE>
std::shared_ptr<TestableAccumulator<MODE>> makeAccumulator( rclcpp::Node &node, double resolution )
{
  return std::make_shared<TestableAccumulator<MODE>>(
      node, resolution, "map", rclcpp::Rate( 1.0 ), std::vector<std::string>{}, /*queue_size=*/10 );
}

class AccumulatorTest : public ::testing::Test
{
protected:
  void SetUp() override { node_ = rclcpp::Node::make_shared( "test_accumulator_node" ); }

  rclcpp::Node::SharedPtr node_;
};

constexpr float kEps = 1e-4f;

TEST_F( AccumulatorTest, SinglePointStored )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 1.0f, 2.0f, 3.0f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE( readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 1, 2, 3 ), kEps ) );
}

TEST_F( AccumulatorTest, PointsInDifferentVoxelsKeptSeparately )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 0, 0, 0 }, { 5, 0, 0 } } ) );

  EXPECT_EQ( acc->accumulatedCloud().width, 2u );
}

TEST_F( AccumulatorTest, AverageMergesPointsInSameVoxel )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  // Both points round to voxel index (0, 0, 0).
  acc->processPointcloud( makeCloud( { { 0.1f, 0.0f, 0.0f }, { 0.3f, 0.0f, 0.0f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_NEAR( readPoint( acc->accumulatedCloud(), 0 ).x(), 0.2f, kEps );
}

TEST_F( AccumulatorTest, AverageAccrossMultipleClouds )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 0.0f, 0.0f, 0.0f } } ) );
  acc->processPointcloud( makeCloud( { { 0.2f, 0.0f, 0.0f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_NEAR( readPoint( acc->accumulatedCloud(), 0 ).x(), 0.1f, kEps );
}

TEST_F( AccumulatorTest, HighestZKeepsTopmostPoint )
{
  auto acc = makeAccumulator<AggregationMode::HIGHEST_Z>( *node_, 1.0 );
  // Both round to voxel (0, 0, 0); the second has the higher z.
  acc->processPointcloud( makeCloud( { { 0.1f, 0.0f, 0.1f }, { 0.2f, 0.0f, 0.3f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE(
      readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 0.2f, 0.0f, 0.3f ), kEps ) );
}

TEST_F( AccumulatorTest, HighestZIgnoresLowerPoint )
{
  auto acc = makeAccumulator<AggregationMode::HIGHEST_Z>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 0.2f, 0.0f, 0.3f }, { 0.1f, 0.0f, 0.1f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE(
      readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 0.2f, 0.0f, 0.3f ), kEps ) );
}

TEST_F( AccumulatorTest, ClosestToCenterKeepsNearestPoint )
{
  auto acc = makeAccumulator<AggregationMode::CLOSEST_TO_CENTER>( *node_, 1.0 );
  // Voxel (0, 0, 0) center is (0.5, 0.5, 0.5); the second point is closer to it.
  acc->processPointcloud( makeCloud( { { 0.1f, 0.1f, 0.1f }, { 0.4f, 0.4f, 0.4f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE(
      readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 0.4f, 0.4f, 0.4f ), kEps ) );
}

TEST_F( AccumulatorTest, ResolutionControlsVoxelSize )
{
  // With a coarse resolution the two points fall into the same voxel (both round to
  // index 0 at resolution 10) and merge.
  auto coarse = makeAccumulator<AggregationMode::AVERAGE>( *node_, 10.0 );
  coarse->processPointcloud( makeCloud( { { 0, 0, 0 }, { 4, 0, 0 } } ) );
  EXPECT_EQ( coarse->accumulatedCloud().width, 1u );

  // With a fine resolution they stay separate.
  auto fine = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  fine->processPointcloud( makeCloud( { { 0, 0, 0 }, { 5, 0, 0 } } ) );
  EXPECT_EQ( fine->accumulatedCloud().width, 2u );
}

TEST_F( AccumulatorTest, ExtraFieldsArePreserved )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 1.0f, 0.0f, 0.0f } }, { { "intensity", 42.0f } } ) );

  const auto &cloud = acc->accumulatedCloud();
  ASSERT_EQ( cloud.width, 1u );
  auto it = std::find_if( cloud.fields.begin(), cloud.fields.end(),
                          []( const auto &f ) { return f.name == "intensity"; } );
  ASSERT_NE( it, cloud.fields.end() );
  const float value = *reinterpret_cast<const float *>( cloud.data.data() + it->offset );
  EXPECT_NEAR( value, 42.0f, kEps );
}

TEST_F( AccumulatorTest, ResetClearsAccumulatedCloud )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 0, 0, 0 }, { 5, 0, 0 } } ) );
  ASSERT_EQ( acc->accumulatedCloud().width, 2u );

  acc->reset();
  EXPECT_EQ( acc->accumulatedCloud().width, 0u );
  EXPECT_TRUE( acc->accumulatedCloud().data.empty() );

  // After a reset accumulation starts over cleanly.
  acc->processPointcloud( makeCloud( { { 0, 0, 0 } } ) );
  EXPECT_EQ( acc->accumulatedCloud().width, 1u );
}

TEST_F( AccumulatorTest, DisabledAccumulatorIgnoresPointclouds )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->setEnabled( false );
  ASSERT_FALSE( acc->isEnabled() );
  acc->processPointcloud( makeCloud( { { 1, 2, 3 } } ) );

  EXPECT_EQ( acc->accumulatedCloud().width, 0u );
}

TEST_F( AccumulatorTest, CloudWithoutXyzIsIgnored )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  // Cloud whose only field is "intensity" - no x/y/z, must be dropped without crashing.
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  msg->header.frame_id = "map";
  msg->height = 1;
  msg->width = 1;
  msg->fields.resize( 1 );
  msg->fields[0].name = "intensity";
  msg->fields[0].offset = 0;
  msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg->fields[0].count = 1;
  msg->point_step = sizeof( float );
  msg->row_step = msg->point_step;
  msg->data.resize( msg->point_step );

  acc->processPointcloud( msg );
  EXPECT_EQ( acc->accumulatedCloud().width, 0u );
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
