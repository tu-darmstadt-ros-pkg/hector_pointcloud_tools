// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "../src/nodes/pointcloud_accumulator_node.hpp"
#include "hector_pointcloud_processing/pointcloud_accumulator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

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

  bool updated() const { return this->updated_; }

  void enqueue( const sensor_msgs::msg::PointCloud2::SharedPtr &msg )
  {
    this->onNewPointcloud( msg );
  }

  //! Waits until the processing thread has finished a cloud and holds the next one.
  void waitForProcessing()
  {
    while ( true ) {
      {
        std::lock_guard lock( this->accumulated_pointcloud_mutex_ );
        if ( this->accumulated_cloud_.width > 0 )
          return;
      }
      std::this_thread::sleep_for( std::chrono::microseconds( 50 ) );
    }
  }
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
std::shared_ptr<TestableAccumulator<MODE>> makeAccumulator( rclcpp::Node &node, double resolution,
                                                            double max_range = 0.0 )
{
  return std::make_shared<TestableAccumulator<MODE>>( node, resolution, "map", rclcpp::Rate( 1.0 ),
                                                      std::vector<std::string>{},
                                                      /*queue_size=*/10, max_range );
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
  // Both round to voxel (0, 0, 0), centred on the origin; the first point is closer to it.
  acc->processPointcloud( makeCloud( { { 0.4f, 0.4f, 0.4f }, { 0.1f, 0.1f, 0.1f } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE(
      readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 0.1f, 0.1f, 0.1f ), kEps ) );
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

// A lidar layout: x, y, z and intensity as FLOAT32, then a FLOAT64 timestamp.
TEST_F( AccumulatorTest, FieldsOfMixedTypesGetTheirOwnBytes )
{
  auto msg = makeCloud( { { 1.0f, 0.0f, 0.0f } }, { { "intensity", 42.0f } } );
  sensor_msgs::msg::PointField timestamp;
  timestamp.name = "timestamp";
  timestamp.offset = 16;
  timestamp.datatype = sensor_msgs::msg::PointField::FLOAT64;
  timestamp.count = 1;
  msg->fields.push_back( timestamp );
  msg->point_step = 24;
  msg->row_step = 24;
  msg->data.resize( 24 );
  const double stamp = 1234.5;
  std::memcpy( msg->data.data() + 16, &stamp, sizeof( stamp ) );

  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( msg );

  const auto &cloud = acc->accumulatedCloud();
  ASSERT_EQ( cloud.fields.size(), 5u );
  EXPECT_EQ( cloud.fields[3].offset, 12u );
  EXPECT_EQ( cloud.fields[4].offset, 16u );
  EXPECT_EQ( cloud.point_step, 24u );
  ASSERT_EQ( cloud.data.size(), 24u );
  double value = 0.0;
  std::memcpy( &value, cloud.data.data() + 16, sizeof( value ) );
  EXPECT_DOUBLE_EQ( value, stamp );
  EXPECT_NEAR( *reinterpret_cast<const float *>( cloud.data.data() + 12 ), 42.0f, kEps );
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

TEST_F( AccumulatorTest, NonFinitePointsAreSkipped )
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { nan, 0, 0 }, { 0, inf, 0 }, { 3, 0, 0 } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE( readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 3, 0, 0 ), kEps ) );
}

TEST_F( AccumulatorTest, MaxRangeSkipsFarPoints )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0, /*max_range=*/5.0 );
  acc->processPointcloud( makeCloud( { { 3, 4, 0 }, { 4, 4, 0 }, { 0, 0, -6 } } ) );

  ASSERT_EQ( acc->accumulatedCloud().width, 1u );
  EXPECT_TRUE( readPoint( acc->accumulatedCloud(), 0 ).isApprox( Eigen::Vector3f( 3, 4, 0 ), kEps ) );
}

TEST_F( AccumulatorTest, CloudsStampedBeforeAResetAreDropped )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  const rclcpp::Time before = node_->get_clock()->now();
  acc->reset();
  auto old_cloud = makeCloud( { { 0, 0, 0 } } );
  old_cloud->header.stamp = before;
  acc->processPointcloud( old_cloud );
  EXPECT_EQ( acc->accumulatedCloud().width, 0u );

  auto new_cloud = makeCloud( { { 5, 0, 0 } } );
  new_cloud->header.stamp = node_->get_clock()->now();
  acc->processPointcloud( new_cloud );
  EXPECT_EQ( acc->accumulatedCloud().width, 1u );
}

TEST_F( AccumulatorTest, ResetPublishesTheEmptyCloud )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 0, 0, 0 } } ) );
  acc->publishPointcloud();
  ASSERT_FALSE( acc->updated() );
  acc->reset();
  EXPECT_TRUE( acc->updated() );
}

TEST_F( AccumulatorTest, ResetStampsTheEmptyCloud )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  auto cloud = makeCloud( { { 0, 0, 0 } } );
  cloud->header.stamp = rclcpp::Time( 5, 0 );
  acc->processPointcloud( cloud );
  acc->reset();
  EXPECT_GT( rclcpp::Time( acc->accumulatedCloud().header.stamp ).nanoseconds(), 5'000'000'000 );
}

TEST_F( AccumulatorTest, FieldsAreTakenFromTheFirstCloudAfterAReset )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( makeCloud( { { 0, 0, 0 } }, { { "intensity", 1.0f }, { "ring", 2.0f } } ) );
  ASSERT_EQ( acc->accumulatedCloud().fields.size(), 5u );
  acc->reset();
  // A different sensor with a UINT8 intensity.
  auto msg = makeCloud( { { 0, 0, 0 } } );
  sensor_msgs::msg::PointField intensity;
  intensity.name = "intensity";
  intensity.offset = 12;
  intensity.datatype = sensor_msgs::msg::PointField::UINT8;
  intensity.count = 1;
  msg->fields.push_back( intensity );
  msg->point_step = 13;
  msg->row_step = 13;
  msg->data.resize( 13 );
  msg->data[12] = 200;
  acc->processPointcloud( msg );

  const auto &cloud = acc->accumulatedCloud();
  ASSERT_EQ( cloud.fields.size(), 4u );
  EXPECT_EQ( cloud.fields[3].datatype, sensor_msgs::msg::PointField::UINT8 );
  EXPECT_EQ( cloud.point_step, 13u );
  ASSERT_EQ( cloud.data.size(), 13u );
  EXPECT_EQ( cloud.data[12], 200 );
}

// A UINT8 field makes the accumulated point_step 13, so the second point is not aligned. Build with
// -fsanitize=alignment to see misaligned accesses.
TEST_F( AccumulatorTest, UnalignedPointsAreAveraged )
{
  const auto make = []( float x ) {
    auto msg = makeCloud( { { 0, 0, 0 }, { x, 0, 0 } } );
    sensor_msgs::msg::PointField ring;
    ring.name = "ring";
    ring.offset = 12;
    ring.datatype = sensor_msgs::msg::PointField::UINT8;
    ring.count = 1;
    msg->fields.push_back( ring );
    msg->point_step = 16;
    msg->row_step = 32;
    msg->data.assign( 32, 0 );
    std::memcpy( msg->data.data() + 16, &x, sizeof( x ) );
    return msg;
  };
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  acc->processPointcloud( make( 5.0f ) );
  acc->processPointcloud( make( 5.2f ) );

  const auto &cloud = acc->accumulatedCloud();
  ASSERT_EQ( cloud.point_step, 13u );
  ASSERT_EQ( cloud.width, 2u );
  float x = 0;
  std::memcpy( &x, cloud.data.data() + 13, sizeof( x ) );
  EXPECT_NEAR( x, 5.1f, kEps );
}

TEST_F( AccumulatorTest, MalformedCloudsAreDropped )
{
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  auto short_data = makeCloud( { { 0, 0, 0 }, { 5, 0, 0 } } );
  short_data->data.resize( short_data->point_step );
  acc->processPointcloud( short_data );
  auto field_beyond = makeCloud( { { 0, 0, 0 } }, { { "intensity", 1.0f } } );
  field_beyond->fields.back().offset = field_beyond->point_step;
  acc->processPointcloud( field_beyond );
  auto big_endian = makeCloud( { { 0, 0, 0 } } );
  big_endian->is_bigendian = true;
  acc->processPointcloud( big_endian );

  EXPECT_EQ( acc->accumulatedCloud().width, 0u );
}

// Resets while the processing thread works through queued clouds. Build with -fsanitize=address
// and -D_GLIBCXX_SANITIZE_VECTOR to see the race, the plain build only checks the map afterwards.
TEST_F( AccumulatorTest, ResetWhileProcessingKeepsTheMapConsistent )
{
  std::vector<Eigen::Vector3f> points;
  for ( int i = 0; i < 20000; ++i ) points.emplace_back( i % 100, i / 100, 0 );
  const auto cloud = makeCloud( points );
  auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
  for ( int k = 0; k < 30; ++k ) {
    for ( int i = 0; i < 3; ++i ) acc->enqueue( cloud );
    acc->waitForProcessing();
    acc->reset();
  }
  std::this_thread::sleep_for( std::chrono::milliseconds( 200 ) );
  acc->reset();
  acc->processPointcloud( cloud );
  EXPECT_EQ( acc->accumulatedCloud().width, 20000u );
}

// Destroys the accumulator while its thread processes a cloud. Build with -fsanitize=address to see
// a use after free of the voxels.
TEST_F( AccumulatorTest, DestroyingWhileProcessingIsSafe )
{
  std::vector<Eigen::Vector3f> points;
  for ( int i = 0; i < 20000; ++i ) points.emplace_back( i % 100, i / 100, 0 );
  const auto cloud = makeCloud( points );
  for ( int k = 0; k < 20; ++k ) {
    auto acc = makeAccumulator<AggregationMode::AVERAGE>( *node_, 1.0 );
    for ( int i = 0; i < 3; ++i ) acc->enqueue( cloud );
    acc->waitForProcessing();
  }
  SUCCEED();
}

TEST( AccumulatorNode, RejectsAnUnknownAggregationModeAndInvalidValues )
{
  for ( const auto &parameter :
        { rclcpp::Parameter( "aggregation_mode", "median" ), rclcpp::Parameter( "resolution", 0.0 ),
          rclcpp::Parameter( "publish_rate", -1.0 ), rclcpp::Parameter( "max_range", -1.0 ) } ) {
    rclcpp::NodeOptions options;
    options.parameter_overrides( { parameter } );
    EXPECT_THROW( hector_pointcloud_processing::PointcloudAccumulatorNode node( options ),
                  std::invalid_argument )
        << parameter.get_name();
  }
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
