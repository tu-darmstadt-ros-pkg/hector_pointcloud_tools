// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_accumulator/pointcloud_accumulator.hpp"

#include "./logging.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

namespace hector_pointcloud_accumulator
{

PointcloudAccumulatorBase::PointcloudAccumulatorBase( rclcpp::Node &node, double resolution,
                                                      const std::string &frame,
                                                      const rclcpp::Rate &publish_rate,
                                                      const std::vector<std::string> &topics )
    : frame_( frame ), resolution_( resolution ), tfBuffer( node.get_clock() ), tfListener( tfBuffer )
{

  reset_service_ = node.create_service<std_srvs::srv::Trigger>(
      "reset_pointcloud", [this]( std_srvs::srv::Trigger::Request::SharedPtr,
                                  std_srvs::srv::Trigger::Response::SharedPtr res ) {
        reset();
        res->success = true;
      } );
  enable_service_ = node.create_service<std_srvs::srv::SetBool>(
      "enable_pointcloud", [this]( std_srvs::srv::SetBool::Request::SharedPtr req,
                                   std_srvs::srv::SetBool::Response::SharedPtr res ) {
        setEnabled( req->data );
        res->success = true;
      } );

  for ( const auto &topic : topics ) {
    pointcloud_subscriptions_.emplace_back( node.create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, 10,
        [this]( sensor_msgs::msg::PointCloud2::SharedPtr msg ) { processPointcloud( msg ); } ) );
  }

  accumulated_publisher_ = node.create_publisher<sensor_msgs::msg::PointCloud2>( "cloud_out", 1 );
  publish_timer_ = node.create_wall_timer( publish_rate.period(), [this]() { publishPointcloud(); } );

  accumulated_cloud_.header.frame_id = frame_;
  accumulated_cloud_.height = 1;
  accumulated_cloud_.fields.resize( 3 );
  accumulated_cloud_.fields[0].name = "x";
  accumulated_cloud_.fields[0].offset = 0;
  accumulated_cloud_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  accumulated_cloud_.fields[0].count = 1;
  accumulated_cloud_.fields[1].name = "y";
  accumulated_cloud_.fields[1].offset = 4;
  accumulated_cloud_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  accumulated_cloud_.fields[1].count = 1;
  accumulated_cloud_.fields[2].name = "z";
  accumulated_cloud_.fields[2].offset = 8;
  accumulated_cloud_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  accumulated_cloud_.fields[2].count = 1;
  accumulated_cloud_.is_bigendian = false;
  accumulated_cloud_.point_step = 12;
  accumulated_cloud_.is_dense = false;

  PCA_LOG_INFO(
      "PointcloudAccumulator initialized with resolution %f and frame %s. Publishing every %fs.",
      resolution_, frame_.c_str(), 1E9 / publish_rate.period().count() );
}

void PointcloudAccumulatorBase::setEnabled( bool enabled ) { enabled_ = enabled; }

void PointcloudAccumulatorBase::reset()
{
  accumulated_cloud_.data.clear();
  accumulated_cloud_.width = 0;
}

void PointcloudAccumulatorBase::publishPointcloud()
{
  if ( !updated_ ) {
    return;
  }
  updated_ = false;
  accumulated_cloud_.row_step = accumulated_cloud_.point_step * accumulated_cloud_.width;
  accumulated_publisher_->publish( accumulated_cloud_ );
  PCA_LOG_INFO( "Published pointcloud with %d points. (%u new, %u processed, %lu total)",
                accumulated_cloud_.width, accumulated_cloud_.width - count_last_published_,
                count_last_processed_, count_total_processed_ );
  count_last_published_ = accumulated_cloud_.width;
  count_last_processed_ = 0;
}

namespace
{
int pointFieldSize( uint8_t datatype )
{
  switch ( datatype ) {
  case sensor_msgs::msg::PointField::UINT8:
  case sensor_msgs::msg::PointField::INT8:
    return 1;
  case sensor_msgs::msg::PointField::UINT16:
  case sensor_msgs::msg::PointField::INT16:
    return 2;
  case sensor_msgs::msg::PointField::UINT32:
  case sensor_msgs::msg::PointField::INT32:
  case sensor_msgs::msg::PointField::FLOAT32:
    return 4;
  case sensor_msgs::msg::PointField::FLOAT64:
    return 8;
  default:
    throw std::runtime_error( "Unknown datatype" );
  }
}
} // namespace

void PointcloudAccumulatorBase::addField( const sensor_msgs::msg::PointField &field )
{
  sensor_msgs::msg::PointField new_field = field;
  const sensor_msgs::msg::PointField &last_field = accumulated_cloud_.fields.back();
  new_field.offset = last_field.offset + last_field.count * pointFieldSize( field.datatype );
  accumulated_cloud_.fields.push_back( new_field );
  accumulated_cloud_.point_step += pointFieldSize( field.datatype ) * field.count;
}

template<AggregationMode AGGREGATION_MODE>
void PointcloudAccumulator<AGGREGATION_MODE>::reset()
{
  PointcloudAccumulatorBase::reset();
  voxel_map_.clear();
}

namespace
{
int32_t findChannelIndex( const sensor_msgs::msg::PointCloud2 &msg, const std::string &name )
{
  for ( size_t i = 0; i < msg.fields.size(); ++i ) {
    if ( msg.fields[i].name == name ) {
      return static_cast<int32_t>( i );
    }
  }
  return -1;
}

} // namespace

template<AggregationMode AGGREGATION_MODE>
template<typename T>
void PointcloudAccumulator<AGGREGATION_MODE>::processPointCloudData(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg )
{
  if ( !tfBuffer.canTransform( frame_, msg->header.frame_id, msg->header.stamp,
                               rclcpp::Duration::from_seconds( 0.01 ) ) ) {
    PCA_LOG_WARN( "Can't transform from %s to %s!", msg->header.frame_id.c_str(), frame_.c_str() );
    return;
  }
  geometry_msgs::msg::TransformStamped transform_msg = tfBuffer.lookupTransform(
      frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds( 0.01 ) );
  const Eigen::Isometry3f transform = tf2::transformToEigen( transform_msg ).cast<float>();

  // When this is called, it was already checked that x, y, z fields exist
  sensor_msgs::msg::PointField x_field = msg->fields[findChannelIndex( *msg, "x" )];
  sensor_msgs::msg::PointField y_field = msg->fields[findChannelIndex( *msg, "y" )];
  sensor_msgs::msg::PointField z_field = msg->fields[findChannelIndex( *msg, "z" )];
  if ( x_field.datatype != y_field.datatype || x_field.datatype != z_field.datatype ) {
    PCA_LOG_ERROR_ONCE( "Different data types for x, y, z fields are not supported." );
    return;
  }
  // Add fields that are not in pointcloud already
  for ( const auto &field : msg->fields ) {
    if ( std::any_of( accumulated_cloud_.fields.begin(), accumulated_cloud_.fields.end(),
                      [&field]( const auto &item ) { return item.name == field.name; } ) ) {
      continue;
    }
    if ( accumulated_cloud_.width != 0 ) {
      PCA_LOG_ERROR_ONCE( "Mixing pointclouds with different fields is not supported." );
      break;
    }
    addField( field );
  }

  const uint8_t *data = msg->data.data();
  const unsigned int width = msg->width;
  const unsigned int height = msg->height;
  for ( unsigned int i = 0; i < height; ++i ) {
    const uint8_t *point_data = data;
    for ( unsigned int k = 0; k < width; ++k ) {
      Eigen::Vector3f point;
      point.x() = static_cast<float>( *reinterpret_cast<const T *>( point_data + x_field.offset ) );
      point.y() = static_cast<float>( *reinterpret_cast<const T *>( point_data + y_field.offset ) );
      point.z() = static_cast<float>( *reinterpret_cast<const T *>( point_data + z_field.offset ) );
      point = transform * point;
      Index index = { static_cast<int>( std::round( point.x() / resolution_ ) ),
                      static_cast<int>( std::round( point.y() / resolution_ ) ),
                      static_cast<int>( std::round( point.z() / resolution_ ) ) };
      auto it = voxel_map_.find( index );
      if ( it == voxel_map_.end() ) {
        PointInfo info = addNewPoint( msg, point_data, index, point );
        voxel_map_.try_emplace( index, info );
      } else {
        updatePoint( index, point, it->second );
      }
      point_data += msg->point_step;
    }
    data += msg->row_step;
  }
  count_last_processed_ += width * height;
  count_total_processed_ += width * height;
  accumulated_cloud_.header.stamp = msg->header.stamp;
  updated_ = true;
}

template<AggregationMode AGGREGATION_MODE>
void PointcloudAccumulator<AGGREGATION_MODE>::processPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg )
{
  if ( !enabled_ ) {
    return;
  }

  int32_t xi = findChannelIndex( *msg, "x" );
  int32_t yi = findChannelIndex( *msg, "y" );
  int32_t zi = findChannelIndex( *msg, "z" );
  if ( xi == -1 || yi == -1 || zi == -1 ) {
    PCA_LOG_WARN( "No x, y, or z channel found in point cloud message!" );
    return;
  }
  switch ( msg->fields[xi].datatype ) {
  case sensor_msgs::msg::PointField::INT8:
    processPointCloudData<int8_t>( msg );
    break;
  case sensor_msgs::msg::PointField::UINT8:
    processPointCloudData<uint8_t>( msg );
    break;
  case sensor_msgs::msg::PointField::INT16:
    processPointCloudData<int16_t>( msg );
    break;
  case sensor_msgs::msg::PointField::UINT16:
    processPointCloudData<uint16_t>( msg );
    break;
  case sensor_msgs::msg::PointField::INT32:
    processPointCloudData<int32_t>( msg );
    break;
  case sensor_msgs::msg::PointField::UINT32:
    processPointCloudData<uint32_t>( msg );
    break;
  case sensor_msgs::msg::PointField::FLOAT32:
    processPointCloudData<float>( msg );
    break;
  case sensor_msgs::msg::PointField::FLOAT64:
    processPointCloudData<double>( msg );
    break;
  default:
    PCA_LOG_ERROR_ONCE( "Unsupported data type %d for x, y, z fields.", msg->fields[xi].datatype );
    return;
  }
}

template<AggregationMode AGGREGATION_MODE>
typename PointcloudAccumulator<AGGREGATION_MODE>::PointInfo
PointcloudAccumulator<AGGREGATION_MODE>::addNewPoint(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg, const uint8_t *point_data,
    const Index &index, const Eigen::Vector3f &point )
{
  size_t point_index = accumulated_cloud_.width;
  accumulated_cloud_.width += 1;
  accumulated_cloud_.data.resize( accumulated_cloud_.width * accumulated_cloud_.point_step );
  auto *out = accumulated_cloud_.data.data();
  out += ( accumulated_cloud_.width - 1 ) * accumulated_cloud_.point_step;
  Eigen::Map<Eigen::Vector3f> point_map( reinterpret_cast<float *>( out ) );
  point_map = point;
  // Copy fields
  for ( size_t i = 3; i < accumulated_cloud_.fields.size(); ++i ) {
    const auto &target_field = accumulated_cloud_.fields[i];
    int32_t source_index = findChannelIndex( *msg, target_field.name );
    if ( source_index == -1 )
      continue;
    const auto &source_field = msg->fields[source_index];
    if ( source_field.datatype != target_field.datatype ) {
      PCA_LOG_ERROR_ONCE( "Different data types for %s field are not supported.",
                          target_field.name.c_str() );
      continue;
    }
    int count = std::min( source_field.count, target_field.count );
    const uint8_t *source_data = point_data + source_field.offset;
    uint8_t *target_data = out + target_field.offset;
    std::copy( source_data, source_data + count * pointFieldSize( source_field.datatype ),
               target_data );
  }

  if constexpr ( AGGREGATION_MODE == AggregationMode::AVERAGE ) {
    return { point_index, 1 };
  } else if ( AGGREGATION_MODE == AggregationMode::HIGHEST_Z ) {
    return { point_index, point.z() };
  } else if ( AGGREGATION_MODE == AggregationMode::CLOSEST_TO_CENTER ) {
    Eigen::Vector3f center( index.x, index.y, index.z );
    center.array() += 0.5;
    center *= resolution_;
    float distance = ( point - center ).squaredNorm();
    return { point_index, distance };
  }
}

template<>
void PointcloudAccumulator<AggregationMode::AVERAGE>::updatePoint( const Index &,
                                                                   const Eigen::Vector3f &point,
                                                                   PointInfo &info )
{
  Eigen::Map<Eigen::Vector3f> point_map( reinterpret_cast<float *>(
      accumulated_cloud_.data.data() + info.index * accumulated_cloud_.point_step ) );
  point_map = point_map + ( point - point_map ) / static_cast<float>( info.count + 1 );
  info.count += 1;
}

template<>
void PointcloudAccumulator<AggregationMode::HIGHEST_Z>::updatePoint( const Index &,
                                                                     const Eigen::Vector3f &point,
                                                                     PointInfo &info )
{
  if ( point.z() > info.z ) {
    info.z = point.z();
    Eigen::Map<Eigen::Vector3f> point_map( reinterpret_cast<float *>(
        accumulated_cloud_.data.data() + info.index * accumulated_cloud_.point_step ) );
    point_map = point;
  }
}

template<>
void PointcloudAccumulator<AggregationMode::CLOSEST_TO_CENTER>::updatePoint(
    const Index &index, const Eigen::Vector3f &point, PointInfo &info )
{
  Eigen::Vector3f center( index.x, index.y, index.z );
  center.array() += 0.5;
  center *= resolution_;
  float distance = ( point - center ).squaredNorm();
  if ( distance < info.distance ) {
    info.distance = distance;
    Eigen::Map<Eigen::Vector3f> point_map( reinterpret_cast<float *>(
        accumulated_cloud_.data.data() + info.index * accumulated_cloud_.point_step ) );
    point_map = point;
  }
}

template class PointcloudAccumulator<AggregationMode::AVERAGE>;
template class PointcloudAccumulator<AggregationMode::HIGHEST_Z>;
template class PointcloudAccumulator<AggregationMode::CLOSEST_TO_CENTER>;
} // namespace hector_pointcloud_accumulator
