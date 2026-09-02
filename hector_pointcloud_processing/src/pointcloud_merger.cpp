#include "hector_pointcloud_processing/pointcloud_merger.hpp"

#include "hector_pointcloud_processing/voxel_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

#include <tf2_eigen/tf2_eigen.hpp>

namespace hector_pointcloud_processing
{

namespace
{

double toSec( const builtin_interfaces::msg::Time &stamp )
{
  return static_cast<double>( stamp.sec ) + static_cast<double>( stamp.nanosec ) * 1e-9;
}

const sensor_msgs::msg::PointField *findField( const sensor_msgs::msg::PointCloud2 &msg,
                                               const std::string &name )
{
  const auto it = std::find_if( msg.fields.begin(), msg.fields.end(),
                                [&name]( const auto &f ) { return f.name == name; } );
  return it == msg.fields.end() ? nullptr : &( *it );
}

bool sameLayout( const std::vector<sensor_msgs::msg::PointField> &a,
                 const std::vector<sensor_msgs::msg::PointField> &b )
{
  if ( a.size() != b.size() )
    return false;
  for ( size_t i = 0; i < a.size(); ++i ) {
    if ( a[i].name != b[i].name || a[i].offset != b[i].offset || a[i].datatype != b[i].datatype ||
         a[i].count != b[i].count )
      return false;
  }
  return true;
}

} // namespace

PointcloudMerger::PointcloudMerger( const rclcpp::NodeOptions &options )
    : Node( "pointcloud_merger", options )
{
  declare_readonly_parameter( "input_topics", input_topics_,
                              "Input cloud topics. The first one is the timing reference the "
                              "others are matched against. A single topic is also valid and turns "
                              "this into a pure reframe: every point is transformed into "
                              "target_frame, nothing is merged" );
  declare_readonly_parameter( "target_frame", target_frame_,
                              "Frame the merged cloud is expressed in. Usually the frame of the "
                              "sensor whose IMU the downstream consumer uses" );
  declare_readonly_parameter(
      "time_field", time_field_,
      "Name of the per-point time field (uint32, ns relative to the cloud's own header stamp). "
      "Rebased onto the merged stamp. Empty disables rebasing" );
  declare_readonly_parameter( "line_field", line_field_,
                              "Name of the scan line / ring field (uint8). Empty disables line "
                              "renumbering" );
  declare_reconfigurable_parameter( "sync_tolerance", std::ref( sync_tolerance_ ),
                                    "Maximum header stamp difference, in seconds, for clouds to be "
                                    "considered part of the same group" );
  declare_reconfigurable_parameter(
      "min_range", std::ref( min_range_ ),
      "Points closer than this to their own sensor origin are dropped before transforming. Guards "
      "against no-return points reported as (0, 0, 0), which would otherwise become a spurious "
      "cluster at the sensor origin once moved into the target frame" );
  declare_readonly_parameter( "line_offset_step", line_offset_step_,
                              "Added to the line field as input_index * step, so lines from "
                              "different sensors stay distinguishable. 0 keeps them as they are" );
  declare_readonly_parameter( "buffer_size", buffer_size_,
                              "Clouds buffered per input while waiting for their counterparts" );
  declare_readonly_parameter(
      "cache_transforms", cache_transforms_,
      "Look each sensor extrinsic up once and reuse it. Correct for rigidly "
      "mounted sensors and avoids a TF lookup per cloud" );

  if ( input_topics_.empty() ) {
    throw std::runtime_error( "pointcloud_merger needs at least one input_topic" );
  }
  if ( target_frame_.empty() ) {
    throw std::runtime_error( "pointcloud_merger requires target_frame to be set" );
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>( get_clock() );
  // Pass the node so the listener uses this node's topics and honours its remappings, rather than
  // creating an internal node subscribed to the global /tf and /tf_static.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>( *tf_buffer_, this );

  buffers_.resize( input_topics_.size() );
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>( "points_merged",
                                                                rclcpp::SensorDataQoS().reliable() );

  subscriptions_.reserve( input_topics_.size() );
  for ( size_t i = 0; i < input_topics_.size(); ++i ) {
    subscriptions_.push_back( create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topics_[i], rclcpp::SensorDataQoS().reliable(),
        [this, i]( const CloudPtr &msg ) { pointcloudCallback( i, msg ); } ) );
  }

  printNodeStatus();
}

void PointcloudMerger::pointcloudCallback( size_t input_index, const CloudPtr &msg )
{
  std::lock_guard<std::mutex> lock( mutex_ );
  auto &buffer = buffers_[input_index];
  buffer.push_back( msg );
  while ( buffer.size() > static_cast<size_t>( buffer_size_ ) ) {
    RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 5000,
                          "Dropping a cloud from '%s': buffer full, its counterparts never arrived",
                          input_topics_[input_index].c_str() );
    buffer.pop_front();
  }
  processBuffers();
}

void PointcloudMerger::processBuffers()
{
  while ( !buffers_[0].empty() ) {
    const CloudPtr reference = buffers_[0].front();
    const double reference_stamp = toSec( reference->header.stamp );

    std::vector<CloudPtr> group{ reference };
    std::vector<size_t> matched_indices{ 0 };
    bool complete = true;
    bool unmatchable = false;

    for ( size_t i = 1; i < buffers_.size(); ++i ) {
      const auto match = findMatch( buffers_[i], reference_stamp );
      if ( !match ) {
        complete = false;
        // If this input has already moved past the reference, its counterpart will never arrive
        // and waiting would stall every later group behind it.
        if ( !buffers_[i].empty() &&
             toSec( buffers_[i].back()->header.stamp ) > reference_stamp + sync_tolerance_ ) {
          unmatchable = true;
        }
        break;
      }
      group.push_back( buffers_[i][*match] );
      matched_indices.push_back( *match );
    }

    if ( !complete ) {
      if ( !unmatchable )
        return; // Counterparts may still be in flight.
      RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 5000,
                            "Dropping a cloud from '%s': no match within %.1f ms on all inputs",
                            input_topics_[0].c_str(), sync_tolerance_ * 1e3 );
      buffers_[0].pop_front();
      continue;
    }

    if ( auto merged = merge( group ) ) {
      publisher_->publish( std::move( merged ) );
    }

    // Consume the matched clouds along with anything older, which can no longer be matched.
    for ( size_t i = 0; i < buffers_.size(); ++i ) {
      buffers_[i].erase( buffers_[i].begin(), buffers_[i].begin() + matched_indices[i] + 1 );
    }
  }
}

std::optional<size_t> PointcloudMerger::findMatch( const std::deque<CloudPtr> &buffer,
                                                   double stamp ) const
{
  std::optional<size_t> best;
  double best_diff = sync_tolerance_;
  for ( size_t i = 0; i < buffer.size(); ++i ) {
    const double diff = std::abs( toSec( buffer[i]->header.stamp ) - stamp );
    if ( diff <= best_diff ) {
      best_diff = diff;
      best = i;
    }
  }
  return best;
}

bool PointcloudMerger::validateLayout( const sensor_msgs::msg::PointCloud2 &msg )
{
  if ( reference_fields_ ) {
    if ( sameLayout( *reference_fields_, msg.fields ) && reference_point_step_ == msg.point_step )
      return true;
    RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 5000,
                           "Refusing to merge '%s': its field layout differs from the first cloud "
                           "seen. All inputs must publish an identical layout",
                           msg.header.frame_id.c_str() );
    return false;
  }

  const auto xyz = voxel_common::findXyzOffsets( msg );
  if ( !xyz ) {
    RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 5000,
                           "Input cloud has no usable x/y/z fields" );
    return false;
  }
  // The merge copies whole points and rewrites xyz in place, which needs the three coordinates
  // contiguous as float32 -- the layout every driver in this stack emits.
  if ( xyz->y != xyz->x + sizeof( float ) || xyz->z != xyz->y + sizeof( float ) ) {
    RCLCPP_ERROR_THROTTLE( get_logger(), *get_clock(), 5000,
                           "Input cloud does not store x, y, z as three contiguous float32" );
    return false;
  }
  x_offset_ = xyz->x;

  if ( !time_field_.empty() ) {
    const auto *field = findField( msg, time_field_ );
    if ( field == nullptr ) {
      RCLCPP_WARN( get_logger(),
                   "Time field '%s' not present; per-point times will not be rebased. Points from "
                   "inputs whose stamps differ will be misplaced in time by the consumer",
                   time_field_.c_str() );
    } else if ( field->datatype != sensor_msgs::msg::PointField::UINT32 ) {
      RCLCPP_ERROR( get_logger(), "Time field '%s' must be UINT32 (nanoseconds), not datatype %u",
                    time_field_.c_str(), field->datatype );
      return false;
    } else {
      time_offset_ = field->offset;
    }
  }

  if ( !line_field_.empty() && line_offset_step_ != 0 ) {
    const auto *field = findField( msg, line_field_ );
    if ( field == nullptr || field->datatype != sensor_msgs::msg::PointField::UINT8 ) {
      RCLCPP_WARN( get_logger(), "Line field '%s' missing or not UINT8; lines are left unchanged",
                   line_field_.c_str() );
    } else {
      line_offset_ = field->offset;
    }
  }

  reference_fields_ = msg.fields;
  reference_point_step_ = msg.point_step;
  return true;
}

std::optional<Eigen::Isometry3f> PointcloudMerger::lookupTransform( const std::string &source_frame,
                                                                    const rclcpp::Time &stamp )
{
  if ( cache_transforms_ ) {
    const auto cached = transform_cache_.find( source_frame );
    if ( cached != transform_cache_.end() )
      return cached->second;
  }

  try {
    const auto transform_msg = tf_buffer_->lookupTransform( target_frame_, source_frame, stamp,
                                                            rclcpp::Duration::from_seconds( 0.1 ) );
    const Eigen::Isometry3f transform = tf2::transformToEigen( transform_msg ).cast<float>();
    if ( cache_transforms_ )
      transform_cache_.emplace( source_frame, transform );
    return transform;
  } catch ( const tf2::TransformException &ex ) {
    RCLCPP_WARN_THROTTLE( get_logger(), *get_clock(), 5000, "Can't transform '%s' to '%s': %s",
                          source_frame.c_str(), target_frame_.c_str(), ex.what() );
    return std::nullopt;
  }
}

sensor_msgs::msg::PointCloud2::UniquePtr PointcloudMerger::merge( const std::vector<CloudPtr> &group )
{
  // The merged stamp is the earliest in the group so that every rebased per-point time stays
  // non-negative, which the unsigned time field requires.
  double base_stamp = toSec( group.front()->header.stamp );
  for ( const auto &msg : group ) base_stamp = std::min( base_stamp, toSec( msg->header.stamp ) );

  std::vector<Eigen::Isometry3f> transforms;
  transforms.reserve( group.size() );
  size_t total_points = 0;
  for ( const auto &msg : group ) {
    if ( !validateLayout( *msg ) )
      return nullptr;
    const auto transform = lookupTransform( msg->header.frame_id, msg->header.stamp );
    if ( !transform )
      return nullptr;
    transforms.push_back( *transform );
    total_points += static_cast<size_t>( msg->width ) * msg->height;
  }

  const uint32_t point_step = reference_point_step_;
  auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
  out->header.frame_id = target_frame_;
  out->header.stamp = rclcpp::Time( static_cast<int64_t>( base_stamp * 1e9 ), RCL_ROS_TIME );
  out->fields = *reference_fields_;
  out->is_bigendian = group.front()->is_bigendian;
  out->point_step = point_step;
  out->height = 1;
  out->is_dense = true;
  out->data.resize( total_points * point_step );

  const float min_range_sq = static_cast<float>( min_range_ * min_range_ );
  size_t write_index = 0;

  for ( size_t i = 0; i < group.size(); ++i ) {
    const auto &msg = *group[i];
    const Eigen::Isometry3f &transform = transforms[i];
    const size_t num_points = static_cast<size_t>( msg.width ) * msg.height;

    // Per-point times are relative to their own cloud's stamp; shifting by the gap to the merged
    // stamp puts every input on one timeline.
    const double stamp_delta = toSec( msg.header.stamp ) - base_stamp;
    const auto time_shift = static_cast<uint32_t>( std::lround( stamp_delta * 1e9 ) );
    const auto line_shift = static_cast<uint8_t>( i * static_cast<size_t>( line_offset_step_ ) );

    for ( size_t p = 0; p < num_points; ++p ) {
      const uint8_t *in_point = &msg.data[p * point_step];
      const float *xyz = reinterpret_cast<const float *>( in_point + x_offset_ );
      const Eigen::Vector3f point( xyz[0], xyz[1], xyz[2] );

      if ( !point.allFinite() || point.squaredNorm() < min_range_sq )
        continue;

      uint8_t *out_point = &out->data[write_index * point_step];
      std::memcpy( out_point, in_point, point_step );

      const Eigen::Vector3f transformed = transform * point;
      auto *out_xyz = reinterpret_cast<float *>( out_point + x_offset_ );
      out_xyz[0] = transformed.x();
      out_xyz[1] = transformed.y();
      out_xyz[2] = transformed.z();

      if ( time_offset_ ) {
        uint32_t time_value;
        std::memcpy( &time_value, in_point + *time_offset_, sizeof( time_value ) );
        time_value += time_shift;
        std::memcpy( out_point + *time_offset_, &time_value, sizeof( time_value ) );
      }
      if ( line_offset_ ) {
        out_point[*line_offset_] = static_cast<uint8_t>( out_point[*line_offset_] + line_shift );
      }

      ++write_index;
    }
  }

  out->width = static_cast<uint32_t>( write_index );
  out->row_step = out->width * point_step;
  out->data.resize( write_index * point_step );
  return out;
}

void PointcloudMerger::printNodeStatus() const
{
  std::stringstream info;
  info << "The node has the following attributes:" << std::endl;
  info << "  inputs:" << std::endl;
  for ( const auto &topic : input_topics_ ) info << "    " << topic << std::endl;
  info << "  output:       " << publisher_->get_topic_name() << std::endl;
  info << "  target_frame: " << target_frame_ << std::endl;
  info << "  time_field:   " << ( time_field_.empty() ? "<none>" : time_field_ ) << std::endl;
  info << "  sync_tol:     " << sync_tolerance_ * 1e3 << " ms";
  RCLCPP_INFO_STREAM( get_logger(), info.str() );
}

} // namespace hector_pointcloud_processing

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( hector_pointcloud_processing::PointcloudMerger )
