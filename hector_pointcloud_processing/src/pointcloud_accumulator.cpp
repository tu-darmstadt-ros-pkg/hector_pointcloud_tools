// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_processing/pointcloud_accumulator.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <cstring>

namespace hector_pointcloud_processing
{

PointcloudAccumulatorBase::PointcloudAccumulatorBase( rclcpp::Node &node, double resolution,
                                                      const std::string &frame,
                                                      const rclcpp::Rate &publish_rate,
                                                      const std::vector<std::string> &topics,
                                                      size_t queue_size, double max_range )
    : frame_( frame ), resolution_( resolution ), max_queue_size_( queue_size ),
      max_range_( max_range ), clock_( node.get_clock() ),
      reset_stamp_( 0, 0, node.get_clock()->get_clock_type() ),
      latest_stamp_( 0, 0, node.get_clock()->get_clock_type() ), tfBuffer( node.get_clock() ),
      tfListener( tfBuffer, &node ), logger_( node.get_logger() )
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

  // Durability is included so the accumulated cloud's latching can be reconfigured too.
  const rclcpp::QosOverridingOptions qos_overriding_options{
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Depth,
      rclcpp::QosPolicyKind::Reliability, rclcpp::QosPolicyKind::Durability };

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.qos_overriding_options = qos_overriding_options;
  for ( const auto &topic : topics ) {
    pointcloud_subscriptions_.emplace_back( node.create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        [this]( sensor_msgs::msg::PointCloud2::SharedPtr msg ) { onNewPointcloud( msg ); },
        subscription_options ) );
  }

  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = qos_overriding_options;
  accumulated_publisher_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      "accumulated_pointcloud", rclcpp::QoS( 1 ).transient_local().reliable(), publisher_options );
  // On the node's clock, so the rate is in simulation time when the node uses it.
  publish_timer_ = rclcpp::create_timer( &node, node.get_clock(), publish_rate.period(),
                                         [this]() { publishPointcloud(); } );

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

  RCLCPP_INFO( logger_,
               "PointcloudAccumulator initialized with resolution %f, max range %f and max queue "
               "size %lu in frame %s. Publishing at %f Hz.",
               resolution_, max_range_, max_queue_size_, frame_.c_str(),
               1E9 / publish_rate.period().count() );

  processing_thread_ = std::thread( &PointcloudAccumulatorBase::processQueue, this );
}

PointcloudAccumulatorBase::~PointcloudAccumulatorBase()
{
  pointcloud_subscriptions_.clear();
  publish_timer_.reset();
  stop();
}

void PointcloudAccumulatorBase::stop()
{
  {
    std::lock_guard lock( pointcloud_queue_mutex_ );
    shutting_down_ = true;
  }
  data_available.notify_all();
  if ( processing_thread_.joinable() ) {
    processing_thread_.join();
  }
}

void PointcloudAccumulatorBase::setEnabled( bool enabled ) { enabled_ = enabled; }

void PointcloudAccumulatorBase::reset()
{
  std::scoped_lock lock{ accumulated_pointcloud_mutex_, pointcloud_queue_mutex_ };
  RCLCPP_INFO( logger_, "Resetting accumulated pointcloud." );
  pointcloud_queue_.clear();
  accumulated_cloud_.data.clear();
  accumulated_cloud_.width = 0;
  clearVoxels();
  reset_stamp_ = clock_->now();
  latest_stamp_ = rclcpp::Time( 0, 0, clock_->get_clock_type() );
  count_last_published_ = 0;
  // Publishes the empty cloud, so late joiners do not get the map from before the reset.
  updated_ = true;
}

void PointcloudAccumulatorBase::publishPointcloud()
{
  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud;
  {
    // Copied under the lock and published outside it, so serializing does not stall processing.
    std::unique_lock lock( accumulated_pointcloud_mutex_ );
    if ( !updated_ ) {
      return;
    }
    updated_ = false;
    accumulated_cloud_.row_step = accumulated_cloud_.point_step * accumulated_cloud_.width;
    cloud = std::make_unique<sensor_msgs::msg::PointCloud2>( accumulated_cloud_ );
    RCLCPP_DEBUG( logger_, "Publishing pointcloud with %u points. (%u new, %u processed, %lu total)",
                  accumulated_cloud_.width, accumulated_cloud_.width - count_last_published_,
                  count_last_processed_, count_total_processed_ );
    count_last_published_ = accumulated_cloud_.width;
    count_last_processed_ = 0;
  }
  accumulated_publisher_->publish( std::move( cloud ) );
  if ( const unsigned int dropped = dropped_pointclouds_.exchange( 0 ); dropped > 0 ) {
    RCLCPP_WARN( logger_, "Dropped %u incoming pointclouds due to full queue.", dropped );
  }
}

void PointcloudAccumulatorBase::onNewPointcloud( const sensor_msgs::msg::PointCloud2::SharedPtr &msg )
{
  if ( !enabled_ ) {
    return;
  }
  std::lock_guard lock( pointcloud_queue_mutex_ );
  if ( max_queue_size_ != 0 && pointcloud_queue_.size() >= max_queue_size_ ) {
    pointcloud_queue_.pop_front();
    ++dropped_pointclouds_;
  }
  pointcloud_queue_.push_back( msg );
  data_available.notify_one();
}

void PointcloudAccumulatorBase::processQueue()
{
  while ( true ) {
    sensor_msgs::msg::PointCloud2::SharedPtr msg;
    {
      std::unique_lock lock( pointcloud_queue_mutex_ );
      data_available.wait( lock, [this] { return shutting_down_ || !pointcloud_queue_.empty(); } );
      if ( shutting_down_ ) {
        return;
      }
      msg = pointcloud_queue_.front();
      pointcloud_queue_.pop_front();
    }
    try {
      processPointcloud( msg );
    } catch ( const std::exception &e ) {
      RCLCPP_ERROR_THROTTLE( logger_, *clock_, 5000, "Dropped a pointcloud: %s", e.what() );
    }
  }
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
  new_field.offset = accumulated_cloud_.point_step;
  accumulated_cloud_.fields.push_back( new_field );
  accumulated_cloud_.point_step += pointFieldSize( field.datatype ) * field.count;
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

bool knownDatatype( uint8_t datatype )
{
  return datatype >= sensor_msgs::msg::PointField::INT8 &&
         datatype <= sensor_msgs::msg::PointField::FLOAT64;
}

//! Why the cloud's layout cannot be read safely, or an empty string.
std::string layoutProblem( const sensor_msgs::msg::PointCloud2 &msg )
{
  if ( msg.is_bigendian ) {
    return "big endian data";
  }
  if ( static_cast<uint64_t>( msg.width ) * msg.point_step > msg.row_step ) {
    return "rows shorter than width * point_step";
  }
  if ( static_cast<uint64_t>( msg.row_step ) * msg.height > msg.data.size() ) {
    return "less data than row_step * height";
  }
  for ( const auto &field : msg.fields ) {
    if ( !knownDatatype( field.datatype ) ) {
      return "unknown datatype of field " + field.name;
    }
    if ( field.offset + static_cast<uint64_t>( pointFieldSize( field.datatype ) ) * field.count >
         msg.point_step ) {
      return "field " + field.name + " beyond point_step";
    }
  }
  return {};
}

template<typename T>
float readValue( const uint8_t *data )
{
  T value;
  std::memcpy( &value, data, sizeof( T ) );
  return static_cast<float>( value );
}

} // namespace

template<AggregationMode AGGREGATION_MODE>
template<typename T>
void PointcloudAccumulator<AGGREGATION_MODE>::processPointCloudData(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg )
{
  using namespace std::chrono_literals;
  // Blocks the queue for up to 1 s, clouds beyond queue_size are dropped meanwhile.
  if ( !tfBuffer.canTransform( frame_, msg->header.frame_id, msg->header.stamp, 1s ) ) {
    RCLCPP_WARN_THROTTLE( logger_, *clock_, 5000, "Can't transform from %s to %s!",
                          msg->header.frame_id.c_str(), frame_.c_str() );
    return;
  }
  geometry_msgs::msg::TransformStamped transform_msg =
      tfBuffer.lookupTransform( frame_, msg->header.frame_id, msg->header.stamp, 10ms );
  const Eigen::Isometry3f transform = tf2::transformToEigen( transform_msg ).cast<float>();

  // When this is called, it was already checked that x, y, z fields exist
  sensor_msgs::msg::PointField x_field = msg->fields[findChannelIndex( *msg, "x" )];
  sensor_msgs::msg::PointField y_field = msg->fields[findChannelIndex( *msg, "y" )];
  sensor_msgs::msg::PointField z_field = msg->fields[findChannelIndex( *msg, "z" )];
  if ( x_field.datatype != y_field.datatype || x_field.datatype != z_field.datatype ) {
    RCLCPP_ERROR_ONCE( logger_, "Different data types for x, y, z fields are not supported." );
    return;
  }

  std::unique_lock lock( accumulated_pointcloud_mutex_ );
  const rclcpp::Time stamp( msg->header.stamp, clock_->get_clock_type() );
  // Taken before the last reset. An unstamped cloud is kept.
  if ( stamp.nanoseconds() != 0 && stamp < reset_stamp_ ) {
    return;
  }
  // The fields of the first cloud after a reset make up the accumulated cloud.
  for ( const auto &field : msg->fields ) {
    if ( std::any_of( accumulated_cloud_.fields.begin(), accumulated_cloud_.fields.end(),
                      [&field]( const auto &item ) { return item.name == field.name; } ) ) {
      continue;
    }
    if ( accumulated_cloud_.width != 0 ) {
      RCLCPP_WARN_ONCE( logger_,
                        "Field %s is not in the accumulated cloud and is left out. The fields "
                        "come from the first cloud.",
                        field.name.c_str() );
      break;
    }
    addField( field );
  }

  const float max_range_squared = static_cast<float>( max_range_ * max_range_ );
  const uint8_t *data = msg->data.data();
  const unsigned int width = msg->width;
  const unsigned int height = msg->height;
  for ( unsigned int i = 0; i < height; ++i ) {
    for ( unsigned int k = 0; k < width; ++k ) {
      const uint8_t *point_data = data + static_cast<size_t>( i ) * msg->row_step +
                                  static_cast<size_t>( k ) * msg->point_step;
      Eigen::Vector3f point( readValue<T>( point_data + x_field.offset ),
                             readValue<T>( point_data + y_field.offset ),
                             readValue<T>( point_data + z_field.offset ) );
      if ( !point.allFinite() || ( max_range_ > 0 && point.squaredNorm() > max_range_squared ) ) {
        continue;
      }
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
    }
  }
  count_last_processed_ += width * height;
  count_total_processed_ += width * height;
  if ( stamp > latest_stamp_ ) {
    latest_stamp_ = stamp;
    accumulated_cloud_.header.stamp = msg->header.stamp;
  }
  updated_ = true;
}

template<AggregationMode AGGREGATION_MODE>
void PointcloudAccumulator<AGGREGATION_MODE>::processPointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg )
{
  if ( !enabled_ ) {
    return;
  }

  if ( const std::string problem = layoutProblem( *msg ); !problem.empty() ) {
    RCLCPP_WARN_THROTTLE( logger_, *clock_, 5000, "Dropped a pointcloud with %s.",
                          problem.c_str() );
    return;
  }
  int32_t xi = findChannelIndex( *msg, "x" );
  int32_t yi = findChannelIndex( *msg, "y" );
  int32_t zi = findChannelIndex( *msg, "z" );
  if ( xi == -1 || yi == -1 || zi == -1 ) {
    RCLCPP_WARN( logger_, "No x, y, or z channel found in point cloud message!" );
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
    RCLCPP_ERROR_ONCE( logger_, "Unsupported data type %d for x, y, z fields.",
                       msg->fields[xi].datatype );
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
      RCLCPP_ERROR_ONCE( logger_, "Different data types for %s field are not supported.",
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
    // round() puts voxel i around i * resolution.
    const Eigen::Vector3f center = Eigen::Vector3f( index.x, index.y, index.z ) * resolution_;
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
  const Eigen::Vector3f center = Eigen::Vector3f( index.x, index.y, index.z ) * resolution_;
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
} // namespace hector_pointcloud_processing
