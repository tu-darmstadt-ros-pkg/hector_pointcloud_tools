// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_POINTCLOUD_PROCESSING_VOXEL_COMMON_HPP
#define HECTOR_POINTCLOUD_PROCESSING_VOXEL_COMMON_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

//! Shared helpers for the voxel filter nodes: field handling and the first-point-per-voxel core.
//! Both VoxelFilter and DistanceAdaptiveVoxelFilter keep the first input point landing in each
//! voxel and copy it verbatim, so all fields are preserved without averaging.
namespace hector_pointcloud_processing::voxel_common
{

//! Byte offsets of the FLOAT32 x/y/z fields within each point, used to read positions.
struct XyzOffsets {
  uint32_t x;
  uint32_t y;
  uint32_t z;
};

//! Locates the x/y/z fields and returns their offsets. Nullopt if any is missing or not FLOAT32.
inline std::optional<XyzOffsets> findXyzOffsets( const sensor_msgs::msg::PointCloud2 &msg )
{
  using sensor_msgs::msg::PointField;
  const PointField *x = nullptr, *y = nullptr, *z = nullptr;
  for ( const auto &field : msg.fields ) {
    if ( field.name == "x" )
      x = &field;
    else if ( field.name == "y" )
      y = &field;
    else if ( field.name == "z" )
      z = &field;
  }
  if ( x == nullptr || y == nullptr || z == nullptr )
    return std::nullopt;
  if ( x->datatype != PointField::FLOAT32 || y->datatype != PointField::FLOAT32 ||
       z->datatype != PointField::FLOAT32 )
    return std::nullopt;
  return XyzOffsets{ x->offset, y->offset, z->offset };
}

inline float readFloat( const uint8_t *base, uint32_t offset )
{
  float value;
  std::memcpy( &value, base + offset, sizeof( float ) );
  return value;
}

//! Size in bytes of a single PointField (datatype size times element count).
inline uint32_t pointFieldSize( uint8_t datatype, uint32_t count )
{
  using sensor_msgs::msg::PointField;
  uint32_t element_size = 0;
  switch ( datatype ) {
  case PointField::INT8:
  case PointField::UINT8:
    element_size = 1;
    break;
  case PointField::INT16:
  case PointField::UINT16:
    element_size = 2;
    break;
  case PointField::INT32:
  case PointField::UINT32:
  case PointField::FLOAT32:
    element_size = 4;
    break;
  case PointField::FLOAT64:
    element_size = 8;
    break;
  default:
    return 0;
  }
  return element_size * std::max<uint32_t>( 1, count );
}

//! A contiguous byte range to copy from each input point to the output point.
struct FieldCopy {
  uint32_t src_offset;
  uint32_t size;
};

//! Output point layout derived from an input cloud and an optional keep-field whitelist.
struct OutputLayout {
  std::vector<sensor_msgs::msg::PointField> fields;
  std::vector<FieldCopy> copies; //!< Empty when all fields are kept (copy the whole point).
  uint32_t point_step = 0;
  bool keep_all = true;
};

//! Builds the output layout. Empty keep_fields keeps every field; blank names are ignored, so
//! [""] also keeps everything (rcl cannot express an empty string array parameter override).
//! Returns nullopt if keep_fields names fields but matches none of the input fields.
inline std::optional<OutputLayout> planOutputLayout( const sensor_msgs::msg::PointCloud2 &msg,
                                                     const std::vector<std::string> &keep_fields )
{
  std::unordered_set<std::string> keep;
  for ( const auto &name : keep_fields )
    if ( !name.empty() )
      keep.insert( name );

  OutputLayout layout;
  if ( keep.empty() ) {
    layout.keep_all = true;
    layout.fields = msg.fields;
    layout.point_step = msg.point_step;
    return layout;
  }
  layout.keep_all = false;
  for ( const auto &field : msg.fields ) {
    if ( keep.find( field.name ) == keep.end() )
      continue;
    const uint32_t size = pointFieldSize( field.datatype, field.count );
    sensor_msgs::msg::PointField out_field = field;
    out_field.offset = layout.point_step;
    layout.fields.push_back( out_field );
    layout.copies.push_back( { field.offset, size } );
    layout.point_step += size;
  }
  if ( layout.fields.empty() )
    return std::nullopt;
  return layout;
}

//! Prepends a tf prefix to a frame id: "prefix/frame". No-op if prefix or frame is empty; a
//! leading slash on the frame is dropped (tf2 frame ids are unslashed).
inline std::string applyTfPrefix( const std::string &prefix, const std::string &frame )
{
  if ( prefix.empty() || frame.empty() )
    return frame;
  const std::string stripped = frame.front() == '/' ? frame.substr( 1 ) : frame;
  return prefix + "/" + stripped;
}

//! Integer voxel coordinate, plus a band id so that adaptive grids with different voxel sizes do
//! not alias each other in the occupancy set (the uniform filter always uses band 0).
struct VoxelKey {
  int32_t x, y, z, band;

  bool operator==( const VoxelKey &o ) const
  {
    return x == o.x && y == o.y && z == o.z && band == o.band;
  }
};

struct VoxelKeyHash {
  std::size_t operator()( const VoxelKey &k ) const
  {
    auto mix = []( std::size_t h, int32_t v ) {
      return h ^ ( std::hash<int32_t>()( v ) + 0x9e3779b9 + ( h << 6 ) + ( h >> 2 ) );
    };
    return mix( mix( mix( std::hash<int32_t>()( k.x ), k.y ), k.z ), k.band );
  }
};

//! The voxel size (as its inverse, to multiply) and band id chosen for a point at a given distance.
//! The uniform filter always returns band 0; the adaptive filter picks a band from the distance
//! schedule so grids with different voxel sizes do not alias each other in the occupancy set.
struct VoxelChoice {
  double inv_voxel_size;
  int32_t band;
};

//! Floor division of a coordinate by the voxel size. inv_voxel_size = 1 / voxel_size.
inline int32_t voxelIndex( double coord, double inv_voxel_size )
{
  return static_cast<int32_t>( std::floor( coord * inv_voxel_size ) );
}

/*!
 * First-point-per-voxel filter shared by both voxel nodes.
 *
 * Iterates the input cloud once, drops non-finite points and points farther than
 * sqrt(max_distance_sq) from the cloud origin, and keeps the first point falling into each voxel.
 * The voxel size per point is chosen by choose_voxel(dist_sq), letting the uniform filter return a
 * constant and the adaptive filter pick a size from the distance bands.
 *
 * @param occupied Occupancy set, cleared on entry; passed in so callers can reuse its capacity.
 * @param transform Optional transform applied to each position before the distance check and the
 *   voxel binning; the copied point bytes are not modified, so the output stays in the input frame.
 * @return The filtered cloud (height 1, layout per `layout`).
 */
template<typename ChooseVoxelFn>
sensor_msgs::msg::PointCloud2
filterCloud( const sensor_msgs::msg::PointCloud2 &msg, const XyzOffsets &xyz,
             const OutputLayout &layout, double max_distance_sq,
             std::unordered_set<VoxelKey, VoxelKeyHash> &occupied, ChooseVoxelFn choose_voxel,
             const Eigen::Isometry3d *transform = nullptr )
{
  occupied.clear();

  const size_t input_size = static_cast<size_t>( msg.height ) * msg.width;

  sensor_msgs::msg::PointCloud2 output;
  output.header = msg.header;
  output.height = 1;
  output.is_bigendian = msg.is_bigendian;
  output.is_dense = true; //!< Non-finite points are dropped below, so the output never has any.
  output.point_step = layout.point_step;
  output.fields = layout.fields;
  output.data.clear();
  // At most every input point survives; reserving the worst case avoids repeated reallocations.
  output.data.reserve( input_size * layout.point_step );

  const uint8_t *src = msg.data.data();
  const size_t point_step = msg.point_step;

  for ( size_t point = 0; point < input_size; ++point ) {
    const uint8_t *sp = src + point * point_step;
    const float x = readFloat( sp, xyz.x );
    const float y = readFloat( sp, xyz.y );
    const float z = readFloat( sp, xyz.z );
    if ( !std::isfinite( x ) || !std::isfinite( y ) || !std::isfinite( z ) )
      continue;

    Eigen::Vector3d position( x, y, z );
    if ( transform != nullptr )
      position = *transform * position;

    const double dist_sq = position.squaredNorm();
    if ( dist_sq > max_distance_sq )
      continue;

    const VoxelChoice choice = choose_voxel( dist_sq );
    const VoxelKey key{ voxelIndex( position.x(), choice.inv_voxel_size ),
                        voxelIndex( position.y(), choice.inv_voxel_size ),
                        voxelIndex( position.z(), choice.inv_voxel_size ), choice.band };
    if ( !occupied.insert( key ).second )
      continue;

    if ( layout.keep_all ) {
      output.data.insert( output.data.end(), sp, sp + point_step );
    } else {
      for ( const auto &copy : layout.copies )
        output.data.insert( output.data.end(), sp + copy.src_offset,
                            sp + copy.src_offset + copy.size );
    }
  }

  output.row_step = output.data.size();
  output.width = layout.point_step == 0 ? 0 : output.row_step / layout.point_step;
  return output;
}

} // namespace hector_pointcloud_processing::voxel_common

#endif // HECTOR_POINTCLOUD_PROCESSING_VOXEL_COMMON_HPP
