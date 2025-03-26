// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "hector_pointcloud_io/pointcloud_io.hpp"

#include <pcl/io/auto_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hector_pointcloud_io
{
bool save_pointcloud( const std::string &path, const sensor_msgs::msg::PointCloud2 &pointcloud )
{
  pcl::PCLPointCloud2 pcl_pointcloud;
  pcl_conversions::toPCL( pointcloud, pcl_pointcloud );
  return pcl::io::save( path, pcl_pointcloud ) == 0;
}

sensor_msgs::msg::PointCloud2::SharedPtr load_pointcloud( const std::string &path )
{
  pcl::PCLPointCloud2 pcl_pointcloud;
  if ( pcl::io::load( path, pcl_pointcloud ) != 0 ) {
    return nullptr;
  }
  auto pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_conversions::fromPCL( pcl_pointcloud, *pointcloud );
  return pointcloud;
}
} // namespace hector_pointcloud_io
