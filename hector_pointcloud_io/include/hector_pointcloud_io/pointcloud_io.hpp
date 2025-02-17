//
// Created by stefan on 15.02.25.
//

#ifndef HECTOR_POINTCLOUD_SAVER_POINTCLOUD_IO_HPP
#define HECTOR_POINTCLOUD_SAVER_POINTCLOUD_IO_HPP

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace hector_pointcloud_io
{

/*!
 * @brief Saves a pointcloud to disk.
 * @param path The path to save the pointcloud to. The file extension determines the output format.
 *             Supported formats are given by PCL. Currently: ifs, pcd, ply, and vtk.
 * @param pointcloud The pointcloud to save.
 * @return True if the pointcloud was saved successfully, false otherwise.
 */
bool save_pointcloud( const std::string &path, const sensor_msgs::msg::PointCloud2 &pointcloud );

/*!
 * @brief Loads a pointcloud from disk.
 * @param path The path to load the pointcloud from. The file extension determines the input format.
 *             Supported formats are given by PCL. Currently: ifs, pcd, ply, and vtk.
 * @return The loaded pointcloud. If the pointcloud could not be loaded, a nullptr is returned.
 */
sensor_msgs::msg::PointCloud2::SharedPtr load_pointcloud( const std::string &path );

} // namespace hector_pointcloud_io

#endif // HECTOR_POINTCLOUD_SAVER_POINTCLOUD_IO_HPP
