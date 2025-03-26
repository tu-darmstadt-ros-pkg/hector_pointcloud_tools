# Hector PointCloud Tools

Some simple packages for common pointcloud needs.

## hector_pointcloud_accumulator

Voxel-filtered pointcloud accumulator. You can choose if it should store a running average, the highest z or the point closest to the center for each voxel.

## hector_pointcloud_io

Executables to load and publish a pointcloud from a file, and to save a pointcloud from a given topic to a file.
Includes a service server node that can save pointclouds from a topic on service call.

Supported formats: `ifs`, `pcd`, `ply`, `vtk`

### Examples

#### Load

```bash
ros2 run hector_pointcloud_io load_pointcloud /pointcloud_topic path_to_file.pcd 
```

#### Save

```bash
ros2 run hector_pointcloud_io save_pointcloud /pointcloud_topic path_to_file.ply
```

#### Service

```bash
ros2 run hector_pointcloud_io pointcloud_saver --ros-args -p topic:=/pointcloud_topic -p output_folder:=/path/to/save/folder -p output_format:=vtk -p output_filename_prefix:=my-pointcloud-prefix
```

All these parameters can be reconfigured.
When the service is called using the `std_srvs::srv::Trigger`, it will wait for a message on the given topic and save it to the given folder with the filename `<prefix>.<timestamp>.<output_format>`.
