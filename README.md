# Hector PointCloud Tools

Some simple packages for common pointcloud needs

## hector_pointcloud_processing

Contains nodes for accumulation and decimating of point clouds.

### `pointcloud_accumulator`

Voxel-filtered pointcloud accumulator. You can choose if it should store a running average, the highest z or the point closest to the center for each voxel.

Example:
```bash
ros2 launch hector_pointcloud_accumulator pointcloud_accumulator.launch.yaml resolution:=0.025 frame:=map publish_rate:=0.2 aggregation_mode:=average topics:=["pointcloud1","pointcloud2"] output_topic:=accumulated_pointcloud
```

#### Subscribed Topics

| Topic         | Type                          | Description                                                          |
| ------------- | ----------------------------- | -------------------------------------------------------------------- |
| `/pointcloud` | `sensor_msgs/msg/PointCloud2` | Default input topic for pointclouds if parameter topics is not used. |

#### Published Topics

| Topic                     | Type                          | Description                             |
| ------------------------- | ----------------------------- | --------------------------------------- |
| `/accumulated_pointcloud` | `sensor_msgs/msg/PointCloud2` | Output topic for accumulated pointcloud |

#### Services

| Service              | Type                   | Description                          |
| -------------------- | ---------------------- | ------------------------------------ |
| `/enable_pointcloud` | `std_srvs/srv/SetBool` | Turn pointcloud processing on or off |
| `/reset_pointcloud`  | `std_srvs/srv/Trigger` | Clear all accumulated data           |

#### Parameters

| Parameter          | Type           | Default                    | Description                                                                                                                     |
| ------------------ | -------------- | -------------------------- | ------------------------------------------------------------------------------------------------------------------------------- |
| `resolution`       | `double`       | `0.025`                    | Resolution of the voxel filter                                                                                                  |
| `frame`            | `string`       | `"map"`                    | The frame into which all pointclouds are accumulated                                                                            |
| `publish_rate`     | `double`       | `0.2`                      | Frequency at which the node publishes on `/cloud_out`                                                                           |
| `aggregation_mode` | `string`       | `"average"`                | Controls how points within the same voxel are integrated. Allowed values are: `"average"`, `"highest_z"`, `"closest_to_center"` |
| `topics`           | `string_array` | `["pointcloud"]`           | The topics names of all topics that should be accumulated together                                                              |
| `output_topic`     | `string`       | `"accumulated_pointcloud"` | The topic name of the accumulated pointcloud                                                                                    |

### `pointcloud_decimator`

This node reduces the pount count of a pointcloud and publishes them in a compressed format via [point_cloud_transport](https://github.com/ros-perception/point_cloud_transport).
The desired ammount of points can be specified as either a fraction of the original point count or as a total number of points.
There are currently two methods to decimate a pointcloud:\
`random` picks points randomly from the pointcloud.
This method should prevent noticeable patterns in the pointcloud, but will likely not match the desired point count/fraction precisely.\
`count` picks points equally from the whole range of the pointcloud. This method should is the more performant and will match the desired point count/fraction.

#### Subscribed Topics

| Topic         | Type                          | Description                 |
| ------------- | ----------------------------- | --------------------------- |
| `/pointcloud` | `sensor_msgs/msg/PointCloud2` | Input topic for pointclouds |


#### Published Topics

| Topic                   | Type                    | Description                            |
| ----------------------- | ----------------------- | -------------------------------------- |
| `/pointcloud_decimated` | `point_cloud_transport` | Output topic for decimated pointclouds |

#### Services

**None**

#### Actions

**None**

#### Parameters

| Parameter                | Type          | Default               | Description                                                                |
| ------------------------ | ------------- | --------------------- | -------------------------------------------------------------------------- |
| `elimination_method`     | `std::string` | `"count"`             | How the pointcloud is decimated (`random`/`count`)                         |
| `elimination_quantifier` | `std::string` | `"fraction"`          | How the amount of points to be kept is quantified (`fraction`/`count`)     |
| `point_fraction`         | `double`      | `0.1`                 | Fraction of points to keep (used if `elimination_quantifier = "fraction"`) |
| `point_count`            | `int`         | `1000`                | Number of points to keep (used if `elimination_quantifier = "count"`)      |

## hector_pointcloud_io

Executables to load and publish a pointcloud from a file, and to save a pointcloud from a given topic to a file.
Includes the service server node `pointcloud_saver` that can save pointclouds from a topic on service call.

Supported formats: `ifs`, `pcd`, `ply`, `vtk`

### `load_pointcloud`

Continuously publishes the selected pointcloud on the selected topic.

Example:
```bash
ros2 run hector_pointcloud_io load_pointcloud /pointcloud_topic path_to_file.pcd
```

### `save_pointcloud`

Stores the first pointcloud received on the selected topic.

Example:
```bash
ros2 run hector_pointcloud_io save_pointcloud /pointcloud_topic path_to_file.ply
```

### `pointcloud_saver`

#### Subscribed Topics

| Topic     | Type                          | Description                                |
| --------- | ----------------------------- | ------------------------------------------ |
| `<topic>` | `sensor_msgs/msg/PointCloud2` | The topic from where pointclouds are saved |

#### Services

| Service             | Type                   | Description                                                                                             |
| ------------------- | ---------------------- | ------------------------------------------------------------------------------------------------------- |
| `~/save_pointcloud` | `std_srvs/srv/Trigger` | Saves the first pointcloud received on \<topic> within <timeout_ms> milliseconds after the service call |

#### Parameters

| Parameter                | Type     | Description                                                                       |
| ------------------------ | -------- | --------------------------------------------------------------------------------- |
| `topic`                  | `string` | Topic to listen for pointclouds on                                                |
| `ouput_folder`           | `string` | Folder to save pointclouds to                                                     |
| `output_format`          | `string` | Output format of pointclouds. Supported: `pcd`, `ifs`, `ply`, `vtk`               |
| `output_filename_prefix` | `string` | Prefix for output filenames. Name will be \<prefix>.\<timestamp>.\<output_format> |
| `timeout_ms`             | `int`    | Timeout for waiting for pointclouds in ms.                                        |

Example:
```bash
ros2 run hector_pointcloud_io pointcloud_saver --ros-args -p topic:=/pointcloud_topic -p output_folder:=/path/to/save/folder -p output_format:=vtk -p output_filename_prefix:=my-pointcloud-prefix
```

All these parameters can be reconfigured.
When the service is called using the `std_srvs::srv::Trigger`, it will wait for a message on the given topic and save it to the given folder with the filename `<prefix>.<timestamp>.<output_format>`.
