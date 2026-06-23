# LiDAR Adapter Guide

This guide documents the boundary between model-specific LiDAR input handling
and the shared FAST-LIO2 core.

## Core Rule

Do not add model-specific branches in the FAST-LIO2 mapping, EKF, IMU
undistortion, or ikd-tree code. A LiDAR adapter must normalize its input before
the core sees it.

The normalized contract is:

```cpp
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
```

Required output fields:

| Field | Meaning |
| --- | --- |
| `x y z` | Point coordinates in the LiDAR frame |
| `intensity` | Reflectivity or intensity |
| `curvature` | Relative point time in milliseconds from the beginning of the scan or sub-scan |
| normal fields | Reset to zero unless the existing feature path fills them |

## Current Adapter Map

Operator-facing model keys are lowercase and match the config and launch file
names.

| Model key | `lidar_type` | Input transport | Handler |
| --- | --- | --- | --- |
| `jt16` | `1` | `PointCloud2` | `hesai_handler` |
| `jt128` | `2` | `PointCloud2` | `hesai_handler` |
| `airy96` | `5` | `PointCloud2` | `robosense_airy_handler` |
| `avia` | `10` | `livox_interfaces/msg/CustomMsg` | `livox_custom_handler` |
| `horizon` | `15` | `livox_interfaces/msg/CustomMsg` | `livox_custom_handler` |
| `mid360` | `11` | `livox_ros_driver2/msg/CustomMsg` | `livox_custom_handler` |
| `ouster64` | `14` | `PointCloud2` | `ouster64_handler` |
| `velodyne` | `13` | `PointCloud2` | `velodyne_handler` |

Reusable code paths without first-class launch files:

| Adapter key | `lidar_type` | Input transport | Handler |
| --- | --- | --- | --- |
| `airy96_break` | `6` | `PointCloud2` | `robosense_airy_handler` with sub-cloud splitting |
| `mid360_pointcloud2` | `12` | `PointCloud2` | `mid360_pointcloud2_handler` |

Only the lowercase model keys with configs and launch files are supported as
first-class operator entry points.

## Add A PointCloud2 LiDAR

1. Add an enum value in `src/preprocess.h`.
2. Define and register a PCL point type if the driver uses non-standard fields.
3. Declare a handler in `Preprocess`.
4. Implement the handler in `src/preprocess.cpp`.
5. Add a `case` in `Preprocess::process(const PointCloud2::UniquePtr&, ...)`.
6. Add `config/<model>.yaml` with `preprocess.lidar_type`, `scan_line`,
   `timestamp_unit`, topics, extrinsics, and publish/save switches.
7. Add `launch/mapping_<model>.launch.py`.
8. Add model expectations to `tools/check_config.py`.
9. Update `README.md` and this guide.

The handler must reject invalid ranges with `blind`, respect
`point_filter_num`, and keep `curvature` in milliseconds.

## Add A Non-PointCloud2 LiDAR

Use this path only when the native driver publishes a custom ROS message.

1. Add the message package to `package.xml` and `CMakeLists.txt`.
2. Add a subscription member and callback in `src/laserMapping.cpp`.
3. Route that `lidar_type` to the custom subscription in node initialization.
4. Add a `Preprocess::process(...)` overload for the message type.
5. Normalize the message to `PointCloudXYZI` in `src/preprocess.cpp`.
6. Add config, launch, validation, and documentation entries.

`avia`, `horizon`, and `mid360` are examples of this path. `avia` and
`horizon` subscribe to `livox_interfaces/msg/CustomMsg` from the local
`Livox_AVIA_ROS2` driver. `mid360` subscribes to
`livox_ros_driver2/msg/CustomMsg` from the local `Livox_MID360` driver. All
three parse `offset_time` as nanoseconds.

## Time Handling

FAST-LIO2 expects relative point time in milliseconds in `PointType.curvature`.

| Incoming unit | `preprocess.timestamp_unit` | Conversion to curvature |
| --- | --- | --- |
| seconds | `0` | `delta * 1000` |
| milliseconds | `1` | `delta` |
| microseconds | `2` | `delta * 0.001` |
| nanoseconds | `3` | `delta * 0.000001` |

For Livox `CustomMsg`, `offset_time` is nanoseconds relative to the packet
timebase. The handler writes `offset_time / 1e6` into `curvature`.

## Validation Checklist

Before calling an adapter finished:

```bash
python3 -m py_compile launch/mapping_<model>.launch.py tools/check_config.py
python3 tools/check_config.py --config config/<model>.yaml --model <model> --ros 2
source /opt/ros/humble/setup.bash
source /home/sax/Livox_MID360/install/setup.bash
source /home/sax/Livox_AVIA_ROS2/install/setup.bash
colcon build --symlink-install --packages-select fast_lio
ros2 launch fast_lio mapping_<model>.launch.py --show-args
```

For `PointCloud2` models, also run `check_input.py` against live or replayed
data. For custom-message models, verify the native message type with
`ros2 topic info -v` and inspect a sample with `ros2 topic echo --once`.
