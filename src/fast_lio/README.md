# fast_lio

ROS 2 Humble FAST-LIO2 package with a shared FAST-LIO2 core and separate
LiDAR input adapters.

This package intentionally keeps the mapping, EKF, IMU undistortion, and
ikd-tree path common for every model. New LiDAR support should enter through
the preprocessing/subscription boundary, then hand the core algorithm a
`PointCloudXYZI` cloud whose `curvature` field stores per-point relative time
in milliseconds.

## Model Matrix

| Model key | `lidar_type` | Message consumed by `fastlio_mapping` | Required point fields | Config | Launch |
| --- | --- | --- | --- | --- | --- |
| `jt16` | `1` | `sensor_msgs/msg/PointCloud2` | `x y z intensity ring timestamp` | `config/jt16.yaml` | `mapping_jt16.launch.py` |
| `jt128` | `2` | `sensor_msgs/msg/PointCloud2` | `x y z intensity ring timestamp` | `config/jt128.yaml` | `mapping_jt128.launch.py` |
| `airy96` | `5` | `sensor_msgs/msg/PointCloud2` | `x y z intensity ring timestamp` preferred; falls back to `PointXYZI` with no per-point time | `config/airy96.yaml` | `mapping_airy96.launch.py` |
| `avia` | `10` | `livox_interfaces/msg/CustomMsg` | `x y z reflectivity tag line offset_time` | `config/avia.yaml` | `mapping_avia.launch.py` |
| `horizon` | `15` | `livox_interfaces/msg/CustomMsg` | `x y z reflectivity tag line offset_time` | `config/horizon.yaml` | `mapping_horizon.launch.py` |
| `mid360` | `11` | `livox_ros_driver2/msg/CustomMsg` | `x y z reflectivity tag line offset_time` | `config/mid360.yaml` | `mapping_mid360.launch.py` |
| `ouster64` | `14` | `sensor_msgs/msg/PointCloud2` | `x y z intensity t reflectivity ring ambient range` | `config/ouster64.yaml` | `mapping_ouster64.launch.py` |
| `velodyne` | `13` | `sensor_msgs/msg/PointCloud2` | `x y z intensity ring time` | `config/velodyne.yaml` | `mapping_velodyne.launch.py` |

Additional enum values exist in code for adapter reuse:
`6` (`airy96_break`) and `12` (`mid360_pointcloud2`). They do not have
first-class configs or launch files in this workspace.

## Build

The Livox message packages are build dependencies even if you only run
PointCloud2 data, because the executable contains the `avia`, `horizon`, and
`mid360` subscription paths.

```bash
cd /home/sax/FAST_LIO_ROS2
source /opt/ros/humble/setup.bash
source /home/sax/Livox_MID360/install/setup.bash
source /home/sax/Livox_AVIA_ROS2/install/setup.bash
colcon build --symlink-install --packages-select fast_lio
source install/setup.bash
```

## Run

```bash
ros2 launch fast_lio mapping_jt16.launch.py
ros2 launch fast_lio mapping_jt128.launch.py
ros2 launch fast_lio mapping_airy96.launch.py
ros2 launch fast_lio mapping_avia.launch.py
ros2 launch fast_lio mapping_horizon.launch.py
ros2 launch fast_lio mapping_mid360.launch.py
ros2 launch fast_lio mapping_ouster64.launch.py
ros2 launch fast_lio mapping_velodyne.launch.py
```

Common launch arguments:

| Argument | Default | Notes |
| --- | --- | --- |
| `use_sim_time` | `false` | Set `true` for simulated `/clock` |
| `rviz` | `true` | Set `false` on headless machines |
| `rviz_cfg` | package `rviz/fastlio.rviz` | RViz config path |
| `config_path` | package `config/` | Available in generic/Airy/Livox launch files |
| `config_file` | model-specific yaml | Available in generic/Airy/Livox launch files |

`mapping_airy96.launch.py` also exposes `map_file_path` as a launch argument.
Other models use the value in their yaml unless the parameter is overridden by
a custom launch file.

## Topics

| Model group | LiDAR topic parameter | IMU topic parameter | Transport |
| --- | --- | --- | --- |
| `jt16`/`jt128` | `common.lid_topic` | `common.imu_topic` | `PointCloud2` + `Imu` |
| `airy96` | `common.lid_topic` | `common.imu_topic` | `PointCloud2` + `Imu` |
| `avia`/`horizon` | `common.lid_topic` | `common.imu_topic` | `livox_interfaces/msg/CustomMsg` + `Imu` |
| `mid360` | `common.lid_topic` | `common.imu_topic` | `livox_ros_driver2/msg/CustomMsg` + `Imu` |
| `ouster64`/`velodyne` | `common.lid_topic` | `common.imu_topic` | `PointCloud2` + `Imu` |

The node publishes `/Odometry`, `/path`, `/cloud_registered`,
`/cloud_registered_body`, `/cloud_effected`, and `/Laser_map` depending on the
`publish.*` switches in the yaml.

## Configuration Rules

| Parameter | Meaning |
| --- | --- |
| `preprocess.lidar_type` | Selects the adapter in `src/preprocess.cpp` and, for Livox, the subscription type in `src/laserMapping.cpp` |
| `preprocess.scan_line` | Physical scan line count used by the adapter |
| `preprocess.timestamp_unit` | Unit of incoming per-point time: `0=s`, `1=ms`, `2=us`, `3=ns` |
| `point_filter_num` | Positive integer downsampling step used by the adapter before mapping |
| `common.imu_gyr_unit` | IMU angular velocity unit before normalization: `deg` or `rad` |
| `mapping.extrinsic_T/R` | LiDAR-to-IMU extrinsic seed |
| `mapping.extrinsic_est_en` | Online extrinsic estimation; keep `false` when calibrated extrinsics are trusted |
| `pcd_save.pcd_save_en` | Enables the `/map_save` service |

Livox `CustomMsg.offset_time` is nanoseconds. The Livox handler converts it to
milliseconds internally, which is what FAST-LIO2 expects in point curvature.

## Validation

Static yaml checks:

```bash
python3 tools/check_config.py --config config/jt16.yaml --model jt16 --ros 2
python3 tools/check_config.py --config config/jt128.yaml --model jt128 --ros 2
python3 tools/check_config.py --config config/airy96.yaml --model airy96 --ros 2
python3 tools/check_config.py --config config/avia.yaml --model avia --ros 2
python3 tools/check_config.py --config config/horizon.yaml --model horizon --ros 2
python3 tools/check_config.py --config config/mid360.yaml --model mid360 --ros 2
python3 tools/check_config.py --config config/ouster64.yaml --model ouster64 --ros 2
python3 tools/check_config.py --config config/velodyne.yaml --model velodyne --ros 2
```

Runtime input checks:

```bash
ros2 run fast_lio check_input.py --lidar_topic /hesai_front/lidar_points --imu_topic /hesai_front/lidar_imu --model jt128 --timestamp-unit 0
ros2 run fast_lio check_input.py --lidar_topic /rslidar_rear/points --imu_topic /rslidar_rear/imu_data --model airy96 --timestamp-unit 0
```

`check_input.py` validates `PointCloud2` streams only. It does not subscribe to
Livox `CustomMsg`; for `avia`, `horizon`, and `mid360`, use `ros2 topic info`,
`ros2 topic echo --once`, and `check_config.py`.

## Map Saving

Save a map while the node is running:

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

The service requires `pcd_save.pcd_save_en: true`. It expands `~`, resolves
relative paths from the `fast_lio` package root, creates the parent directory
when needed, flattens the current ikd-tree, and writes a binary PCD to
`map_file_path`.

## Adapter Boundary

Files that should change for a new LiDAR model:

| Need | File |
| --- | --- |
| New point struct, enum, or handler declaration | `src/preprocess.h` |
| Message-specific parsing and time extraction | `src/preprocess.cpp` |
| New ROS message transport type | `src/laserMapping.cpp` subscription setup |
| New default model parameters | `config/<model>.yaml` |
| New launch entry | `launch/mapping_<model>.launch.py` |
| Static validation expectations | `tools/check_config.py` |
| Operator documentation | this README and `doc/LIDAR_ADAPTERS.md` |

Do not fork the FAST-LIO2 core for a model-specific input format. The adapter
must normalize the input into the shared point cloud contract.
