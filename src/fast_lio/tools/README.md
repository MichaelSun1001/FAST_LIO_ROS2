# FAST_LIO_ROS2 Tools

Helper tools for checking configs, validating runtime inputs, analyzing output
maps, and preparing Hesai PCAP data before running FAST-LIO2.

Run commands from `src/fast_lio` when using relative paths, or use the installed
executables through `ros2 run fast_lio ...` after building the workspace.

## Support Summary

| Tool | `jt16`/`jt128` | `airy96` | Livox/Ouster/Velodyne | Notes |
| --- | --- | --- | --- | --- |
| `check_config.py` | Yes | Yes | Yes | Static yaml validation |
| `check_input.py` | Yes | Yes | PointCloud2 only | Does not subscribe to Livox `CustomMsg`; model-specific checks are tuned for JT/Airy |
| `check_map.py` | Yes | Yes | Yes | Works on saved PCD or `/Laser_map` |
| `pcap_to_rosbag/*` | Hesai only | No | No | Wraps Hesai ROS Driver PCAP playback |

## `check_config.py`

Static yaml validator. It does not require ROS to be running.

Checks:

| Item | Failure means |
| --- | --- |
| `preprocess.lidar_type` matches model and ROS version | Wrong adapter selected |
| `preprocess.scan_line` matches model | Wrong line count |
| `preprocess.timestamp_unit` is `0..3` | Per-point time conversion may be wrong |
| `common.imu_gyr_unit` is `deg` or `rad` | IMU angular velocity may be scaled wrong |
| `point_filter_num` is a positive integer | Adapter downsampling may divide by zero or drop all points |
| `preprocess.blind` is positive and below `mapping.det_range` | Points may be over-filtered |
| `mapping.extrinsic_R/T` are well-formed | Bad LiDAR-IMU extrinsic seed |
| `pcd_save.pcd_save_en` and `map_file_path` are coherent | Map save may fail |

Examples:

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

Options:

| Option | Required | Values |
| --- | --- | --- |
| `--config` | Yes | Path to yaml |
| `--model` | Yes | `jt16`, `jt128`, `airy96`, `avia`, `horizon`, `mid360`, `ouster64`, `velodyne` |
| `--ros` | Yes | `1` or `2` |

## `check_input.py`

Runtime validator for `sensor_msgs/msg/PointCloud2` plus
`sensor_msgs/msg/Imu`.

Use it for `jt16`, `jt128`, and `airy96` streams. It checks topic presence,
PointCloud2 fields, ring range, per-point timestamp behavior, frame drops, IMU
frequency, gyro unit sanity, frame IDs, and LiDAR/IMU clock alignment.

Examples:

```bash
ros2 run fast_lio check_input.py --lidar_topic /hesai_front/lidar_points --imu_topic /hesai_front/lidar_imu --model jt128 --timestamp-unit 0
ros2 run fast_lio check_input.py --lidar_topic /rslidar_rear/points --imu_topic /rslidar_rear/imu_data --model airy96 --timestamp-unit 0

# Standalone from src/fast_lio after ROS is sourced
python3 tools/check_input.py --lidar_topic /lidar_points --imu_topic /lidar_imu --model auto
```

Options:

| Option | Default | Values |
| --- | --- | --- |
| `--lidar_topic` | `/lidar_points` | PointCloud2 topic |
| `--imu_topic` | `/lidar_imu` | IMU topic |
| `--model` | `auto` | `jt16`, `airy96`, `jt128`, `auto` |
| `--timeout` | `8.0` | Seconds |
| `--timestamp-unit` | unset | `0=s`, `1=ms`, `2=us`, `3=ns` |

Limitations:

- It does not subscribe to Livox `CustomMsg`.
- For `avia`, `horizon`, and `mid360` direct input, use `ros2 topic info -v <topic>` and
  `ros2 topic echo --once <topic>` to verify the native driver message type and
  fields.

## `check_map.py`

Map quality analyzer for saved PCD files or the live `/Laser_map` topic.

Examples:

```bash
python3 tools/check_map.py --pcd PCD/jt128_map.pcd
python3 tools/check_map.py --pcd PCD/mid360_map.pcd --model mid360
ros2 run fast_lio check_map.py --map-topic /Laser_map --odom-topic /Odometry
```

Metrics:

| Metric | High value indicates |
| --- | --- |
| Surface thickness | Ghosting, double walls, bad sync, bad extrinsics |
| Point count / density | Sparse map, excessive filtering, dropped frames |
| Trajectory Z drift | Poor initialization, degeneracy, IMU/extrinsic issues |

Dependencies: `numpy` is required. `open3d` is optional and only accelerates PCD
loading.

## `pcap_to_rosbag`

Hesai-only scripts for converting PCAP playback into a rosbag by temporarily
driving the Hesai ROS Driver in PCAP mode.

ROS 2:

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh \
  --model jt128 \
  --pcap /data/input.pcap \
  --correction /data/correction.csv \
  --firetime /data/firetime.csv \
  --output /data/output \
  --driver-ws ~/hesai_ros2_ws
```

ROS 1:

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros1.sh \
  --model jt128 \
  --pcap /data/input.pcap \
  --correction /data/correction.csv \
  --firetime /data/firetime.csv \
  --output /data/output.bag \
  --driver-ws ~/hesai_ros_ws
```

The scripts require a driver config layout compatible with the local Hesai ROS
Driver and expect the output topics to contain both LiDAR and IMU data.
