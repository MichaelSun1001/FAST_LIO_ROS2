---
name: run-fastlio2-ros2-multilidar
description: >-
  Set up, validate, and run the unified FAST_LIO_ROS2 workspace on ROS 2
  Humble using lowercase model keys such as jt16, jt128, airy96, avia,
  horizon, mid360, ouster64, or velodyne.
---

# Run FAST-LIO2 Multi-LiDAR — ROS 2 Humble

Run commands from `/home/sax/FAST_LIO_ROS2` unless stated otherwise.

## Step 0: Choose the model

| Model key | Launch | Config | Runtime input |
| --- | --- | --- | --- |
| `jt16` | `mapping_jt16.launch.py` | `jt16.yaml` | `PointCloud2` + `Imu` |
| `jt128` | `mapping_jt128.launch.py` | `jt128.yaml` | `PointCloud2` + `Imu` |
| `airy96` | `mapping_airy96.launch.py` | `airy96.yaml` | `PointCloud2` + `Imu` |
| `avia` | `mapping_avia.launch.py` | `avia.yaml` | `livox_interfaces/msg/CustomMsg` + `Imu` |
| `horizon` | `mapping_horizon.launch.py` | `horizon.yaml` | `livox_interfaces/msg/CustomMsg` + `Imu` |
| `mid360` | `mapping_mid360.launch.py` | `mid360.yaml` | `livox_ros_driver2/msg/CustomMsg` + `Imu` |
| `ouster64` | `mapping_ouster64.launch.py` | `ouster64.yaml` | `PointCloud2` + `Imu` |
| `velodyne` | `mapping_velodyne.launch.py` | `velodyne.yaml` | `PointCloud2` + `Imu` |

Do not auto-detect Livox models with `check_input.py`; that tool subscribes to
PointCloud2 only. Use `ros2 topic info -v <topic>` to confirm Livox
`CustomMsg` streams.

## Step 1: Source and build

```bash
source /opt/ros/humble/setup.bash
source /home/sax/Livox_MID360/install/setup.bash
source /home/sax/Livox_AVIA_ROS2/install/setup.bash
colcon build --symlink-install --packages-select fast_lio
source install/setup.bash
```

Verify package visibility:

```bash
ros2 pkg executables fast_lio
```

## Step 2: Validate config

```bash
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/jt16.yaml --model jt16 --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/jt128.yaml --model jt128 --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/airy96.yaml --model airy96 --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/avia.yaml --model avia --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/horizon.yaml --model horizon --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/mid360.yaml --model mid360 --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/ouster64.yaml --model ouster64 --ros 2
python3 src/fast_lio/tools/check_config.py --config src/fast_lio/config/velodyne.yaml --model velodyne --ros 2
```

Resolve every FAIL before launching.

## Step 3: Validate input

For PointCloud2 models:

```bash
ros2 run fast_lio check_input.py --lidar_topic /hesai_front/lidar_points --imu_topic /hesai_front/lidar_imu --model jt128 --timestamp-unit 0
ros2 run fast_lio check_input.py --lidar_topic /rslidar_rear/points --imu_topic /rslidar_rear/imu_data --model airy96 --timestamp-unit 0
```

For Livox models:

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/custom_msg
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/custom_msg --once
```

Expected Livox message types:

```text
avia/horizon: livox_interfaces/msg/CustomMsg
mid360:       livox_ros_driver2/msg/CustomMsg
```

The FAST-LIO2 Livox adapter needs `points[].offset_time`, `line`, `tag`, and
reflectivity fields from the native driver stream.

## Step 4: Launch FAST-LIO2

```bash
ros2 launch fast_lio mapping_jt128.launch.py
ros2 launch fast_lio mapping_airy96.launch.py
ros2 launch fast_lio mapping_avia.launch.py
ros2 launch fast_lio mapping_horizon.launch.py
ros2 launch fast_lio mapping_mid360.launch.py
ros2 launch fast_lio mapping_ouster64.launch.py
ros2 launch fast_lio mapping_velodyne.launch.py
```

Use `rviz:=false` on headless machines.

## Step 5: Save and inspect a map

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
python3 src/fast_lio/tools/check_map.py --pcd src/fast_lio/PCD/jt128_map.pcd
```

Use the model's `map_file_path` when it differs from the example.

## Hesai PCAP path

The PCAP helper scripts are Hesai-only:

```bash
bash src/fast_lio/tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh \
  --model jt128 \
  --pcap /path/to/input.pcap \
  --correction /path/to/correction.csv \
  --firetime /path/to/firetime.csv \
  --output /path/to/output \
  --driver-ws ~/hesai_ros2_ws
```

After conversion, play the bag and launch the matching JT entry.
