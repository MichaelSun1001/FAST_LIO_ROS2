# FAST_LIO_ROS2

这是一个 ROS 2 Humble 下的 FAST-LIO2 统一工程，用同一套
FAST-LIO2 核心算法支持多种 LiDAR 输入。

本工程的原则是：FAST-LIO2 的建图、EKF、IMU 去畸变和 ikd-tree 路径
保持共享基准；不同 LiDAR 的差异只放在订阅和预处理适配层，最后统一
转换成 FAST-LIO2 使用的 `PointCloudXYZI`。

## 工程内容

| 包 | 作用 |
| --- | --- |
| `fast_lio` | FAST-LIO2 建图节点、LiDAR 配置、launch 文件和检查工具 |

## 当前支持的 LiDAR

| 型号名 | `lidar_type` | 默认输入消息 | 默认 LiDAR topic | 配置文件 | launch 文件 |
| --- | --- | --- | --- | --- | --- |
| `jt16` | `1` | `sensor_msgs/msg/PointCloud2` | `/lidar_points` | `jt16.yaml` | `mapping_jt16.launch.py` |
| `jt128` | `2` | `sensor_msgs/msg/PointCloud2` | `/hesai_front/lidar_points` | `jt128.yaml` | `mapping_jt128.launch.py` |
| `airy96` | `5` | `sensor_msgs/msg/PointCloud2` | `/rslidar_rear/points` | `airy96.yaml` | `mapping_airy96.launch.py` |
| `avia` | `10` | `livox_interfaces/msg/CustomMsg` | `/livox/lidar` | `avia.yaml` | `mapping_avia.launch.py` |
| `horizon` | `15` | `livox_interfaces/msg/CustomMsg` | `/livox/lidar` | `horizon.yaml` | `mapping_horizon.launch.py` |
| `mid360` | `11` | `livox_ros_driver2/msg/CustomMsg` | `/livox/custom_msg` | `mid360.yaml` | `mapping_mid360.launch.py` |
| `ouster64` | `14` | `sensor_msgs/msg/PointCloud2` | `/os_cloud_node/points` | `ouster64.yaml` | `mapping_ouster64.launch.py` |
| `velodyne` | `13` | `sensor_msgs/msg/PointCloud2` | `/velodyne_points` | `velodyne.yaml` | `mapping_velodyne.launch.py` |

`airy96` 当前默认使用 GigaAI rear Airy 配置；`airy96_rear` 不再作为单独
型号名维护。`mapping.launch.py` 是通用入口，默认使用 `jt128.yaml`；日常
使用优先选择上表中的型号专用 launch。`gdb_debug_example.launch` 只作为
调试示例，不是 LiDAR 型号入口。

## 依赖

基础环境：

- ROS 2 Humble
- `colcon`
- `rviz2`
- PCL / Eigen 相关 ROS 依赖
- Python 3 和 `PyYAML`，用于运行配置检查工具

本工程直接订阅 Livox 原生 `CustomMsg`，因此编译和运行时需要能找到下面
两个消息包：

| 消息包 | 本机来源 | 用途 |
| --- | --- | --- |
| `livox_interfaces` | `/home/sax/Livox_AVIA_ROS2` | `avia`、`horizon` 的 `CustomMsg` |
| `livox_ros_driver2` | `/home/sax/Livox_MID360` | `mid360` 的 `CustomMsg` |

这两个 Livox 工程是外部驱动/消息来源，不属于本 FAST-LIO2 工程。本工程
只保留 FAST-LIO2 节点、配置、launch 和检查工具。

本机如果安装了 MVS 等 SDK，可能会把 `/opt/MVS/lib/64` 放到
`LD_LIBRARY_PATH` 前面，导致 PCL 运行时误加载旧 `libusb`。本工程的
FAST-LIO launch 会优先使用系统 `/lib/x86_64-linux-gnu` 和
`/usr/lib/x86_64-linux-gnu`，避免 `libpcl_io.so` 启动时报
`undefined symbol: libusb_set_option`。

## 编译

如果 Livox 消息包不是系统级安装，需要先 source 对应驱动工作空间：

```bash
cd /home/sax/FAST_LIO_ROS2
source /opt/ros/humble/setup.bash
source /home/sax/Livox_MID360/install/setup.bash
source /home/sax/Livox_AVIA_ROS2/install/setup.bash
colcon build --symlink-install --packages-select fast_lio
```

编译完成后：

```bash
source /home/sax/FAST_LIO_ROS2/install/setup.bash
ros2 pkg executables fast_lio
```

## 运行

建议按下面顺序 source：

```bash
source /opt/ros/humble/setup.bash
source /home/sax/Livox_MID360/install/setup.bash
source /home/sax/Livox_AVIA_ROS2/install/setup.bash
source /home/sax/FAST_LIO_ROS2/install/setup.bash
```

选择对应型号的 launch：

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

无图形界面时关闭 RViz：

```bash
ros2 launch fast_lio mapping_mid360.launch.py rviz:=false
```

## 配置检查

所有正式支持型号都应通过静态配置检查：

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

`check_input.py` 目前只检查 `PointCloud2` 输入，适用于 `jt16`、`jt128`
和 `airy96`。`avia`、`horizon`、`mid360` 使用 Livox 原生 `CustomMsg`，
应使用下面命令确认 topic 类型和字段：

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/custom_msg
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/custom_msg --once
```

## 保存地图

每个配置文件都有 `map_file_path` 和 `pcd_save.pcd_save_en`。节点运行时可用
下面的服务保存当前 ikd-tree 地图：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

相对路径会按 `fast_lio` 包根目录解析；`~` 和绝对路径会在保存前展开。

## 后续新增 LiDAR 型号

新增型号时不要改 FAST-LIO2 的建图、EKF、IMU 去畸变和 ikd-tree 核心路径。
正确做法是新增一个 LiDAR adapter，把原始点云或自定义消息转换成
`PointCloudXYZI`，其中 `curvature` 表示点相对当前 scan 起点的毫秒级时间。

最少需要同步修改：

| 内容 | 文件 |
| --- | --- |
| 新增 `lidar_type`、点类型或 handler 声明 | `src/fast_lio/src/preprocess.h` |
| 实现点云字段解析、时间单位转换、盲区过滤 | `src/fast_lio/src/preprocess.cpp` |
| 需要自定义 ROS 消息时，新增订阅分发 | `src/fast_lio/src/laserMapping.cpp` |
| 新型号默认参数 | `src/fast_lio/config/<model>.yaml` |
| 新型号启动入口 | `src/fast_lio/launch/mapping_<model>.launch.py` |
| 配置静态检查 | `src/fast_lio/tools/check_config.py` |
| 文档支持矩阵和验证说明 | `README.md`、`src/fast_lio/README.md`、`src/fast_lio/doc/LIDAR_ADAPTERS.md` |

新增后至少要跑：配置检查、launch `--show-args`、节点短启动、`colcon build`
和 `colcon test`。有真实设备或 rosbag 时，还应使用 `check_input.py` 或
`ros2 topic info -v` 验证输入 topic 类型和字段。

## 相关文档

| 文件 | 作用 |
| --- | --- |
| `src/fast_lio/README.md` | 包级运行和配置说明 |
| `src/fast_lio/doc/LIDAR_ADAPTERS.md` | 新增 LiDAR 适配的开发说明 |
| `src/fast_lio/tools/README.md` | 检查工具说明 |
| `src/fast_lio/THIRD_PARTY_NOTICES.md` | 来源和许可证说明 |
