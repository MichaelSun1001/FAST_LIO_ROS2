# Third-Party Notices

FAST_LIO_ROS2 is a derivative work based on third-party open-source software and
local LiDAR-specific integration work. This file records the main sources and
license boundaries.

## FAST_LIO / FAST-LIO2

**Source**: https://github.com/hku-mars/FAST_LIO  
**Authors**: Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, Fu Zhang — MARS Lab, HKU  
**License**: GPL-2.0  
**Derived files**: `src/laserMapping.cpp`, `src/preprocess.cpp`,
`src/preprocess.h`, `src/IMU_Processing.hpp`, `include/common_lib.h`,
`include/so3_math.h`, `include/use-ikfom.hpp`, `include/Exp_mat.h`,
`CMakeLists.txt`, `package.xml`

Reference:

> W. Xu, Y. Cai, D. He, J. Lin, and F. Zhang, "FAST-LIO2: Fast Direct
> LiDAR-Inertial Odometry," IEEE Transactions on Robotics, 2022.

The full GPL-2.0 license text is provided in `LICENSE`.

## LOAM

**Authors**: Ji Zhang, Carnegie Mellon University; further contributions
copyright (c) 2016, Southwest Research Institute  
**License**: BSD 3-Clause, as noted in `src/laserMapping.cpp`

Reference:

> J. Zhang and S. Singh, "LOAM: Lidar Odometry and Mapping in Real-time,"
> Robotics: Science and Systems Conference (RSS), Berkeley, CA, July 2014.

## Livox FAST-LIO Contributions

FAST-LIO contains historical Livox-oriented modifications in
`src/laserMapping.cpp`.

**Modifier**: Livox  
**License**: BSD notice retained in source

## ikd-Tree

**Source**: https://github.com/hku-mars/ikd-Tree  
**Authors**: Yixi Cai, Wei Xu, Fu Zhang — MARS Lab, HKU  
**Included as**: `include/ikd-Tree/`

Refer to `include/ikd-Tree/` for the current upstream license text.

Reference:

> Y. Cai, W. Xu, and F. Zhang, "ikd-Tree: An Incremental KD Tree for Robotic
> Applications," arXiv:2102.10808, 2021.

## IKFoM

**Source**: https://github.com/hku-mars/IKFoM  
**Author**: Dongjiao HE — The University of Hong Kong  
**License**: BSD 3-Clause  
**Included as**: `include/IKFoM_toolkit/`

## Local Integration Changes

Local source configs reviewed during the merge:

| Source config directory | Covered model keys in this workspace |
| --- | --- |
| `/home/sax/FAST_LIO_JT128_ROS2/src/FAST_LIO_Hesai/config/` | `jt16`, `jt128` |
| `/home/sax/FAST_LIO_Airy_ROS2/src/fast_lio_robosenseAiry/config/` | `airy96`, `avia`, `horizon`, `mid360`, `ouster64`, `velodyne` |
| `/home/sax/FAST_LIO_AVIA_Humble/src/FAST_LIO/config/` | `avia`, `horizon`, `mid360`, `ouster64`, `velodyne` |
| `/home/sax/FAST_LIO_MID360_Humble/src/FAST_LIO/config/` | `avia`, `horizon`, `mid360`, `ouster64`, `velodyne` |

The unified configs use one lowercase model key per first-class launch entry.
They intentionally do not preserve old fork-local `lidar_type` numbering when
that numbering conflicts with the shared adapter enum.

Files added or maintained for the unified multi-LiDAR workspace:

| File | Purpose |
| --- | --- |
| `config/jt16.yaml` | `jt16` config |
| `config/jt128.yaml` | `jt128` config |
| `config/airy96.yaml` | `airy96` config |
| `config/avia.yaml` | `avia` config |
| `config/horizon.yaml` | `horizon` config |
| `config/mid360.yaml` | `mid360` config |
| `config/ouster64.yaml` | `ouster64` config |
| `config/velodyne.yaml` | `velodyne` config |
| `launch/mapping_jt16.launch.py` | `jt16` launch entry |
| `launch/mapping_jt128.launch.py` | `jt128` launch entry |
| `launch/mapping_airy96.launch.py` | `airy96` launch entry |
| `launch/mapping_avia.launch.py` | `avia` launch entry |
| `launch/mapping_horizon.launch.py` | `horizon` launch entry |
| `launch/mapping_mid360.launch.py` | `mid360` launch entry |
| `launch/mapping_ouster64.launch.py` | `ouster64` launch entry |
| `launch/mapping_velodyne.launch.py` | `velodyne` launch entry |
| `tools/check_input.py` | Runtime PointCloud2/IMU validation |
| `tools/check_config.py` | Static config validation |
| `tools/check_map.py` | Map quality analysis |
| `tools/pcap_to_rosbag/*.sh` | Hesai PCAP to bag helpers |
| `doc/LIDAR_ADAPTERS.md` | Adapter development guide |
| `README.md` | Package documentation |

Substantially modified FAST-LIO2 files:

| File | Integration changes |
| --- | --- |
| `src/preprocess.h` | LiDAR adapter type declarations |
| `src/preprocess.cpp` | Device-specific parsing and per-point time normalization |
| `src/laserMapping.cpp` | ROS 2 node wiring, Livox `CustomMsg` subscription path, map save service, IMU gyro unit handling |

Model-specific adapter changes are released under GPL-2.0 where they modify
FAST-LIO2-derived files.
