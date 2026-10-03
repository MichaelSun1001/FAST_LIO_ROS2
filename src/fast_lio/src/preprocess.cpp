// Copyright 2026 Hesai Technology. All rights reserved.
// SPDX-License-Identifier: GPL-2.0
//
// This file is part of FAST_LIO_ROS2, a fork of FAST_LIO
// (https://github.com/hku-mars/FAST_LIO) by the MARS Lab, HKU.
// The upstream preprocess code has been substantially rewritten for
// jt16 / jt128 LiDARs.  Original upstream structure is retained
// where applicable; original copyright notices are preserved in the
// accompanying LICENSE file.
//
// Modified by Hesai Technology, 2026-06:
//   - Added hesai_handler() for jt16 / jt128 PointCloud2 parsing
//   - Added per-point ring + timestamp extraction and frame accumulation
//
// Consolidated for multi-LiDAR FAST-LIO2, 2026-06:
//   - Added airy96 PointCloud2 parsing as a separate handler
//   - Added avia / horizon / mid360 CustomMsg parsing as a separate handler

#include "preprocess.h"

#include <algorithm>
#include <iostream>
#include <pcl/common/common.h>

namespace
{
bool is_livox_custom_lidar(int lidar_type)
{
  return lidar_type == avia || lidar_type == mid360 || lidar_type == horizon;
}

bool has_point_field(const sensor_msgs::msg::PointCloud2 &msg, const std::string &field_name)
{
  return std::any_of(msg.fields.begin(), msg.fields.end(),
                     [&field_name](const sensor_msgs::msg::PointField &field) {
                       return field.name == field_name;
                     });
}

void reset_point(PointType &point)
{
  point.normal_x = 0;
  point.normal_y = 0;
  point.normal_z = 0;
}
}  // namespace

Preprocess::Preprocess() : feature_enabled(0), lidar_type(jt16), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 16;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disA = 0.1;  // B?
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess()
{
}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

#ifdef FAST_LIO_HAS_LIVOX
void Preprocess::process(const livox_interfaces::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out)
{
  livox_custom_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out)
{
  livox_custom_handler(msg);
  *pcl_out = pl_surf;
}
#endif

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
    case jt16:
      hesai_handler(msg);
      break;
    case jt128:
      hesai_handler(msg);
      break;

    case airy96:
    case airy96_break:
    {
      double start_time = rclcpp::Time(msg->header.stamp).seconds();
      double end_time = start_time;
      robosense_airy_handler(msg, 0, 1, start_time, end_time);
      break;
    }

    case mid360_pointcloud2:
      mid360_pointcloud2_handler(msg);
      break;

    case ouster64:
      ouster64_handler(msg);
      break;

    case velodyne:
      velodyne_handler(msg);
      break;

    default:
      default_handler(msg);
      break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out,
                         int i_sub_cloud, int num_sub_cloud, double &start_time, double &end_time)
{
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
    case airy96:
    case airy96_break:
      robosense_airy_handler(msg, i_sub_cloud, num_sub_cloud, start_time, end_time);
      break;

    case jt16:
    case jt128:
      hesai_handler(msg);
      start_time = rclcpp::Time(msg->header.stamp).seconds();
      end_time = start_time;
      break;

    default:
      std::cerr << "Error LiDAR Type: " << lidar_type << std::endl;
      break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::hesai_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<hesai_point_type::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  const int filter_step = std::max(1, point_filter_num);

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < (uint)plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      reset_point(added_pt);
      added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * time_unit_scale;
      if (pl_orig.points[i].ring < (uint16_t)N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2)
      {
        continue;
      }
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < (uint)linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < (int)pl_orig.points.size(); i++)
    {
      if (i % filter_step != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                     pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < (blind * blind)) continue;

      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      reset_point(added_pt);
      added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * time_unit_scale;

      pl_surf.points.push_back(added_pt);
    }
  }
}

#ifdef FAST_LIO_HAS_LIVOX
template <typename LivoxCustomMsgUniquePtr>
void Preprocess::livox_custom_handler_impl(const LivoxCustomMsgUniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  const int plsize = std::min(static_cast<int>(msg->point_num), static_cast<int>(msg->points.size()));
  if (plsize <= 0)
  {
    return;
  }

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }

  uint valid_num = 0;
  if (feature_enabled)
  {
    for (int i = 1; i < plsize; i++)
    {
      const auto &src_pt = msg->points[i];
      if (src_pt.line >= N_SCANS)
      {
        continue;
      }
      if ((src_pt.tag & 0x30) != 0x10 && (src_pt.tag & 0x30) != 0x00)
      {
        continue;
      }

      pl_full[i].x = src_pt.x;
      pl_full[i].y = src_pt.y;
      pl_full[i].z = src_pt.z;
      pl_full[i].intensity = src_pt.reflectivity;
      reset_point(pl_full[i]);
      pl_full[i].curvature = src_pt.offset_time / 1000000.0f;

      if ((std::abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
          (std::abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
          (std::abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
      {
        pl_buff[src_pt.line].push_back(pl_full[i]);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = static_cast<int>(pl.size());
      if (linesize <= 5)
      {
        continue;
      }

      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (int i = 0; i < linesize; i++)
      {
        types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = std::sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[linesize].range = std::sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    const int filter_step = std::max(1, point_filter_num);
    for (int i = 1; i < plsize; i++)
    {
      const auto &src_pt = msg->points[i];
      if (src_pt.line >= N_SCANS)
      {
        continue;
      }
      if ((src_pt.tag & 0x30) != 0x10 && (src_pt.tag & 0x30) != 0x00)
      {
        continue;
      }

      valid_num++;
      if (valid_num % filter_step != 0)
      {
        continue;
      }

      PointType added_pt;
      added_pt.x = src_pt.x;
      added_pt.y = src_pt.y;
      added_pt.z = src_pt.z;
      added_pt.intensity = src_pt.reflectivity;
      reset_point(added_pt);
      added_pt.curvature = src_pt.offset_time / 1000000.0f;

      const bool is_not_duplicate =
          (std::abs(added_pt.x - pl_full[i - 1].x) > 1e-7) ||
          (std::abs(added_pt.y - pl_full[i - 1].y) > 1e-7) ||
          (std::abs(added_pt.z - pl_full[i - 1].z) > 1e-7);
      if (is_not_duplicate &&
          added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind * blind)
      {
        pl_surf.push_back(added_pt);
      }

      pl_full[i] = added_pt;
    }
  }
}

void Preprocess::livox_custom_handler(const livox_interfaces::msg::CustomMsg::UniquePtr &msg)
{
  livox_custom_handler_impl(msg);
}

void Preprocess::livox_custom_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg)
{
  livox_custom_handler_impl(msg);
}
#endif

void Preprocess::robosense_airy_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg,
                                        int i_sub_cloud, int num_sub_cloud,
                                        double &start_time, double &end_time)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  const double msg_time = rclcpp::Time(msg->header.stamp).seconds();
  start_time = msg_time;
  end_time = msg_time;

  const bool has_ring = has_point_field(*msg, "ring");
  const bool has_timestamp = has_point_field(*msg, "timestamp");
  const int filter_step = std::max(1, point_filter_num);

  if (!has_ring || !has_timestamp)
  {
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.size());
    if (plsize == 0)
    {
      return;
    }

    pl_surf.reserve(plsize / filter_step + 1);
    if (feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize / std::max(1, N_SCANS) + 1);
      }
    }

    const int points_per_scan = std::max(1, plsize / std::max(1, N_SCANS));
    for (int i = 0; i < plsize; i++)
    {
      if (i % filter_step != 0)
      {
        continue;
      }

      const double range = std::sqrt(pl_orig.points[i].x * pl_orig.points[i].x +
                                     pl_orig.points[i].y * pl_orig.points[i].y +
                                     pl_orig.points[i].z * pl_orig.points[i].z);
      if (range >= 150.0 || range <= blind)
      {
        continue;
      }

      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      reset_point(added_pt);
      added_pt.curvature = 0.0;

      if (feature_enabled)
      {
        const int estimated_ring = (i / points_per_scan) % std::max(1, N_SCANS);
        pl_buff[estimated_ring].push_back(added_pt);
      }
      else
      {
        pl_surf.push_back(added_pt);
      }
    }

    if (feature_enabled)
    {
      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = static_cast<int>(pl.size());
        if (linesize < 2)
        {
          continue;
        }

        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (int i = 0; i < linesize; i++)
        {
          types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = std::sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }
    }
    return;
  }

  pcl::PointCloud<robosense_airy_point_type::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const int plsize = static_cast<int>(pl_orig.size());
  if (plsize == 0)
  {
    return;
  }

  const int safe_num_sub_cloud = std::max(1, num_sub_cloud);
  const int safe_sub_cloud = std::min(std::max(0, i_sub_cloud), safe_num_sub_cloud - 1);
  const int slice_begin = plsize * safe_sub_cloud / safe_num_sub_cloud;
  const int slice_end = plsize * (safe_sub_cloud + 1) / safe_num_sub_cloud;
  if (slice_begin >= slice_end)
  {
    return;
  }

  start_time = pl_orig.points[slice_begin].timestamp;
  end_time = pl_orig.points[slice_end - 1].timestamp;
  pl_surf.reserve((slice_end - slice_begin) / filter_step + 1);

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve((slice_end - slice_begin) / std::max(1, N_SCANS) + 1);
    }
  }

  for (int i = slice_begin; i < slice_end; i++)
  {
    if ((i - slice_begin) % filter_step != 0)
    {
      continue;
    }

    const auto &src_pt = pl_orig.points[i];
    const double range = std::sqrt(src_pt.x * src_pt.x + src_pt.y * src_pt.y + src_pt.z * src_pt.z);
    if (range >= 150.0 || range <= blind)
    {
      continue;
    }

    PointType added_pt;
    added_pt.x = src_pt.x;
    added_pt.y = src_pt.y;
    added_pt.z = src_pt.z;
    added_pt.intensity = src_pt.intensity;
    reset_point(added_pt);
    added_pt.curvature = (src_pt.timestamp - start_time) * time_unit_scale;

    if (feature_enabled)
    {
      if (src_pt.ring < static_cast<uint16_t>(N_SCANS))
      {
        pl_buff[src_pt.ring].push_back(added_pt);
      }
    }
    else
    {
      pl_surf.push_back(added_pt);
    }
  }

  if (!feature_enabled)
  {
    return;
  }

  for (int j = 0; j < N_SCANS; j++)
  {
    PointCloudXYZI &pl = pl_buff[j];
    int linesize = static_cast<int>(pl.size());
    if (linesize < 2)
    {
      continue;
    }

    vector<orgtype> &types = typess[j];
    types.clear();
    types.resize(linesize);
    linesize--;
    for (int i = 0; i < linesize; i++)
    {
      types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
      vx = pl[i].x - pl[i + 1].x;
      vy = pl[i].y - pl[i + 1].y;
      vz = pl[i].z - pl[i + 1].z;
      types[i].dista = vx * vx + vy * vy + vz * vz;
    }
    types[linesize].range = std::sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
    give_feature(pl, types);
  }
}

void Preprocess::mid360_pointcloud2_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<livox_point_type::LivoxPointXyzitl> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const int plsize = static_cast<int>(pl_orig.points.size());
  if (plsize == 0)
  {
    return;
  }
  pl_surf.reserve(plsize);

  const double omega_l = 0.361 * SCAN_RATE;
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);
  std::vector<float> yaw_last(N_SCANS, 0.0);
  std::vector<float> time_last(N_SCANS, 0.0);

  const int filter_step = std::max(1, point_filter_num);
  for (int i = 0; i < plsize; ++i)
  {
    if (i % filter_step != 0)
    {
      continue;
    }

    const auto &src_pt = pl_orig.points[i];
    const int layer = src_pt.line;
    if (layer >= N_SCANS)
    {
      continue;
    }

    PointType added_pt;
    added_pt.x = src_pt.x;
    added_pt.y = src_pt.y;
    added_pt.z = src_pt.z;
    added_pt.intensity = src_pt.intensity;
    reset_point(added_pt);
    added_pt.curvature = 0.0;

    const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;
    if (is_first[layer])
    {
      yaw_fp[layer] = yaw_angle;
      is_first[layer] = false;
      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
      continue;
    }

    if (yaw_angle <= yaw_fp[layer])
    {
      added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
    }
    else
    {
      added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
    }

    if (added_pt.curvature < time_last[layer])
    {
      added_pt.curvature += 360.0 / omega_l;
    }

    yaw_last[layer] = yaw_angle;
    time_last[layer] = added_pt.curvature;

    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind * blind)
    {
      pl_surf.push_back(added_pt);
    }
  }
}

void Preprocess::ouster64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<ouster_point_type::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const int plsize = static_cast<int>(pl_orig.size());
  if (plsize == 0)
  {
    return;
  }
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      const auto &src_pt = pl_orig.points[i];
      const double range = src_pt.x * src_pt.x + src_pt.y * src_pt.y + src_pt.z * src_pt.z;
      if (range < blind * blind || src_pt.ring >= N_SCANS)
      {
        continue;
      }

      PointType added_pt;
      added_pt.x = src_pt.x;
      added_pt.y = src_pt.y;
      added_pt.z = src_pt.z;
      added_pt.intensity = src_pt.intensity;
      reset_point(added_pt);
      added_pt.curvature = src_pt.t * time_unit_scale;
      pl_buff[src_pt.ring].push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = static_cast<int>(pl.size());
      if (linesize < 2)
      {
        continue;
      }
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (int i = 0; i < linesize; i++)
      {
        types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = std::sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    const int filter_step = std::max(1, point_filter_num);
    for (int i = 0; i < plsize; i++)
    {
      if (i % filter_step != 0)
      {
        continue;
      }

      const auto &src_pt = pl_orig.points[i];
      const double range = src_pt.x * src_pt.x + src_pt.y * src_pt.y + src_pt.z * src_pt.z;
      if (range < blind * blind)
      {
        continue;
      }

      PointType added_pt;
      added_pt.x = src_pt.x;
      added_pt.y = src_pt.y;
      added_pt.z = src_pt.z;
      added_pt.intensity = src_pt.intensity;
      reset_point(added_pt);
      added_pt.curvature = src_pt.t * time_unit_scale;
      pl_surf.push_back(added_pt);
    }
  }
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<velodyne_point_type::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const int plsize = static_cast<int>(pl_orig.points.size());
  if (plsize == 0)
  {
    return;
  }
  pl_surf.reserve(plsize);

  const double omega_l = 0.361 * SCAN_RATE;
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);
  std::vector<float> yaw_last(N_SCANS, 0.0);
  std::vector<float> time_last(N_SCANS, 0.0);
  given_offset_time = pl_orig.points[plsize - 1].time > 0;

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }
  }

  const int filter_step = std::max(1, point_filter_num);
  for (int i = 0; i < plsize; i++)
  {
    const auto &src_pt = pl_orig.points[i];
    const int layer = src_pt.ring;
    if (layer >= N_SCANS)
    {
      continue;
    }

    PointType added_pt;
    added_pt.x = src_pt.x;
    added_pt.y = src_pt.y;
    added_pt.z = src_pt.z;
    added_pt.intensity = src_pt.intensity;
    reset_point(added_pt);
    added_pt.curvature = src_pt.time * time_unit_scale;

    if (!given_offset_time)
    {
      const double yaw_angle = std::atan2(added_pt.y, added_pt.x) * 57.2957;
      if (is_first[layer])
      {
        yaw_fp[layer] = yaw_angle;
        is_first[layer] = false;
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      if (yaw_angle <= yaw_fp[layer])
      {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      }
      else
      {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.curvature < time_last[layer])
      {
        added_pt.curvature += 360.0 / omega_l;
      }

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
    }

    if (feature_enabled)
    {
      pl_buff[layer].push_back(added_pt);
    }
    else if (i % filter_step == 0 &&
             added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind * blind)
    {
      pl_surf.push_back(added_pt);
    }
  }

  if (!feature_enabled)
  {
    return;
  }

  for (int j = 0; j < N_SCANS; j++)
  {
    PointCloudXYZI &pl = pl_buff[j];
    int linesize = static_cast<int>(pl.size());
    if (linesize < 2)
    {
      continue;
    }
    vector<orgtype> &types = typess[j];
    types.clear();
    types.resize(linesize);
    linesize--;
    for (int i = 0; i < linesize; i++)
    {
      types[i].range = std::sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
      vx = pl[i].x - pl[i + 1].x;
      vy = pl[i].y - pl[i + 1].y;
      vz = pl[i].z - pl[i + 1].z;
      types[i].dista = vx * vx + vy * vy + vz * vz;
    }
    types[linesize].range = std::sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
    give_feature(pl, types);
  }
}

void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  const int plsize = static_cast<int>(pl_orig.points.size());
  if (plsize == 0)
  {
    return;
  }
  pl_surf.reserve(plsize);

  const int filter_step = std::max(1, point_filter_num);
  for (int i = 0; i < plsize; ++i)
  {
    if (i % filter_step != 0)
    {
      continue;
    }

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    reset_point(added_pt);
    added_pt.curvature = 0.0;

    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind * blind)
    {
      pl_surf.push_back(added_pt);
    }
  }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while (head < types.size() && types[head].range < blind)
  {
    head++;
  }
  if (head >= types.size())
  {
    return;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < (uint)plsize2; i++)
  {
    if (types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)
    {
      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }

      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else  // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < (uint)plsize2; i++)
  {
    if (types[i].range < blind || types[i].ftype >= Real_Plane)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }

      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if (types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if (types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
        types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 &&
             types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < (uint)plsize2; i++)
  {
    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }

    if (types[i].ftype == Nor)
    {
      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < (uint)plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }

      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if (last_surface != -1)
      {
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI& pl, const rclcpp::Time& ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI& pl, vector<orgtype>& types, uint i_cur, uint& i_nex,
                            Eigen::Vector3d& curr_direct)
{
  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size()))
      break;

    if (types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    if (two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid)
    {
      leng_wid = lw;
    }
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if (is_livox_custom_lidar(lidar_type))
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI& pl, vector<orgtype>& types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind)
    {
      return false;
    }
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind)
    {
      return false;
    }
  }
  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {
    return false;
  }

  return true;
}
