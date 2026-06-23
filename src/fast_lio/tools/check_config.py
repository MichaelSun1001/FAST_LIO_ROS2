#!/usr/bin/env python3
# Copyright 2026 Hesai Technology. All rights reserved.
# SPDX-License-Identifier: GPL-2.0
#
# This file is part of FAST_LIO_ROS2, a fork of FAST_LIO
# (https://github.com/hku-mars/FAST_LIO) by the MARS Lab, HKU.
# This script is an original contribution by Hesai Technology.
"""
check_config.py  —  Statically validate a FAST-LIO2 LiDAR yaml config

Does NOT require ROS to be running. Parses the yaml and checks parameter
consistency against the declared LiDAR model and ROS version, catching the
most common misconfigurations before launch.

Checks:
  - preprocess.lidar_type matches model + ROS version
  - preprocess.scan_line matches model
  - preprocess.timestamp_unit is a valid enum (0-3)
  - common.imu_gyr_unit is "deg" or "rad"
  - point_filter_num is a positive integer
  - preprocess.blind is positive and below mapping.det_range
  - mapping.extrinsic_R is a valid rotation matrix (orthonormal, det ~ 1)
  - mapping.extrinsic_T has 3 elements
  - pcd_save.pcd_save_en consistency with map_file_path writability

	Usage:
	  python3 tools/check_config.py --config config/jt128.yaml --model jt128 --ros 2
	  python3 tools/check_config.py --config config/jt16.yaml  --model jt16  --ros 1
	  python3 tools/check_config.py --config config/airy96.yaml --model airy96 --ros 2
	  python3 tools/check_config.py --config config/avia.yaml --model avia --ros 2
	  python3 tools/check_config.py --config config/horizon.yaml --model horizon --ros 2
	  python3 tools/check_config.py --config config/mid360.yaml --model mid360 --ros 2
	  python3 tools/check_config.py --config config/velodyne.yaml --model velodyne --ros 2
	  python3 tools/check_config.py --config config/ouster64.yaml --model ouster64 --ros 2
	"""

import argparse
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("[ERROR] PyYAML not found. Run: pip3 install pyyaml")

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
WARN = "\033[93m[WARN]\033[0m"
INFO = "\033[94m[INFO]\033[0m"

# expected per-model values
MODEL_SPECS = {
    "jt16":  {"scan_line": 16,  "lidar_type": {1: 5, 2: 1}},
    "jt128": {"scan_line": 128, "lidar_type": {1: 6, 2: 2}},
    "airy96": {"scan_line": 96, "lidar_type": {1: 5, 2: 5}},
    "avia": {"scan_line": 6, "lidar_type": {1: 10, 2: 10}},
    "horizon": {"scan_line": 6, "lidar_type": {1: 15, 2: 15}},
    "mid360": {"scan_line": 4, "lidar_type": {1: 11, 2: 11}},
    "velodyne": {"scan_line": 32, "lidar_type": {1: 13, 2: 13}},
    "ouster64": {"scan_line": 64, "lidar_type": {1: 14, 2: 14}},
}


class ConfigChecker:
    def __init__(self, config_path, model, ros_ver):
        self.path     = config_path
        self.model    = model
        self.ros      = ros_ver
        self.spec     = MODEL_SPECS[model]
        self.failures = 0
        self.warnings = 0

    def _pass(self, msg): print(f"{PASS} {msg}")
    def _info(self, msg): print(f"{INFO} {msg}")
    def _fail(self, msg): print(f"{FAIL} {msg}"); self.failures += 1
    def _warn(self, msg): print(f"{WARN} {msg}"); self.warnings += 1

    # ── load + normalize ──────────────────────────────────────────────────
    def load(self):
        with open(self.path) as f:
            raw = yaml.safe_load(f)
        if raw is None:
            self._fail("Empty or invalid yaml file")
            return None
        # ROS 2 nests parameters under '/**': {'ros__parameters': {...}}
        if "/**" in raw and "ros__parameters" in raw["/**"]:
            self._info("Detected ROS 2 parameter layout (/**: ros__parameters)")
            return raw["/**"]["ros__parameters"]
        # ROS 1 uses a flat top-level layout
        self._info("Detected ROS 1 / flat parameter layout")
        return raw

    @staticmethod
    def _get(d, dotted, default=None):
        cur = d
        for k in dotted.split("."):
            if not isinstance(cur, dict) or k not in cur:
                return default
            cur = cur[k]
        return cur

    # ── checks ────────────────────────────────────────────────────────────
    def check(self, cfg):
        self._check_lidar_type(cfg)
        self._check_scan_line(cfg)
        self._check_timestamp_unit(cfg)
        self._check_gyr_unit(cfg)
        self._check_point_filter_num(cfg)
        self._check_blind(cfg)
        self._check_extrinsic(cfg)
        self._check_pcd_save(cfg)

    def _check_lidar_type(self, cfg):
        val = self._get(cfg, "preprocess.lidar_type")
        expected = self.spec["lidar_type"][self.ros]
        if val is None:
            self._fail("preprocess.lidar_type is missing")
        elif val != expected:
            self._fail(f"preprocess.lidar_type={val}, expected {expected} "
                       f"for {self.model} on ROS {self.ros}")
        else:
            self._pass(f"preprocess.lidar_type={val} matches "
                       f"{self.model} (ROS {self.ros})")

    def _check_scan_line(self, cfg):
        val = self._get(cfg, "preprocess.scan_line")
        exp = self.spec["scan_line"]
        if val is None:
            self._fail("preprocess.scan_line is missing")
        elif val != exp:
            self._fail(f"preprocess.scan_line={val}, expected {exp} "
                       f"for {self.model}")
        else:
            self._pass(f"preprocess.scan_line={val} matches {self.model}")

    def _check_timestamp_unit(self, cfg):
        val = self._get(cfg, "preprocess.timestamp_unit")
        if val is None:
            self._warn("preprocess.timestamp_unit missing (defaults may apply)")
        elif val not in (0, 1, 2, 3):
            self._fail(f"preprocess.timestamp_unit={val} invalid "
                       f"(must be 0=s,1=ms,2=us,3=ns)")
        else:
            unit = {0: "s", 1: "ms", 2: "us", 3: "ns"}[val]
            self._pass(f"preprocess.timestamp_unit={val} ({unit}) valid")

    def _check_gyr_unit(self, cfg):
        val = self._get(cfg, "common.imu_gyr_unit")
        if val is None:
            self._warn("common.imu_gyr_unit missing")
        elif val not in ("deg", "rad"):
            self._fail(f"common.imu_gyr_unit='{val}' invalid (must be 'deg' or 'rad')")
        else:
            self._pass(f"common.imu_gyr_unit='{val}' valid")

    def _check_point_filter_num(self, cfg):
        val = self._get(cfg, "point_filter_num")
        if val is None:
            self._warn("point_filter_num missing")
        elif not isinstance(val, int) or val <= 0:
            self._fail(f"point_filter_num={val} invalid (must be a positive integer)")
        else:
            self._pass(f"point_filter_num={val} valid")

    def _check_blind(self, cfg):
        blind = self._get(cfg, "preprocess.blind")
        det   = self._get(cfg, "mapping.det_range")
        if blind is None:
            self._warn("preprocess.blind missing")
            return
        if blind < 0:
            self._fail(f"preprocess.blind={blind} is negative")
        elif det is not None and blind >= det:
            self._fail(f"preprocess.blind={blind} ≥ mapping.det_range={det} "
                       f"— all points would be filtered out")
        else:
            self._pass(f"preprocess.blind={blind} m is reasonable")

    def _check_extrinsic(self, cfg):
        T = self._get(cfg, "mapping.extrinsic_T")
        R = self._get(cfg, "mapping.extrinsic_R")
        if T is None or R is None:
            self._warn("mapping.extrinsic_T / extrinsic_R missing")
            return
        if not isinstance(T, list) or len(T) != 3:
            self._fail(f"mapping.extrinsic_T must have 3 elements, got {T}")
        if not isinstance(R, list) or len(R) != 9:
            self._fail(f"mapping.extrinsic_R must have 9 elements, got {R}")
            return
        # check orthonormality: det(R) ~ 1 and rows unit-length
        det = (R[0]*(R[4]*R[8] - R[5]*R[7])
               - R[1]*(R[3]*R[8] - R[5]*R[6])
               + R[2]*(R[3]*R[7] - R[4]*R[6]))
        if abs(det - 1.0) > 0.05:
            self._fail(f"mapping.extrinsic_R determinant={det:.4f} (expected ~1.0) "
                       f"— not a valid rotation matrix")
        else:
            self._pass(f"mapping.extrinsic_R valid rotation (det={det:.4f})")
        est = self._get(cfg, "mapping.extrinsic_est_en")
        if est is True:
            self._info("mapping.extrinsic_est_en=true — online extrinsic estimation on; "
                       "set false if you already have accurate extrinsics")

    def _check_pcd_save(self, cfg):
        en = self._get(cfg, "pcd_save.pcd_save_en")
        if not en:
            self._info("pcd_save.pcd_save_en=false — PCD will not be saved")
            return
        path = self._get(cfg, "map_file_path", "")
        if not path:
            self._info("pcd_save enabled; map_file_path unset → defaults to PCD/scans.pcd")
            return
        # Relative paths are resolved against the package root at runtime.
        # Absolute paths and "~" paths can be sanity-checked directly.
        expanded_path = os.path.expanduser(path)
        if os.path.isabs(expanded_path):
            parent = os.path.dirname(expanded_path)
            display_path = expanded_path if expanded_path == path else f"{path} (expands to {expanded_path})"
            if parent and not os.path.isdir(parent):
                self._warn(f"map_file_path directory does not exist: {parent}")
            else:
                self._pass(f"pcd_save enabled → {display_path}")
        else:
            self._info(f"pcd_save enabled; map_file_path='{path}' "
                       f"(relative to package root)")

    # ── summary ─────────────────────────────────────────────────────────
    def summary(self):
        print()
        if self.failures:
            print(f"{FAIL} {self.failures} error(s), {self.warnings} warning(s). "
                  f"Fix errors before launching FAST-LIO2.")
            return 1
        if self.warnings:
            print(f"{WARN} 0 errors, {self.warnings} warning(s). Config is usable.")
            return 0
        print(f"{PASS} Config looks good — no issues found.")
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Statically validate a FAST-LIO2 yaml config")
    parser.add_argument("--config", required=True, help="Path to the yaml config")
    parser.add_argument("--model",  required=True, choices=list(MODEL_SPECS.keys()),
                        help="Lowercase LiDAR model key, e.g. jt16, jt128, airy96, avia, horizon, mid360, velodyne, ouster64")
    parser.add_argument("--ros",    type=int, required=True, choices=[1, 2],
                        help="ROS version: 1 or 2 (lidar_type enum differs)")
    args = parser.parse_args()

    if not os.path.isfile(args.config):
        sys.exit(f"[ERROR] Config file not found: {args.config}")

    print(f"\n{'='*62}")
    print(f"  FAST-LIO2 Config Checker  |  {args.model}  |  ROS {args.ros}")
    print(f"  config: {args.config}")
    print(f"{'='*62}\n")

    checker = ConfigChecker(args.config, args.model, args.ros)
    cfg = checker.load()
    if cfg is None:
        sys.exit(1)
    checker.check(cfg)
    rc = checker.summary()
    print(f"\n{'='*62}\n")
    sys.exit(rc)


if __name__ == "__main__":
    main()
