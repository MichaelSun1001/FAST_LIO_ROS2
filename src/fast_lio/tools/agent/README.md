# FAST_LIO_ROS2 Agent Playbooks

Plain Markdown playbooks for an AI agent or operator assistant that needs to
set up, validate, and run this unified ROS 2 FAST-LIO2 workspace.

These files are generic instructions. They are not tied to one AI product or
agent runtime.

## Files

| File | Target | Use |
| --- | --- | --- |
| `run_fastlio2_ros2.md` | ROS 2 Humble | Build with `colcon`, choose a LiDAR model, validate config/input, and launch FAST-LIO2 |

## Scope

The current workspace is ROS 2 Humble only. It contains:

- `fast_lio`, the unified FAST-LIO2 mapping package.

The playbook covers the lowercase model keys documented in
`run_fastlio2_ros2.md`. Hesai PCAP conversion is still documented as a
Hesai-only helper workflow.
