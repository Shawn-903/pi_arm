#!/usr/bin/env bash
set -euo pipefail

if [[ ! -r /etc/os-release ]]; then
  echo "错误：无法读取 /etc/os-release，不能确认目标操作系统。" >&2
  exit 1
fi

# /etc/os-release is part of the base Ubuntu system, so lsb-release is not a
# prerequisite for running this script on a newly installed system.
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "警告：本项目目标平台是 Ubuntu 24.04 (noble)。" >&2
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  can-utils \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-yaml \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-moveit \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rosdep install --from-paths "${workspace_root}/src" --ignore-src -r -y \
  --rosdistro jazzy
