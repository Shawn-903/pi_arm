#!/usr/bin/env bash
# Load this workspace and run one software-zero capture with automatic service cleanup.
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "请先执行 bash scripts/build.sh" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
tasks_prefix="$(ros2 pkg prefix pi_arm_tasks)"
exec "$tasks_prefix/lib/pi_arm_tasks/pi_arm_zero_once" "$@"
