#!/usr/bin/env bash
# Starts the ROS control chain and serves only the web/ directory.
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$workspace_root/install/setup.bash" ]]; then
  echo "请先执行 bash scripts/build.sh" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"
set -u
web_port="${PI_ARM_WEB_PORT:-8000}"
launch_file="bringup.launch.py"
launch_args=(use_rviz:=false)
if [[ "${1:-}" == "--monitor" ]]; then
  shift
  launch_file="monitor_web.launch.py"
  launch_args=()
elif [[ "${1:-}" == "--commissioning" ]]; then
  shift
  launch_file="commissioning_web.launch.py"
  launch_args=()
fi
children=()
cleanup() {
  trap - EXIT INT TERM
  for child in "${children[@]}"; do kill -INT "$child" 2>/dev/null || true; done
  for child in "${children[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# Background jobs otherwise inherit SIGINT ignored from non-interactive bash.
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  python3 -m http.server "$web_port" --bind 0.0.0.0 --directory "$workspace_root/web" &
children+=("$!")
python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
  ros2 launch pi_arm_bringup "$launch_file" "${launch_args[@]}" "$@" &
children+=("$!")
echo "网页：http://localhost:${web_port}/control.html （远程访问请替换 localhost 为本机 IP）"
if [[ "$launch_file" == "commissioning_web.launch.py" ]]; then
  echo "电机方向测试：输出轴单步最大 5°、距首次测试起点 ±30°、速度最大 1°/s；仅在网页点击后执行。"
elif [[ "$launch_file" == "monitor_web.launch.py" ]]; then
  echo "实机只读排查：轮询 CAN 反馈，不启动电机控制服务。Ctrl+C 停止服务。"
else
  echo "默认 mock 模拟后端；--monitor 启动实机只读排查。Ctrl+C 停止服务。"
fi
wait -n "${children[@]}"
