#!/usr/bin/env bash
set -euo pipefail

mode="${1:-real}"
interface="${2:-can0}"
bitrate="${3:-1000000}"

# Validate arguments before changing an interface or loading a kernel module.
case "${mode}" in
  real)
    if ! [[ "${bitrate}" =~ ^[1-9][0-9]*$ ]]; then
      echo "bitrate 必须是正整数" >&2
      exit 2
    fi
    ;;
  vcan) interface="${2:-vcan0}" ;;
  down) ;;
  *)
    echo "用法：$0 real|vcan|down [can0|vcan0] [bitrate]" >&2
    exit 2
    ;;
esac

if [[ "${EUID}" -ne 0 ]]; then
  echo "请以 root 运行，例如：sudo bash $0 ${mode} ${interface}" >&2
  exit 1
fi

case "${mode}" in
  real)
    if [[ -z "$(ip -details link show dev "${interface}" type can)" ]]; then
      echo "接口 ${interface} 不存在或不是物理 CAN 接口" >&2
      exit 2
    fi
    ip link set "${interface}" down
    ip link set "${interface}" type can bitrate "${bitrate}" restart-ms 100
    ip link set "${interface}" txqueuelen 1000
    ip link set "${interface}" up
    ;;
  vcan)
    if ip link show dev "${interface}" >/dev/null 2>&1; then
      if [[ -z "$(ip -details link show dev "${interface}" type vcan)" ]]; then
        echo "接口 ${interface} 已存在且不是 vcan，未作修改" >&2
        exit 2
      fi
    else
      modprobe vcan
      ip link add dev "${interface}" type vcan
    fi
    ip link set "${interface}" up
    ;;
  down)
    ip link set "${interface}" down
    ;;
esac

ip -details -statistics link show "${interface}"
