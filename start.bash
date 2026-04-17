#!/usr/bin/env bash

set -euo pipefail

cd "$(dirname "$0")"

# 启动蓝牙代理（后台）
sudo python3 bt_agent.py &
BT_PID=$!
trap "kill $BT_PID 2>/dev/null" EXIT

# 启动视觉主程序
exec ./build/car --camera-device=/dev/video0 --http-mjpeg=8080 --headless /dev/ttyS2
