#!/usr/bin/env bash

set -euo pipefail

cd "$(dirname "$0")"
exec ./build/car --camera-device=/dev/video0 --http-mjpeg=8080 --headless /dev/ttyS2
