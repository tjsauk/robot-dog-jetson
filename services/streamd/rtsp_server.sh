#!/usr/bin/env bash
set -euo pipefail

PORT="${RTSP_PORT:-8554}"
BITRATE="${BITRATE:-8000000}"  # bits/s
W="${CAM_W:-1920}"
H="${CAM_H:-1080}"
FPS="${CAM_FPS:-30}"

# Two mounts: /cam0 and /cam1
# Using H264 hardware encoder (nvv4l2h264enc)

gst-rtsp-launch -p "$PORT" "( \
  nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=$W,height=$H,framerate=$FPS/1,format=NV12 ! \
  nvv4l2h264enc bitrate=$BITRATE insert-sps-pps=true iframeinterval=$FPS ! h264parse ! rtph264pay name=pay0 pt=96 \
)" "( \
  nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=$W,height=$H,framerate=$FPS/1,format=NV12 ! \
  nvv4l2h264enc bitrate=$BITRATE insert-sps-pps=true iframeinterval=$FPS ! h264parse ! rtph264pay name=pay1 pt=96 \
)"
