#!/usr/bin/env bash
# 右ディスプレイ (HDMI-0, 1920x1080, offset +1920+0) を録画
# Usage: bash scripts/record_screen.sh [output.mp4] [duration_sec]
#   Ctrl+C で手動停止も可能

OUT="${1:-output/awsim_shinjuku_slam/demo.mp4}"
DUR="${2:-90}"

mkdir -p "$(dirname "${OUT}")"

echo "Recording right display (1920x1080 @ +1920,0) for ${DUR}s"
echo "Output: ${OUT}"
echo "Press Ctrl+C to stop early"

ffmpeg -y \
  -f x11grab \
  -framerate 10 \
  -video_size 1920x1080 \
  -i :1+1920,0 \
  -t "${DUR}" \
  -c:v libx264 -preset ultrafast -crf 25 -pix_fmt yuv420p \
  "${OUT}"

echo "Saved: ${OUT}"
ls -lh "${OUT}"
