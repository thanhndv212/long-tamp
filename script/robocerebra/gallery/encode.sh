#!/bin/bash
# Encode <dir>/.../frames (20 Hz control steps) to 30 fps real-time H.264, one
# <name>/<name>.mp4 beside each frames/. <dir> defaults to out_video.
set -e
cd "$(dirname "$0")/${1:-out_video}"
find . -type d -name frames | sort | while read -r f; do
  d=$(dirname "$f"); n=$(basename "$d")
  ffmpeg -y -loglevel error -framerate 20 -i "$f/%04d.jpg" -vf fps=30 \
    -c:v libx264 -pix_fmt yuv420p -crf 22 -movflags +faststart "$d/$n.mp4" </dev/null
  echo "$d $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$d/$n.mp4")s"
done
