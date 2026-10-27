#!/bin/bash
# Encode out_video/<class>/frames (20 Hz control steps) to 30 fps real-time H.264.
set -e
cd "$(dirname "$0")/out_video"
for d in */; do
  n=${d%/}; [ -d "$n/frames" ] || continue
  ffmpeg -y -loglevel error -framerate 20 -i "$n/frames/%04d.jpg" -vf fps=30 \
    -c:v libx264 -pix_fmt yuv420p -crf 22 -movflags +faststart "$n/$n.mp4"
  echo "$n $(ffprobe -v error -show_entries format=duration -of csv=p=0 "$n/$n.mp4")s"
done
