#!/bin/bash
# Run N seeded missions in parallel and summarize them.
#
#   bash run_batch.sh [N=10] [PAR=3] [OUT=batch]
#
# Run inside the hpp-agimus-arm64 container, from this folder, after
# build_scene.py. Each seed writes OUT/seed_NNN.{json,log}. PAR processes
# at a time: the scene is crash-free now (see the problem-distance fix), so
# parallel runs only cost some timing noise.
set -u
N=${1:-10}
PAR=${2:-3}
OUT=${3:-batch}
HERE=$(cd "$(dirname "$0")" && pwd)
cd "$HERE"
mkdir -p "$OUT"
seq 1 "$N" | xargs -P "$PAR" -I{} bash -c '
  s=$(printf %03d {})
  PYTHONFAULTHANDLER=1 python3 -u task_screw_assembly.py --seed {} --no-viewer \
    --summary "'"$OUT"'/seed_$s.json" > "'"$OUT"'/seed_$s.log" 2>&1
  echo "seed {} exit $?"
'
python3 summarize.py "$OUT"
