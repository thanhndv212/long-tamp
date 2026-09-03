#!/usr/bin/env bash
# Regenerate robotiq_2f85_generated.urdf from the vendored xacro sources.
# Local prototype only — see ../README.md.
#
# The upstream xacro files (assets/robotiq_2f85/urdf/*.xacro, vendored
# as-is from ros-industrial/robotiq's robotiq_2f_85_gripper_visualization,
# BSD-2-Clause) use ROS-only substitutions this standalone `pip install
# xacro` can't resolve without a real ROS install:
#   - `$(find robotiq_2f_85_gripper_visualization)` (needs ament_index /
#     rospkg) — patched in-place to this vendored directory's absolute
#     path in robotiq_arg2f_85_model.xacro / _macro.xacro.
#   - `package://robotiq_2f_85_gripper_visualization/...` mesh URIs — same
#     patch, in robotiq_arg2f_85_model_macro.xacro AND robotiq_arg2f.xacro
#     (a separate xacro:include'd from inside the macro file, easy to miss).
#
# Output mesh paths are baked in as this HOST's absolute path (same
# environment-dependent caveat as generate_urdf_srdf.py) — re-run this
# inside the container (or sed the host prefix to the container mount
# point) before loading there.
#
# Requires: pip install xacro (already done once, into agimus_spacelab's
# .venv on this machine — not a long_tamp dependency, purely a one-off
# local conversion tool).

set -euo pipefail
cd "$(dirname "$0")/assets/robotiq_2f85/urdf"

python3 -c "
import xacro, sys
sys.argv = ['xacro', 'robotiq_arg2f_85_model.xacro', '-o', 'robotiq_2f85_generated.urdf']
xacro.main()
"

echo "wrote robotiq_2f85_generated.urdf"
if grep -q 'find\|package://' robotiq_2f85_generated.urdf; then
    echo "WARNING: unresolved ROS substitution remains — check for a new" >&2
    echo "xacro:include this script's patching doesn't cover yet." >&2
    exit 1
fi
