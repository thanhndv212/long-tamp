#!/usr/bin/env python3
"""Load the combined UR10+Robotiq arm alone in viser, for visual
verification of the tool0-mount offset and TCP frame placement (both
placeholders in merge_ur10_robotiq.py — see its docstring).

Local prototype only — see README.md. Run inside the hpp-agimus-arm64
container (after sourcing config.sh) with mesh paths already baked to that
environment (see generate_urdf_srdf.py's note on regenerating per
environment).
"""

from pathlib import Path

from long_tamp.backends.pyhpp import PyHPPBackend

HERE = Path(__file__).parent
# .container.urdf has mesh paths baked to the hpp-agimus-arm64 container's
# mount point (see merge_ur10_robotiq.py's note on per-environment paths).
# Use the plain ur10_robotiq.urdf instead when running on the host.
URDF = HERE / "generated" / "ur10_robotiq.container.urdf"
SRDF = HERE / "generated" / "ur10_robotiq.srdf"


def main() -> None:
    backend = PyHPPBackend(viewer_type="viser")
    backend.load_robot(
        "arm",
        str(URDF),
        str(SRDF),
        root_joint_type="anchor",
    )
    backend.setup_viewer(viewer_type="viser")
    # backend.neutral_config() calls Device.neutralConfiguration(), which
    # doesn't exist on this pyhpp version's Device (real bug, unrelated to
    # this prototype — flagged separately). currentConfiguration() works.
    q = backend.device.currentConfiguration()
    print("nq:", len(q), "q:", list(q))
    backend.visualize(q)
    print("Viewer running — open the URL above in your browser.")
    print("Ctrl-C to stop.")
    import time

    while True:
        time.sleep(60)


if __name__ == "__main__":
    main()
