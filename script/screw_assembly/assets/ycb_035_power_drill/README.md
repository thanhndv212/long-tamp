# YCB 035_power_drill

The cordless drill/driver the screw-assembly cell uses as its screwdriver: a 3D scan
of a real hand-held drill from the YCB Object and Model Set (object 035, "power
drill", Google 16k-face scan). It is one of the most-used objects in robot grasping
benchmarks.

| File | Source |
|---|---|
| `textured.obj`, `textured.mtl` | `035_power_drill_google_16k.tgz`, unchanged |
| `texture_map.png` | same archive, downscaled 4096 → 1024 px |

Source: <http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/data/google/035_power_drill_google_16k.tgz>

License: [Creative Commons Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/).

> B. Calli, A. Walsman, A. Singh, S. Srinivasa, P. Abbeel, A. M. Dollar,
> "Benchmarking in Manipulation Research: Using the Yale-CMU-Berkeley Object and
> Model Set", IEEE Robotics & Automation Magazine, 22(3), 2015.

The mesh is only the *visual* geometry. `build_scene.py` gives the drill a few
box/cylinder collision proxies fitted to this scan (battery, handle, trigger guard,
motor body, chuck) and a 50 mm driver bit the scan does not include. The mesh frame
(lying on its side, chuck toward -X, handle along -Y) is mapped to the drill's own
frame by the URDF visual origin; see `build_scene.py`.
