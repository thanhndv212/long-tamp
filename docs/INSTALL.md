# Installation

`long_tamp` has two dependency tiers:

| Tier | Packages | Source |
|------|----------|--------|
| **Python (PyPI)** | `numpy`, `pyyaml`, `pin` (pinocchio), and the viser viewer stack (`viser`, `trimesh`, `pycollada`) | `pip` — cross-platform (Linux/macOS via cmeel) |
| **HPP native bindings** | `hpp-python` (pyhpp), `hpp-gepetto-viewer`, `hpp-toppra`, … | **PyPI (prebuilt wheels) — Linux only today; robotpkg/source build otherwise** |

The HPP native bindings are C++ extension modules. On Linux (x86_64/aarch64, Python
3.10-3.14) they're now published as prebuilt wheels by the [cmeel](https://github.com/cmake-wheel/cmeel)
project — the same packaging used for `pin` (pinocchio), `coal`, and the rest of the
Gepetto/Stack-of-Tasks ecosystem — so **`pip install` is the primary, recommended path**.
`pip install long-tamp` alone (no extras) gives you a working *pure-Python* package (config
parsing, planning-graph construction, transforms via pinocchio, run logging, viser viewer)
on any platform cmeel publishes `pin` for, including macOS — verified directly in a clean
venv. Add the `[hpp]` and `[toppra]` extras below for the actual planning backends, which
are Linux-only for now. Instantiating a backend without its bindings raises an
`ImportError` explaining exactly what is missing.

---

## Recommended: pip only, no system packages, no Docker

```bash
# Editable install with the PyHPP backend + viewer + TOPPRA optimizer
pip install -e ".[hpp,toppra]"

# With dev tooling (pytest, black, ruff, sphinx)
pip install -e ".[hpp,toppra,dev]"
```

This resolves the entire stack from PyPI in one shot: `long_tamp` itself, `hpp-python`
(pyhpp bindings), `hpp-gepetto-viewer` (pulls in `pin`/pinocchio, the Gepetto Qt viewer,
and the `pyhpp_viser` browser-viewer bindings), and `hpp-toppra` (pulls in the `toppra`
C++ lib via `cmeel-toppra`). Verify with:

```bash
python -c "from long_tamp import get_available_backends; print(get_available_backends())"
```

**Platform support today: Linux only** (`manylinux_2_28`, x86_64/aarch64), CPython
3.10–3.14. No macOS or Windows wheels are published yet — verified directly: on macOS,
`pip install --no-deps --only-binary=:all: hpp-python` fails fast and cleanly with "No
matching distribution found." **Without `--only-binary=:all:`, pip instead falls back to
the sdist and attempts a from-source build** (needs boost/eigen/cmake and can hang or fail
deep into the C++ build) — on an unsupported platform, pass `--only-binary=:all:` (or just
expect this and Ctrl-C) rather than waiting on it, and use the robotpkg/source-build path
below instead. (`pin`/pinocchio itself, a hard dependency below, *does* publish macOS
wheels via cmeel — it's specifically the HPP packages, `hpp-python`/`hpp-gepetto-viewer`/
`hpp-toppra`, that are Linux-only today.)

**NumPy version — the opposite constraint from robotpkg.** The cmeel wheels are built
against NumPy 2.x (`cmeel-boost`, a transitive dependency of `hpp-python`, requires
`numpy>=2`). This is unrelated to — and incompatible with — the robotpkg path below, which
needs NumPy 1.x. Don't mix a `pip install -e ".[hpp,toppra]"` environment with a
robotpkg/`/opt/openrobots` install on the same interpreter.

**Known gap: the Gepetto (Qt) viewer needs a recent `long_tamp`.** The PyPI
`hpp-gepetto-viewer` wheel exposes its viewer as `pyhpp_gepetto.viewer` (a flat top-level
module), whereas the robotpkg/source-built package nests it as `pyhpp.gepetto.viewer`.
`long_tamp`'s backend tries both, so this is transparent — just noting it in case you're
comparing against an older checkout. The default `viser` browser viewer isn't affected
either way (`pyhpp_viser` matches both install paths as-is).

---

## Alternative: robotpkg / source build

Use this path only if you're on a platform without cmeel wheels (not Linux x86_64/aarch64),
need to match an existing robotpkg/ROS 2 workspace, or need bleeding-edge `devel`-branch
HPP that hasn't cut a release yet.

### System packages via robotpkg

Follow the **[official HPP installation guide](https://humanoid-path-planner.github.io/hpp-doc/installation/installation.html)** to add the robotpkg APT repository and set up your environment (`PATH`, `LD_LIBRARY_PATH`, `PYTHONPATH`, `CMAKE_PREFIX_PATH` under `/opt/openrobots`).

```bash
pyver=312   # adjust to your Python version — Ubuntu 24.04 → 312, 22.04 → 310, 20.04 → 38

sudo apt-get install \
  robotpkg-py${pyver}-hpp-python \
  robotpkg-py${pyver}-qt5-hpp-gepetto-viewer
```

| Package | Provides | Pulls in (transitively) |
|---------|----------|-------------------------|
| `robotpkg-py${pyver}-hpp-python` | PyHPP backend (`pyhpp.*`) | pinocchio, eigenpy, coal, hpp-util, hpp-pinocchio, hpp-core, hpp-constraints, hpp-manipulation, hpp-manipulation-urdf |
| `robotpkg-py${pyver}-qt5-hpp-gepetto-viewer` | Gepetto + `pyhpp_viser` viewers | gepetto-viewer-corba, qgv, qtbase5 |

Both are required — `hpp-python` alone plans headlessly with no viewer; without the second
package, `task_lift_ball.py --viewer-type viser` from Quick Start has nothing to display
into.

**TOPPRA is not in robotpkg** (checked `pub` and `wip`, all distros — absent). If you need
it on this path, build `toppra` and `hpp-toppra` from source, or switch to the pip path
above where `hpp-toppra` is a real wheel.

> **NumPy ABI — do not install `pinocchio` from PyPI on top of robotpkg.** The
> robotpkg/`/opt/openrobots` pinocchio is compiled against **NumPy 1.x**; a NumPy 2.x in
> your user/site path shadows the system numpy and **segfaults** the pinocchio
> C-extension. Install `long_tamp` here with `pip install --no-deps -e .` and let
> robotpkg provide numpy/pinocchio — don't let `pip` resolve `long_tamp`'s own
> `numpy>=1.26` dependency (which is happy to satisfy itself with a NumPy 2.x wheel) in
> this environment.

### Source build / container

For a matching HPP stack built from source (`devel` branches, needed if robotpkg is
missing symbols your target relies on), use the prebuilt Docker image, which compiles
HPP from source under `$DEVEL_HPP_DIR` (`~/devel/hpp`).

The Docker definitions live in a separate repository: [gitlab.laas.fr/dvtnguyen/dockers](https://gitlab.laas.fr/dvtnguyen/dockers), with one image per ROS 2 distribution (`hpp/` → Jazzy/24.04, `hpp-humble/` → Humble/22.04). Pick the one matching your target, build it (`run_docker.sh` in each directory), and work inside the container. To reproduce the build outside Docker, follow the same steps on the host and point `PYTHONPATH`/`LD_LIBRARY_PATH` at your source-install prefix instead of `/opt/openrobots`.

This path is primarily maintained for `agimus_spacelab` (the sibling, ROS 2-attached
repo) — for `long_tamp` itself, prefer the pip path above unless you have a specific
reason to need a source build.

### CMake (in an HPP workspace)

Installs `long_tamp` into `PYTHON_SITELIB` alongside a source-built HPP stack — the source
of truth for the native backends if you're building HPP from source anyway:

```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_HPP_DIR
make install
```

Build options (see `CMakeLists.txt`):

| Option | Default | Provides | Native prerequisites |
|--------|:-------:|----------|----------------------|
| `WITH_PYHPP`  | **ON**  | PyHPP backend (default) | `hpp-python` |
| `WITH_TOPPRA` | OFF     | TOPPRA time-parameterization optimizer | `hpp-toppra` (which requires the `toppra` C++ lib ≥0.6.2) |

`hpp-gepetto-viewer` is picked up whenever `WITH_PYHPP` is enabled.

```bash
# Enable the optional TOPPRA optimizer:
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_HPP_DIR -DWITH_TOPPRA=ON
```

## Backend availability at runtime

The viser browser viewer ships by default. The other optimizers/viewers are detected at
import time and expose `HAS_*` flags in `long_tamp.backends.pyhpp` (`HAS_PYHPP`,
`HAS_TOPPRA`, `HAS_VISER`, `HAS_GEPETTO_VIEWER`). A missing backend fails loudly only when
you try to construct it, with guidance on how to obtain the bindings.
