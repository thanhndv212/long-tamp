#!/usr/bin/env python3
"""Standalone-install smoke check: config parsing only, no planning/solving.

Confirms that `long_tamp` imports and a real YAML task config loads and
validates end-to-end with **zero HPP native bindings** present — only the
`[standalone]` extra (pinocchio from PyPI). Never constructs a backend,
builds a constraint graph, or calls `ManipulationTask.setup()`/`run()`;
those all require pyhpp and are covered separately once an HPP-stack image
is available in CI (see `.github/workflows/lint.yml`).

Usage:
    pip install -e ".[standalone]"
    python script/validate_standalone_install.py
"""

from pathlib import Path

import long_tamp
from long_tamp.config.yaml_loader import YamlTaskLoader

CONFIG = Path(__file__).parent / "twin" / "config" / "twin_lift_ball_config.yaml"


def main() -> None:
    print(f"long_tamp {long_tamp.__version__}")
    print(f"available backends: {long_tamp.get_available_backends()!r}")

    loader = YamlTaskLoader(CONFIG)

    file_paths = loader.file_paths
    assert file_paths, "file_paths must not be empty"

    joint_bounds_class = loader.joint_bounds_class
    assert joint_bounds_class is not None

    task_config = loader.task_config
    assert task_config is not None

    q_init = loader.build_initial_config(objects=getattr(task_config, "OBJECTS", None))
    assert q_init, "build_initial_config must return a non-empty configuration"

    print(
        f"loaded {CONFIG.name}: {len(file_paths)} path groups, "
        f"{len(q_init)}-dim initial config"
    )
    print("OK — config parses and validates with zero HPP bindings installed.")


if __name__ == "__main__":
    main()
