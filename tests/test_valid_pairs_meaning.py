"""What ``VALID_PAIRS`` means to the graph factory (#225).

``{}`` and ``None`` both look like "nothing listed" and mean opposite things:
``{}`` (the default) allows no grasp, ``None`` allows every one. The warning
is tested without HPP; the graphs are built for real on the TWIN lift-ball
scene (two grippers, two handles on one ball) where HPP is installed.
"""

import logging
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from long_tamp.config.base_config import BaseTaskConfig
from long_tamp.planning.graph import GraphBuilder, allows_no_grasp, warn_if_no_grasp

try:
    from long_tamp.backends.pyhpp import HAS_PYHPP
except ImportError:
    HAS_PYHPP = False

_TWIN_SCRIPT_DIR = Path(__file__).resolve().parent.parent / "script" / "twin"
_LOGGER = "long_tamp.planning.graph"


def _config(pairs, rules=None):
    return SimpleNamespace(VALID_PAIRS=pairs, RULES=rules)


def test_an_empty_dict_allows_nothing_and_none_allows_everything():
    assert allows_no_grasp(_config({}))
    assert allows_no_grasp(_config({"arm/gripper": []}))
    assert not allows_no_grasp(_config(None))
    assert not allows_no_grasp(_config({"arm/gripper": ["ball/handle"]}))
    assert not allows_no_grasp(_config({}, rules=["a rule"]))  # rules replace pairs


def test_the_default_task_config_allows_no_grasp():
    # The trap this module is about: a config that says nothing gets `{}`.
    assert BaseTaskConfig.VALID_PAIRS == {}
    assert allows_no_grasp(BaseTaskConfig)


def test_the_warning_names_the_setting_and_the_consequence(caplog):
    with caplog.at_level(logging.WARNING, logger=_LOGGER):
        warn_if_no_grasp(_config({"arm/gripper": ["ball/handle"]}))
        warn_if_no_grasp(_config(None))
    assert not caplog.records
    with caplog.at_level(logging.WARNING, logger=_LOGGER):
        warn_if_no_grasp(_config({}))
    (record,) = caplog.records
    assert "VALID_PAIRS" in record.getMessage() and "free" in record.getMessage()


def test_the_task_entry_point_warns(monkeypatch, caplog):
    class Config(BaseTaskConfig):
        GRIPPERS = ["arm/gripper"]
        OBJECTS = ["ball"]
        HANDLES_PER_OBJECT = [["ball/handle"]]

    builder = object.__new__(GraphBuilder)  # no planner behind it
    monkeypatch.setattr(
        builder, "create_factory_graph", lambda config, **kw: "graph", raising=False
    )
    with caplog.at_level(logging.WARNING, logger=_LOGGER):
        assert builder.build_graph_for_task(Config, mode="factory") == "graph"
    assert len(caplog.records) == 1


@pytest.fixture(scope="module")
def twin():
    """The TWIN scene, set up once, without its graph."""
    if not HAS_PYHPP:
        pytest.skip("PyHPP backend not available")
    if str(_TWIN_SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(_TWIN_SCRIPT_DIR))
    import task_lift_ball  # noqa: PLC0415 -- path-dependent import

    task = task_lift_ball.LiftBallTask(backend="pyhpp")
    try:
        task.setup(
            validation_step=task.task_config.PATH_VALIDATION_STEP,
            projector_step=task.task_config.PATH_PROJECTOR_STEP,
            freeze_joint_substrings=task.FREEZE_JOINT_SUBSTRINGS,
            skip_graph=True,
        )
    except Exception as exc:
        pytest.skip(f"TWIN scene could not be set up in this environment: {exc}")
    return task


def _states(task, pairs):
    config = SimpleNamespace()
    for name in dir(task.task_config):
        value = getattr(task.task_config, name)
        if not name.startswith("_") and not callable(value):
            setattr(config, name, value)
    config.VALID_PAIRS = pairs
    task.graph_builder.create_factory_graph(
        config,
        graph_constraints=getattr(task, "_graph_constraints", None),
        q_init=task.q_init,
    )
    return set(task.graph_builder.graph.getStateNames())


def _grasps(states):
    return {s for s in states if " grasps " in s and " : " not in s}


def test_declared_pairs_give_only_those_grasps(twin):
    states = _states(twin, twin.task_config.VALID_PAIRS)
    assert _grasps(states) == {
        "panda_left/gripper grasps ball/handle",
        "panda_right/gripper grasps ball/handle2",
    }


def test_empty_pairs_give_a_graph_with_the_free_state_alone(twin, caplog):
    with caplog.at_level(logging.WARNING, logger=_LOGGER):
        assert _states(twin, {}) == {"free"}
    # A phase graph passes empty pairs on purpose (plan_loop with nothing
    # held), so the factory call itself says nothing.
    assert not [r for r in caplog.records if "VALID_PAIRS" in r.getMessage()]


def test_no_pairs_at_all_let_any_gripper_grasp_any_handle(twin):
    states = _states(twin, None)
    assert _grasps(states) == {
        f"panda_{side}/gripper grasps ball/{handle}"
        for side in ("left", "right")
        for handle in ("handle", "handle2")
    }
