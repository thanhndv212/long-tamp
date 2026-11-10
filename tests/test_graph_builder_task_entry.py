"""``GraphBuilder``'s task-config entry point (#219). No HPP needed: the
factory call itself is replaced."""

from long_tamp.config.base_config import BaseTaskConfig
from long_tamp.planning.graph import GraphBuilder


class _Config(BaseTaskConfig):
    GRIPPERS = ["arm/gripper"]
    OBJECTS = ["ball"]
    HANDLES_PER_OBJECT = [["ball/handle"]]


def _builder(monkeypatch):
    calls = []
    builder = object.__new__(GraphBuilder)  # no planner behind it
    monkeypatch.setattr(
        builder,
        "create_factory_graph",
        lambda config, **kwargs: calls.append((config, kwargs)) or "graph",
        raising=False,
    )
    return builder, calls


def test_factory_mode_hands_the_task_config_to_the_factory(monkeypatch):
    builder, calls = _builder(monkeypatch)
    assert builder.build_graph_for_task(_Config, mode="factory") == "graph"
    assert calls == [(_Config, {})]


def test_a_config_instance_and_the_default_mode_work_too(monkeypatch):
    builder, calls = _builder(monkeypatch)
    assert builder.build_graph_for_task(_Config()) == "graph"  # no STATES/EDGES
    assert calls == [(_Config, {})]


def test_a_class_based_config_has_the_rules_the_factory_reads():
    # create_factory_graph reads config.RULES; only the YAML loader set it.
    assert BaseTaskConfig.RULES is None
    assert _Config.RULES is None
