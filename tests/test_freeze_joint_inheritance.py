"""GraspSequencePlanner keeps the joints ``task.setup()`` froze (issue #28).

``task.setup(freeze_joint_substrings=...)`` locks cosmetic joints (fingers)
in the global graph, but phase graphs are rebuilt per phase from the frozen
*arms* only. Unless the planner also knew the patterns, the moving arm's
"frozen" fingers took random values in every generated configuration: on
TWIN a finger closed to 14.8 mm inside a 25 mm-radius ball and the grasp
pose itself collided.
"""

from types import SimpleNamespace

from long_tamp.tasks.grasp_sequence import GraspSequencePlanner

_TASK_CONFIG = SimpleNamespace(
    GRIPPERS=["arm/gripper"], HANDLES_PER_OBJECT=[["ball/handle"]]
)


def _planner(graph_builder, **kwargs):
    return GraspSequencePlanner(
        graph_builder=graph_builder,
        config_gen=None,
        planner=None,
        task_config=_TASK_CONFIG,
        **kwargs,
    )


def test_planner_inherits_the_patterns_setup_froze():
    builder = SimpleNamespace(frozen_joint_substrings=["finger_joint"])
    assert _planner(builder).freeze_joint_substrings == ["finger_joint"]


def test_explicit_patterns_win_and_empty_list_opts_out():
    builder = SimpleNamespace(frozen_joint_substrings=["finger_joint"])
    explicit = _planner(builder, freeze_joint_substrings=["knuckle"])
    assert explicit.freeze_joint_substrings == ["knuckle"]
    assert _planner(builder, freeze_joint_substrings=[]).freeze_joint_substrings == []


def test_graph_builder_without_record_freezes_nothing():
    assert _planner(SimpleNamespace()).freeze_joint_substrings == []
