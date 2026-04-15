"""
Unit tests for RunLogger and related log utilities.

These tests require no HPP backend — they only exercise the pure-Python
logging infrastructure.
"""

import json
import os
import tempfile
from typing import Any, Dict

import pytest

from agimus_spacelab.logging import (
    RunLogger,
    load_run_log,
    iter_events,
    get_replay_config,
)
from agimus_spacelab.logging.run_logger import (
    _make_serializable,
    _serialize_task_config,
)


# ---------------------------------------------------------------------------
# _make_serializable helpers
# ---------------------------------------------------------------------------


class TestMakeSerializable:
    def test_primitives(self):
        assert _make_serializable(None) is None
        assert _make_serializable(True) is True
        assert _make_serializable(42) == 42
        assert _make_serializable(3.14) == 3.14
        assert _make_serializable("hello") == "hello"

    def test_list(self):
        assert _make_serializable([1, 2, 3]) == [1, 2, 3]

    def test_tuple(self):
        assert _make_serializable((1, 2)) == [1, 2]

    def test_dict(self):
        assert _make_serializable({"a": 1, "b": [2, 3]}) == {
            "a": 1,
            "b": [2, 3],
        }

    def test_dict_key_coercion(self):
        # Non-string keys should become strings
        result = _make_serializable({1: "one"})
        assert result == {"1": "one"}

    def test_nested(self):
        obj = {"x": [{"y": 1}]}
        assert _make_serializable(obj) == {"x": [{"y": 1}]}

    def test_numpy_scalar(self):
        pytest.importorskip("numpy")
        import numpy as np

        assert _make_serializable(np.int64(7)) == 7
        assert _make_serializable(np.float32(1.5)) == pytest.approx(
            1.5, abs=1e-5
        )

    def test_numpy_array(self):
        pytest.importorskip("numpy")
        import numpy as np

        arr = np.array([1.0, 2.0, 3.0])
        assert _make_serializable(arr) == [1.0, 2.0, 3.0]

    def test_dataclass(self):
        from dataclasses import dataclass

        @dataclass
        class Foo:
            x: int
            y: str

        assert _make_serializable(Foo(x=1, y="a")) == {"x": 1, "y": "a"}

    def test_object_with_dict(self):
        class Bar:
            def __init__(self):
                self.a = 10
                self.b = "hello"
                self._private = "skip"

        result = _make_serializable(Bar())
        assert result["a"] == 10
        assert result["b"] == "hello"
        # Private attrs are NOT excluded by _make_serializable but by
        # _serialize_task_config; _make_serializable uses __dict__ wholesale.

    def test_object_with_empty_dict_serialises_as_dict(self):
        # Objects with an empty __dict__ are serialised as {} (not a string),
        # because the __dict__ branch fires before the str() fallback.
        class Obj:
            pass

        result = _make_serializable(Obj())
        assert result == {}

    def test_fallback_to_str_for_unserializable(self):
        # Objects that raise inside __dict__ access fall through to str().
        class Unserializable:
            @property
            def __dict__(self):
                raise RuntimeError("no dict")

            def __str__(self):
                return "unserializable_object"

        result = _make_serializable(Unserializable())
        assert result == "unserializable_object"


# ---------------------------------------------------------------------------
# _serialize_task_config
# ---------------------------------------------------------------------------


class TestSerializeTaskConfig:
    def test_none(self):
        assert _serialize_task_config(None) == {}

    def test_class_attrs(self):
        class MyConfig:
            ROBOT_NAMES = ["robot_a", "robot_b"]
            GRIPPERS = ["g1", "g2"]
            MAX_ITERATIONS = 500

            def _private(self):
                pass

            def method(self):
                pass

        result = _serialize_task_config(MyConfig)
        assert result["ROBOT_NAMES"] == ["robot_a", "robot_b"]
        assert result["GRIPPERS"] == ["g1", "g2"]
        assert result["MAX_ITERATIONS"] == 500
        assert "_private" not in result

    def test_instance_attrs(self):
        class DynConfig:
            GRIPPERS = ["g1"]

            def __init__(self):
                self.dynamic_field = "hello"

        obj = DynConfig()
        result = _serialize_task_config(obj)
        assert result["GRIPPERS"] == ["g1"]
        assert result["dynamic_field"] == "hello"


# ---------------------------------------------------------------------------
# RunLogger
# ---------------------------------------------------------------------------


class TestRunLogger:
    def test_creates_jsonl_file(self, tmp_path):
        logger = RunLogger(str(tmp_path))
        assert os.path.exists(logger.jsonl_path)
        logger.close()

    def test_log_writes_event(self, tmp_path):
        logger = RunLogger(str(tmp_path))
        logger.log("run_start", task_name="Test", backend="pyhpp")
        logger.close()

        events = list(iter_events(logger.jsonl_path))
        assert len(events) == 1
        assert events[0]["event"] == "run_start"
        assert events[0]["task_name"] == "Test"
        assert events[0]["backend"] == "pyhpp"

    def test_multiple_events(self, tmp_path):
        logger = RunLogger(str(tmp_path))
        logger.log("run_start", task_name="demo")
        logger.log("sequence_start", q_init=[0.0, 1.0])
        logger.log("run_end", success=True, total_time=5.0)
        logger.close()

        events = list(iter_events(logger.jsonl_path))
        assert len(events) == 3
        assert events[0]["event"] == "run_start"
        assert events[1]["event"] == "sequence_start"
        assert events[2]["event"] == "run_end"

    def test_run_id_consistent(self, tmp_path):
        logger = RunLogger(str(tmp_path), run_id="abc12345")
        logger.log("run_start", task_name="x")
        logger.close()

        events = list(iter_events(logger.jsonl_path))
        assert events[0]["run_id"] == "abc12345"

    def test_snapshot_written_on_close(self, tmp_path):
        logger = RunLogger(str(tmp_path))
        logger.log("run_start", task_name="snap_test")
        logger.close()

        assert os.path.exists(logger.snapshot_path)
        with open(logger.snapshot_path) as f:
            snap = json.load(f)
        assert "run_start" in snap
        assert snap["run_start"]["task_name"] == "snap_test"

    def test_context_manager(self, tmp_path):
        with RunLogger(str(tmp_path)) as logger:
            logger.log("run_start", task_name="ctx_test")
        assert os.path.exists(logger.snapshot_path)

    def test_close_idempotent(self, tmp_path):
        logger = RunLogger(str(tmp_path))
        logger.log("run_start", task_name="x")
        logger.close()
        logger.close()  # second close should not raise

    def test_log_after_close_is_noop(self, tmp_path):
        logger = RunLogger(str(tmp_path))
        logger.log("run_start", task_name="x")
        logger.close()
        logger.log("extra_event")  # should not raise or write

        events = list(iter_events(logger.jsonl_path))
        assert len(events) == 1  # only the run_start

    def test_log_task_config(self, tmp_path):
        class Cfg:
            GRIPPERS = ["g1", "g2"]
            MAX_ITERATIONS = 100

        logger = RunLogger(str(tmp_path))
        logger.log_task_config(
            Cfg,
            setup_params={"validation_step": 0.01},
            backend="pyhpp",
            task_name="cfg_test",
        )
        logger.close()

        events = list(iter_events(logger.jsonl_path))
        assert events[0]["event"] == "config_snapshot"
        assert events[0]["task_config"]["GRIPPERS"] == ["g1", "g2"]
        assert events[0]["setup_params"]["validation_step"] == 0.01

    def test_serialises_numpy(self, tmp_path):
        pytest.importorskip("numpy")
        import numpy as np

        logger = RunLogger(str(tmp_path))
        logger.log("sequence_start", q_init=np.array([1.0, 2.0, 3.0]))
        logger.close()

        events = list(iter_events(logger.jsonl_path))
        assert events[0]["q_init"] == [1.0, 2.0, 3.0]


# ---------------------------------------------------------------------------
# load_run_log / get_replay_config
# ---------------------------------------------------------------------------


class TestLogLoader:
    def _write_sample_log(self, tmp_path) -> str:
        logger = RunLogger(str(tmp_path))
        logger.log("run_start", task_name="loader_test", backend="pyhpp")
        logger.log(
            "config_snapshot",
            task_config={"GRIPPERS": ["g1"]},
            setup_params={"validation_step": 0.01},
        )
        logger.log(
            "sequence_start",
            grasp_sequence=[["g1", "h1"]],
            q_init=[0.0, 0.0],
            validate=True,
            max_iterations_per_edge=1000,
            timeout_per_edge=60.0,
            frozen_arms_mode="auto",
            time_parameterize=True,
            reset_roadmap=True,
        )
        logger.log(
            "phase_end",
            phase=1,
            gripper="g1",
            handle="h1",
            success=True,
            phase_time=3.5,
            phase_gen_time=0.5,
            phase_plan_time=3.0,
            final_config=[1.0, 2.0],
            state_after="grasped",
            saved_files=[],
            error=None,
        )
        logger.log(
            "run_end",
            success=True,
            total_time=4.0,
            total_planning_time=3.5,
            phase_count=1,
            final_config=[1.0, 2.0],
            error=None,
        )
        logger.close()
        return logger.jsonl_path

    def test_load_jsonl(self, tmp_path):
        path = self._write_sample_log(tmp_path)
        run = load_run_log(path)
        assert run["run_id"] is not None
        assert run["run_start"]["task_name"] == "loader_test"
        assert run["run_end"]["success"] is True
        assert len(run["phase_results"]) == 1

    def test_load_snapshot_json(self, tmp_path):
        path = self._write_sample_log(tmp_path)
        snap_path = path.replace(".jsonl", ".json")
        run = load_run_log(snap_path)
        assert run["run_start"]["task_name"] == "loader_test"

    def test_get_replay_config(self, tmp_path):
        path = self._write_sample_log(tmp_path)
        cfg = get_replay_config(path)
        assert cfg["backend"] == "pyhpp"
        assert cfg["task_name"] == "loader_test"
        assert cfg["task_config"]["GRIPPERS"] == ["g1"]
        assert cfg["sequence"]["grasp_sequence"] == [["g1", "h1"]]
        assert cfg["sequence"]["q_init"] == [0.0, 0.0]

    def test_iter_events_skips_bad_lines(self, tmp_path):
        p = tmp_path / "bad.jsonl"
        p.write_text('{"event": "ok"}\nNOT JSON\n{"event": "also_ok"}\n')
        events = list(iter_events(str(p)))
        assert len(events) == 2
        assert events[0]["event"] == "ok"
        assert events[1]["event"] == "also_ok"
