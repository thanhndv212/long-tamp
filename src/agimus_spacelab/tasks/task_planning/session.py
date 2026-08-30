"""Persistent execution session for validated task plans."""

from __future__ import annotations

import json
import threading
from typing import Any

from .capabilities import CapabilityRegistry
from .model import TaskPlan


def _response(**values: Any) -> str:
    return json.dumps(values, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


class TaskPlanningSession:
    """Dispatch validated operation nodes through a trusted registry."""

    def __init__(self, plan: TaskPlan, registry: CapabilityRegistry) -> None:
        registry.freeze()
        self.plan = plan
        self.registry = registry
        self._nodes = self._index_nodes(plan.document["root"])
        self._completed: set[str] = set()
        self._stop_requested = threading.Event()

    @classmethod
    def _index_nodes(cls, root: dict[str, Any]) -> dict[str, dict[str, Any]]:
        nodes: dict[str, dict[str, Any]] = {}

        def visit(node: dict[str, Any]) -> None:
            nodes[node["id"]] = node
            for child in node.get("children", []):
                visit(child)
            if "child" in node:
                visit(node["child"])

        visit(root)
        return nodes

    def setup(self, options_json: str = "{}") -> str:
        options = json.loads(options_json)
        return _response(status="success", options=options)

    def get_plan(self) -> str:
        return self.plan.canonical_json

    def check_precondition(self, step_id: str) -> str:
        exists = step_id in self._nodes
        return _response(status="success" if exists else "failure", ready=exists)

    def is_step_complete(self, step_id: str) -> str:
        return _response(status="success", complete=step_id in self._completed)

    def evaluate_condition(self, step_id: str) -> str:
        node = self._nodes.get(step_id)
        if node is None or node["type"] != "condition":
            return _response(status="failure", value=False)
        try:
            implementation = self.registry.implementation(node["capability"])
            value = implementation(dict(node.get("parameters", {})))
        except Exception as error:  # noqa: BLE001 - capability boundary
            return _response(status="failure", value=False, message=str(error))
        if not isinstance(value, bool):
            return _response(
                status="failure", value=False, message="condition must return bool"
            )
        return _response(status="success", value=value)

    def execute_step(self, step_id: str) -> str:
        """Dispatch a transaction's single restartable operation.

        Atomicity contract: this framework does NOT provide generic
        snapshot/restore of session state across a failed attempt. Instead,
        every capability bound into a transaction MUST be idempotently
        restartable from the session state as it existed when the
        transaction began -- i.e. the capability itself must either (a)
        return only on success, having internally discarded any partial
        progress on failure, or (b) raise without having mutated any
        session-visible state (``self.q_current``, ``self.snapshots``,
        grasp-tracker state, recorder state, etc.).

        ``ScrewdrivingPlanningSession`` upholds this via
        ``run_block_nonstop``/``move_arm_to_target_nonstop`` in
        ``test_screwdriving_sequence.py``: their return value (not a
        side-effecting mutation) is the only thing that updates
        ``self.q_current``, so a call that raises or is retried by
        ``RetryUntilSuccessful`` never leaves the session in a partially
        mutated state -- internally, ``_replan_from_entry`` explicitly
        resets the grasp tracker and rewinds the path recorder before
        retrying from the transaction's entry configuration. This is
        validated at the plan level: ``TaskPlan`` rejects any transaction
        whose single child capability is not registered as
        ``restartable=True`` (see ``model.py``).
        """
        if step_id in self._completed:
            return _response(
                status="skipped", step_id=step_id, message="already complete"
            )
        if self._stop_requested.is_set():
            return _response(
                status="cancelled", step_id=step_id, message="stop requested"
            )
        node = self._nodes.get(step_id)
        if node is None:
            return _response(
                status="failure", step_id=step_id, message="not executable"
            )
        executable = node
        if node["type"] == "transaction":
            children = node.get("children", [])
            if len(children) != 1:
                return _response(
                    status="failure",
                    step_id=step_id,
                    message="transaction must contain one executable operation",
                )
            executable = children[0]
        if executable["type"] not in {"operation", "condition"}:
            return _response(
                status="failure", step_id=step_id, message="not executable"
            )
        try:
            implementation = self.registry.implementation(executable["capability"])
            metrics = implementation(dict(executable.get("parameters", {})))
        except Exception as error:  # noqa: BLE001 - capability boundary
            return _response(status="retry", step_id=step_id, message=str(error))
        self._completed.add(step_id)
        return _response(
            status="success",
            step_id=step_id,
            message="completed",
            metrics=metrics if isinstance(metrics, dict) else {},
        )

    def request_stop(self) -> str:
        self._stop_requested.set()
        return _response(status="success")

    def finalize(self) -> str:
        return _response(status="success", completed=sorted(self._completed))

    def get_report(self) -> str:
        return _response(
            status="success",
            plan_fingerprint=self.plan.plan_fingerprint,
            completed=sorted(self._completed),
        )
