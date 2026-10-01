"""Render a TaskPlan as a Mermaid or Graphviz diagram (pure Python).

Useful to review a plan before running it, generated plans especially, and to
illustrate one in docs. Node kinds get distinct shapes::

    sequence     → label          rectangle
    fallback     ? label          hexagon; alternatives after the first child
                                  are dashed "else" edges (recovery branches)
    retry        ↻ ×N label       stadium
    condition    label?           diamond
    transaction  label            rectangle: capability(parameters) ×attempts
    operation    label            rectangle: capability(parameters)

Pass the plan's ``registry`` to also show each step's grounded effects.
"""

from __future__ import annotations

import re
from typing import Any

from .capabilities import CapabilityRegistry
from .model import TaskPlan, grounded_literals


def _walk(node: dict[str, Any]):
    """(node, parent, is_alternative) in depth-first order."""
    stack = [(node, None, False)]
    while stack:
        current, parent, alternative = stack.pop()
        yield current, parent, alternative
        children = list(current.get("children", []))
        if "child" in current:
            children = [current["child"]]
        for index, child in reversed(list(enumerate(children))):
            stack.append((child, current, current["type"] == "fallback" and index > 0))


def _call(node: dict[str, Any]) -> str:
    parameters = node.get("parameters", {})
    args = ", ".join(str(parameters[key]) for key in sorted(parameters))
    return f"{node['capability']}({args})"


def _lines(
    plan: TaskPlan, node: dict[str, Any], registry: CapabilityRegistry | None
) -> tuple[str, list[str]]:
    """(kind, text lines) describing ``node``."""
    kind = node["type"]
    label = node.get("label", node["id"])
    attempts = plan.effective_attempts.get(node["id"], 1)
    if kind == "sequence":
        return kind, [f"→ {label}"]
    if kind == "parallel":
        return kind, [f"⇉ {label}"]
    if kind == "fallback":
        return kind, [f"? {label}"]
    if kind == "retry":
        return kind, [f"↻ ×{attempts} {label}"]
    if kind == "condition":
        return kind, [f"{label}?"]
    executable = node["children"][0] if kind == "transaction" else node
    lines = [label, _call(executable) + (f" ×{attempts}" if attempts > 1 else "")]
    if registry is not None:
        _, effects = grounded_literals(
            executable, registry.descriptor(executable["capability"])
        )
        lines += [f"⇒ {effect}" for effect in effects]
    return kind, lines


def _mermaid_id(node_id: str, used: dict[str, str]) -> str:
    base = re.sub(r"[^A-Za-z0-9_]", "_", node_id)
    candidate, n = base, 1
    while candidate in used.values():
        n += 1
        candidate = f"{base}_{n}"
    used[node_id] = candidate
    return candidate


def to_mermaid(plan: TaskPlan, registry: CapabilityRegistry | None = None) -> str:
    """A Mermaid ``flowchart TD`` of the plan (top-down, one box per node)."""
    shapes = {
        "sequence": '["{}"]',
        "parallel": '[["{}"]]',
        "fallback": '{{{{"{}"}}}}',
        "retry": '(["{}"])',
        "condition": '{{"{}"}}',
        "transaction": '["{}"]',
        "operation": '["{}"]',
    }
    ids: dict[str, str] = {}
    nodes, edges = [], []
    for node, parent, alternative in _walk(plan.document["root"]):
        if node["type"] == "operation" and parent and parent["type"] == "transaction":
            continue  # shown inside its transaction
        kind, lines = _lines(plan, node, registry)
        text = "<br/>".join(line.replace('"', "#quot;") for line in lines)
        mid = _mermaid_id(node["id"], ids)
        nodes.append(f"    {mid}{shapes[kind].format(text)}")
        if parent is not None:
            arrow = "-.->|else|" if alternative else "-->"
            edges.append(f"    {ids[parent['id']]} {arrow} {mid}")
    return "\n".join(["flowchart TD", *nodes, *edges]) + "\n"


def to_dot(plan: TaskPlan, registry: CapabilityRegistry | None = None) -> str:
    """A Graphviz ``digraph`` of the plan, with the same shapes as Mermaid."""
    shapes = {
        "sequence": "box",
        "parallel": "box, peripheries=2",
        "fallback": "hexagon",
        "retry": "box, style=rounded",
        "condition": "diamond",
        "transaction": "box",
        "operation": "box",
    }

    def quote(text: str) -> str:
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'

    lines = [f"digraph {quote(plan.document['mission_id'])} {{", "    rankdir=TB;"]
    edges = []
    for node, parent, alternative in _walk(plan.document["root"]):
        if node["type"] == "operation" and parent and parent["type"] == "transaction":
            continue
        kind, text = _lines(plan, node, registry)
        label = quote("\\n".join(text))
        lines.append(f"    {quote(node['id'])} [label={label}, shape={shapes[kind]}];")
        if parent is not None:
            style = ' [style=dashed, label="else"]' if alternative else ""
            edges.append(f"    {quote(parent['id'])} -> {quote(node['id'])}{style};")
    return "\n".join([*lines, *edges, "}"]) + "\n"
