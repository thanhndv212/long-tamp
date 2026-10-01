"""The chat panel of the web mission viewer (#90): the operator chat driven
over HTTP, one turn at a time, its tool calls in the event stream. No API
calls: the model is scripted (see test_ai_chat)."""

import json
import re
import threading
import urllib.error
import urllib.request

import pytest

from long_tamp.ai.chat import ChatSession
from long_tamp.tasks.task_planning.events import (
    PLAN_ROLE,
    JsonlEventWriter,
    make_event,
    plan_event,
)
from long_tamp.viewer import ChatBridge, ViewerServer
from tests.test_ai_chat import (
    GOAL,
    act,
    counter_tools,
    scripted_client,
    stub_mission_module,
)


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return json.loads(r.read()) if "/api/" in url else r.read().decode()


def _post(url, body):
    request = urllib.request.Request(
        url,
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def _say(server, bridge, message):
    """Send ``message`` from the page and wait for its turn to finish."""
    status, body = _post(server.url + "api/chat", {"message": message})
    assert status == 200 and body["accepted"], body
    assert bridge.wait(10)


@pytest.fixture
def server(tmp_path):
    (tmp_path / "events.jsonl").touch()
    with ViewerServer(tmp_path / "events.jsonl", port=0) as server:
        yield server


def test_the_page_has_a_chat_panel_only_with_a_bridge(server):
    boot = re.search(r'type="application/json">(.*?)</script>', _get(server.url), re.S)
    assert json.loads(boot.group(1))["chat"] is False
    client, _ = scripted_client()
    ChatBridge(ChatSession(client, [])).attach(server)
    boot = re.search(r'type="application/json">(.*?)</script>', _get(server.url), re.S)
    assert json.loads(boot.group(1))["chat"] is True


def test_a_message_from_the_page_runs_a_turn_and_shows_its_transcript(server):
    log = []
    client, _ = scripted_client(
        act("adding", ("add", {"n": 2}), done=False), act("2 added")
    )
    bridge = ChatBridge(ChatSession(client, counter_tools(log))).attach(server)
    _say(server, bridge, "add two")
    assert log == [2]
    body = _get(server.url + "api/chat?since=0")
<<<<<<< HEAD
    # each step as it came: the model's message, then the tool it called
    assert [(e["who"], e["text"][:6]) for e in body["entries"]] == [
        ("operator", "add tw"),
        ("model", "adding"),
        ("tool", 'add({"'),
        ("model", "2 adde"),
    ]
=======
    assert [e["who"] for e in body["entries"]] == ["operator", "tool", "model"]
>>>>>>> origin/dev
    assert body["entries"][0] == {
        **body["entries"][0],
        "text": "add two",
        "source": "web",
    }
<<<<<<< HEAD
    assert body["entries"][2]["ok"] and body["entries"][2]["tool"] == "add"
    assert not body["busy"] and not body["ended"]
    assert _get(server.url + "api/chat?since=3")["entries"][0]["text"] == "2 added"


def test_the_page_sees_a_turn_while_it_runs(server):
    """Messages and tool calls show as they come, not when the turn ends."""
    release, seen = threading.Event(), []

    def slow_add(n):
        seen.extend(e["who"] for e in bridge.transcript())  # mid-turn
        release.wait(5)
        return {"total": n}

    from long_tamp.ai.chat import Tool

    client, _ = scripted_client(
        act("adding now", ("add", {"n": 1}), done=False), act("done")
    )
    bridge = ChatBridge(ChatSession(client, [Tool("add", "add", slow_add)])).attach(
        server
    )
    assert _post(server.url + "api/chat", {"message": "add one"})[0] == 200
    deadline = threading.Event()
    for _ in range(100):
        if seen:
            break
        deadline.wait(0.05)
    assert seen == ["operator", "model"]  # the message before the tool ends
    mid = _get(server.url + "api/chat?since=0")
    assert mid["busy"] and mid["entries"][-1]["text"] == "adding now"
    release.set()
    assert bridge.wait(5)
    assert [e["who"] for e in bridge.transcript()] == [
        "operator",
        "model",
        "tool",
        "model",
    ]
=======
    assert body["entries"][1]["ok"] and body["entries"][1]["tool"] == "add"
    assert body["entries"][2]["text"] == "2 added"
    assert not body["busy"] and not body["ended"]
    assert _get(server.url + "api/chat?since=2")["entries"][0]["who"] == "model"
>>>>>>> origin/dev


def test_one_turn_at_a_time_whoever_sends_it(server):
    release = threading.Event()

    class SlowSession:
        def turn(self, text):
            release.wait(5)
            return type("Turn", (), {"calls": [], "error": "", "say": "done " + text})()

    bridge = ChatBridge(SlowSession()).attach(server)
    assert _post(server.url + "api/chat", {"message": "first"})[0] == 200
    status, body = _post(server.url + "api/chat", {"message": "second"})
    assert status == 400 and "still answering" in body["error"]
    assert _get(server.url + "api/chat?since=0")["busy"]
    release.set()
    assert bridge.wait(5)
    bridge.turn("from the terminal")  # the terminal shares the bridge
    texts = [(e["who"], e["text"]) for e in bridge.transcript()]
    assert texts == [
        ("operator", "first"),
        ("model", "done first"),
        ("operator", "from the terminal"),
        ("model", "done from the terminal"),
    ]


def test_bad_messages_and_quit(server):
    client, _ = scripted_client()
    bridge = ChatBridge(ChatSession(client, [])).attach(server)
    assert _post(server.url + "api/chat", {"text": "hi"})[0] == 400
    assert _post(server.url + "api/chat", {"message": "  "})[0] == 400
    status, body = _post(server.url + "api/chat", {"message": "quit"})
    assert status == 200 and body["ended"] and bridge.ended.is_set()
    assert _post(server.url + "api/chat", {"message": "hello"})[0] == 400


def test_a_failing_session_is_reported_and_the_chat_goes_on(server):
    class Broken:
        def turn(self, text):
            raise RuntimeError("gateway down")

    bridge = ChatBridge(Broken()).attach(server)
    _say(server, bridge, "hello")
    entries = _get(server.url + "api/chat?since=0")["entries"]
    assert entries[-1] == {
        **entries[-1],
        "who": "error",
        "text": "RuntimeError: gateway down",
    }
    assert not bridge.busy


def test_the_scripted_mission_conversation_works_from_the_viewer(tmp_path):
    """#90's acceptance: the terminal chat's scripted conversation (#89),
    sent from the page; tool calls and the new plan land in the event
    stream the viewer shows."""
    pytest.importorskip("unified_planning")
    from types import SimpleNamespace

    from chat_tools import INTRO, MissionChat

    T = stub_mission_module(world=[])
    task = SimpleNamespace(task_config=SimpleNamespace(VALID_PAIRS={}))
    work = MissionChat(T, task, planner=None, recorded=None, n_parts=2, mission={})
    first = ["screwed(part2, part2/h_hole1)", "screwed(part2, part2/h_hole2)"]
    client, _ = scripted_client(
        act(
            "setting the goal",
            ("set_goal", {"literals": GOAL}),
            ("plan", {}),
            done=False,
        ),
        act("planned both parts"),
        act(
            "part 2 first",
            ("plan", {"first": ["screwed(part3, part3/h_hole1)"]}),
            done=False,
        ),
        act("again", ("plan", {"first": first}), done=False),
        act("part 2 comes first now"),
        act("running", ("run", {}), done=False),
        act("it failed"),
        act("explaining", ("explain_failure", {}), done=False),
        act("clamp 2 can't reach part 2's seat"),
    )
    events = JsonlEventWriter(tmp_path / "events.jsonl")

    def on_tool(call):  # as task_screw_assembly.run_chat does
        events(
            make_event(
                "chat",
                "tool",
                call.tool,
                "SUCCESS" if call.ok else "FAILURE",
                message=call.error,
                metrics={"arguments": call.arguments},
            )
        )
        if call.tool == "plan" and call.ok and work.document is not None:
            events(plan_event(work.document, "planned in the chat"))

    session = ChatSession(client, work.tools(), INTRO, on_tool=on_tool)
    with ViewerServer(tmp_path / "events.jsonl", port=0) as server:
        bridge = ChatBridge(session).attach(server)
        for message in (
            "assemble parts 1 and 2",
            "do part 2 first",
            "run it",
            "why did that step fail?",
        ):
            _say(server, bridge, message)

        assert work.goal == GOAL and len(T.runs) == 1
        entries = _get(server.url + "api/chat?since=0")["entries"]
        assert entries[-1]["text"] == "clamp 2 can't reach part 2's seat"
        rejected = [e for e in entries if e["who"] == "tool" and not e["ok"]]
        assert len(rejected) == 1 and "not in it" in rejected[0]["text"]

        stream = _get(server.url + "api/events?since=0")["events"]
        tools = [(e["name"], e["status"]) for e in stream if e["role"] == "tool"]
        assert tools == [
            ("set_goal", "SUCCESS"),
            ("plan", "SUCCESS"),
            ("plan", "FAILURE"),
            ("plan", "SUCCESS"),
            ("run", "SUCCESS"),
            ("explain_failure", "SUCCESS"),
        ]
        plans = [e for e in stream if e["role"] == PLAN_ROLE]
        assert len(plans) == 2  # the plan tree follows the chat's plans
        clamps = [
            n["parameters"]["part"]
            for n in _walk(plans[-1]["plan"]["root"])
            if n.get("capability") == "clamp_and_screw"
        ]
        assert clamps[0] == "part2"  # the reordered plan
    events.close()


def _walk(node):
    yield node
    for child in node.get("children", []) + (
        [node["child"]] if "child" in node else []
    ):
        yield from _walk(child)
<<<<<<< HEAD


def test_an_action_runs_without_the_model_and_a_plan_shows_as_a_card(server):
    log = []
    client, prompts = scripted_client(
        act("planning", ("plan", {}), done=False), act("here is the plan")
    )
    state = {"plan": None}

    def plan():
        state["plan"] = ["grasp part1", "screw part1"]
        return {"plan": state["plan"]}

    def run():
        log.append("ran")
        state["plan"] = None
        return {"success": True}

    from long_tamp.ai.chat import Tool

    session = ChatSession(client, [Tool("plan", "plan", plan), Tool("run", "run", run)])
    bridge = ChatBridge(session).attach(server)
    bridge.add_action(
        "start",
        "Start mission",
        lambda: session.call("run", {}),
        enabled=lambda: state["plan"] is not None,
    )
    body = _get(server.url + "api/chat?since=0")
    assert body["actions"] == [
        {"name": "start", "label": "Start mission", "enabled": False}
    ]
    status, error = _post(server.url + "api/action", {"name": "start"})
    assert status == 400 and "not available now" in error["error"]

    _say(server, bridge, "plan it")
    entries = _get(server.url + "api/chat?since=0")["entries"]
    card = [e for e in entries if e["who"] == "plan"]
    assert card and card[0]["steps"] == ["grasp part1", "screw part1"]
    assert _get(server.url + "api/chat?since=0")["actions"][0]["enabled"]

    calls_before = len(prompts)
    assert _post(server.url + "api/action", {"name": "start"})[0] == 200
    assert bridge.wait(5)
    assert log == ["ran"] and len(prompts) == calls_before  # no model call
    entries = _get(server.url + "api/chat?since=0")["entries"]
    assert entries[-2] == {**entries[-2], "who": "operator", "action": "start"}
    assert entries[-1]["who"] == "tool" and entries[-1]["ok"]
    assert not _get(server.url + "api/chat?since=0")["actions"][0]["enabled"]
    assert _post(server.url + "api/action", {"name": "nope"})[0] == 400


def test_an_action_waits_for_a_running_turn(server):
    release = threading.Event()

    class Slow:
        def turn(self, text):
            release.wait(5)
            return type("Turn", (), {"calls": [], "error": "", "say": "ok"})()

    bridge = ChatBridge(Slow()).attach(server)
    bridge.add_action("start", "Start mission", lambda: "started")
    assert _post(server.url + "api/chat", {"message": "hi"})[0] == 200
    status, body = _post(server.url + "api/action", {"name": "start"})
    assert status == 400 and "still running" in body["error"]
    release.set()
    assert bridge.wait(5)
    assert _post(server.url + "api/action", {"name": "start"})[0] == 200
    assert bridge.wait(5)
    assert bridge.transcript()[-1] == {
        **bridge.transcript()[-1],
        "who": "system",
        "text": "started",
    }


def test_reset_clears_a_stop_for_the_next_run():
    from long_tamp.execution import ExecutionControl

    control = ExecutionControl()
    control.pause()
    control.stop()
    assert control.checkpoint("s", "before") is False
    control.reset()
    assert control.checkpoint("s", "before") is True and not control.paused
=======
>>>>>>> origin/dev
