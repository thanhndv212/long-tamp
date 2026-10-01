"""The web mission viewer (#24): plan and pause events, the replay page, its
configuration, and the live server."""

import json
import re
import threading
import time
import urllib.error
import urllib.request

import pytest

from long_tamp.execution import ExecutionControl, MockBackend
from long_tamp.execution.executor import PlanExecutor
from long_tamp.tasks.task_planning.events import (
    PAUSE_ROLE,
    PLAN_ROLE,
    JsonlEventWriter,
    plan_event,
)
from long_tamp.tasks.task_planning.host import create_fake_session
from long_tamp.tasks.task_planning.runner import run_plan
from long_tamp.viewer import (
    PANELS,
    EventTail,
    ViewerConfig,
    ViewerServer,
    load_run,
    render_html,
    write_replay,
)
from long_tamp.viewer.__main__ import main


def _boot(html):
    match = re.search(
        r'<script id="boot" type="application/json">(.*?)</script>', html, re.S
    )
    assert match, "no boot data"
    return json.loads(match.group(1))


@pytest.fixture
def run_dir(tmp_path):
    """A recorded run: a plan event, then the plan's transitions."""
    session = create_fake_session('{"shape": "composite"}')
    with JsonlEventWriter(tmp_path / "events.jsonl") as write:
        write(plan_event(session.plan))
        run_plan(session, on_event=write)
    return tmp_path


def test_a_plan_event_carries_the_plan_and_its_attempt_budgets():
    session = create_fake_session('{"shape": "composite"}')
    event = plan_event(session.plan, "replanned")
    assert event["role"] == PLAN_ROLE and event["status"] == "SUCCESS"
    assert event["plan"] == session.plan.document
    assert event["ir_id"] == session.plan.document["root"]["id"]
    assert event["metrics"]["fingerprint"] == session.plan.plan_fingerprint
    assert event["metrics"]["attempts"] == session.plan.effective_attempts
    assert event["message"] == "replanned"
    # a bare IR document works too, without the TaskPlan's extras
    bare = plan_event(session.plan.document)
    assert bare["plan"] == session.plan.document and "metrics" not in bare


def test_run_plan_alone_emits_no_plan_event():
    events = []
    run_plan(create_fake_session("{}"), on_event=events.append)
    assert PLAN_ROLE not in {e["role"] for e in events}


def test_a_breakpoint_pause_emits_pause_then_resume_events():
    session = create_fake_session("{}")
    control = ExecutionControl()
    control.add_breakpoint("move-home", "before")
    events = []
    executor = PlanExecutor(
        session,
        backend=MockBackend(rtf=1000.0),
        control=control,
        on_event=events.append,
    )
    result = {}
    worker = threading.Thread(target=lambda: result.update(run=executor.run()))
    worker.start()
    deadline = time.time() + 5
    while control.waiting_at is None and time.time() < deadline:
        time.sleep(0.01)
    assert control.waiting_at == ("move-home", "before")
    control.resume()
    worker.join(5)
    assert result["run"].success
    pauses = [e for e in events if e["role"] == PAUSE_ROLE]
    assert [(e["status"], e["previous"]) for e in pauses] == [
        ("RUNNING", "IDLE"),
        ("SUCCESS", "RUNNING"),
    ]
    assert pauses[0]["ir_id"] == "move-home"
    assert pauses[0]["metrics"] == {"when": "before"}


def test_a_stop_while_paused_ends_the_pause_with_a_failure():
    control = ExecutionControl()
    seen = []
    control.on_wait = lambda step, when, waiting: seen.append((step, when, waiting))
    control.pause()
    threading.Timer(0.05, control.stop).start()
    assert control.checkpoint("s", "after") is False
    assert seen == [("s", "after", True), ("s", "after", False)]


def test_the_replay_page_embeds_the_run_and_needs_no_network(run_dir):
    out = write_replay(run_dir)
    assert out == run_dir / "viewer.html"
    html = out.read_text()
    boot = _boot(html)
    assert not boot["live"]
    assert boot["events"][0]["role"] == PLAN_ROLE
    assert len(boot["events"]) == len(load_run(run_dir))
    assert "/*" + "VIEWER_JS" + "*/" not in html and "LongTamp.start()" in html
    # self-contained: no external scripts, styles or fonts
    assert not re.search(r'(src|href)="https?://', html)


def test_text_from_the_stream_cannot_close_the_script(tmp_path):
    with JsonlEventWriter(tmp_path / "events.jsonl") as write:
        session = create_fake_session("{}")
        write(plan_event(session.plan, "</script><script>alert(1)</script>"))
    html = write_replay(tmp_path).read_text()
    assert "<script>alert(1)" not in html
    assert _boot(html)["events"][0]["message"].startswith("</script>")


def test_a_plan_file_can_be_given_for_streams_without_one(tmp_path):
    session = create_fake_session("{}")
    with JsonlEventWriter(tmp_path / "events.jsonl") as write:
        run_plan(session, on_event=write)
    plan_file = tmp_path / "plan.json"
    plan_file.write_text(json.dumps(session.plan.document))
    events = load_run(tmp_path / "events.jsonl", plan=plan_file)
    assert events[0]["role"] == PLAN_ROLE
    assert events[0]["t"] == events[1]["t"]  # the timeline starts with the run


def test_config_from_json_with_custom_panels_and_scripts(tmp_path):
    (tmp_path / "panel.js").write_text(
        "LongTamp.addPanel({id: 'grip', render: function () {}});"
    )
    (tmp_path / "theme.css").write_text(":root { --accent: #ff00aa; }")
    (tmp_path / "viewer.json").write_text(
        json.dumps(
            {
                "title": "Cell 3 <night shift>",
                "panels": ["timeline", "plan", "grip"],
                "hidden_roles": ["motion"],
                "status_colors": {"failure": "#ff0000"},
                "metric_labels": {"seconds": "planning (s)"},
                "extra_js": ["panel.js"],
                "extra_css": ["theme.css"],
            }
        )
    )
    config = ViewerConfig.load(tmp_path / "viewer.json")
    html = render_html([], config)
    settings = _boot(html)["config"]
    assert settings["panels"] == ["timeline", "plan", "grip"]
    assert settings["status_colors"] == {"failure": "#ff0000"}
    assert "extra_js" not in settings  # inlined, not sent as paths
    assert "LongTamp.addPanel({id: 'grip'" in html
    assert "--accent: #ff00aa" in html
    assert "<title>Cell 3 &lt;night shift&gt;</title>" in html


def test_unknown_config_keys_are_rejected():
    with pytest.raises(ValueError, match="colour"):
        ViewerConfig.from_dict({"colour": "red"})
<<<<<<< HEAD
    with pytest.raises(ValueError, match="layout"):
        ViewerConfig.from_dict({"layout": "grid"})
    config = ViewerConfig.from_dict({"layout": "page", "screen": {"left": ["plan"]}})
    assert config.page_settings()["screen"] == {"left": ["plan"]}
    assert ViewerConfig().layout == "screen"
=======
>>>>>>> origin/dev
    assert ViewerConfig().panels == list(PANELS)


def test_the_tail_leaves_a_partly_written_line_for_later(tmp_path):
    path = tmp_path / "events.jsonl"
    tail = EventTail(path)
    assert tail.since(0) == []  # not written yet
    path.write_text('{"a": 1}\n{"b": ')
    assert tail.since(0) == [{"a": 1}]
    with open(path, "a") as f:
        f.write("2}\n")
    assert tail.since(0) == [{"a": 1}, {"b": 2}]
    assert tail.since(1) == [{"b": 2}]


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.status, r.read().decode()


def _post(url, body):
    request = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json"}, method="POST"
    )
    try:
        with urllib.request.urlopen(request, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as error:
        return error.code, json.loads(error.read())


def test_the_live_server_serves_the_page_events_and_control(run_dir):
    control = ExecutionControl()
    with ViewerServer(run_dir / "events.jsonl", port=0, control=control) as server:
        url = server.url
        assert url.startswith("http://localhost:")
        status, html = _get(url)
        boot = _boot(html)
        assert status == 200 and boot["live"] and boot["control"]
        assert boot["events"] == []  # fetched, not embedded
        status, body = _get(url + "api/events?since=0")
        events = json.loads(body)["events"]
        assert events[0]["role"] == PLAN_ROLE
        status, body = _get(url + f"api/events?since={len(events) - 2}")
        assert len(json.loads(body)["events"]) == 2

        status, body = _post(url + "api/control", b'{"action": "pause"}')
        assert status == 200 and body["paused"] and control.paused
        assert _post(url + "api/control", b'{"action": "resume"}')[1]["paused"] is False
        assert _post(url + "api/control", b'{"action": "jump"}')[0] == 400
        assert _post(url + "api/control", b"not json")[0] == 400
        assert _post(url + "api/nothing", b"{}")[0] == 404


def test_the_live_server_has_no_control_route_without_a_control(run_dir):
    with ViewerServer(run_dir / "events.jsonl", port=0) as server:
        assert not _boot(_get(server.url)[1])["control"]
        assert _post(server.url + "api/control", b'{"action": "stop"}')[0] == 404


def test_a_separate_process_serves_the_page_while_this_one_is_busy(run_dir):
    control = ExecutionControl()
    server = ViewerServer(
        run_dir / "events.jsonl", port=0, control=control, separate_process=True
    )
    with server:
        url = server.url
        assert server._front is not None and server._front.poll() is None
        assert _boot(_get(url)[1])["live"]
        events = json.loads(_get(url + "api/events?since=0")[1])["events"]
        assert events[0]["role"] == PLAN_ROLE
        features = json.loads(_get(url + "api/features")[1])  # proxied
        assert features == {"control": True, "chat": False}
        assert _post(url + "api/control", b'{"action": "pause"}')[0] == 200
        assert control.paused
        assert _post(url + "api/control", b'{"action": "jump"}')[0] == 400
        assert _post(url + "api/nothing", b"{}")[0] == 404
        front = server._front
    assert front.poll() is not None  # closed with the server


def test_extra_routes(run_dir):
    with ViewerServer(run_dir / "events.jsonl", port=0) as server:
        server.route("POST", "/api/echo", lambda body, query: {"got": body})
        assert _post(server.url + "api/echo", b'{"x": 1}') == (200, {"got": {"x": 1}})


def test_the_command_line_writes_a_replay(run_dir, tmp_path, capsys):
    out = tmp_path / "page.html"
    assert main(["replay", str(run_dir), "-o", str(out), "--title", "Run 7"]) == 0
    assert "Run 7" in out.read_text()
    assert str(out) in capsys.readouterr().out
