# Mission viewer

`long_tamp.viewer` shows a mission in a web browser, built from its
[event stream](events.md). It works in two ways:

- **Replay.** One self-contained HTML file per run, with the events embedded and no
  network access needed. You can open it from disk, attach it to an issue or archive it with
  the run.
- **Live.** A small server (standard library only) runs next to the mission. It follows
  `events.jsonl` as the file grows, and it can pause, resume and stop the mission.

The Python executor and the BehaviorTree.CPP host write the same stream, so the viewer works
for both.

```bash
# replay a recorded run: writes <run>/viewer.html
python -m long_tamp.viewer replay runs/seed3_20261001_221500 --open

# follow a run as it happens
python -m long_tamp.viewer serve runs/seed3_20261001_221500 --port 8090

# the screw assembly serves it while it runs, with pause/resume/stop
python task_screw_assembly.py --backend mujoco --web-port 8090
```

## What it shows

<<<<<<< HEAD
By default everything fits on one screen:

- **Top:** the summary.
- **Left:** the plan tree, with Details and Events as tabs below it.
- **Center:** the scene, with the timeline under it.
- **Right:** the chat.

Each panel scrolls inside itself, and you can rearrange the areas with `screen` (below).

=======
>>>>>>> origin/dev
| Panel | What it shows |
|---|---|
| **Summary** | Elapsed time; steps done, skipped and failed out of the total; retries; time spent planning and in motion; drift replans; pauses. When a model took part, it adds model calls and tokens, tool calls, and the number of plans. |
| **Timeline** | Play, pause and seek controls with a speed setting (1×–600×, or one event at a time), and *follow live* on a live page. Below them, a Gantt bar per step, coloured by its state, with a cursor at the current position. Parallel lanes show up as overlapping bars. |
| **Plan** | The plan tree: sequences (→), fallbacks (?), retries (↻), parallel groups (⇉), conditions (◇) and steps, each with its `capability(parameters)` and state. Badges show the attempt (k/N), *skipped: completed_this_run* when the step's effect already held, the motion time, *moving*, the number of drift replans, and ⏸ while paused there. Branches fold, and the running node scrolls into view. |
| **Details** | The selected node: its call, state, attempts, why it was skipped, planning and motion time, start time and duration, failure messages, and all of its events. |
| **Events** | The raw stream up to the cursor. You can filter it by text, show only failures, or toggle each role; `ready` and `precondition` are hidden by default. Click an event to move the cursor there. |
| **Chat** | On a live page with a chat session: the operator chat with the mission model (see [AI models](ai-models.md)). It shows your messages, each tool call (accepted calls in green, rejected ones in red, with the reason) and the model's answers. Turns typed in the terminal show up here too. |
| **Scene** | Another page embedded next to the rest, normally the Viser server the mission plays on (`scene_url`). |

Everything is computed from the events up to the cursor. Scrubbing back shows the mission as
it was at that moment: which steps were running, which had failed, and where it was paused.

### Events the viewer uses

Two event roles exist for the viewer (see [Mission events](events.md)):

- `plan`: the plan about to run, with its IR document in the event's `plan` field. It is
  opt-in: call `plan_event(task_plan)` before running the plan, and again when a new plan
  replaces it after a replan or a chat edit. The screw assembly does both. Without a `plan`
  event, the viewer infers the tree from the transitions (from which composite was running
  when each node started). For an old run, pass the plan with `replay --plan plan.json`.
- `pause`: emitted by the executor when an `ExecutionControl` holds it at a step boundary
  (`RUNNING`), and again when it resumes (`SUCCESS`) or stops (`FAILURE`).

## Customizing it

### Settings: `ViewerConfig`

The settings are plain data. You can set them in Python or keep them in a JSON file
(`--config viewer.json`):

```json
{
  "title": "Cell 3: screw assembly",
  "panels": ["summary", "timeline", "plan", "details", "events", "gripper"],
  "hidden_roles": ["ready", "precondition", "attempts"],
  "status_colors": {"failure": "#ff3b30", "running": "#0a84ff"},
  "theme": {"--accent": "#7c3aed"},
  "metric_labels": {"seconds": "planning (s)", "duration": "motion (s)"},
  "scene_url": "http://localhost:8081",
  "extra_css": ["cell3.css"],
  "extra_js": ["gripper_panel.js"],
  "poll_ms": 500
}
```

| Setting | Effect |
|---|---|
| `title` | The page title. |
| `panels` | Which panels to show, and in what order. These can be built-in panels (`summary`, `timeline`, `plan`, `details`, `events`, `scene`) or panels your own scripts register. Registered panels you don't list are added at the end. |
| `hidden_roles` | Event roles that start unchecked in the event list. The plan tree still uses them. |
| `status_colors` | A color per state: `running`, `success`, `failure`, `skipped`, `paused` or `idle`. |
| `theme` | CSS variables (`--bg`, `--panel`, `--fg`, `--muted`, `--border`, `--accent`) that override the built-in light and dark themes. |
| `metric_labels` | Display names for event metrics, such as backend-specific ones. |
| `scene_url` | The page shown in the Scene panel. |
| `extra_css`, `extra_js` | Files inlined into the page after the built-in style and script. A replay stays a single file. Relative paths are resolved against the JSON file's folder. |
| `poll_ms` | How often a live page asks the server for new events. |
<<<<<<< HEAD
| `layout` | `"screen"` (default) fits everything on one screen and each panel scrolls inside. Below 1000 px wide, the panels stack. `"page"` stacks the panels in a scrolling page. |
| `screen` | Where each panel goes on one screen, by area: `top`, `left`, `center`, `right`. An inner list is a group of tabs. The default is `{"top": ["summary"], "left": ["plan", ["details", "events"]], "center": ["scene", "timeline"], "right": ["chat"]}`. Panels you don't place join the left tabs. Without a scene, the tabs move to the center. |
=======
>>>>>>> origin/dev

### Your own panels, badges and formats: `window.LongTamp`

A script in `extra_js` can extend the page. Everything a panel needs is in `view`: the state
after the events up to the cursor.

```js
// gripper_panel.js: a panel and a badge for a backend that reports grip force
LongTamp.addPanel({
  id: "gripper",
  title: "Grip force",
  render: function (view, el) {
    var last = LongTamp.events.slice(0, LongTamp.cursor).reverse()
      .find(function (e) { return e.role === "motion" && e.metrics && e.metrics.grip_force; });
    el.textContent = last ? last.metrics.grip_force.toFixed(1) + " N" : "no reading yet";
  },
});

// a badge on every step that took more than 3 attempts
LongTamp.addBadge(function (node) {
  return node.attempt > 3 ? { text: "flaky", cls: "bad" } : null;
});

// show a metric in the unit you want
LongTamp.formatMetric("tracking_error", function (v) { return (v * 1000).toFixed(1) + " mm"; });

// react to each event as it is applied (live or replay), e.g. to raise an alert
LongTamp.onEvent(function (event, view) {
  if (event.role === "drift") console.warn("drift at", event.ir_id);
});
```

| API | Use |
|---|---|
| `addPanel({id, title, wide, render(view, element)})` | Adds a panel. `render` runs on every update, with an emptied `element`. |
| `addBadge(fn(node, view))` | Adds badges to plan nodes. Return a string, `{text, cls}` (`cls`: `warn`, `bad`) or `null`. |
| `formatMetric(key, fn(value, event))` | Sets how a metric is displayed in the event list. |
| `onEvent(fn(event, view))` | Runs a function on each event as it is applied. |
| `view.nodes[id]` | The state of each node: `status`, `attempt`, `maxAttempts`, `skipped`, `skipReason`, `drifts`, `paused`, `moving`, `motionSeconds`, `planSeconds`, `started`, `ended`, `failures`, `def` (its IR node). |
| `LongTamp.nodeState(node)` | The node's state: `running`, `success`, `failure`, `skipped`, `paused` or `idle`. |
| `LongTamp.events`, `LongTamp.cursor`, `LongTamp.select(id)`, `LongTamp.seek(i)` | The stream, the current position, and navigation. |

Your own events show up too. Anything a sink writes, such as a `role` the viewer doesn't
know (for example `make_event("vision", "detection", ...)`), lands in the event list with
its own role toggle. Your panels can read those events.

<<<<<<< HEAD
### The mission UI

One command starts the whole thing (in the HPP environment):

```bash
cd script/screw_assembly
python mission_ui.py                     # MuJoCo; --backend playback for planned paths only
```

It prints the page's URL (and opens a browser, outside a container). Then:

1. In the chat, type an instruction, such as *assemble parts 1 and 2, with part 2 first*. The
   model writes the goal, and the task planner builds the plan.
2. The plan shows as a card in the chat, and in the plan tree, with a **Start mission**
   button.
3. **Start mission** runs it without asking the model, and brings the plan monitor into view.
   The plan tree, the timeline and the events follow the run live, and the scene plays the
   motion. **pause**, **resume** and **stop** act at step boundaries. A stop ends that run,
   not the chat: you can change the plan and start again. The model sees the run in the
   chat history, so you can ask *why did that step fail?*

The model and its endpoint come from the AI env file (see [AI models](ai-models.md)).
`--model API:MODEL` overrides it, and arguments after `--` go to `task_screw_assembly.py`.

=======
>>>>>>> origin/dev
### The chat panel

The operator chat (`--chat`) also works from the viewer. Run it with `--web-port`:

```bash
python task_screw_assembly.py --chat --goal-model openai:my-model --web-port 8090
```

The terminal and the page share one session, and one turn runs at a time, whoever sends
it. Everything the model does goes through the same gated tools as in the terminal. Tool
calls appear in the event list (role `tool`). When the chat makes a new plan, the plan tree
switches to it through a `plan` event. When the chat runs the plan, the mission's events and
the Viser scene follow the run. After stdin closes, the chat goes on in the page until you
send `quit` there or press Ctrl-C.

In your own application, wrap any `ChatSession` and attach it to the server:

```python
from long_tamp.viewer import ChatBridge

bridge = ChatBridge(session).attach(server)  # POST/GET /api/chat
bridge.turn("plan part 2 first")             # a terminal turn, shown on the page too
<<<<<<< HEAD

# a button that acts without the model, when enabled
bridge.add_action("start", "Start mission", lambda: session.call("run", {}),
                  enabled=lambda: work.document is not None)
```

A tool result with a `plan` list of step labels shows as a plan card. The `start` action's
button sits on the latest card, and other actions sit under the chat.

=======
```

>>>>>>> origin/dev
### The live server

`ViewerServer` can be embedded in your own application:

```python
from long_tamp.execution import ExecutionControl
from long_tamp.viewer import ViewerConfig, ViewerServer

control = ExecutionControl()  # pass the same one to PlanExecutor(control=...)
server = ViewerServer("run/events.jsonl", ViewerConfig(title="Cell 3"), port=8090,
                      control=control)
print(server.start())         # http://localhost:8090/
server.route("GET", "/api/cell", lambda body, query: {"door": "closed"})
```

Its routes are JSON: `GET /api/events?since=N`, `POST /api/control` (`pause`, `resume`,
<<<<<<< HEAD
`stop`, only when a control is given), `POST /api/chat`, `GET /api/chat?since=N` and
`POST /api/action` (with a `ChatBridge`), `GET /api/features`, and whatever you add with `route(method, path,
=======
`stop`, only when a control is given), `POST /api/chat` and `GET /api/chat?since=N` (with
a `ChatBridge`), and whatever you add with `route(method, path,
>>>>>>> origin/dev
handler)`. The server binds to `127.0.0.1` by default because the control and chat routes move a
robot. Pass `host="0.0.0.0"` only on a network you trust: the server has no
authentication.
