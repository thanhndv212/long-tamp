/* long-tamp mission viewer (#24): the plan tree, a timeline, the event list
 * and per-node details, computed from the event stream
 * (docs/usage/events.md). No dependencies.
 *
 * Extension API (window.LongTamp), for ViewerConfig.extra_js files:
 *   LongTamp.addPanel({id, title, wide, render(view, element)})
 *   LongTamp.onEvent(fn(event, view))         each event as it is applied
 *   LongTamp.addBadge(fn(node, view) -> text | {text, cls} | null)
 *   LongTamp.formatMetric(key, fn(value, event) -> text)
 *   LongTamp.view, LongTamp.events, LongTamp.config, LongTamp.select(id)
 */
(function () {
  "use strict";

  var boot = JSON.parse(document.getElementById("boot").textContent);
  var config = boot.config;

  var COMPOSITES = ["sequence", "fallback", "retry", "parallel"];
  var MAIN_ROLES = ["sequence", "fallback", "retry", "parallel", "condition",
                    "operation", "transaction"];
  var PLAN_ROLES = MAIN_ROLES.concat(["complete", "ready", "precondition",
                    "attempts", "execute", "motion", "drift", "pause", "plan",
                    "progress", "watchdog"]);
  var KIND_ICONS = { sequence: "→", fallback: "?", retry: "↻",
                     parallel: "⇉", condition: "◇", operation: "▸",
                     transaction: "■" };

  var LT = window.LongTamp = {
    config: config,
    events: boot.events || [],
    live: !!boot.live,
    view: null,
    cursor: 0,
    selected: null,
    follow: true,
    _panels: {},
    _eventHooks: [],
    _badges: [],
    _formatters: {},
    _collapsed: {},
    _roleShown: {},
    _filter: "",
    _onlyFailures: false,
  };

  // -- extension API -------------------------------------------------------

  LT.addPanel = function (panel) {
    if (!panel || !panel.id || typeof panel.render !== "function") {
      throw new Error("addPanel needs {id, render}");
    }
    LT._panels[panel.id] = panel;
    if (LT._started) { layout(); render(); }
  };
  LT.onEvent = function (fn) { LT._eventHooks.push(fn); };
  LT.addBadge = function (fn) { LT._badges.push(fn); };
  LT.formatMetric = function (key, fn) { LT._formatters[key] = fn; };
  LT.select = function (id) {
    LT.selected = id;
    var d = sections.details;
    if (d && d.group) { LT._tabs[d.group] = "details"; showTab(d.group); }
    render();
  };

  // -- helpers -------------------------------------------------------------

  function h(tag, attrs) {
    var el = document.createElement(tag);
    if (attrs) {
      Object.keys(attrs).forEach(function (k) {
        var v = attrs[k];
        if (v === null || v === undefined || v === false) return;
        if (k === "text") el.textContent = v;
        else if (k === "cls") el.className = v;
        else if (k.slice(0, 2) === "on") el.addEventListener(k.slice(2), v);
        else el.setAttribute(k, v === true ? "" : v);
      });
    }
    for (var i = 2; i < arguments.length; i++) {
      var c = arguments[i];
      if (c === null || c === undefined || c === false) continue;
      if (Array.isArray(c)) c.forEach(function (x) { if (x) el.appendChild(x); });
      else el.appendChild(typeof c === "string" ? document.createTextNode(c) : c);
    }
    return el;
  }

  function fmtSeconds(s) {
    if (s === null || s === undefined || isNaN(s)) return "–";
    if (s < 60) return s.toFixed(1) + " s";
    var m = Math.floor(s / 60);
    if (m < 60) return m + " min " + Math.round(s - 60 * m) + " s";
    return Math.floor(m / 60) + " h " + (m % 60) + " min";
  }

  function metricText(key, value, event) {
    var f = LT._formatters[key];
    if (f) return f(value, event);
    if (typeof value === "number") {
      return String(Math.round(value * 1000) / 1000);
    }
    return typeof value === "string" ? value : JSON.stringify(value);
  }

  function metricLabel(key) {
    return (config.metric_labels && config.metric_labels[key]) || key;
  }

  function stateClass(state) { return "st-" + (state === "unmet" ? "idle" : state); }

  function isSide(event) { return PLAN_ROLES.indexOf(event.role) < 0; }

  // -- the view: the state after the first `cursor` events -----------------

  function emptyView() {
    return {
      plan: null, planVersion: 0, attemptsCap: {},
      nodes: {}, roots: [], stack: [],
      t0: null, t: null, applied: 0,
      paused: null, planning: null, side: [], roles: {},
      model: { calls: 0, tokens: 0, failures: 0 },
      tools: { calls: 0, failures: 0 },
      // the monitor's behavior-tree elements (#105): "<ir id>/<role>", plus
      // the live phases and motions under each step's execute
      elements: {}, dyn: {}, visits: 0,
    };
  }

  // -- behavior-tree elements: every element a step compiles to, each with
  //    its state, how long it has had it, its visit order and its history

  var BT_ROLES = ["sequence", "fallback", "retry", "parallel", "condition", "operation",
                  "transaction", "complete", "ready", "precondition", "attempts", "execute"];

  function element(view, key, t) {
    var e = view.elements[key];
    if (!e) e = view.elements[key] = { status: "IDLE", since: t, history: [], order: null, attempt: 0 };
    return e;
  }

  function setElement(view, key, status, t, message) {
    var e = element(view, key, t);
    if (e.order === null && status !== "IDLE") e.order = ++view.visits;
    if (e.status !== status || status !== "RUNNING") {
      e.history.unshift({ t: t, from: e.status, to: status, message: message || "" });
      if (e.history.length > 30) e.history.pop();
    }
    e.status = status; e.since = t;
    if (message) e.message = message;
    return e;
  }

  function dynChild(view, stepId, key, label, kind) {
    var list = view.dyn[stepId] || (view.dyn[stepId] = []);
    if (!list.some(function (c) { return c.key === key; })) list.push({ key: key, label: label, kind: kind });
  }

  function trackElements(view, n, event) {
    var role = event.role, t = event.t, id = n.id;
    if (BT_ROLES.indexOf(role) >= 0) {
      var e = setElement(view, id + "/" + role, event.status, t, event.message);
      if (role === "execute") {
        e.attempt = (event.metrics && event.metrics.attempt) || e.attempt + 1;
        // the attempt's phases and motions end with it
        (view.dyn[id] || []).forEach(function (c) {
          var ce = view.elements[c.key];
          if (ce && ce.status === "RUNNING") setElement(view, c.key, event.status === "FAILURE" ? "FAILURE" : "SUCCESS", t);
        });
      }
      return;
    }
    var attempt = (view.elements[id + "/execute"] || { attempt: 0 }).attempt + 1;
    if (role === "progress" && event.metrics && event.metrics.phase) {
      var key = id + "/a" + attempt + "/phase" + event.metrics.phase;
      (view.dyn[id] || []).forEach(function (c) {  // the previous phase is done
        var ce = view.elements[c.key];
        if (c.kind === "phase" && c.key !== key && ce && ce.status === "RUNNING") setElement(view, c.key, "SUCCESS", t);
      });
      dynChild(view, id, key, (attempt > 1 ? "attempt " + attempt + ", " : "") + event.message, "phase");
      if (element(view, key, t).status !== "RUNNING") setElement(view, key, "RUNNING", t, event.message);
    } else if (role === "motion") {
      if (event.status === "RUNNING") n.motionSeq = (n.motionSeq || 0) + 1;
      var mkey = id + "/motion" + (n.motionSeq || 1);
      dynChild(view, id, mkey, "motion " + (n.motionSeq || 1), "motion");
      setElement(view, mkey, event.status, t, event.message);
    }
  }

  function node(view, id) {
    var n = view.nodes[id];
    if (!n) {
      n = view.nodes[id] = {
        id: id, type: null, label: id, def: null, children: [], parent: null,
        status: "IDLE", skipped: false, skipReason: null, attempt: 0,
        maxAttempts: null, failures: [], drifts: 0, paused: false,
        moving: false, motions: 0, motionSeconds: 0, planSeconds: 0,
        activity: null, progress: [], watchdog: [],
        started: null, ended: null, events: [],
      };
    }
    return n;
  }

  function adoptPlan(view, event) {
    view.plan = event.plan;
    view.planVersion += 1;
    view.attemptsCap = (event.metrics && event.metrics.attempts) || {};
    view.roots = [];
    Object.keys(view.nodes).forEach(function (id) {
      view.nodes[id].children = []; view.nodes[id].parent = null; view.nodes[id].def = null;
    });
    (function walk(def, parent) {
      var n = node(view, def.id);
      n.def = def; n.type = def.type; n.label = def.label || def.id;
      n.maxAttempts = view.attemptsCap[def.id] || def.attempts || null;
      if (parent) { n.parent = parent.id; parent.children.push(def.id); }
      else view.roots.push(def.id);
      var kids = def.children || (def.child ? [def.child] : []);
      if (def.type === "transaction" && kids.length === 1 && kids[0].type === "operation") {
        return; // its operation is what it runs: shown on it (call), not as a node
      }
      kids.forEach(function (k) { walk(k, n); });
    })(event.plan.root, null);
  }

  function apply(view, event) {
    if (view.t0 === null) view.t0 = event.t;
    view.t = event.t;
    view.applied += 1;
    view.roles[event.role] = (view.roles[event.role] || 0) + 1;
    if (event.role === "plan" && event.plan) { adoptPlan(view, event); return; }
    if (isSide(event)) {
      view.side.push(event);
      if (event.role === "model") {
        view.model.calls += 1;
        if (event.status === "FAILURE") view.model.failures += 1;
        var m = event.metrics || {};
        view.model.tokens += (m.input_tokens || 0) + (m.output_tokens || 0);
      } else if (event.role === "tool") {
        view.tools.calls += 1;
        if (event.status === "FAILURE") view.tools.failures += 1;
      }
      return;
    }
    var known = !!view.nodes[event.ir_id];
    var n = node(view, event.ir_id);
    n.events.push(view.applied - 1);
    if (!known && !view.plan) {
      // No plan event: the tree is inferred, under the running composite.
      var parent = view.stack.length ? view.stack[view.stack.length - 1] : null;
      if (parent && parent !== n.id) { n.parent = parent; node(view, parent).children.push(n.id); }
      else view.roots.push(n.id);
    } else if (!known) {
      view.roots.push(n.id); // not in the plan: listed at the top level
    }
    if (MAIN_ROLES.indexOf(event.role) >= 0) {
      if (!n.type) n.type = event.role;
      if (n.label === n.id && event.name) n.label = event.name.replace(/ transaction$/, "");
      n.status = event.status;
      if (event.status !== "RUNNING") n.activity = null;
      if (event.status === "RUNNING") {
        if (n.started === null) n.started = event.t;
        n.ended = null;
        if (COMPOSITES.indexOf(event.role) >= 0) view.stack.push(n.id);
      } else {
        n.ended = event.t;
        if (n.started === null) n.started = event.t;
        var i = view.stack.lastIndexOf(n.id);
        if (i >= 0) view.stack.splice(i, 1);
      }
      if (event.status === "FAILURE" && event.message) n.failures.push(event.message);
    }
    trackElements(view, n, event);
    switch (event.role) {
      case "complete":
        if (event.status === "SUCCESS") { n.skipped = true; n.skipReason = event.message || "effect holds"; }
        break;
      case "execute":
        n.activity = null;
        if (view.planning === n.id) view.planning = null;
        n.attempt = (event.metrics && event.metrics.attempt) || n.attempt + 1;
        n.planSeconds += (event.metrics && event.metrics.seconds) || 0;
        if (event.status === "FAILURE") n.failures.push(event.message || "attempt failed");
        if (n.started === null) n.started = event.t;
        break;
      case "motion":
        if (event.status === "RUNNING") n.moving = true;
        else {
          n.moving = false; n.motions += 1;
          var mm = event.metrics || {};
          n.motionSeconds += mm.duration || mm.seconds || 0;
          if (event.status === "FAILURE") n.failures.push("motion: " + (event.message || mm.reason || "failed"));
        }
        break;
      case "drift":
        n.drifts += 1;
        break;
      case "progress":
        n.activity = { message: event.message || "", metrics: event.metrics || {}, t: event.t };
        n.progress.push(n.activity);
        if (n.progress.length > 200) n.progress.shift();
        view.planning = n.id;
        break;
      case "watchdog":
        n.watchdog.push({ message: event.message || "", metrics: event.metrics || {} });
        break;
      case "pause":
        n.paused = event.status === "RUNNING";
        view.paused = n.paused ? { id: n.id, when: (event.metrics || {}).when } : null;
        break;
    }
  }

  function nodeState(n) {
    if (n.paused) return "paused";
    if (n.type === "condition" && n.status === "FAILURE") return "unmet";
    if (n.skipped && n.status !== "FAILURE" && n.attempt === 0) return "skipped";
    if (n.status === "RUNNING") return "running";
    if (n.status === "SUCCESS") return "success";
    if (n.status === "FAILURE") return "failure";
    if (n.status === "SKIPPED") return "skipped";
    return "idle";
  }
  LT.nodeState = nodeState;

  function computeTo(cursor) {
    var view = LT.view;
    if (!view || cursor < view.applied) view = emptyView();
    var hooks = LT._eventHooks.length > 0;
    for (var i = view.applied; i < cursor; i++) {
      apply(view, LT.events[i]);
      if (hooks) LT._eventHooks.forEach(function (fn) { fn(LT.events[i], view); });
    }
    LT.view = view;
    LT.cursor = cursor;
  }

  // -- panels --------------------------------------------------------------

  function steps(view) {
    return Object.keys(view.nodes).map(function (id) { return view.nodes[id]; })
      .filter(function (n) { return n.type === "transaction" || n.type === "operation"; });
  }

  var builtins = {};

  builtins.summary = {
    title: "Summary", wide: true,
    render: function (view, el) {
      var all = steps(view), done = 0, skipped = 0, failed = 0, retries = 0,
          motion = 0, planning = 0, drifts = 0;
      all.forEach(function (n) {
        var s = nodeState(n);
        if (s === "success") done += 1;
        if (s === "skipped") skipped += 1;
        if (s === "failure") failed += 1;
        retries += Math.max(0, n.attempt - 1);
        motion += n.motionSeconds; planning += n.planSeconds; drifts += n.drifts;
      });
      var pauses = view.roles.pause ? Math.ceil(view.roles.pause / 2) : 0;
      var stat = function (value, label) { return h("div", { cls: "stat" }, h("b", { text: String(value) }), h("span", { text: label })); };
      var items = [
        stat(view.t0 === null ? "–" : fmtSeconds(view.t - view.t0), "elapsed"),
        stat(done + " / " + all.length, "steps done"),
        stat(skipped, "skipped (effect held)"),
        stat(failed, "failed"),
        stat(retries, "retries"),
        stat(fmtSeconds(planning), "planning"),
        stat(fmtSeconds(motion), "motion"),
        stat(drifts, "drift replans"),
        stat(pauses, "pauses"),
      ];
      if (view.model.calls) items.push(stat(view.model.calls + " / " + view.model.tokens, "model calls / tokens"));
      if (view.tools.calls) items.push(stat(view.tools.calls, "tool calls"));
      if (view.planVersion > 1) items.push(stat(view.planVersion, "plans"));
      el.appendChild(h("div", { cls: "stats" }, items));
    },
  };

  function badges(n, view) {
    var out = [];
    var state = nodeState(n);
    if (n.type === "transaction" || n.type === "operation") {
      if (n.attempt > 0) {
        out.push(h("span", { cls: "badge" + (n.attempt > 1 ? " warn" : ""),
          text: "attempt " + n.attempt + (n.maxAttempts ? "/" + n.maxAttempts : "") }));
      }
      if (state === "skipped") out.push(h("span", { cls: "badge", text: "skipped: " + n.skipReason }));
      if (n.motionSeconds) out.push(h("span", { cls: "badge", text: "motion " + fmtSeconds(n.motionSeconds) }));
      if (n.moving) out.push(h("span", { cls: "badge warn", text: "moving" }));
    }
    if (n.activity && n.status === "RUNNING") {
      var a = n.activity, msg = a.message.length > 70 ? a.message.slice(0, 69) + "…" : a.message;
      out.push(h("span", { cls: "badge activity", title: a.message,
        text: msg + (a.metrics.elapsed !== undefined ? " · " + fmtSeconds(a.metrics.elapsed) : "") }));
    }
    if (n.drifts) out.push(h("span", { cls: "badge warn", text: "drift ×" + n.drifts }));
    if (n.watchdog.length) {
      var w = n.watchdog[n.watchdog.length - 1];
      out.push(h("span", { cls: "badge warn", title: w.message, text: "watchdog: " + (w.metrics.action || "?") }));
    }
    if (n.paused) out.push(h("span", { cls: "badge warn", text: "⏸ paused" }));
    LT._badges.forEach(function (fn) {
      var b = fn(n, view);
      if (!b) return;
      if (typeof b === "string") b = { text: b };
      out.push(h("span", { cls: "badge " + (b.cls || ""), text: b.text }));
    });
    return out;
  }

  function call(def) {
    if (def && !def.capability && def.type === "transaction") {
      var kids = def.children || (def.child ? [def.child] : []);
      def = kids.length === 1 ? kids[0] : def;
    }
    if (!def || !def.capability) return null;
    var p = def.parameters || {};
    var args = Object.keys(p).sort().map(function (k) { return p[k]; }).join(", ");
    return def.capability + "(" + args + ")";
  }

  // monitor (#105): the plan as a live behavior tree, like a BT monitor.
  // Three levels of detail (plan steps / behavior tree / + phases & motions),
  // three layouts (top down, left to right, radial); links to running nodes
  // flow, finished ones take their result's colour, the active node pulses,
  // the tree glides when it changes; visit order, follow, fit, full screen
  // with a side pane (status counts, details, status changes).
  var NS = "http://www.w3.org/2000/svg";
  var GLYPH = { sequence: "→", fallback: "?", retry: "↻", parallel: "⇉", condition: "◇",
                action: "▶", phase: "∙", motion: "≈", root: "◎" };
  var KIND_NAME = { sequence: "Sequence", fallback: "Fallback", retry: "Retry", parallel: "Parallel",
                    condition: "Condition", action: "Action", phase: "Planning phase", motion: "Motion" };
  var STATES = ["running", "success", "failure", "false", "skipped", "paused", "idle"];
  var STATE_NAME = { running: "Running", success: "Success", failure: "Failure", "false": "Not met",
                     skipped: "Skipped", paused: "Paused", idle: "Idle" };
  var mon = { level: "full", orient: "LR", order: true, follow: true, full: false,
              collapsed: {}, selected: null, tf: { k: 1, x: 40, y: 40 }, els: {}, links: {},
              pos: {}, tree: null, byKey: {}, sig: "", active: null, fitted: false, anim: null };
  LT._monitor = mon;  // for extensions and debugging
  (function restore() {
    try {
      var saved = JSON.parse(window.localStorage.getItem("lt-monitor") || "{}");
      ["level", "orient", "order", "follow"].forEach(function (k) { if (k in saved) mon[k] = saved[k]; });
    } catch (e) { /* no storage: defaults */ }
  })();
  function remember() {
    try {
      window.localStorage.setItem("lt-monitor", JSON.stringify(
        { level: mon.level, orient: mon.orient, order: mon.order, follow: mon.follow }));
    } catch (e) { /* fine */ }
  }

  function sv(tag, attrs) {
    var el = document.createElementNS(NS, tag);
    Object.keys(attrs || {}).forEach(function (k) { el.setAttribute(k, attrs[k]); });
    return el;
  }
  function clip(text, n) { text = String(text || ""); return text.length > n ? text.slice(0, n - 1) + "…" : text; }

  // -- the tree at the chosen level of detail ----------------------------------
  function monBuildTree(view) {
    var byKey = {};
    function mk(key, label, kind, children, ir) {
      var t = { key: key, label: label, kind: kind, children: children, ir: ir, parent: null };
      children.forEach(function (c) { c.parent = t; });
      byKey[key] = t;
      return t;
    }
    function dyn(id) {
      if (mon.level !== "full") return [];
      return (view.dyn[id] || []).map(function (c) { return mk(c.key, c.label, c.kind, [], view.nodes[id]); });
    }
    function build(id) {
      var n = view.nodes[id];
      if (!n) return null;
      var type = n.type || "operation";
      if (type === "transaction") {
        if (mon.level === "steps") return mk(id + "/transaction", n.label, "action", [], n);
        var exec = mk(id + "/execute", n.label, "action", dyn(id), n);
        var retry = mk(id + "/attempts", "retry ×" + (n.maxAttempts || 1), "retry", [exec], n);
        var pre = mk(id + "/precondition", "preconditions hold?", "condition", [], n);
        var ready = mk(id + "/ready", "ready", "sequence", [pre, retry], n);
        var done = mk(id + "/complete", "effect holds?", "condition", [], n);
        return mk(id + "/transaction", n.label, "fallback", [done, ready], n);
      }
      if (type === "operation") return mk(id + "/operation", n.label, "action", dyn(id), n);
      var kind = type === "retry" || type === "fallback" || type === "parallel" || type === "condition" ? type : "sequence";
      return mk(id + "/" + type, n.label, kind, n.children.map(build).filter(Boolean), n);
    }
    var roots = view.roots.map(build).filter(Boolean);
    var top = roots.length === 1 ? roots[0] : mk("__root", "mission", "root", roots, null);
    top.isRoot = true;
    mon.byKey = byKey;
    return top;
  }

  function monState(view, t) {
    var e = view.elements[t.key];
    var n = t.ir;
    // a leaf reports only its result: while its retry runs, it is what runs
    if (n && t.key === n.id + "/execute") {
      var attempts = view.elements[n.id + "/attempts"];
      if (attempts && attempts.status === "RUNNING") return n.paused ? "paused" : "running";
    }
    if (n && n.paused && (t.kind === "fallback" || t.key === n.id + "/operation")) return "paused";
    if (e && e.status !== "IDLE") {
      if (e.status === "RUNNING") return "running";
      if (e.status === "SUCCESS" || e.status === "SKIPPED") return "success";
      return t.kind === "condition" ? "false" : "failure";
    }
    if (t.isRoot && t.children.length) {  // a synthetic root: from its children
      var kids = t.children.map(function (c) { return monState(view, c); });
      if (kids.indexOf("running") >= 0) return "running";
      if (kids.indexOf("failure") >= 0) return "failure";
      return kids.every(function (k) { return k === "success" || k === "skipped"; }) ? "success" : "idle";
    }
    // never visited, though its parent finished: skipped
    var p = t.parent;
    if (p) {
      var ps = monState(view, p);
      if (ps === "success" || ps === "failure" || ps === "skipped" || ps === "false") return "skipped";
    }
    return "idle";
  }

  // -- layout ---------------------------------------------------------------------
  var SIZE = { TD: [118, 104], LR: [64, 220], RAD: [0, 120] };
  function monLayout(top) {
    var leaves = 0, pos = {};
    (function place(t, depth) {
      var kids = mon.collapsed[t.key] ? [] : t.children;
      var slot;
      if (!kids.length) slot = leaves++;
      else {
        var slots = kids.map(function (c) { return place(c, depth + 1); });
        slot = (slots[0] + slots[slots.length - 1]) / 2;
      }
      pos[t.key] = { slot: slot, depth: depth };
      return slot;
    })(top, 0);
    Object.keys(pos).forEach(function (k) {
      var p = pos[k];
      if (mon.orient === "TD") { p.x = p.slot * SIZE.TD[0]; p.y = p.depth * SIZE.TD[1]; }
      else if (mon.orient === "LR") { p.x = p.depth * SIZE.LR[1]; p.y = p.slot * SIZE.LR[0]; }
      else {
        var a = 2 * Math.PI * (p.slot + 0.5) / Math.max(leaves, 1) - Math.PI / 2, r = p.depth * SIZE.RAD[1];
        p.x = r * Math.cos(a); p.y = r * Math.sin(a);
      }
    });
    return pos;
  }

  function linkPath(a, b) {
    if (mon.orient === "TD") {
      var my = (a.y + b.y) / 2;
      return "M" + a.x + " " + a.y + " C" + a.x + " " + my + " " + b.x + " " + my + " " + b.x + " " + b.y;
    }
    if (mon.orient === "LR") {
      var mx = (a.x + b.x) / 2;
      return "M" + a.x + " " + a.y + " C" + mx + " " + a.y + " " + mx + " " + b.y + " " + b.x + " " + b.y;
    }
    return "M" + a.x + " " + a.y + " Q" + (a.x + b.x) * 0.55 + " " + (a.y + b.y) * 0.55 + " " + b.x + " " + b.y;
  }

  function radius(t) { return t.kind === "phase" || t.kind === "motion" ? 9 : t.children.length || t.kind === "fallback" ? 18 : 14; }

  function visible(top) {
    var out = [];
    (function walk(t) { out.push(t); if (!mon.collapsed[t.key]) t.children.forEach(walk); })(top);
    return out;
  }
  function countBelow(t) { var c = -1; (function w(x) { c++; x.children.forEach(w); })(t); return c; }

  // -- drawing ----------------------------------------------------------------------
  function monDraw(view) {
    var top = mon.tree, nodes = visible(top), next = monLayout(top), old = mon.pos;
    var keep = {};
    nodes.forEach(function (t) { keep[t.key] = true; });
    Object.keys(mon.els).forEach(function (k) {
      if (!keep[k]) { mon.els[k].g.remove(); delete mon.els[k]; }
    });
    Object.keys(mon.links).forEach(function (k) {
      if (!keep[k]) { mon.links[k].remove(); delete mon.links[k]; }
    });
    function from(t) {  // where a new node grows from: its nearest drawn ancestor
      for (var a = t.parent; a; a = a.parent) if (old[a.key]) return old[a.key];
      return next[t.key];
    }
    nodes.forEach(function (t) {
      if (t.parent && !mon.links[t.key]) {
        mon.links[t.key] = sv("path", { "class": "mlink" });
        mon.gLinks.appendChild(mon.links[t.key]);
      }
      if (mon.els[t.key]) return;
      var g = sv("g", { "class": "mnode", tabindex: 0, role: "button" });
      var r = radius(t);
      var e = { g: g, t: t,
        pulse: sv("circle", { "class": "pulse", r: r }), halo: sv("circle", { "class": "halo", r: r + 6 }),
        body: sv("circle", { "class": "body", r: r }), glyph: sv("text", { "class": "glyph", "text-anchor": "middle", dy: "0.36em" }),
        name: sv("text", { "class": "name", "text-anchor": "middle", y: r + 15 }),
        type: sv("text", { "class": "type", "text-anchor": "middle", y: r + 28 }),
        order: sv("g", { "class": "order", transform: "translate(" + r * 0.8 + "," + -r * 0.8 + ")" }),
        title: sv("title") };
      e.orderText = sv("text", { "text-anchor": "middle", dy: "0.35em" });
      e.order.appendChild(sv("circle", { r: 8.5 })); e.order.appendChild(e.orderText);
      [e.title, e.pulse, e.halo, e.body, e.glyph, e.name, e.type, e.order].forEach(function (c) { g.appendChild(c); });
      g.addEventListener("click", function (ev) { ev.stopPropagation(); monSelect(e.t.key); });
      g.addEventListener("dblclick", function (ev) { ev.stopPropagation(); monToggle(e.t.key); });
      g.addEventListener("keydown", function (ev) {
        if (ev.key === "Enter" || ev.key === " ") { ev.preventDefault(); monSelect(e.t.key); }
        if (ev.key === "c") monToggle(e.t.key);
      });
      var o = from(t) || { x: 0, y: 0 };
      e.x = o.x; e.y = o.y;
      g.setAttribute("transform", "translate(" + o.x + "," + o.y + ")");
      mon.gNodes.appendChild(g);
      mon.els[t.key] = e;
    });
    nodes.forEach(function (t) {  // texts may change with the level (labels, counts)
      var e = mon.els[t.key];
      e.t = t;
      var collapsed = mon.collapsed[t.key] && t.children.length;
      var gl = collapsed ? "+" + countBelow(t) : t.isRoot ? GLYPH.root : GLYPH[t.kind] || "";
      e.glyph.textContent = gl;
      e.glyph.setAttribute("class", "glyph" + (gl.length > 2 ? " small" : ""));
      e.name.textContent = clip(t.label, mon.orient === "LR" ? 30 : 18);
      var call_ = t.ir && (t.kind === "action" || t.kind === "fallback") ? call(t.ir.def) : null;
      e.type.textContent = clip(KIND_NAME[t.kind] || (t.isRoot ? "mission" : t.kind), 24);
      e.title.textContent = t.label + "\n" + (KIND_NAME[t.kind] || t.kind) + (call_ ? "\n" + call_ : "") +
        (t.children.length ? "\nDouble-click to fold or unfold" : "");
      e.g.setAttribute("aria-label", t.label + ", " + (KIND_NAME[t.kind] || t.kind));
    });
    // glide from where things were to where they go
    var start = performance.now(), dur = reduceMotion() ? 0 : 420;
    var froms = {};
    nodes.forEach(function (t) { var e = mon.els[t.key]; froms[t.key] = { x: e.x, y: e.y }; });
    if (mon.anim) cancelAnimationFrame(mon.anim);
    (function step() {
      var s = progress(start, dur), k = s * (2 - s);
      nodes.forEach(function (t) {
        var e = mon.els[t.key], a = froms[t.key], b = next[t.key];
        e.x = a.x + (b.x - a.x) * k; e.y = a.y + (b.y - a.y) * k;
        e.g.setAttribute("transform", "translate(" + e.x + "," + e.y + ")");
      });
      nodes.forEach(function (t) {
        if (!t.parent) return;
        var pe = mon.els[t.parent.key], ce = mon.els[t.key];
        if (pe && ce) mon.links[t.key].setAttribute("d", linkPath(pe, ce));
      });
      mon.anim = s < 1 ? requestAnimationFrame(step) : null;
    })(start);
    mon.pos = next;
  }

  // how far an animation started at ``start`` has got, in [0, 1]: read from
  // the clock, not from requestAnimationFrame's timestamp, which can be a frame
  // earlier than ``start`` (and an easing then extrapolates)
  function progress(start, dur) {
    return dur ? Math.max(0, Math.min(1, (performance.now() - start) / dur)) : 1;
  }

  function reduceMotion() {
    return window.matchMedia && window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  }

  function monPaint(view) {
    var nodes = visible(mon.tree), active = null, deepest = -1;
    nodes.forEach(function (t) {
      var e = mon.els[t.key], state = monState(view, t), el = view.elements[t.key];
      var folded = mon.collapsed[t.key] && t.children.length;
      var runningBelow = folded && t.children.some(function hasRun(c) {
        return monState(view, c) === "running" || c.children.some(hasRun);
      });
      var leafActive = state === "running" && (!t.children.length || folded ||
        !t.children.some(function (c) { return monState(view, c) === "running"; }));
      if (leafActive || runningBelow) {
        var d = (mon.pos[t.key] || { depth: 0 }).depth;
        if (d > deepest) { deepest = d; active = t.key; }
      }
      e.g.setAttribute("class", "mnode s-" + state + (leafActive || runningBelow ? " active" : "") +
        (mon.selected === t.key ? " selected" : ""));
      var show = mon.order && el && el.order;
      e.order.style.display = show ? "" : "none";
      if (show) e.orderText.textContent = el.order;
      if (t.parent) mon.links[t.key].setAttribute("class", "mlink s-" + state);
      var extra = "";
      if (t.kind === "action" && el && el.attempt) extra = "attempt " + el.attempt + (t.ir && t.ir.maxAttempts ? "/" + t.ir.maxAttempts : "");
      if (t.ir && t.ir.activity && t.ir.status === "RUNNING" && (t.key === t.ir.id + "/execute" || t.key === t.ir.id + "/operation" ||
          (mon.level === "steps" && t.key === t.ir.id + "/transaction"))) extra = t.ir.activity.message;
      e.type.textContent = clip(extra || KIND_NAME[t.kind] || (t.isRoot ? "mission" : t.kind), 30);
    });
    if (mon.follow && active && active !== mon.active) monCenter(active);
    mon.active = active;
    monSide(view);
  }

  // -- view transform: zoom, pan, fit, center --------------------------------------
  function monApply() {
    var f = mon.tf;
    mon.gRoot.setAttribute("transform", "translate(" + f.x + "," + f.y + ") scale(" + f.k + ")");
  }
  function monAnimate(to) {
    var from = { k: mon.tf.k, x: mon.tf.x, y: mon.tf.y }, start = performance.now(), dur = reduceMotion() ? 0 : 480;
    (function step() {
      var s = progress(start, dur), k = s * (2 - s);
      mon.tf = { k: from.k + (to.k - from.k) * k, x: from.x + (to.x - from.x) * k, y: from.y + (to.y - from.y) * k };
      monApply();
      if (s < 1) requestAnimationFrame(step);
    })(start);
  }
  function monFit() {
    var rect = mon.svg.getBoundingClientRect();
    if (!rect.width || !rect.height) return;
    var x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    Object.keys(mon.pos).forEach(function (k) {
      var p = mon.pos[k];
      x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y);
    });
    if (x0 === Infinity) return;
    x0 -= 80; x1 += mon.orient === "LR" ? 160 : 80; y0 -= 40; y1 += 50;
    var k = Math.min(1.4, 0.94 * Math.min(rect.width / (x1 - x0), rect.height / (y1 - y0)));
    monAnimate({ k: k, x: rect.width / 2 - k * (x0 + x1) / 2, y: rect.height / 2 - k * (y0 + y1) / 2 });
  }
  function monCenter(key) {
    var p = mon.pos[key];
    for (var t = mon.byKey[key]; !p && t; t = t.parent) p = mon.pos[t.key];  // folded: its visible ancestor
    if (!p) return;
    var rect = mon.svg.getBoundingClientRect(), k = Math.max(mon.tf.k, 0.85);
    // the node toward the leading side, so its ancestors stay in view
    var fx = mon.orient === "LR" ? 0.7 : 0.5, fy = mon.orient === "TD" ? 0.62 : 0.5;
    monAnimate({ k: k, x: rect.width * fx - k * p.x, y: rect.height * fy - k * p.y });
  }

  function monSelect(key) {
    mon.selected = key;
    var t = mon.byKey[key];
    if (t && t.ir) LT.selected = t.ir.id;  // the Details panel follows
    render();
  }
  function monToggle(key) {
    var t = mon.byKey[key];
    if (!t || !t.children.length) return;
    mon.collapsed[key] = !mon.collapsed[key];
    mon.sig = "";
    render();
  }

  // -- the side pane: status counts, the selected node, status changes -------------
  function dot(state) { return h("i", { cls: "mdot ms-" + state }); }
  function fmtClock(t) {
    var d = new Date(t * 1000);
    return d.toLocaleTimeString([], { hour12: false }) + "." + String(d.getMilliseconds()).padStart(3, "0");
  }
  function monSide(view) {
    if (!mon.side) return;
    var counts = {};
    STATES.forEach(function (s) { counts[s] = 0; });
    visible(mon.tree).forEach(function (t) { counts[monState(view, t)]++; });
    var legend = h("div", { cls: "mlegend" },
      h("div", { cls: "mline" }, STATES.filter(function (s) { return counts[s] || s === "idle"; }).map(function (s) {
        return h("span", null, dot(s), " " + STATE_NAME[s] + " ", h("b", { text: String(counts[s]) }));
      })),
      h("div", { cls: "mline muted" }, ["sequence", "fallback", "retry", "condition", "action", "phase", "motion"].map(function (k) {
        return h("span", null, h("strong", { text: GLYPH[k] }), " " + KIND_NAME[k]);
      })));
    var details = h("div", { cls: "mdetails" });
    var t = mon.selected && mon.byKey[mon.selected];
    if (!t) details.appendChild(h("p", { cls: "muted", text: "Click a node for its details. Double-click to fold a branch." }));
    else {
      var el = view.elements[t.key], state = monState(view, t), path = [];
      for (var a = t.parent; a; a = a.parent) path.unshift(a);
      details.appendChild(h("div", { cls: "crumbs" }, path.map(function (p, i) {
        return [i ? " › " : "", h("a", { text: p.label, onclick: function () { monSelect(p.key); monCenter(p.key); } })];
      }).reduce(function (x, y) { return x.concat(y); }, [])));
      details.appendChild(h("h3", { text: t.label }));
      details.appendChild(h("div", { cls: "muted", text: (KIND_NAME[t.kind] || t.kind) + " · " + t.key +
        (el && el.order ? " · visited as step " + el.order : "") }));
      mon.since = el ? el.since : null;
      details.appendChild(h("p", null, h("span", { cls: "pill " + stateClass(state === "false" ? "idle" : state), text: STATE_NAME[state] }),
        h("span", { cls: "muted mfor", text: el ? " for " + fmtSeconds((LT.live ? Date.now() / 1000 : view.t) - el.since) : "" })));
      details.appendChild(h("div", { cls: "mbtns" },
        h("button", { text: "Center in view", onclick: function () { monCenter(t.key); } }),
        t.children.length ? h("button", { text: mon.collapsed[t.key] ? "Unfold" : "Fold", onclick: function () { monToggle(t.key); } }) : null));
      var rows = [];
      if (t.ir && t.ir.def && t.ir.def.capability) rows.push(["call", call(t.ir.def)]);
      if (el && el.attempt) rows.push(["attempt", el.attempt + (t.ir && t.ir.maxAttempts ? " of " + t.ir.maxAttempts : "")]);
      if (el && el.message) rows.push(["message", el.message]);
      if (rows.length) details.appendChild(h("table", { cls: "mtable" }, rows.map(function (r) {
        return h("tr", null, h("td", { text: r[0] }), h("td", { text: r[1] }));
      })));
      if (t.children.length) {
        details.appendChild(h("h4", { text: "Children (" + t.children.length + ")" }));
        details.appendChild(h("table", { cls: "mtable" }, t.children.map(function (c) {
          var cs = monState(view, c);
          return h("tr", null, h("td", null, dot(cs)), h("td", null,
            h("a", { text: c.label, onclick: function () { monSelect(c.key); monCenter(c.key); } }), " ",
            h("span", { cls: "muted", text: STATE_NAME[cs] })));
        })));
      }
      details.appendChild(h("h4", { text: "Recent changes" }));
      var hist = el ? el.history.slice(0, 8) : [];
      details.appendChild(hist.length ? h("table", { cls: "mtable" }, hist.map(function (x) {
        return h("tr", null, h("td", { text: fmtClock(x.t) }), h("td", { text: x.from.toLowerCase() + " → " + x.to.toLowerCase() + (x.message ? ": " + x.message : "") }));
      })) : h("p", { cls: "muted", text: "No changes yet." }));
    }
    var changes = [];
    Object.keys(view.elements).forEach(function (k) {
      if (!mon.byKey[k]) return;
      view.elements[k].history.forEach(function (x) { changes.push({ t: x.t, key: k, to: x.to }); });
    });
    changes.sort(function (a, b) { return b.t - a.t; });
    var log = h("div", { cls: "mlog" }, h("h4", { text: "Status changes" }), changes.slice(0, 120).map(function (c) {
      var tt = mon.byKey[c.key], st = c.to === "FAILURE" && tt.kind === "condition" ? "false" : c.to.toLowerCase();
      return h("div", { cls: "mrow", onclick: function () { monSelect(c.key); monCenter(c.key); } },
        h("time", { text: fmtClock(c.t) }), dot(st), h("span", { cls: "nm", text: tt.label }), h("span", { cls: "muted", text: STATE_NAME[st] || c.to }));
    }));
    mon.side.textContent = "";
    mon.side.appendChild(legend); mon.side.appendChild(details); mon.side.appendChild(log);
  }
  setInterval(function () {  // the selected node's "for …" ticks
    var el = mon.side && mon.side.querySelector(".mfor");
    if (el && mon.since && LT.live) el.textContent = " for " + fmtSeconds(Date.now() / 1000 - mon.since);
  }, 500);

  // -- the panel ------------------------------------------------------------------
  function monCreate(el) {
    function select(options, value, onchange, label) {
      return h("label", null, label + " ", h("select", { onchange: function (e) { onchange(e.target.value); } },
        options.map(function (o) { return h("option", { value: o[0], selected: o[0] === value, text: o[1] }); })));
    }
    function check(value, label, onchange) {
      return h("label", null, h("input", { type: "checkbox", checked: value, onchange: function (e) { onchange(e.target.checked); } }), " " + label);
    }
    var bar = h("div", { cls: "mbar" },
      h("button", { text: "Fit", onclick: monFit }),
      select([["steps", "Plan steps"], ["bt", "Behavior tree"], ["full", "+ phases & motions"]], mon.level,
        function (v) { mon.level = v; mon.sig = ""; mon.fitted = false; remember(); render(); }, "Detail"),
      select([["TD", "Top down"], ["LR", "Left to right"], ["RAD", "Radial"]], mon.orient,
        function (v) { mon.orient = v; mon.sig = ""; mon.fitted = false; remember(); render(); }, "Layout"),
      check(mon.order, "Visit order", function (v) { mon.order = v; remember(); render(); }),
      check(mon.follow, "Follow active", function (v) { mon.follow = v; remember(); }),
      h("button", { text: "⤢ Full screen", title: "Esc to leave", onclick: function () { monFull(!mon.full); } }));
    mon.svg = sv("svg", { "class": "msvg", role: "img", "aria-label": "plan monitor" });
    mon.gRoot = sv("g"); mon.gLinks = sv("g"); mon.gNodes = sv("g");
    mon.gRoot.appendChild(mon.gLinks); mon.gRoot.appendChild(mon.gNodes); mon.svg.appendChild(mon.gRoot);
    mon.els = {}; mon.links = {}; mon.pos = {}; mon.sig = ""; mon.fitted = false;
    mon.svg.addEventListener("wheel", function (ev) {
      ev.preventDefault();
      var rect = mon.svg.getBoundingClientRect(), cx = ev.clientX - rect.left, cy = ev.clientY - rect.top;
      var f = mon.tf, k = Math.min(3, Math.max(0.08, f.k * Math.exp(-ev.deltaY * 0.0015)));
      mon.tf = { k: k, x: cx - (cx - f.x) * k / f.k, y: cy - (cy - f.y) * k / f.k };
      monApply();
    }, { passive: false });
    var drag = null;
    mon.svg.addEventListener("pointerdown", function (ev) {
      if (ev.target.closest && ev.target.closest(".mnode")) return;
      drag = { x: ev.clientX, y: ev.clientY, tx: mon.tf.x, ty: mon.tf.y };
      mon.svg.setPointerCapture(ev.pointerId); mon.svg.classList.add("grabbing");
    });
    mon.svg.addEventListener("pointermove", function (ev) {
      if (!drag) return;
      mon.tf.x = drag.tx + ev.clientX - drag.x; mon.tf.y = drag.ty + ev.clientY - drag.y; monApply();
    });
    mon.svg.addEventListener("pointerup", function () { drag = null; mon.svg.classList.remove("grabbing"); });
    mon.side = h("aside", { cls: "mside" });
    if (mon.full) document.body.classList.add("mon-full");
    mon.box = h("div", { cls: "monitor" + (mon.full ? " full" : "") }, bar,
      h("div", { cls: "mmain" }, h("div", { cls: "mstage" }, mon.svg,
        h("p", { cls: "mhint", text: "Click: details · double-click: fold · drag: pan · scroll: zoom" })), mon.side));
    el.appendChild(mon.box);
  }

  function monFull(on) {
    mon.full = on;
    if (mon.box) mon.box.classList.toggle("full", on);
    document.body.classList.toggle("mon-full", on);
    requestAnimationFrame(monFit);
  }
  document.addEventListener("keydown", function (ev) { if (ev.key === "Escape" && mon.full) monFull(false); });
  // a link can open it full screen: <page>#monitor-full
  if (/monitor-full/.test(window.location.hash)) mon.full = true;

  builtins.monitor = {
    title: "Monitor",
    keep: true,
    render: function (view, el) {
      if (!mon.svg || !el.contains(mon.svg)) { el.textContent = ""; monCreate(el); }
      if (!view.roots.length) return;
      var dynCount = Object.keys(view.dyn).reduce(function (n, k) { return n + view.dyn[k].length; }, 0);
      var sig = [view.planVersion, Object.keys(view.nodes).length, mon.level === "full" ? dynCount : 0, mon.orient,
        Object.keys(mon.collapsed).filter(function (k) { return mon.collapsed[k]; }).join(",")].join(":");
      if (sig !== mon.sig) {
        mon.tree = monBuildTree(view);
        monDraw(view);
        mon.sig = sig;
        if (!mon.fitted) {  // the whole tree first, then the running step
          mon.fitted = true;
          setTimeout(function () {
            monFit();
            if (mon.follow && mon.active) setTimeout(function () { monCenter(mon.active); }, 900);
          }, 450);
        }
      }
      monPaint(view);
    },
  };

  builtins.plan = {
    title: "Plan",
    render: function (view, el) {
      if (!view.roots.length) { el.appendChild(h("p", { cls: "muted", text: "No plan events yet." })); return; }
      if (!view.plan) el.appendChild(h("p", { cls: "muted", text: "No plan event in the stream: the tree is inferred from the transitions." }));
      function item(id) {
        var n = view.nodes[id], state = nodeState(n), kids = n.children;
        var li = h("li", { cls: LT._collapsed[id] ? "collapsed" : null });
        var caret = h("span", { cls: "caret", text: kids.length ? (LT._collapsed[id] ? "▸" : "▾") : "",
          onclick: function (e) { e.stopPropagation(); LT._collapsed[id] = !LT._collapsed[id]; render(); } });
        var c = call(n.def);
        li.appendChild(h("div", { cls: "node" + (LT.selected === id ? " selected" : ""), "data-id": id,
            onclick: function () { LT.select(id); } },
          caret,
          h("span", { cls: "kind", title: n.type || "", text: KIND_ICONS[n.type] || "·" }),
          h("span", { text: n.label, title: c || null }),
          c ? h("span", { cls: "call mono", title: c, text: c.length > 60 ? c.slice(0, 59) + "\u2026" : c }) : null,
          h("span", { cls: "pill " + stateClass(state), text: state }),
          badges(n, view)));
        if (kids.length) li.appendChild(h("ul", null, kids.map(item)));
        return li;
      }
      var tree = h("ul", { cls: "tree" }, view.roots.map(item));
      var box = h("div", { cls: "scroll" }, tree);
      el.appendChild(box);
      if (LT.follow && view.stack.length) {
        var running = box.querySelector('[data-id="' + cssEscape(view.stack[view.stack.length - 1]) + '"]');
        if (running && running.scrollIntoView) requestAnimationFrame(function () { running.scrollIntoView({ block: "nearest" }); });
      }
    },
  };

  function cssEscape(s) { return window.CSS && CSS.escape ? CSS.escape(s) : s.replace(/"/g, '\\"'); }

  // timeline: transport controls and a bar per step
  var player = { playing: false, speed: 10, last: null, simT: 0 };

  function tAt(i) {
    var ev = LT.events;
    return ev.length ? ev[Math.max(0, Math.min(i, ev.length) - 1)].t : 0;
  }

  function tick(now) {
    if (!player.playing) return;
    var ev = LT.events;
    if (LT.cursor >= ev.length) { player.playing = false; render(); return; }
    if (player.speed === "step") {
      if (player.last === null || now - player.last > 150) { player.last = now; seek(LT.cursor + 1, true); }
    } else {
      if (player.last === null) { player.last = now; player.simT = tAt(LT.cursor) - ev[0].t; }
      player.simT += (now - player.last) / 1000 * player.speed;
      player.last = now;
      var c = LT.cursor;
      while (c < ev.length && ev[c].t - ev[0].t <= player.simT) c++;
      if (c === LT.cursor) {
        // jump idle gaps longer than 2 s of playback
        var gap = ev[c].t - ev[0].t - player.simT;
        if (gap > 2 * player.speed) player.simT += gap - 0.1;
      } else seek(c, true);
    }
    requestAnimationFrame(tick);
  }

  builtins.timeline = {
    title: "Timeline", wide: true,
    render: function (view, el) {
      var ev = LT.events, n = ev.length;
      var play = h("button", { text: player.playing ? "⏸ Pause" : "▶ Play", onclick: function () {
        player.playing = !player.playing; player.last = null;
        if (player.playing) { LT.follow = false; if (LT.cursor >= n) seek(0, true); requestAnimationFrame(tick); }
        render();
      } });
      var speed = h("select", { title: "playback speed", onchange: function (e) {
        player.speed = e.target.value === "step" ? "step" : Number(e.target.value); player.last = null; } },
        ["1", "10", "60", "600", "step"].map(function (v) {
          return h("option", { value: v, selected: String(player.speed) === v, text: v === "step" ? "event by event" : v + "×" });
        }));
      var range = h("input", { type: "range", min: 0, max: n, value: LT.cursor, "aria-label": "position in the event stream",
        oninput: function (e) { LT.follow = false; player.playing = false; seek(Number(e.target.value)); } });
      var follow = LT.live ? h("label", null, h("input", { type: "checkbox", checked: LT.follow,
        onchange: function (e) { LT.follow = e.target.checked; if (LT.follow) seek(LT.events.length); } }), " follow live") : null;
      var at = n ? fmtSeconds((LT.cursor ? ev[LT.cursor - 1].t : ev[0].t) - ev[0].t) : "–";
      el.appendChild(h("div", { cls: "transport" },
        h("button", { text: "⏮", title: "start", onclick: function () { LT.follow = false; seek(0); } }),
        play,
        h("button", { text: "⏭", title: "end", onclick: function () { seek(n); } }),
        speed, range,
        h("span", { cls: "mono", text: at + "  ·  event " + LT.cursor + "/" + n }),
        follow));
      // one bar per step over the whole run, coloured by its state now
      if (!n) return;
      var t0 = ev[0].t, span = Math.max(ev[n - 1].t - t0, 1e-6);
      var rows = steps(view).filter(function (s) { return s.started !== null; })
        .sort(function (a, b) { return a.started - b.started; });
      var g = h("div", { cls: "gantt" });
      var now = LT.cursor ? ev[LT.cursor - 1].t : t0;
      rows.forEach(function (s) {
        var end = s.ended !== null ? s.ended : now;
        var left = (s.started - t0) / span * 100, width = Math.max((end - s.started) / span * 100, 0.2);
        g.appendChild(h("div", { cls: "row" }, h("div", {
          cls: "bar " + stateClass(nodeState(s)),
          style: "left:" + left + "%;width:" + width + "%",
          title: s.label + " · " + nodeState(s) + " · " + fmtSeconds(end - s.started),
          onclick: function () { LT.select(s.id); } })));
      });
      g.appendChild(h("div", { cls: "cursor", style: "left:" + ((now - t0) / span * 100) + "%" }));
      el.appendChild(g);
    },
  };

  builtins.events = {
    title: "Events",
    render: function (view, el) {
      var roles = Object.keys(view.roles).sort();
      roles.forEach(function (r) {
        if (!(r in LT._roleShown)) LT._roleShown[r] = (config.hidden_roles || []).indexOf(r) < 0;
      });
      var search = h("input", { type: "search", placeholder: "filter", value: LT._filter, "aria-label": "filter events",
        oninput: function (e) { LT._filter = e.target.value; renderSoon(); } });
      el.appendChild(h("div", { cls: "filters" }, search,
        h("label", null, h("input", { type: "checkbox", checked: LT._onlyFailures,
          onchange: function (e) { LT._onlyFailures = e.target.checked; render(); } }), " failures only"),
        roles.map(function (r) {
          return h("label", null, h("input", { type: "checkbox", checked: LT._roleShown[r],
            onchange: function (e) { LT._roleShown[r] = e.target.checked; render(); } }), " " + r);
        })));
      var q = LT._filter.toLowerCase(), rows = [];
      for (var i = LT.cursor - 1; i >= 0 && rows.length < 400; i--) {
        var e = LT.events[i];
        if (!LT._roleShown[e.role]) continue;
        if (LT._onlyFailures && e.status !== "FAILURE") continue;
        if (q && (e.ir_id + " " + e.name + " " + e.role + " " + (e.message || "")).toLowerCase().indexOf(q) < 0) continue;
        rows.push(eventRow(e, i, view));
      }
      rows.reverse();
      var box = h("div", { cls: "scroll" }, h("table", { cls: "events" }, h("tbody", null, rows)));
      el.appendChild(box);
      requestAnimationFrame(function () { box.scrollTop = box.scrollHeight; });
    },
  };

  function eventStatus(e) {
    // "not complete yet" is the normal way into a step, not a failure
    if (e.role === "complete" && e.status === "FAILURE") return "idle";
    return e.status === "SUCCESS" ? "success" : e.status === "FAILURE" ? "failure"
      : e.status === "RUNNING" ? "running" : "skipped";
  }

  function metricsText(e) {
    var m = e.metrics || {};
    return Object.keys(m).filter(function (k) { return k !== "arguments"; }).slice(0, 4)
      .map(function (k) { return metricLabel(k) + " " + metricText(k, m[k], e); }).join(", ");
  }

  function eventRow(e, i, view) {
    return h("tr", { cls: i === LT.cursor - 1 ? "current" : null,
        onclick: function () { LT.follow = false; LT.selected = isSide(e) ? LT.selected : e.ir_id; seek(i + 1); } },
      h("td", { cls: "mono muted", text: "+" + (e.t - LT.events[0].t).toFixed(1) }),
      h("td", null, h("span", { cls: "dot " + stateClass(eventStatus(e)), title: e.status })),
      h("td", { cls: "mono", text: e.ir_id }),
      h("td", { text: e.role }),
      h("td", { cls: "msg", text: [e.message, metricsText(e)].filter(Boolean).join(" · ") }));
  }

  builtins.details = {
    title: "Details",
    render: function (view, el) {
      var n = LT.selected && view.nodes[LT.selected];
      if (!n) { el.appendChild(h("p", { cls: "muted", text: "Select a node in the plan, a bar in the timeline or an event." })); return; }
      var rows = [
        ["id", n.id], ["type", n.type || "–"], ["state", nodeState(n)],
        ["call", call(n.def)],
        ["attempts", n.attempt ? n.attempt + (n.maxAttempts ? " of " + n.maxAttempts : "") : null],
        ["skipped", n.skipped ? n.skipReason : null],
        ["planning", n.planSeconds ? fmtSeconds(n.planSeconds) : null],
        ["motion", n.motions ? n.motions + " command(s), " + fmtSeconds(n.motionSeconds) : null],
        ["drift replans", n.drifts || null],
        ["started", n.started !== null ? "+" + fmtSeconds(n.started - view.t0) : null],
        ["took", n.started !== null && n.ended !== null ? fmtSeconds(n.ended - n.started) : null],
      ];
      el.appendChild(h("h3", { text: n.label }));
      el.appendChild(h("dl", { cls: "kv" }, rows.filter(function (r) { return r[1] !== null && r[1] !== undefined; })
        .map(function (r) { return [h("dt", { text: r[0] }), h("dd", { text: String(r[1]) })]; })
        .reduce(function (a, b) { return a.concat(b); }, [])));
      if (n.activity && n.status === "RUNNING") {
        el.appendChild(h("p", null, h("b", { text: "Now: " }), n.activity.message));
        el.appendChild(stepButtons(n));
      }
      if (n.watchdog.length) {
        el.appendChild(h("p", { text: "Watchdog:" }));
        el.appendChild(h("ul", null, n.watchdog.map(function (w) {
          return h("li", { text: "+" + fmtSeconds(w.metrics.elapsed) + " " + w.message + " (" + (w.metrics.by || "") + ")" });
        })));
      }
      if (n.failures.length) {
        el.appendChild(h("p", { text: "Failures:" }));
        el.appendChild(h("ul", null, n.failures.map(function (f) { return h("li", { cls: "mono", text: f }); })));
      }
      el.appendChild(h("div", { cls: "scroll" }, h("table", { cls: "events" }, h("tbody", null,
        n.events.map(function (i) { return eventRow(LT.events[i], i, view); })))));
    },
  };

  builtins.scene = {
    title: "Scene", wide: true,
    available: function () { return !!config.scene_url; },
    render: function (view, el) {
      if (!el.querySelector("iframe")) {
        el.appendChild(h("div", { cls: "scene" }, h("iframe", { src: config.scene_url, title: "3D scene" })));
      }
    },
    keep: true, // the iframe is not rebuilt on every update
  };

  // chat (#90): the operator chat of a live server, next to everything else
  var chat = { entries: [], busy: false, ended: false, log: null, status: null, input: null,
               actions: [], bar: null, cards: [] };

  function actionButton(a, primary) {
    return h("button", { cls: primary ? "primary" : null, disabled: !a.enabled || chat.busy || chat.ended,
      "data-action": a.name, text: a.label, onclick: function () { runAction(a.name); } });
  }

  function runAction(name) {
    post("api/action", { name: name }).then(function (r) {
      if (r.error) { chat.log.appendChild(chatEntry({ who: "error", text: r.error, t: Date.now() / 1000 })); return; }
      chat.busy = true;
      chatStatus();
      if (name === "start") openMonitor();
    });
  }

  // Start: follow the run live, with the plan monitor in view.
  function openMonitor() {
    LT.follow = true;
    seek(LT.events.length);
    var m = sections.monitor;
    if (m && m.group) { activate(m.group, "monitor"); }
    var target = document.querySelector('[data-panel="monitor"]') || document.querySelector('[data-panel="plan"]');
    if (target) {
      target.classList.add("flash");
      target.scrollIntoView({ behavior: "smooth", block: "start" });
      setTimeout(function () { target.classList.remove("flash"); }, 1600);
    }
  }
  LT.openMonitor = openMonitor;

  function planCard(e) {
    var start = chat.actions.filter(function (a) { return a.name === "start"; })[0];
    var card = h("div", { cls: "msg plan-card" },
      h("b", { text: "Plan: " + e.steps.length + " step" + (e.steps.length === 1 ? "" : "s") }),
      h("ol", null, e.steps.map(function (s) { return h("li", { text: s }); })));
    var bar = h("div", { cls: "card-actions" });
    if (start) bar.appendChild(actionButton(start, true));
    card.appendChild(bar);
    chat.cards.push(bar);
    return card;
  }

  function chatEntry(e) {
    if (e.who === "plan") return planCard(e);
    var cls = "msg " + e.who + (e.who === "tool" && !e.ok ? " rejected" : "");
    var text = e.who === "operator" && e.source && e.source !== "web" ? e.text + "  (" + e.source + ")" : e.text;
    return h("div", { cls: cls, title: new Date(e.t * 1000).toLocaleTimeString() }, text || "\u2026");
  }

  function chatStatus() {
    if (!chat.status) return;
    var last = chat.entries.length ? chat.entries[chat.entries.length - 1] : null;
    var acting = chat.busy && last && last.who === "operator" && last.action;
    chat.status.textContent = chat.ended ? "The chat has ended." :
      acting ? last.text + ": running\u2026" : chat.busy ? "The model is working\u2026" : "";
    chat.input.disabled = chat.ended;
    // action buttons: the bar under the log, and the latest plan card's
    chat.bar.textContent = "";
    var onCard = chat.cards.length ? "start" : null; // the latest plan card has it
    chat.actions.forEach(function (a) { if (a.name !== onCard) chat.bar.appendChild(actionButton(a, false)); });
    chat.cards.forEach(function (bar, i) {
      bar.textContent = "";
      var start = chat.actions.filter(function (a) { return a.name === "start"; })[0];
      if (start && i === chat.cards.length - 1) bar.appendChild(actionButton(start, true));
    });
  }

  function chatPoll() {
    if (chat.polling) return;
    chat.polling = true;
    fetch("api/chat?since=" + chat.entries.length).then(function (r) { return r.json(); })
      .then(function (data) {
        if (data.since === chat.entries.length && data.entries.length) {
          data.entries.forEach(function (e) { chat.entries.push(e); if (chat.log) chat.log.appendChild(chatEntry(e)); });
          if (chat.log) chat.log.scrollTop = chat.log.scrollHeight;
        }
        chat.busy = data.busy; chat.ended = data.ended; chat.actions = data.actions || [];
        chatStatus();
      })
      .catch(function () {})
      .then(function () {
        chat.polling = false;
        if (!chat.ended) setTimeout(chatPoll, config.poll_ms || 500);
      });
  }

  builtins.chat = {
    title: "Chat",
    available: function () { return !!boot.chat; },
    keep: true,
    render: function (view, el) {
      if (chat.log && el.contains(chat.log)) return;
      chat.cards = [];
      chat.log = h("div", { cls: "log", "aria-live": "polite" }, chat.entries.map(chatEntry));
      chat.status = h("div", { cls: "busy" });
      chat.bar = h("div", { cls: "actions" });
      chat.input = h("input", { type: "text", placeholder: "Ask the mission model, e.g. \u201cplan part 2 first\u201d", "aria-label": "message to the model" });
      var form = h("form", { onsubmit: function (e) {
        e.preventDefault();
        var text = chat.input.value.trim();
        if (!text) return;
        post("api/chat", { message: text }).then(function (r) {
          if (r.error) { chat.log.appendChild(chatEntry({ who: "error", text: r.error, t: Date.now() / 1000 })); }
          else { chat.input.value = ""; chat.busy = !r.ended; chat.ended = r.ended; chatStatus(); }
        });
      } }, chat.input, h("button", { type: "submit", text: "Send" }));
      el.appendChild(h("div", { cls: "chat" }, chat.log, chat.status, chat.bar, form));
      chatStatus();
    },
  };

  // -- layout and rendering --------------------------------------------------

  var sections = {};

  function panelFor(id) {
    return LT._panels[id] || builtins[id] || null;
  }

  // Where each panel goes on one screen (ViewerConfig.layout "screen"): a
  // list per area; an inner list is a group of tabs.
  var SCREEN = config.screen || { top: ["summary"], left: [["monitor", "plan"], ["details", "events"]],
                 center: ["scene", "timeline"], right: ["chat"] };
  var FIT = ["summary", "timeline"]; // their natural height, the rest share
  LT._tabs = {};

  function usable(id) {
    var p = panelFor(id);
    if (!p && LT._started && (config.panels || []).indexOf(id) >= 0) {
      p = { title: id, render: function (v, el) { el.appendChild(h("p", { cls: "muted", text: "No panel named " + id + " was registered." })); } };
    }
    if (!p || (p.available && !p.available())) return null;
    return p;
  }

  function enabled() {
    var ids = (config.panels || []).slice();
    Object.keys(LT._panels).forEach(function (id) { if (ids.indexOf(id) < 0) ids.push(id); });
    return ids;
  }

  function section(id, p, extraCls) {
    var body = h("div", { cls: "body" });
    var sec = h("section", { cls: "panel" + (p.wide ? " wide" : "") + (extraCls || ""), "data-panel": id },
      h("h2", { text: p.title || id }), body);
    sections[id] = { panel: p, body: body, sec: sec };
    return sec;
  }

  function tabGroup(ids, key) {
    var shown = ids.filter(function (id) { return usable(id); });
    if (!shown.length) return null;
    if (shown.length === 1) return section(shown[0], usable(shown[0]));
    var active = LT._tabs[key] && shown.indexOf(LT._tabs[key]) >= 0 ? LT._tabs[key] : shown[0];
    var bar = h("div", { cls: "tabbar", role: "tablist" });
    var group = h("section", { cls: "panel tabs", "data-group": key }, bar);
    shown.forEach(function (id) {
      var p = usable(id), body = h("div", { cls: "body" });
      var tab = h("button", { cls: "tab", role: "tab", "data-tab": id, text: p.title || id,
        onclick: function () { activate(key, id); } });
      bar.appendChild(tab);
      var pane = h("div", { cls: "pane", "data-panel": id }, body);
      group.appendChild(pane);
      sections[id] = { panel: p, body: body, sec: pane, group: key, tab: tab };
    });
    LT._tabs[key] = active;
    showTab(key);
    return group;
  }

  function showTab(key) {
    Object.keys(sections).forEach(function (id) {
      var s = sections[id];
      if (s.group !== key) return;
      var on = LT._tabs[key] === id;
      s.sec.hidden = !on;
      s.tab.classList.toggle("active", on);
      s.tab.setAttribute("aria-selected", on ? "true" : "false");
    });
  }

  function activate(key, id) {
    LT._tabs[key] = id; showTab(key); render();
    if (id === "monitor" && mon.svg) setTimeout(monFit, 30); // sized now
  }

  function screenLayout(main) {
    var on = enabled();
    var placed = {};
    var areas = {};
    Object.keys(SCREEN).forEach(function (area) {
      areas[area] = SCREEN[area].map(function (entry) {
        var ids = (Array.isArray(entry) ? entry : [entry]).filter(function (id) { return on.indexOf(id) >= 0; });
        ids.forEach(function (id) { placed[id] = true; });
        return ids;
      }).filter(function (ids) { return ids.length; });
    });
    // panels of yours (and any not placed) join the left tabs
    var rest = on.filter(function (id) { return !placed[id]; });
    if (rest.length) {
      var tabs = areas.left.filter(function (ids) { return ids.length > 1; })[0];
      if (tabs) Array.prototype.push.apply(tabs, rest); else areas.left.push(rest);
    }
    // no scene: the tabs take the centre instead
    if (!usable("scene")) {
      var moved = areas.left.filter(function (ids) { return ids.length > 1 || ids[0] !== "plan"; });
      areas.left = areas.left.filter(function (ids) { return moved.indexOf(ids) < 0; });
      areas.center = areas.center.concat(moved);
    }
    var top = h("div", { cls: "top" });
    areas.top.forEach(function (ids, i) {
      var g = tabGroup(ids, "top" + i);
      if (g) { g.classList.add("fit", "compact"); top.appendChild(g); }
    });
    if (top.childNodes.length) main.appendChild(top);
    var grid = h("div", { cls: "columns" }), widths = [];
    ["left", "center", "right"].forEach(function (area) {
      var col = h("div", { cls: "col col-" + area });
      areas[area].forEach(function (ids, i) {
        var g = tabGroup(ids, area + i);
        if (!g) return;
        if (ids.length === 1 && FIT.indexOf(ids[0]) >= 0) g.classList.add("fit");
        if (ids.indexOf("plan") >= 0 || ids.indexOf("monitor") >= 0) g.classList.add("tall");
        col.appendChild(g);
      });
      if (!col.childNodes.length) return;
      grid.appendChild(col);
      widths.push(area === "center" ? "minmax(0, 1fr)" : area === "left" ? "minmax(300px, 30%)" : "minmax(280px, 26%)");
    });
    grid.style.gridTemplateColumns = widths.join(" ");
    main.appendChild(grid);
  }

  function layout() {
    var main = document.getElementById("panels");
    main.textContent = "";
    sections = {};
    var screen = (config.layout || "screen") === "screen";
    document.body.classList.toggle("screen", screen);
    if (screen) { screenLayout(main); return; }
    enabled().forEach(function (id) {
      var p = usable(id);
      if (p) main.appendChild(section(id, p));
    });
  }

  // Skip search / Abort step, for the step being planned (#108)
  function stepButtons(n) {
    var box = h("span", { cls: "step-buttons" });
    if (!boot.control || !LT.live || !n || n.status !== "RUNNING" || !n.activity) return box;
    var label = n.label;
    if (n.activity.metrics.search) {
      box.appendChild(h("button", { text: "Skip search", title: "plan " + label + " without finishing: " + n.activity.metrics.search,
        onclick: function () { post("api/control", { action: "skip" }); } }));
    }
    box.appendChild(h("button", { cls: "danger", text: "Abort step", title: "end " + label + " now, as a failure",
      onclick: function () {
        if (window.confirm("Abort " + label + "? The run stops at this step.")) post("api/control", { action: "abort_step" });
      } }));
    return box;
  }

  function header(view) {
    var slot = document.getElementById("step-controls");
    if (slot) {
      slot.textContent = "";
      var pn = view.planning && view.nodes[view.planning];
      if (pn && pn.status === "RUNNING" && pn.activity) {
        var buttons = stepButtons(pn);
        if (buttons.childNodes.length) {
          slot.appendChild(h("span", { cls: "muted", text: pn.label + ": " }));
          slot.appendChild(buttons);
        }
      }
    }
    document.getElementById("title").textContent = config.title;
    var root = view.roots.length ? view.nodes[view.roots[0]] : null;
    var state = view.paused ? "paused" : root ? nodeState(root) : "idle";
    var pill = document.getElementById("state");
    pill.className = "pill " + stateClass(state);
    pill.textContent = (LT.live ? "live · " : "replay · ") + state;
    document.getElementById("clock").textContent =
      view.t0 === null ? "" : "t = " + fmtSeconds(view.t - view.t0);
  }

  function render() {
    var view = LT.view;
    header(view);
    Object.keys(sections).forEach(function (id) {
      var s = sections[id];
      if (s.group && LT._tabs[s.group] !== id && !s.panel.keep) return; // hidden tab
      if (!s.panel.keep) s.body.textContent = "";
      try { s.panel.render(view, s.body); }
      catch (err) { s.body.textContent = "panel error: " + err.message; }
    });
  }

  var pending = false;
  function renderSoon() {
    if (pending) return;
    pending = true;
    requestAnimationFrame(function () { pending = false; render(); });
  }

  function seek(cursor, soon) {
    computeTo(Math.max(0, Math.min(cursor, LT.events.length)));
    if (soon) renderSoon(); else render();
  }
  LT.seek = seek;

  // -- live updates and controls ---------------------------------------------

  function post(path, body) {
    return fetch(path, { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body) }).then(function (r) { return r.json(); });
  }
  LT.post = post;

  function poll() {
    fetch("api/events?since=" + LT.events.length).then(function (r) { return r.json(); })
      .then(function (data) {
        if (data.events && data.events.length && data.since === LT.events.length) {
          Array.prototype.push.apply(LT.events, data.events);
          if (LT.follow) seek(LT.events.length, true); else renderSoon();
        }
      })
      .catch(function () { /* the server stopped: keep what we have */ })
      .then(function () { setTimeout(poll, config.poll_ms || 500); });
  }

  function controls() {
    var box = document.getElementById("controls");
    box.textContent = "";
    if (!boot.control) return;
    ["pause", "resume", "stop"].forEach(function (action) {
      box.appendChild(h("button", { text: action, onclick: function () {
        if (action === "stop" && !window.confirm("Stop the mission?")) return;
        post("api/control", { action: action });
      } }));
    });
  }

  function theme() {
    var root = document.documentElement.style;
    Object.keys(config.theme || {}).forEach(function (k) {
      root.setProperty(k.slice(0, 2) === "--" ? k : "--" + k, config.theme[k]);
    });
    Object.keys(config.status_colors || {}).forEach(function (k) {
      root.setProperty("--st-" + k.toLowerCase(), config.status_colors[k]);
    });
  }

  // What the server offers can change after the page was made (a chat
  // attached later), and a separate front process does not know it.
  function features() {
    fetch("api/features").then(function (r) { return r.json(); })
      .then(function (f) {
        var changed = !!f.control !== !!boot.control || !!f.chat !== !!boot.chat;
        boot.control = !!f.control; boot.chat = !!f.chat;
        if (changed) { layout(); controls(); render(); LT._featureHooks.forEach(function (fn) { fn(f); }); }
      })
      .catch(function () {})
      .then(function () { setTimeout(features, 5000); });
  }
  LT._featureHooks = [];

  LT.start = function () {
    LT._started = true;
    theme();
    layout();
    controls();
    seek(LT.events.length);
    if (LT.live) {
      poll();
      features();
      if (boot.chat) chatPoll();
      LT._featureHooks.push(function (f) { if (f.chat) chatPoll(); });
    }
  };
})();
