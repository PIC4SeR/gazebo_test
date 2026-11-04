"""Campaign control dashboard for gazebo_test.

Runs a campaign plan (plan.py) as a queue of batches over parallel lanes
(campaign.py, lanes.py): each batch is one ``ros2 gazeboexp run`` call, each lane
one ROS domain. Progress, results and the pedestrian-draw pairing are read back
from the gazebo_test outcomes (status.py). The page starts and pauses the
campaign, stops and cleans lanes, reorders, skips and requeues batches.

Binds to 127.0.0.1 by default: the page starts processes on this machine.
"""

from __future__ import annotations

import argparse
import csv
import html
import io
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from flask import Flask, Response, jsonify, request

from .campaign import HEALTH_EVERY_S, Campaign
from .lanes import LaneBackend, TmuxLaneBackend
from .plan import Plan, gazebo_episode_resolver, known_names, load_plan, plan_warnings

TICK_S = 2.0


def _loop(stop: threading.Event, period: float, action) -> None:
    while not stop.wait(period):
        try:
            action()
        except Exception as exc:  # noqa: BLE001 - keep the dashboard alive, show it in the log
            print(f"dashboard: {action.__name__} failed: {exc!r}")


def create_app(plan: Plan, backend: Optional[LaneBackend] = None, episode_resolver=None,
               warnings: Optional[List[str]] = None, threads: bool = True) -> Flask:
    app = Flask(__name__)
    campaign = Campaign(plan, backend or TmuxLaneBackend(), episode_resolver)
    app.config["campaign"] = campaign
    if threads:
        stop = threading.Event()
        app.config["stop"] = stop
        for period, action in ((TICK_S, campaign.tick), (HEALTH_EVERY_S, campaign.sample_health)):
            threading.Thread(target=_loop, args=(stop, period, action), daemon=True).start()

    def guarded(action):
        try:
            return jsonify(action())
        except PermissionError as exc:
            return jsonify({"ok": False, "error": str(exc)}), 409
        except (KeyError, ValueError, RuntimeError) as exc:
            return jsonify({"ok": False, "error": str(exc)}), 400

    @app.get("/")
    def index():
        title = html.escape(plan.path.stem)
        return Response(PAGE.replace("__TITLE__", title), mimetype="text/html")

    @app.get("/api/state")
    def api_state():
        state = campaign.snapshot()
        state["warnings"] = list(warnings or [])
        return jsonify(state)

    @app.post("/api/windows")
    def api_windows():
        return guarded(lambda: campaign.set_windows(request.get_json(silent=True) or {}))

    @app.post("/api/campaign/<action>")
    def api_campaign(action: str):
        if action not in ("start", "pause"):
            return jsonify({"ok": False, "error": f"unknown action {action}"}), 400
        return guarded(lambda: campaign.set_running(action == "start"))

    @app.post("/api/lanes/<int:domain>/<action>")
    def api_lane(domain: int, action: str):
        actions = {
            "stop": lambda: campaign.stop_lane(domain),
            "clean": lambda: campaign.clean_lane(domain),
            "enable": lambda: campaign.enable_lane(domain, True),
            "disable": lambda: campaign.enable_lane(domain, False),
        }
        if action not in actions:
            return jsonify({"ok": False, "error": f"unknown action {action}"}), 400
        return guarded(actions[action])

    @app.post("/api/batches/<int:batch_id>/<action>")
    def api_batch(batch_id: int, action: str):
        value = str((request.get_json(silent=True) or {}).get("value", ""))
        return guarded(lambda: campaign.edit(batch_id, action, value))

    @app.post("/api/requeue_failed")
    def api_requeue_failed():
        return guarded(campaign.requeue_failed)

    def csv_response(rows: List[Dict[str, Any]], columns: List[str], name: str) -> Response:
        buffer = io.StringIO()
        writer = csv.DictWriter(buffer, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        return Response(buffer.getvalue(), mimetype="text/csv",
                        headers={"Content-Disposition": f"attachment; filename={name}"})

    @app.get("/export/campaign.csv")
    def export_campaign():
        rows = []
        for b in campaign.snapshot()["batches"]:
            p = b["progress"]
            rows.append({**b, "done": p["done"], "expected": p["expected"],
                         "success_rate": p["success_rate"], "outcomes": p["outcomes"]})
        columns = ["order", "id", "experiment", "algorithm", "episodes", "repetitions", "status",
                   "attempts", "done", "expected", "success_rate", "lane", "started", "finished",
                   "reason", "notes", "outcomes", "last_log"]
        return csv_response(rows, columns, "campaign.csv")

    @app.get("/export/matrix.csv")
    def export_matrix():
        cells = campaign.snapshot()["matrix"]["cells"].values()
        columns = ["experiment", "algorithm", "status", "done", "expected", "success_rate",
                   "attempts", "outcomes"]
        return csv_response(list(cells), columns, "campaign_matrix.csv")

    return app


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Gazebo campaign · __TITLE__</title>
<style>
:root {
  --bg: #f4f5f7; --panel: #ffffff; --ink: #18202b; --muted: #5d6878; --line: #dfe3ea;
  --accent: #1f4e79; --accent-ink: #ffffff; --soft: #eef2f7;
  --ok: #1e7b45; --ok-bg: #e3f3e9; --warn: #8a5a00; --warn-bg: #fff3d6;
  --bad: #b3261e; --bad-bg: #fde7e5; --info: #155e8a; --info-bg: #e2f0fa;
  --idle: #5d6878; --idle-bg: #eceff3;
  --mono: ui-monospace, "SFMono-Regular", Menlo, Consolas, monospace;
}
@media (prefers-color-scheme: dark) {
  :root {
    --bg: #11151b; --panel: #1a2029; --ink: #e6eaf0; --muted: #98a3b3; --line: #2c3440;
    --accent: #5b9bd5; --accent-ink: #0b1220; --soft: #222a35;
    --ok: #6fd39a; --ok-bg: #173526; --warn: #f2c45a; --warn-bg: #3a2f12;
    --bad: #ff8a80; --bad-bg: #3d1d1b; --info: #7cc4f2; --info-bg: #15303f;
    --idle: #98a3b3; --idle-bg: #262e39;
  }
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--ink);
       font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; }
header { display: flex; align-items: center; gap: 16px; flex-wrap: wrap;
         padding: 12px 24px; border-bottom: 1px solid var(--line); background: var(--panel); }
header h1 { margin: 0; font-size: 18px; }
header .meta { color: var(--muted); font-size: 12px; }
header .spacer { flex: 1; }
main { padding: 16px 24px 40px; display: grid; gap: 16px; max-width: 1600px; margin: 0 auto; }
.banner { padding: 10px 14px; border-radius: 8px; background: var(--warn-bg); color: var(--warn); }
.card { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 16px 18px;
        min-width: 0; }
.card h2 { margin: 0 0 10px; font-size: 13px; text-transform: uppercase; letter-spacing: .06em;
           color: var(--muted); font-weight: 600; display: flex; gap: 10px; align-items: center; }
.card h2 .spacer { flex: 1; }
.lanes { display: grid; grid-template-columns: repeat(auto-fill, minmax(360px, 1fr)); gap: 12px; }
.lane { border: 1px solid var(--line); border-radius: 8px; padding: 12px; display: grid; gap: 6px; min-width: 0; }
.lane .title { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
.lane .title strong { font-size: 15px; }
.lane .kv { color: var(--muted); font-size: 12px; }
.lane .kv b { color: var(--ink); font-weight: 600; }
.now { font-size: 13px; color: var(--info); }
.lane .msg { font-size: 12px; color: var(--warn); }
.bar { height: 6px; background: var(--soft); border-radius: 3px; overflow: hidden; }
.bar > div { height: 100%; background: var(--accent); }
pre { margin: 0; padding: 8px; background: var(--soft); border-radius: 6px; font: 11px/1.35 var(--mono);
      max-height: 180px; overflow: auto; white-space: pre-wrap; word-break: break-all; }
details summary { cursor: pointer; color: var(--muted); font-size: 12px; }
.chip { display: inline-block; padding: 1px 8px; border-radius: 999px; font-size: 12px; font-weight: 600;
        background: var(--idle-bg); color: var(--idle); white-space: nowrap; }
.chip.running { background: var(--info-bg); color: var(--info); }
.chip.done { background: var(--ok-bg); color: var(--ok); }
.chip.failed, .chip.occupied { background: var(--bad-bg); color: var(--bad); }
.chip.pending { background: var(--warn-bg); color: var(--warn); }
button { font: inherit; font-size: 12px; padding: 4px 10px; border-radius: 6px; cursor: pointer;
         border: 1px solid var(--line); background: var(--panel); color: var(--ink); }
button:hover { background: var(--soft); }
button.primary { background: var(--accent); color: var(--accent-ink); border-color: var(--accent); font-size: 14px; padding: 6px 16px; }
button.danger { color: var(--bad); border-color: var(--bad); }
button:disabled { opacity: .45; cursor: default; }
.tablewrap { overflow-x: auto; }
table { border-collapse: collapse; width: 100%; font-size: 13px; }
th, td { padding: 6px 8px; border-bottom: 1px solid var(--line); text-align: left; vertical-align: top; }
th { color: var(--muted); font-weight: 600; font-size: 12px; white-space: nowrap; }
td.num { font-variant-numeric: tabular-nums; white-space: nowrap; }
td.actions { white-space: nowrap; }
td.actions button { padding: 2px 7px; }
.cell { display: grid; gap: 2px; }
.cell .sub { color: var(--muted); font-size: 11px; }
.muted { color: var(--muted); }
.mono { font-family: var(--mono); font-size: 12px; }
.ok { color: var(--ok); } .bad { color: var(--bad); }
svg.spark { width: 120px; height: 24px; vertical-align: middle; }
svg.spark polyline { fill: none; stroke: var(--accent); stroke-width: 1.5; }
.toast { position: fixed; bottom: 16px; right: 16px; background: var(--bad-bg); color: var(--bad);
         padding: 10px 14px; border-radius: 8px; display: none; max-width: 480px; }
@media (max-width: 600px) { header, main { padding-left: 16px; padding-right: 16px; }
  .lanes { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>Gazebo campaign · __TITLE__</h1>
  <span class="meta" id="meta"></span>
  <span class="spacer"></span>
  <span id="counts" class="meta"></span>
  <span class="meta" title="Optional tmux windows of the next launches (running lanes keep theirs)">windows:
    <label><input type="checkbox" id="w_gazebo_gui" onchange="setWindow(this)"> Gazebo GUI</label>
    <label><input type="checkbox" id="w_rviz" onchange="setWindow(this)"> RViz</label>
    <label><input type="checkbox" id="w_hunav_eval" onchange="setWindow(this)"> HuNav eval</label></span>
  <button class="primary" id="runbtn" onclick="toggleRun()">Start campaign</button>
</header>
<main>
  <div id="warnings"></div>
  <section class="card"><h2>Lanes <span class="spacer"></span><span class="muted" id="stagger"></span></h2>
    <div class="lanes" id="lanes"></div></section>
  <section class="card"><h2>Matrix <span class="spacer"></span>
    <a href="/export/matrix.csv"><button>Export matrix CSV</button></a></h2>
    <div class="tablewrap" id="matrix"></div></section>
  <section class="card" id="pairingcard" style="display:none"><h2>Pedestrian-draw pairing</h2>
    <div class="tablewrap" id="pairing"></div></section>
  <section class="card"><h2>Queue <span class="spacer"></span>
    <button onclick="post('/api/requeue_failed')">Requeue failed</button>
    <a href="/export/campaign.csv"><button>Export campaign CSV</button></a></h2>
    <div class="tablewrap" id="queue"></div></section>
</main>
<div class="toast" id="toast"></div>
<script>
let state = null;
const esc = s => String(s ?? "").replace(/[&<>"']/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));
const pct = v => v == null ? "–" : Math.round(v * 100) + "%";
const dur = s => s == null ? "–" : s < 120 ? s + " s" : s < 7200 ? Math.round(s / 60) + " min" : (s / 3600).toFixed(1) + " h";

function toast(msg) {
  const t = document.getElementById("toast"); t.textContent = msg; t.style.display = "block";
  clearTimeout(t._h); t._h = setTimeout(() => t.style.display = "none", 6000);
}
async function post(url, body) {
  const r = await fetch(url, {method: "POST", headers: {"Content-Type": "application/json"},
                              body: JSON.stringify(body || {})});
  const j = await r.json().catch(() => ({}));
  if (!r.ok || j.ok === false) toast(j.error || ("HTTP " + r.status));
  refresh();
  return j;
}
function setWindow(box) { post("/api/windows", {[box.id.slice(2)]: box.checked}); }
function toggleRun() { post("/api/campaign/" + (state && state.running ? "pause" : "start")); }
function laneAction(d, a) {
  if (a === "stop" && !confirm("Stop lane " + d + "? Its batch goes back to the queue and the lane is disabled.")) return;
  if (a === "clean" && !confirm("Kill every process of lane " + d + " (tmux session and anything with its Gazebo URI)?")) return;
  post(`/api/lanes/${d}/${a}`);
}
function batchAction(id, a) {
  if (a === "notes") {
    const b = state.batches.find(x => x.id === id);
    const v = prompt("Notes for batch " + id, b ? b.notes : "");
    if (v === null) return;
    return post(`/api/batches/${id}/notes`, {value: v});
  }
  post(`/api/batches/${id}/${a}`);
}
function spark(values) {
  if (!values || values.length < 2) return "";
  const max = Math.max(1, ...values), n = values.length;
  const pts = values.map((v, i) => `${(i / (n - 1) * 118 + 1).toFixed(1)},${(23 - v / max * 22).toFixed(1)}`).join(" ");
  return `<svg class="spark" viewBox="0 0 120 24"><polyline points="${pts}"/></svg>`;
}
function now(c) {
  if (!c) return `<div class="kv">now: <span class="muted">starting up (no run started yet)</span></div>`;
  const ep = c.episode_index ? ` <span class="muted">(${c.episode_index}/${c.episodes})</span>` : "";
  const state = c.recorded ? ` <span class="muted">· recorded, resetting</span>` : "";
  return `<div class="now">now: <b>${esc(c.episode)}</b>${ep} · run <b>${c.run_id}</b>/${c.repetitions}${state}</div>`;
}
function renderLanes(s) {
  document.getElementById("lanes").innerHTML = s.lanes.map(l => {
    const b = l.batch, p = b ? b.progress : null;
    const frac = p && p.expected ? Math.min(1, p.done / p.expected) : 0;
    const open = document.querySelector(`#lane-${l.domain} details[open]`) ? "open" : "";
    return `<div class="lane" id="lane-${l.domain}">
      <div class="title"><strong>Lane ${l.domain}</strong><span class="chip ${l.state}">${l.state}</span>
        <span class="muted mono">${esc(l.session)}</span><span class="spacer" style="flex:1"></span>
        ${b ? `<button class="danger" onclick="laneAction(${l.domain},'stop')">Stop</button>` :
              `<button onclick="laneAction(${l.domain},'clean')">Clean</button>
               <button onclick="laneAction(${l.domain},'${l.enabled ? "disable" : "enable"}')">${l.enabled ? "Disable" : "Enable"}</button>`}
      </div>
      ${b ? `<div><b>${esc(b.experiment)}</b> · ${esc(b.algorithm)}${b.episodes ? " · " + esc(b.episodes) : ""}
              <span class="muted">(batch ${b.id}, attempt ${b.attempt})</span></div>
        ${now(b.current)}
        <div class="bar"><div style="width:${(frac * 100).toFixed(1)}%"></div></div>
        <div class="kv">runs <b>${p.done}/${p.expected ?? "?"}</b> · success <b>${pct(p.success_rate)}</b>
          · elapsed <b>${dur(l.elapsed_s)}</b> · last row <b>${dur(l.since_progress_s)}</b> ago</div>
        <div class="kv">RTF <b>${l.rtf == null ? "–" : l.rtf.toFixed(2)}</b> ${spark(l.rtf_history)}
          sim <b>${l.sim_time == null ? "–" : l.sim_time.toFixed(0) + " s"}</b>${l.paused ? " (paused)" : ""}
          ${l.health_error ? `<span class="muted">· ${esc(l.health_error)}</span>` : ""}</div>
        ${p.last ? `<div class="kv">last: ${esc(p.last.episode)} run ${p.last.run_id} → <b>${esc(p.last.result)}</b></div>` : ""}
        <details ${open}><summary>Logs · <span class="mono">tmux attach -t ${esc(l.session)}</span></summary>
          <div class="kv">experiment manager window</div><pre>${esc(l.pane_tail) || "(no tmux output yet)"}</pre>
          <div class="kv">launcher · <span class="mono">${esc(l.log)}</span></div><pre>${esc(l.log_tail)}</pre>
        </details>` : `<div class="muted">idle</div>`}
      ${l.message ? `<div class="msg">${esc(l.message)}</div>` : ""}
    </div>`;
  }).join("");
}
function renderMatrix(s) {
  const m = s.matrix;
  let h = "<table><tr><th>Experiment</th>" + m.algorithms.map(a => `<th>${esc(a)}</th>`).join("") + "</tr>";
  for (const e of m.experiments) {
    h += `<tr><td><b>${esc(e)}</b></td>`;
    for (const a of m.algorithms) {
      const c = m.cells[e + "|" + a];
      if (!c) { h += "<td class='muted'>–</td>"; continue; }
      h += `<td><div class="cell"><span><span class="chip ${c.status}">${c.status}</span>
            <b>${c.done}/${c.expected ?? "?"}</b></span>
            <span class="sub">success ${pct(c.success_rate)} · ${c.attempts} launch${c.attempts === 1 ? "" : "es"}</span>
            ${c.outcomes ? `<span class="sub mono" title="${esc(c.outcomes)}">${esc(c.outcomes.split("/").slice(-3).join("/"))}</span>` : ""}
          </div></td>`;
    }
    h += "</tr>";
  }
  document.getElementById("matrix").innerHTML = h + "</table>";
}
function renderPairing(s) {
  const exps = Object.keys(s.pairing);
  document.getElementById("pairingcard").style.display = exps.length ? "" : "none";
  if (!exps.length) return;
  let h = "<table><tr><th>Experiment</th><th>Common draws</th><th>Per algorithm: paired / runs</th><th></th></tr>";
  for (const e of exps) {
    const p = s.pairing[e];
    const per = Object.entries(p.algorithms).map(([a, v]) =>
      `${esc(a)} <b>${v.paired}/${v.runs}</b>${v.repeated ? ` <span class="bad">(${v.repeated} repeated)</span>` : ""}`).join(" · ");
    const extra = p.without_draws.length ? `<span class="muted">no draws yet: ${p.without_draws.map(esc).join(", ")}</span>` : "";
    h += `<tr><td><b>${esc(e)}</b></td><td class="num">${p.common_draws}</td><td>${per}</td>
          <td>${p.ok ? '<span class="ok">paired</span>' : '<span class="bad">unpaired runs</span>'} ${extra}</td></tr>`;
  }
  document.getElementById("pairing").innerHTML = h + "</table>" +
    `<p class="muted">Paired tests blocked on <span class="mono">ped_draw</span> use only draws every algorithm has. While a campaign runs, the algorithms behind make the others look unpaired.</p>`;
}
function renderQueue(s) {
  let h = `<table><tr><th>#</th><th>Batch</th><th>Experiment</th><th>Algorithm</th><th>Episodes</th><th>Status</th>
           <th>Runs</th><th>Success</th><th>Attempts</th><th>Lane</th><th>Reason / notes</th><th></th></tr>`;
  for (const b of s.batches) {
    const p = b.progress, live = b.status === "running";
    h += `<tr><td class="num">${b.order}</td><td class="num">${b.id}</td><td>${esc(b.experiment)}</td>
      <td>${esc(b.algorithm)}</td><td class="muted">${esc(b.episodes || "all")}${b.current
        ? `<div class="now">▶ ${esc(b.current.episode)} run ${b.current.run_id}</div>` : ""}</td>
      <td><span class="chip ${b.status}">${b.status}</span></td>
      <td class="num">${p.done}/${p.expected ?? "?"}</td><td class="num">${pct(p.success_rate)}</td>
      <td class="num">${b.attempts}</td><td class="num">${esc(b.lane)}</td>
      <td>${b.reason ? `<div class="muted">${esc(b.reason)}</div>` : ""}${b.notes ? `<div>${esc(b.notes)}</div>` : ""}</td>
      <td class="actions">
        <button title="Move up" onclick="batchAction(${b.id},'up')">↑</button>
        <button title="Move down" onclick="batchAction(${b.id},'down')">↓</button>
        ${b.status === "skipped" ? `<button onclick="batchAction(${b.id},'unskip')">Unskip</button>`
          : `<button ${live ? "disabled" : ""} onclick="batchAction(${b.id},'skip')">Skip</button>`}
        <button ${live ? "disabled" : ""} onclick="batchAction(${b.id},'requeue')">Requeue</button>
        <button onclick="batchAction(${b.id},'notes')">Notes</button>
        ${live ? "" : s.lanes.filter(l => l.state === "idle").map(l =>
          `<button title="Run now in tmux session ${esc(l.session)}" onclick="post('/api/batches/${b.id}/run', {value: ${l.domain}})">▶ ${l.domain}</button>`).join("")}
      </td></tr>`;
  }
  document.getElementById("queue").innerHTML = h + "</table>";
}
function render(s) {
  state = s;
  document.getElementById("meta").textContent = s.results;
  const c = s.counts;
  document.getElementById("counts").textContent =
    `${c.done} done · ${c.running} running · ${c.pending} pending · ${c.failed} failed · ${c.skipped} skipped`;
  for (const [k, v] of Object.entries(s.windows)) document.getElementById("w_" + k).checked = v;
  const btn = document.getElementById("runbtn");
  btn.textContent = s.running ? "Pause campaign" : "Start campaign";
  btn.title = s.running ? "No new batches; running ones finish" : "Free lanes take pending batches";
  document.getElementById("stagger").textContent = s.stagger_left_s ? `next start in ≥ ${s.stagger_left_s} s (stagger)` : "";
  document.getElementById("warnings").innerHTML = s.warnings.map(w => `<div class="banner">${esc(w)}</div>`).join("");
  renderLanes(s); renderMatrix(s); renderPairing(s); renderQueue(s);
}
async function refresh() {
  try { const r = await fetch("/api/state"); render(await r.json()); }
  catch (e) { toast("dashboard not reachable"); }
}
refresh(); setInterval(refresh, 3000);
</script>
</body>
</html>
"""


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--plan", type=Path, required=True, help="Campaign plan YAML.")
    parser.add_argument("--start", action="store_true",
                        help="Start the campaign right away (otherwise press Start on the page).")
    parser.add_argument("--no-gz-stats", action="store_true",
                        help="Do not sample Gazebo (real-time factor, sim-time stall check).")
    parser.add_argument("--host", default="127.0.0.1",
                        help="Bind address (default local only: the page launches simulations).")
    parser.add_argument("--port", type=int, default=8060)
    args = parser.parse_args(argv)
    plan = load_plan(args.plan)
    experiments, navigators = known_names()
    warnings = plan_warnings(plan, experiments, navigators)
    for warning in warnings:
        print(f"Warning: {warning}")
    app = create_app(plan, TmuxLaneBackend(gz_stats=not args.no_gz_stats),
                     gazebo_episode_resolver(), warnings)
    if args.start:
        app.config["campaign"].set_running(True)
    print(f"Gazebo campaign: http://{args.host}:{args.port}  ({plan.state_file})")
    try:
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        app.config["stop"].set()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
