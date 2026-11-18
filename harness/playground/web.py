# SPDX-License-Identifier: AGPL-3.0-only
"""The HTTP pages beside /mcp: the home page, the viewer of an episode, health and the source offer."""
import asyncio
import html

from .common import LICENSE, source_link, source_offer, span, utc

PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>{title}</title><style>
body{{font:15px/1.45 system-ui,sans-serif;margin:24px auto;max-width:1040px;padding:0 16px;color:#16202a}}
code,pre{{font:13px ui-monospace,Menlo,monospace;background:#f1f4f7;border-radius:4px}}
code{{padding:1px 4px}}pre{{padding:10px;overflow:auto;max-height:260px;white-space:pre-wrap}}
.box{{border:1px solid #d5dce3;border-radius:8px;padding:12px 14px;margin:12px 0}}
.ok{{background:#e8f6ec;border-color:#8ccf9f}}.no{{background:#fdecec;border-color:#e5a3a3}}
.warn{{background:#fff6e0;border-color:#e2c26a}}
.row{{display:flex;gap:16px;flex-wrap:wrap}}img{{max-width:100%;border-radius:6px;border:1px solid #d5dce3}}
.gone{{display:inline-block;width:256px;padding:40px 8px;text-align:center;color:#5b6875;border:1px dashed #c3ccd5;
border-radius:6px;font-size:13px}}
small{{color:#5b6875}}h1{{font-size:22px}}h2{{font-size:17px;margin:18px 0 6px}}
table{{border-collapse:collapse}}td,th{{padding:4px 10px;border-bottom:1px solid #e3e8ed;text-align:left}}
</style></head><body>{body}</body></html>"""

VIEWER_JS = """
const id = location.pathname.split('/').pop(); let since = 0;
const esc = s => String(s).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const gone = "this.outerHTML='<span class=gone>picture no longer kept</span>'";
function verdict(v){
  const cls = v.solved ? 'ok' : 'no';
  return `<div class="box ${cls}"><b>Benchmark verdict: ${v.solved ? 'solved' : 'not solved'}</b><br>`
    + `control steps used ${esc(v.steps_used)} of ${esc(v.budget_steps)}`
    + (v.first_success_step != null ? `, success check first true at step ${esc(v.first_success_step)}` : '')
    + `; inside the suite's reference limit of ${esc(v.reference_steps)} steps: ${v.within_reference_steps ? 'yes' : 'no'}`
    + `<br>the model claimed ${v.claimed_success === true ? 'success' : v.claimed_success === false ? 'no success' : 'nothing'}`
    + (v.false_claim ? ' (a false claim: it said done and the check says no)' : '')
    + `<br><small>${esc(v.source)}</small></div>`;
}
function row(e){
  if (e.kind === 'status') return `<div class="box"><small>${e.t} s</small> ${esc(e.text)}</div>`;
  if (e.kind === 'verdict') return verdict(e.verdict);
  if (e.kind === 'read') return `<div class="box"><small>${e.t} s</small> read <code>${esc(e.name)}</code></div>`;
  if (e.kind === 'call') return `<div class="box"><small>${e.t} s</small> call <b>${esc(e.name)}</b> `
    + `<code>${esc(JSON.stringify(e.args))}</code></div>`;
  const imgs = (e.frames || []).map(f =>
    `<img src="/frames/${id}/${encodeURIComponent(f)}" width="256" onerror="${gone}">`).join(' ');
  return `<div class="box ${e.kind === 'claim' ? 'ok' : ''}"><small>${e.t} s, took ${e.seconds} s</small> `
    + `${e.kind === 'claim' ? '<b>claim</b>' : 'result of <b>' + esc(e.name) + '</b>'}`
    + `<div class="row">${imgs}</div><details><summary>answer</summary><pre>${esc(e.text)}</pre></details></div>`;
}
function head(d){
  return `<b>${esc(d.title)}</b>: ${esc(d.status)}` + (d.reason ? ` (${esc(d.reason)})` : '')
    + `, ${d.calls} calls, simulator seconds ${d.simulator_seconds}`;
}
function notes(d){
  let n = '';
  if (d.reason === 'server_stopped') n += `<div class="box warn"><b>The sim server was stopped</b> while this `
    + `episode was running. What was recorded up to then is below. Connect again to start a new episode.</div>`;
  if (d.reason === 'idle') n += `<div class="box warn"><b>Stopped for being idle.</b> No call arrived for the `
    + `idle limit, so the simulator was stopped. Connect again to start a new episode.</div>`;
  if (d.status === 'away' || d.reason === 'server_lost') n += `<div class="box warn">${esc(d.why || '')}</div>`;
  if (d.pictures_trimmed) n += `<div class="box"><small>Pictures are kept in full for KEEP_FULL. This episode is `
    + `older: its transcript and its last ${d.pictures_trimmed.kept.length} pictures are kept, `
    + `${d.pictures_trimmed.removed} earlier pictures were removed.</small></div>`;
  return n;
}
async function tick(){
  try {
    const r = await fetch(`/api/episode/${id}?since=${since}`);
    if (r.status === 404) {
      document.getElementById('head').innerHTML = 'This episode is no longer kept. A viewer link works for KEEP_VIEWER.';
      return;
    }
    const d = await r.json();
    document.getElementById('head').innerHTML = head(d);
    document.getElementById('notes').innerHTML = notes(d);
    const log = document.getElementById('log');
    for (const e of d.events) {
      log.insertAdjacentHTML('afterbegin', row(e));
      if (e.frames && e.frames.length) document.getElementById('live').src =
        `/frames/${id}/${encodeURIComponent(e.frames[e.frames.length - 1])}`;
    }
    since = d.events_total;
    if (d.status === 'ended' || d.status === 'failed') { if (!d.events.length) return; }
  } catch (err) {
    document.getElementById('notes').innerHTML = `<div class="box warn">The sim server is not answering right `
      + `now. This page keeps trying and shows the rest when it is back.</div>`;
  }
  setTimeout(tick, 1000);
}
tick();
"""

# The viewer page and its document tell crawlers not to index them, browsers not to pass the link on and caches
# not to keep them; a picture carries the line for crawlers. None of it is private: /episodes lists every
# episode's id.
NOT_INDEXED = {"x-robots-tag": "noindex, nofollow", "referrer-policy": "no-referrer", "cache-control": "no-store"}


def build_routes(settings, pool, url):
    """The pages beside /mcp. url: the MCP address the home page shows."""
    from starlette.responses import FileResponse, HTMLResponse, JSONResponse, PlainTextResponse
    from starlette.routing import Route

    link = source_link(settings)
    where = (f"<a href='{html.escape(link, quote=True)}'>{html.escape(link)}</a>" if link
             else html.escape(settings.source_url))
    keep_viewer, keep_full = span(settings.keep_viewer_s), span(settings.keep_full_pictures_s)
    viewer_js = VIEWER_JS.replace("KEEP_VIEWER", keep_viewer).replace("KEEP_FULL", keep_full)
    foot = (f"<hr><p><small>{html.escape(source_offer(settings).rsplit(': ', 1)[0])}: {where}. The simulated "
            "scene is LIBERO-PRO on LIBERO, robosuite and MuJoCo; these and the other third-party parts keep "
            "their own licenses, listed in harness/LICENSES.md of that source.</small></p>")

    def page(title, body):
        return PAGE.format(title=html.escape(title), body=body + foot)

    def states_table():
        def note(label, state):
            said = ["the default"] if label == settings.default_state else []
            said += [state["note"]] if state.get("note") else []
            return html.escape("; ".join(said))
        rows = "".join(f"<tr><td><code>{html.escape(k)}</code></td><td>{html.escape(v.get('title', ''))}</td>"
                       f"<td>{note(k, v)}</td></tr>" for k, v in settings.states.items())
        lazy = ("<p><small>The first episode of a start state can start one to two minutes later, once: when "
                "its robot body has no calibration record on this machine yet, the simulated robot measures "
                "itself first, the record is stored, and the episode says how long it took.</small></p>"
                if settings.lazy_records else "")
        return (f"<h2>Start states</h2><table><tr><th>start state</th><th>task</th><th></th></tr>{rows}</table>"
                f"<p><small>With no choice you get <code>{html.escape(settings.default_state)}</code>. Choose "
                "another with <code>?state=LABEL</code> on the URL. These are development start states: the "
                "ones the tools were built on. They are not the start states of the published results."
                "</small></p>" + lazy)

    async def home(request):
        name = html.escape(settings.name)
        body = (f"<h1>{name} simulated robot, on this machine</h1>"
                "<p>Add one MCP URL to your own model client. Each session gets its own simulated robot arm on a "
                "tabletop task, with the robot's tools served as MCP tools. Your model does the thinking; the "
                "simulator runs here, on this machine.</p>"
                f"<h2>Add it to your client</h2><pre>{html.escape(url)}</pre>"
                f"<p><small>For example, in Claude Code:</small></p>"
                f"<pre>claude mcp add --transport http cerulion-sim {html.escape(url)}</pre>")
        return HTMLResponse(page("simulated robot", body + states_table()))

    async def lookup(eid):
        ep = pool.episodes.get(eid)
        return ep, (None if ep else await asyncio.to_thread(pool.store.load, eid))

    async def view(request):
        ep, kept = await lookup(request.path_params["eid"])
        if ep is None and kept is None:
            return HTMLResponse(page("episode", "<p>This episode is not kept here. A viewer link works for "
                                                f"{keep_viewer} after its episode.</p>"),
                                status_code=404, headers=NOT_INDEXED)
        body = ("<h1>Episode</h1><div class='box' id='head'>loading</div><div id='notes'></div>"
                f"<p><small>This page is not private: anyone who can reach this server can watch this episode, "
                f"and <code>/episodes</code> lists every episode. The link works for {keep_viewer} after the "
                "episode ends.</small></p>"
                "<div class='row'>"
                "<div><img id='live' width='512' alt='the latest picture the robot returned'></div></div>"
                "<h2>Calls and results, newest first</h2><div id='log'></div>"
                f"<script>{viewer_js}</script>")
        return HTMLResponse(page("episode", body).replace(
            "<meta charset=\"utf-8\">", "<meta charset=\"utf-8\"><meta name=\"robots\" "
            "content=\"noindex, nofollow\"><meta name=\"referrer\" content=\"no-referrer\">", 1), headers=NOT_INDEXED)

    async def api_episode(request):
        try:
            since = max(0, int(request.query_params.get("since", "0")))
        except ValueError:
            since = 0
        eid = request.path_params["eid"]
        ep = pool.episodes.get(eid)
        doc = ep.public(since) if ep else await asyncio.to_thread(pool.store.load, eid, since)
        if doc is None:
            return JSONResponse({"error": "no such episode"}, status_code=404, headers=NOT_INDEXED)
        return JSONResponse(doc, headers=NOT_INDEXED)

    async def frame(request):
        path = await asyncio.to_thread(pool.store.frame_path, request.path_params["eid"],
                                       request.path_params["name"])
        if not path:
            return JSONResponse({"error": f"no such picture (pictures are kept in full for {keep_full})"},
                                status_code=404, headers=NOT_INDEXED)
        return FileResponse(path, headers={"x-robots-tag": NOT_INDEXED["x-robots-tag"]})

    async def episodes(request):
        live = [{k: v for k, v in e.meta().items() if k in ("id", "state", "status", "simulator_seconds", "calls")}
                for e in pool.live()]
        ended = await asyncio.to_thread(pool.history.episodes)
        return JSONResponse({"live": live, "ended": ended})

    async def robots(request):
        return PlainTextResponse("User-agent: *\nDisallow: /view/\nDisallow: /api/\nDisallow: /frames/\n")

    async def health(request):
        going = pool.stopping
        idle_s, idle_from = pool.idle_limit()
        return JSONResponse({"ok": not going, "live_simulators": len(pool.live()), "pool_size": settings.pool_size,
                             "states": list(settings.states), "default_state": settings.default_state,
                             "idle_limit_s": idle_s, "idle_limit_from": idle_from, "history": pool.history.state(),
                             "utc": utc()}, status_code=503 if going else 200)

    async def source(request):
        return JSONResponse({"license": LICENSE, "source_url": settings.source_url, "offer": source_offer(settings),
                             "third_party": "harness/LICENSES.md in the source"})

    return [Route("/", home), Route("/view/{eid}", view), Route("/api/episode/{eid}", api_episode),
            Route("/frames/{eid}/{name}", frame), Route("/episodes", episodes), Route("/health", health),
            Route("/source", source), Route("/robots.txt", robots)]
