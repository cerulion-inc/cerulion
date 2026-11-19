# SPDX-License-Identifier: AGPL-3.0-only
"""Light history of the waiting package: what the wrist camera saw of the light, and when.

One record per level the light held ("run"): {can_see, from_s, to_s, samples, brightness}. A run grows while the
level stays and a new one starts when it changes (journaled as the event "light"). Sources: every look of the
session (read off the executor's look records, so looks taken inside other tools count too), a camera sample taken
by the idle hook when the world idled and the newest reading is older than every_s, the samples wait_for_light
takes, and the one frame the dark gate reads before it refuses a call that needs light. A sample is one frame's
mean brightness through rcpx.perception (the same dark, dim, ok levels as a look);
it saves no frame and takes no world time. Times are the world clock (the backend's t, as robot://state t_s).

Nothing here reads simulator ground truth: the only input is the camera frame.
"""

import numpy as np

from rcpx import perception, bounds
from rcpx import bounds_ratios as R
from rcpx.context import refusal

PKG = "waiting"
SAMPLE_EVERY_S = 5.0     # the idle hook and wait_for_light look at the light this often (world seconds)
FRESH_S = SAMPLE_EVERY_S  # the dark gate reads the light again when its newest reading is older than this
MAX_RUNS = 12            # runs served in robot://state and in results (the newest)
LEVELS = ("dark", "dim", "ok")
NEEDS_LIGHT = ("standard_look",)   # primitives that need light NOW; others add names in ex.state["waiting"]
# moves toward the world: refused while the NEWEST light reading is dark (any look, any sample refreshes it)
TOWARD_WORLD = ("move_tip", "touch", "press", "orient_tool", "move_joints", "measure_base_give", "impulse",
                "probe_motion", "approach", "grasp", "pinch", "zero_check", "self_check", "survey",
                "gripper")      # a jaw that opens or closes beside a part pushes on it as hard as any move
REST_POSES = ("home", "park")      # move_joints to these runs in the dark (round one's rest moves)
DARK_CHECKS = ("can_see", "dark")  # refusal checks that mean "it is dark": a dark look, a move after a dark look


def new_state():
    return {"light": {"runs": [], "last": None, "looks_seen": 0, "every_s": SAMPLE_EVERY_S, "sampling": True,
                      "busy": False, "problem": None},
            "needs_light": set(NEEDS_LIGHT), "toward_world": set(TOWARD_WORLD), "park_lift": None,
            "waited_s": 0.0, "wait_calls": 0}


def wait_left_s(ex):
    """Seconds left to the task's own deadline for waiting (ex.context deadline_s: a world time on the t_s clock,
    stated by the runner), or None when no deadline is stated: then only the caller's max_s bounds a wait."""
    deadline = ex.context.get("deadline_s")
    return None if deadline is None else max(0.0, float(deadline) - ex.sim_time())


def spent(ex):
    """The stated deadline has passed: under one control period is left, so nothing more can be waited for."""
    left = wait_left_s(ex)
    return left is not None and left < ex.dt


def offer(ex, every=None):
    """The wait RCP offers where a text or a pointer names one: as long again as it has been dark so far, at
    least one light reading, cut to what is left to the task's deadline when one is stated."""
    every = SAMPLE_EVERY_S if every is None else every
    runs = st_of(ex)["light"]["runs"]
    since = max(0, ex.sim_time() - runs[-1]["from_s"]) if runs and runs[-1]["can_see"] == "dark" else 0.0
    value = max(every, since)
    left = wait_left_s(ex)
    return value if left is None else min(value, left)


def wait_words(ex):
    left = wait_left_s(ex)
    return "as long as you state" if left is None else f"until the task's deadline ({left:g} s left)"


def dark_hint(ex):
    """What to do while it is dark, naming only this package's tool and the claim: wait_for_light with the offer;
    once a stated deadline has passed: claim cannot with reason dark."""
    st = st_of(ex)
    if spent(ex):
        return (f"the light did not return in {st['waited_s']:g} s of waiting and the task's deadline for it has "
                "passed: claim with success false and cannot, reason \"dark\" (the scene stayed dark, nothing was "
                "located or pressed), evidence the seq of the wait_for_light answer that says so; park, home, "
                "retreat and ease still run")
    runs = st["light"]["runs"]
    since = max(0, ex.sim_time() - runs[-1]["from_s"]) if runs and runs[-1]["can_see"] == "dark" else 0.0
    return (f"wait_for_light(max_s={offer(ex):.2g}) parks and waits {wait_words(ex)}; it has been dark for "
            f"{since:g} s, so the offer waits as long again, at least one light reading. If light stays away, "
            "claim with success false and cannot, reason \"dark\"; park, home, retreat and ease still run")


def fresh(ex, older_than_s=FRESH_S):
    """The newest light reading, read again from one frame when there is none or it is older than older_than_s of
    world time. None when the camera gives no frame."""
    sync_looks(ex)
    L = st_of(ex)["light"]
    last = L["last"]
    if last is None or float(ex.sim_time()) - float(last["t"]) >= older_than_s:
        if sample(ex, "gate") is None:
            return None
    return L["last"]


def mark_dark(ex, last):
    """Round one's dark mark (Executor._dark_look, read by every move inside a primitive too) follows the NEWEST
    reading: set while it reads dark, cleared when it does not, so one dark look never latches."""
    if last is None:
        return
    if last["can_see"] == "dark":
        tag = last.get("look_id") or f"light reading at t {float(last['t']):.1f} s"
        ex._dark_look = (tag, float(last["brightness"] or 0.0))
    else:
        ex._dark_look = None


def camera_facts(ex):
    """The camera's own measured levels (its hidden level and noise) as the robot's self-knowledge holds them,
    for rcpx.perception's dark and dim levels; None when they were not measured (perception then takes the levels
    of a frame with no measured noise)."""
    try:
        value = bounds.of(ex).know.get("camera")[0]
    except Exception:      # self-knowledge that cannot answer is no camera fact: the light is still read
        return None
    return value if isinstance(value, dict) and ("hidden" in value or "noise" in value) else None


def st_of(ex):
    return ex.state.setdefault(PKG, new_state())


def note(ex, can_see, brightness, source, t=None, look_id=None):
    """Add one reading to the history. Returns the run it belongs to."""
    if can_see not in LEVELS:
        return None
    L = st_of(ex)["light"]
    t = float(ex.sim_time() if t is None else t)
    b = None if brightness is None else round(float(brightness), 1)
    last = L["runs"][-1] if L["runs"] else None
    if last is not None and last["can_see"] == can_see:
        last["to_s"] = max(last["to_s"], t)
        last["samples"] += 1
        last["brightness"] = b
        run = last
    else:
        run = {"can_see": can_see, "from_s": t, "to_s": t, "samples": 1, "brightness": b}
        L["runs"].append(run)
        if last is not None:      # the light changed: a world event other packages may listen for
            ex.note_event("light", can_see=can_see, was=last["can_see"], brightness=b, source=source,
                          t_s=round(t, 3))
    L["last"] = {"t": t, "can_see": can_see, "brightness": b, "source": source, "look_id": look_id}
    if not L["busy"]:            # wait_for_light keeps the mark itself while it takes its samples
        mark_dark(ex, L["last"])
    return run


def sync_looks(ex):
    """Take in the looks made since the last call (the executor keeps one record per look, nested ones too)."""
    L = st_of(ex)["light"]
    looks = getattr(ex, "_looks", None) or {}
    n = len(looks)
    if n < L["looks_seen"]:
        L["looks_seen"] = 0
    if n == L["looks_seen"]:
        return
    for lid, rec in list(looks.items())[L["looks_seen"]:]:
        if isinstance(rec, dict):
            note(ex, rec.get("can_see"), rec.get("brightness"), "look", t=rec.get("t"), look_id=lid)
    if n > L["looks_seen"]:       # a look gave a frame: the camera works, so the idle hook may sample again
        L["sampling"], L["problem"] = True, None
    L["looks_seen"] = n


def sample(ex, source):
    """One camera frame's light level, added to the history: {can_see, brightness, t_s}, or None when the camera
    gave no frame (sampling by the idle hook then stops until a look or a sample gives a frame again; the reason
    is kept and journaled once per failure spell)."""
    sync_looks(ex)
    L = st_of(ex)["light"]
    try:
        img = np.asarray(ex.backend.render("wrist"))
        b = perception.brightness(img)
        cs = perception.can_see(img, cam=camera_facts(ex))
    except Exception as exc:
        if L["problem"] is None:
            ex._journal("r2_problem", package=PKG, where="light sample", trace=f"{type(exc).__name__}: {exc}")
        L["problem"] = f"{type(exc).__name__}: {exc}"
        L["sampling"] = False
        return None
    L["sampling"], L["problem"] = True, None
    note(ex, cs, b, source)
    return {"can_see": cs, "brightness": round(float(b), 1), "t_s": round(float(ex.sim_time()), 3)}


def history(ex, since_s=None):
    """The newest runs, oldest first, as the operator reads them."""
    runs = st_of(ex)["light"]["runs"]
    if since_s is not None:
        runs = [r for r in runs if r["to_s"] >= since_s]
    return [{"can_see": r["can_see"], "from_s": round(r["from_s"], 2), "to_s": round(r["to_s"], 2),
             "samples": r["samples"], "brightness": r["brightness"]} for r in runs[-MAX_RUNS:]]


def doc(ex):
    """The light block of robot://state: {can_see, brightness, since_s, seen_s_ago, history}."""
    sync_looks(ex)
    L = st_of(ex)["light"]
    if not L["runs"]:
        out = {"can_see": None, "wait_left_s": wait_left_s(ex), "brightness": None, "since_s": None, "seen_s_ago": None, "history": [],
               "note": "no look yet in this session: look() reads the light"}
        if L["problem"]:
            out["camera_problem"] = L["problem"]
        return out
    now = float(ex.sim_time())
    cur = L["runs"][-1]
    out = {"can_see": cur["can_see"], "brightness": cur["brightness"],
           "since_s": round(max(0.0, now - cur["from_s"]), 1), "seen_s_ago": round(max(0.0, now - cur["to_s"]), 1),
           "history": history(ex),
           "note": "since_s: how long the light has read this level; seen_s_ago: age of the newest reading; "
                   "history: one row per level it held, times on the t_s clock"}
    if cur["can_see"] == "dark":
        out["advice"] = "dark: moves toward the world are refused while the light reads dark; " + dark_hint(ex)
    st = st_of(ex)
    out["waited_s"] = round(float(st["waited_s"]), 3)
    out["wait_left_s"] = wait_left_s(ex)
    if L["problem"]:
        out["camera_problem"] = L["problem"]
    return out


def setup(ex):
    """Register the light hooks: the idle hook, the look listener, the dark gate and the pointer on dark results."""
    st = st_of(ex)
    L = st["light"]

    def on_idle(seconds):
        sync_looks(ex)
        if L["busy"] or not L["sampling"] or seconds <= 0:
            return
        last = L["last"]
        if last is None or float(ex.sim_time()) - last["t"] >= L["every_s"]:
            sample(ex, "idle")

    def on_look(**_fields):
        sync_looks(ex)

    def dark_gate(name, kwargs):
        """The dark gate, keyed on the LIGHT: the newest reading (any look, any sample) decides, read again from one
        frame when there is none or it is older than FRESH_S. A call that needs light now (standard_look) is refused
        with check can_see, a move toward the world with check dark, while it reads dark; when it does not, round
        one's mark of an older dark look is cleared, so one dark look never latches."""
        if name in st["needs_light"]:
            last = fresh(ex, 0.0 if (L["last"] or {}).get("can_see") == "dark" else FRESH_S)
            mark_dark(ex, last)
            if last is None or last["can_see"] != "dark":
                return None
            d = doc(ex)
            return refusal("can_see", f"it is dark (brightness {last['brightness']:g} of 255, dark for "
                           f"{d['since_s']:g} s): {name} needs light and would see nothing", dark_hint(ex))
        if name not in st["toward_world"]:
            return None
        q = kwargs.get("q_deg")
        if name == "move_joints" and isinstance(q, str) and q in REST_POSES:
            return None
        last = fresh(ex)
        mark_dark(ex, last)
        if last is None or last["can_see"] != "dark":
            return None
        d = doc(ex)
        seen = f"look {last['look_id']}" if last.get("look_id") else f"a frame read {d['seen_s_ago']:g} s ago"
        return refusal("dark", f"it is dark (brightness {last['brightness']:g} of 255 in {seen}, dark for "
                       f"{d['since_s']:g} s): {name} moves toward the world and would run unseen", dark_hint(ex))

    def dark_pointer(name, kwargs, result):
        """A dark look, or a refusal for darkness (a look-dependent call on a dark look, a move toward the world
        after one), points at what to do: wait_for_light while its bound lasts, else the cannot claim (a key is
        added, none removed). The light block is read from a frame when no reading exists yet."""
        if name == "wait_for_light" or not isinstance(result, dict) or "wait" in result:
            return None
        sync_looks(ex)
        ref, inner = result.get("refused"), result.get("look")
        dark = (isinstance(ref, dict) and ref.get("check") in DARK_CHECKS) or \
            (result.get("can_see") == "dark" and result.get("look_id") is not None) or \
            (isinstance(inner, dict) and inner.get("can_see") == "dark")      # a look another package attached
        if not dark:
            return None
        if L["last"] is None:
            fresh(ex)
        d = doc(ex)
        light = {"can_see": d["can_see"], "brightness": d["brightness"], "since_s": d["since_s"],
                 "waited_s": d.get("waited_s"), "wait_left_s": d.get("wait_left_s")}
        if spent(ex):
            return {**result, "wait": {"tool": "claim", "args": {"success": False,
                                                                 "cannot": {"reason": "dark", "evidence": []}},
                                       "why": dark_hint(ex), "light": light}}
        return {**result, "wait": {"tool": "wait_for_light", "args": {"max_s": offer(ex)},
                                   "why": "it is dark: moves toward the world are refused while the light reads "
                                          "dark; " + dark_hint(ex),
                                   "light": light}}

    ex.add_idle(on_idle)
    ex.on_event("look", on_look)
    ex.add_pre(dark_gate)
    ex.add_post(dark_pointer)
