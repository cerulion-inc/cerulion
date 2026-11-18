# SPDX-License-Identifier: AGPL-3.0-only
"""What backs a push's or a lift's words (a result never
says changed or reached the end when what the robot sensed and saw does not show it). The same view before and
after (the witness package, read lazily from ex.state["witness"]), the look between ramp legs, whether the push
line is known square to the face, and the stop word those back. Nothing here names a robot or a device or reads
truth.

    witness_q, witness_look, witness_compare, at_list    P1 and P7
    StepWatch(ex, run, info, u).changed()                True, False or None (no look to judge by)
    line_trust(fit, source)                              (trusted, why not)
    backed(stopped, fields, record, step_seen, fit, source, felt)   the stop word the evidence backs
"""

import numpy as np

from rcpx import r3_limits as L
from rcpx import sheet_words, side_words
from rcpx.r2 import act_engine as E
from rcpx.r2 import act_guard as G
from rcpx.r2.act_engine import Stop

# A felt normal leans toward the touch line under friction. The review
# measured this, so it cannot certify a naming ray or radial line as square.
# Stored geometry or a converged surrounding face fit must back that claim.
TRUSTED_LINES = ("way", "normal", "centre")


def witness_q(ex, info, B=None):
    """Use the naming camera pose when the part spans enough pixels; otherwise ask sight for the distance from its focal length and the part size."""
    B = L.bounds_of(ex) if B is None else B
    q_n, P = info["q_naming"], info["P"]
    if q_n is None:
        return None
    big = E.largest_size(info["part"])
    if big is not None and P is not None:
        u, v, depth = ex.model.project(q_n, P / E.MM)
        if depth > 0 and big * ex.model.f_px / (depth * E.MM) >= L.WITNESS_MIN_PX:
            return q_n
    plan = ((getattr(ex, "state", None) or {}).get("sight") or {}).get("plan_camera")
    if callable(plan) and P is not None and info["origin"] is not None and big is not None:
        try:
            side = side_words.side_of(info["origin"] - P, P)
            got = plan(list(P), side if side in side_words.SIDES else "robot_side", L.witness_plan_mm(B, f_px=ex.model.f_px, size_mm=big))
            if isinstance(got, dict) and got.get("q") is not None:
                return np.asarray(got["q"], dtype=float)
        except Exception:
            pass
    return q_n


def witness_look(run, q_w, note):
    """Take the witness look and retain its runtime phase record; a dark result stops the tool."""
    wit = (getattr(run.ex, "state", None) or {}).get("witness") or {}
    fn = wit.get("go_and_look")
    if not callable(fn) or q_w is None:
        return None
    run.check_time("witness")
    t1 = run.ex.sim_time()
    try:
        out, activity = motion_call(run.ex, run.B, "go_and_look", {"note": note},
                                    fn=lambda **kw: fn(q_w, **kw))
    except Exception as exc:
        run.notes.append(f"the witness look failed: {type(exc).__name__}")
        return None
    rec = {"by": "runtime", "tool": run.tool, "stage": "witness", "primitive": "go_and_look", "t0": round(t1, 3),
           "t1": round(run.ex.sim_time(), 3), "phase": "witness", "did": "go_and_look",
           **activity, "bounds": run.items[0] if run.items else None}
    run.legs.append(rec)
    run.ex.note_event("r3_leg", **rec)
    if isinstance(out, dict) and (out.get("refused") or {}).get("check") == "dark":
        raise Stop("dark", stage="witness")
    return out if isinstance(out, dict) and out.get("look_id") else None


def witness_compare(ex, before, after, at):
    """Compare the same-view pair when the witness package and both looks are present."""
    wit = (getattr(ex, "state", None) or {}).get("witness") or {}
    fn = wit.get("compare")
    if not callable(fn) or before is None or after is None:
        return None
    try:
        return fn(before.get("look_id"), after.get("look_id"), at=at)
    except Exception:
        return None


def at_list(ex, info):
    """
    compare's at: the target, then every fresh located entry that names a cue part or a part of the
    target's own
    device (a part that springs back shows its change at another part of
    the device).
    """
    at = [info["id"]]
    parts = G.plist(ex)
    for e in ex.world.query(kind="located", include_stale=False):
        p = sheet_words.label_part(e["label"], parts)
        if p is None or e["id"] in at or e.get("point_mm") is None:
            continue
        if (p["role"] == "cue" and (info["device"] is None or p["device"] == info["device"])) or \
                (info["device"] is not None and p["device"] == info["device"]):
            at.append(e["id"])
    return at


def line_trust(fit, source):
    """Whether the line is square from stored geometry or a converged face fit, with the sensed fit-angle error as its tolerance."""
    fit = fit or {}
    turns = fit.get("turns") or []
    if fit.get("diverged"):
        return False, "the face fit did not settle"
    if fit.get("near_an_edge"):
        return False, "the touches around found no one face"
    if turns:
        last = float(turns[-1].get("off_deg") or 0.0)
        return (True, None) if last <= float(fit.get("done_deg") or 0) else (False, f"the last fit read the face {last:g} deg off")
    if source in TRUSTED_LINES:
        return True, None
    return False, "the face was not fitted"


def backed(stopped, fields, record, step_seen, fit, source, felt):
    """
    The stop word the evidence backs (a result never says changed or reached the end
    when
    what the robot sensed and saw does not show it).
    - changed stands with a picture (the same-view
    pair reads yes at the part or at a located part of its device,
      or the look between ramp legs did),
    or with a give the press itself detected on a line known square to
      the face (line_trust: square
    on, the finger cannot slide along the face, which is what read as a change
      without one). A move
    that only stayed in, or a give on a line not known square, is unconfirmed: the said
      gives what was
    felt and that no picture shows a change, and the next is a look, never the claim fork.
    - reached_end
    stands only on a line known square to the face (line_trust): a push off square can bind a
      sliding
    part short of its end and the stop feels the same. Otherwise stalled.
    """
    fields = dict(fields)
    if stopped == "changed":
        yes = [a for a in ((record or {}).get("at") or []) if a.get("changed") == "yes"]
        if yes or step_seen:
            return stopped, fields
        last = felt[-1] if felt else {}
        if last.get("word") == "gave" and line_trust(fit, source)[0]:
            return stopped, fields
        fields["how"] = "gave" if last.get("word") == "gave" else "stayed in"
        return "unconfirmed", fields
    if stopped == "reached_end":
        ok, why = line_trust(fit, source)
        if ok:
            return stopped, fields
        fields["why"] = why
        return "stalled", fields
    return stopped, fields


class StepWatch:

    """Compare the same wrist view between legs after backing off by the tool width; a returning part can show its state only there."""
    def __init__(self, ex, run, info, u):
        """Initialize this stage record from its caller."""
        self.ex, self.run, self.u, self.L0, self.tip0, self.ids = ex, run, u, None, None, []
        wit = (getattr(ex, "state", None) or {}).get("witness") or {}
        if not callable(wit.get("compare")):
            return
        q = ex._sense().q
        for eid in at_list(ex, info):
            e = ex.world.get(eid)
            if e is None or e.get("point_mm") is None:
                continue
            px, py, depth = ex.model.project(q, np.asarray(e["point_mm"], dtype=float) / E.MM)
            if depth > 0 and 0 <= px < ex.model.cam_w and 0 <= py < ex.model.cam_h:
                self.ids.append(eid)
        if not self.ids:
            run.notes.append("no located point in the wrist view at the press pose: no look between steps")
            return
        run.move("step_look", -u * L.step_look_back_mm(run.B))
        lk = run.call("step_look", "look", note="before the first push (same view between steps)")
        if isinstance(lk, dict) and lk.get("look_id") and lk.get("can_see") != "dark":
            self.L0, self.tip0 = lk["look_id"], run.tip()

    def changed(self):
        """
        True: the look from the press pose shows a change at a watched point; False: it was taken and shows
        none; None: no such look (nothing to judge by).
        """
        if self.L0 is None:
            return None
        self.run.move("step_look", self.tip0 - self.run.tip())
        lk = self.run.call("step_look", "look", note="after a push that came back (same view between steps)")
        if not (isinstance(lk, dict) and lk.get("look_id")) or lk.get("can_see") == "dark":
            return None
        rec = ((self.ex.state.get("witness") or {}).get("compare") or (lambda *a, **k: None))(
            self.L0, lk["look_id"], at=list(self.ids))
        if not isinstance(rec, dict) or "refused" in rec or not rec.get("at"):
            return None
        yes = [a for a in ((rec or {}).get("at") or []) if a.get("changed") == "yes"]
        if yes:
            self.run.notes.append(f"the look between steps shows a change at {yes[0].get('label') or yes[0].get('id')}"
                                  f" ({(rec or {}).get('record')})")
        return bool(yes)

# The shared geometry and counters describe what the runtime itself measured.
def tool_reach_mm(ex):
    """The tool bodies' furthest reach from the tip in the loaded collision model, in mm."""
    try:
        s = ex._sense()
        tool = getattr(ex, "_tool_bodies", None) or set()
        tip = E._v(s.tip)
        out = 0.0
        for c in ex.model.collision_capsules(np.asarray(s.q, dtype=float)):
            if c[0] in tool:
                out = max(out, float(np.linalg.norm(E._v(c[1]) - tip)) + float(c[3]),
                          float(np.linalg.norm(E._v(c[2]) - tip)) + float(c[3]))
        return round(out * E.MM, 1)
    except Exception:
        return 0.0


def hand_lands_first(ex, B, P, u, standoff, thing=None):
    """The non-tool body that would precede the finger at this touch target, using the model and its derived clearance."""
    s = ex._sense()
    goal = s.tip + E._v(u) * float(standoff) / E.MM
    try:
        q_end, ok, _ = ex.model.ik(s.q, goal, s.R)
    except Exception:
        return None
    if not ok:
        return None
    p = E._v(P) / E.MM
    tool = getattr(ex, "_tool_bodies", set())
    for name, a, b, rad in ex.model.collision_capsules(np.asarray(q_end, dtype=float))[1:]:
        if name in tool:
            continue
        a, b = E._v(a), E._v(b)
        ab = b - a
        t = 0.0 if float(ab @ ab) == 0 else min(1.0, max(0.0, float((p - a) @ ab) / float(ab @ ab)))
        if float(np.linalg.norm(p - (a + t * ab))) - float(rad) < L.hand_lands_mm(B, thing=thing, body=name) / E.MM:
            return name
    return None


def _period_snapshot(ex):
    """A copy of the executor's native counters, when it offers them."""
    value = getattr(ex, "periods", None)
    value = value() if callable(value) else value
    return dict(value) if isinstance(value, dict) else value


def motion_call(ex, B, name, kwargs, fn=None):
    """Use result periods or executor counter deltas, else label an elapsed-time estimate.
    No path replaces ex._period, including refusals and exceptions. An older core
    cannot split its periods exactly: only idle is known to wait in that fallback.
    """
    before, t0 = _period_snapshot(ex), ex.sim_time()
    result = (getattr(ex, name) if fn is None else fn)(**kwargs)
    seconds = ex.sim_time() - t0
    after = _period_snapshot(ex)
    source, counts = "core result periods", None
    value = result.get("periods") if isinstance(result, dict) else None
    if isinstance(value, dict):
        counts = {k: value.get(k, 0) for k in ("moving", "waiting", "jaw")}
        value = value.get("periods", sum(counts.values()))
    elif isinstance(value, list):
        value = sum(v[-1] if isinstance(v, (tuple, list)) else v for v in value)
    if value is None and after is not None and before is not None:
        source = "executor native counter delta"
        if isinstance(after, dict) and isinstance(before, dict):
            counts = {k: after.get(k, 0) - before.get(k, 0) for k in ("moving", "waiting", "jaw")}
            value = after.get("periods", sum(after.get(k, 0) for k in counts)) - before.get("periods", sum(before.get(k, 0) for k in counts))
        elif isinstance(after, (int, float)) and isinstance(before, (int, float)):
            value = after - before
    if value is None:
        source = "elapsed world seconds / dt; activity split estimated by primitive"
        value = round(seconds / B.dt())
    if counts is None:
        counts = {k: (result.get(k) if isinstance(result, dict) else None) for k in ("moving", "waiting", "jaw")}
        if any(v is None for v in counts.values()):
            counts = {"moving": 0, "waiting": 0, "jaw": 0}
            counts["waiting" if name == "idle" else "moving"] = value
            source += "; activity split estimated by primitive"
    return result, {**counts, "periods": value, "seconds": seconds, "counts_source": source,
                    "free_share": result.get("free_share") if isinstance(result, dict) else None}


def install_press_guard(ex):
    """Install the carry press checks and retain operator press evidence."""
    st = G.state(ex)
    pending = {}

    def pre(name, kwargs):
        """Record this press start and run its preflight check."""
        if name != "press":
            return None
        pending.clear()
        s = ex._sense()
        d = G._direction(ex, kwargs, s)
        if d is not None:
            pending.update(start=np.asarray(s.tip, dtype=float) * E.MM, q=s.q.copy(), d=d, force=kwargs.get("force_n"))
        return G.check(ex, kwargs)

    def post(name, kwargs, result):
        """Store a completed operator press's contact evidence."""
        if name != "press" or not isinstance(result, dict) or "refused" in result or not pending:
            return None
        along = float(result.get("along_push_mm") or 0.0)
        rigid = along < G.rigid_now(ex, pending["q"], pending["d"]) and not result.get("gave")
        try:
            force = float(pending.get("force"))
        except (TypeError, ValueError):
            force = None
        st["presses"].append({"call": ex._calls, "tool": "press", "start_mm": [round(float(x), 1)
                                                                              for x in pending["start"]],
                              "force_n": force, "along_mm": round(along, 2), "gave": bool(result.get("gave")),
                              "rigid": bool(rigid)})
        pending.clear()
        return None

    ex.add_pre(pre, names=["press"])
    ex.add_post(post, names=["press"])
    return ex


__all__ = ["StepWatch", "TRUSTED_LINES", "at_list", "backed", "line_trust",
           "witness_compare", "witness_look", "witness_q"]
