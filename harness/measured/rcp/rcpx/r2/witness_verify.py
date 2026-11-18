# SPDX-License-Identifier: AGPL-3.0-only
"""verify gains same_view (the witness package, SPEC-R3.md 4.4).

FILTER wraps the merged verify handler (the device package's evidence-checked verify), so it runs first and this
reads its answer. Without the device package, a post hook on the round one verify primitive does the same.

For a goal state whose deciding part is not a cue (sheet_words.step's done class is not cue: a rocker, a drawer
face, a lid's head), the newest same-view record that spans the last device act (its before look ahead of the act,
its after look after it: two looks both after the act say nothing of what the act did) decides what is added. The
record's item for the part is the one whose label names the part; else (an operator's own label that names no sheet
part) the item whose point lies within max(ACT_NEAR_MIN_MM, ACT_NEAR_SIZE_FRAC x the part's rough size) of
device_verify's own evidence point or of the last device act's point: the claim rests on that point, so what the
same view shows there is evidence for or against it. same_view always carries answer, one of witness_device's
readings, so unclear, unchanged and not judged are never mistaken for each other:
- changed: same_view {record, answer: changed, changed: "yes"} (the verdict stands);
- unchanged (a solid no: 24 px or more across, frames not dim, the light unchanged) with no other change on the
  device: the verdict becomes contradicted, accepted false, and a claim this verify had confirmed is unconfirmed
  again (a check is kept);
- unchanged at the point but the device changed elsewhere (away from every judged point): answer unclear with the
  count and "locate the part again"; no contradiction (the point may not be on the part), the verdict stands;
- device_moved (the newest record that says the device itself moved, surely or possibly, with or without an act
  of the robot's, and no point of the part located again since): the verdict becomes unobservable, accepted
  false, the claim unconfirmed: a point the part has left cannot show its state; such a record is never read
  for changed or unchanged;
- unclear: same_view {record, answer: unclear, why} and the verdict stands;
- no such record: same_view {checked: false, answer: not_judged, hint: "look like=Lk then compare"}, Lk the newest
  look taken before the last act from which the deciding part's located point was in view (else any look before
  it) and whose pose the gates would let the camera go back to now.
A cue part's state is read by device_verify's own cue reading; nothing changes for it here.
"""

from rcpx import r3_limits as L
from rcpx import sheet_words
from rcpx.r2 import witness_device as wd
from rcpx.r2 import witness_pose as pose

PKG = "witness"
T_EPS = 1e-6                # sim seconds: a look at the very time an act starts was taken before it
HINT_TRIES = 6              # looks asked (dry) for a way back before the hint names none


def _st(ex):
    return ex.state.get(PKG) or {"records": [], "acts": []}


def _deciding(ex, device, state):
    doc = (getattr(ex, "context", None) or {}).get("devices")
    if not isinstance(doc, dict) or not state:
        return None
    try:
        step = sheet_words.step(str(state).strip().lower(), doc, device)
    except Exception:          # a sheet this reader cannot parse decides nothing here
        return None
    if step.get("done_class") == "cue" or not step.get("deciding"):
        return None
    part = next((p for p in sheet_words.parts(doc) if p["name"] == step["deciding"]
                 and (device is None or p["device"] == device)), None)
    if part is None or part["role"] == "cue":
        return None
    return part


def _like_hint(ex, part, t_act):
    """The newest look taken before t_act from which the part's located point was in view (any look before t_act
    when no point of the part is located) AND whose pose the camera can go back to now: the gates an operator's
    own move would meet are asked dry (a look taken with the tool on the device is no way back). None when no
    such look passes (of the HINT_TRIES newest)."""
    plist = sheet_words.parts((ex.context or {}).get("devices"))
    pts = [e["point_mm"] for e in ex.world.query(kind="located", include_stale=False)
           if e.get("point_mm") is not None and (sheet_words.label_part(e["label"], plist) or {}).get("name")
           == part["name"]]
    cands = []
    for k, r in ex._looks.items():
        t = r.get("t")
        if t is None or float(t) > t_act or r.get("can_see") == "dark":
            continue
        if pts:
            pp = pose.point_px(ex.model, r["q"], pts[-1])
            if pp is None or not pose.in_frame(ex.model, pp[0], pp[1]):
                continue
        cands.append(k)
    at_point = getattr(pose, "_goal_at_point", None)    # go_to's own refusal: the tool at a stored point
    for k in reversed(cands[-HINT_TRIES:]):
        q = ex._looks[k]["q"]
        try:
            if (at_point is None or at_point(ex, q) is None) and pose._gate(ex, q) is None:
                return k
        except Exception:      # a gate that cannot be asked names no look
            continue
    return None


def _points(ex, out, acts):
    """Where the claim's evidence is: device_verify's own evidence point and the last device act's point."""
    pts = [(out.get("evidence_checked") or {}).get("point_mm")]
    if acts and acts[-1].get("point_mm") is not None:
        pts.append(acts[-1]["point_mm"])
    return [p for p in pts if p is not None]


def _device_moved(ex, part, st):
    """The newest record that says the device itself moved (surely or possibly), unless a point of the part was
    located again after that record's after look: until then every stored point of the device is where its part
    WAS, whoever moved it (no act of the robot's is needed for a device to be moved)."""
    rec = next((r for r in reversed(st.get("records") or []) if r.get("device_moved")), None)
    if rec is None:
        return None
    plist = sheet_words.parts((ex.context or {}).get("devices"))
    for e in ex.world.query(kind="located", include_stale=False):
        at = e.get("observed_at") if isinstance(e.get("observed_at"), dict) else {}
        later = (at.get("call") is not None and int(at["call"]) > int(rec.get("seq", 0))) or (
            at.get("t_s") is not None and float(at["t_s"]) > float(rec.get("after_t", 0.0)))
        if e.get("point_mm") is not None and later \
                and (sheet_words.label_part(e["label"], plist) or {}).get("name") == part["name"]:
            return None
    return rec


def _unconfirm(ex, out, rec, why, verdict, sense="same_view"):
    """The claim this verify answered gets a failed check and is unconfirmed again; the answer says so."""
    cid = out.get("claim_id")
    e = ex.world.entries.get(str(cid)) if cid is not None else None
    if e is not None and e.get("kind") == "state":
        e["checks"].append({"sense": sense, "ref": rec["id"], "at_call": int(getattr(ex, "_calls", 0)),
                            "accepted": False, "why": why})
        if e.get("confirmed"):
            e["confirmed"], e["mark"], e["confirmed_by"] = False, "unconfirmed", None
        ex._journal("witness_verify", claim_id=e["id"], record=rec["id"], verdict=verdict, why=why)
    out.update(verdict=verdict, accepted=False, confirmed=False, why=why, mark="unconfirmed")
    return out


def same_view_check(ex, args, out):
    """verify's answer with evidence_checked.same_view added (module text); out unchanged when it does not apply."""
    if not isinstance(out, dict) or "refused" in out:
        return out
    args = args or {}
    device = out.get("device") or args.get("device")
    state = out.get("state") or args.get("state")
    if args.get("id") is not None and (device is None or state is None):
        e = ex.world.entries.get(str(args["id"])) or {}
        device, state = e.get("device", device), e.get("state", state)
    part = _deciding(ex, device, state)
    if part is None:
        return out
    st = _st(ex)
    acts = st.get("acts") or []
    t_act = float(acts[-1]["t0"]) if acts else None
    # a record witnesses the last act when its before look came ahead of the act and its after look after it
    recs = [r for r in st.get("records") or [] if t_act is not None and not r.get("device_moved")
            and float(r.get("before_t", 0.0)) <= t_act + T_EPS and t_act < float(r.get("after_t", 0.0))]
    reach = max(L.ACT_NEAR_MIN_MM, L.ACT_NEAR_SIZE_FRAC * (pose.part_size_mm(part) or 0.0))
    points = _points(ex, out, acts)
    hit = None
    for how in ("part", "near"):        # the part by name in any spanning record first, then a point at the evidence
        for r in reversed(recs):
            rd = wd.reading(r, part["name"], points, reach)
            if rd["answer"] != "not_judged" and rd.get("how") == how:
                hit = (r, rd)
                break
        if hit is not None:
            break
    out = dict(out)
    ec = dict(out.get("evidence_checked") or {})
    mv = _device_moved(ex, part, st)
    if mv is not None:
        why = mv["device_moved"].get("why") or "the whole device moved"
        ec["same_view"] = {"record": mv["id"], "answer": "device_moved", "changed": "unclear", "why": why,
                           "before": mv.get("before"), "after": mv.get("after")}
        out["evidence_checked"] = ec
        return _unconfirm(ex, out, mv, f"{why} ({mv['id']}: {mv['before']} against {mv['after']}); the state of "
                          f"the {part['name']} cannot be read at a point it has left", "unobservable")
    if hit is None:
        k = _like_hint(ex, part, t_act) if t_act is not None else None
        ec["same_view"] = {"checked": False, "answer": "not_judged",
                           "hint": f"look like={k} then compare" if k else
                           "look like=Lk (a look taken before the act, from clear of the device) then compare"}
        out["evidence_checked"] = ec
        return out
    r, rd = hit
    ans = rd["answer"]
    ec["same_view"] = {"record": r["id"], "answer": ans,
                       "changed": {"changed": "yes", "unchanged": "no"}.get(ans, "unclear"),
                       "at": rd.get("at"), "by": rd.get("how"), "before": r.get("before"), "after": r.get("after")}
    out["evidence_checked"] = ec
    where = part["name"] if rd.get("how") == "part" else f"{rd.get('at')} (your point at the {part['name']})"
    if ans == "unchanged" and rd.get("elsewhere"):
        ec["same_view"].update(answer="unclear", changed="unclear", elsewhere=rd["elsewhere"],
                               why=f"no change at {where}, but the device changed elsewhere "
                                   f"({rd['elsewhere']} place{'s' if rd['elsewhere'] > 1 else ''}): the point may "
                                   "not be on the part, locate the part again")
        return out
    if ans == "unchanged":
        return _unconfirm(ex, out, r, f"the same view before and after shows no change at the {where} ({r['id']}: "
                          f"{r['before']} against {r['after']})", "contradicted")
    if ans == "unclear":
        ec["same_view"]["why"] = rd.get("why")
    return out


def wrap(ex, inner):
    def verify(**args):
        return same_view_check(ex, args, inner(**args))
    verify.__wrapped_by_witness__ = True
    return verify


def filter_tables(surface, resources, handlers, ex):
    """FILTER: the merged verify handler wrapped (the device package's runs first). No handler: nothing here (the
    post hook on the verify primitive covers the round one verify)."""
    if ex is None or not callable(handlers.get("verify")):
        return surface, resources, handlers
    handlers = dict(handlers)
    handlers["verify"] = wrap(ex, handlers["verify"])
    return surface, resources, handlers


def post_verify(ex, name, kwargs, result):
    """The round one verify primitive (served when no package handler serves verify)."""
    if getattr(ex, "_act_name", None) != "verify":
        return None
    out = same_view_check(ex, kwargs, result)
    return None if out is result else out
