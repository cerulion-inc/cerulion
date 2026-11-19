# SPDX-License-Identifier: AGPL-3.0-only
"""Package witness: the same view before and after.

A change of viewpoint is not a change of state. Weak operators took a top view for a front view and a new camera
pose for a moved part; this package compares two looks only when the camera stood in the same place.

- compare(before, after, at=None, detail="brief"), free (its entry is marked free), computes only: the two looks
  must be the same view by FK at their stored (measured) joint poses (the same-view tolerance r3_limits gives at
  the arm's own resolution), else refused not_same_view with the numbers; a
  dark look is refused dark; an unknown look unknown_look. Then witness_diff lines the frames up (ECC), reads the
  light's gain (an even gain with both looks ok is the camera's own exposure and is evened out; any other gain
  change over 15 percent is light_changed: only a compact colour change counts, never no), finds the changes
  (colour names raw, no floor gains) and judges each target: changed yes, no (a "solid" no: a part 24 px or more
  across in frames that are not dim by the camera's own noise, rcpx.perception.dim_level) or unclear. at: a located
  or contact id, a part label, a survey candidate id,
  {"px", "radius_px"} or {"box_px"}; default every located device point in view. Every compare is a record
  (ex.state["witness"]["records"], ids SV1, SV2, ...). A record and a result carry reading, one word a consumer
  can act on (witness_device: changed, unchanged, unclear, device_moved; a refused compare is not_judged), and
  device_moved {answer: yes | possible, ...} when the device itself moved between the looks: every target on it
  then reads unclear and no claim is offered. elsewhere counts the changes on the device at no judged point.
- look gains like (a look id) and, with it, clear (the operator's word that the camera's way is clear: true or
  {"look": id}): the pre hook pops them, gates the move, puts the camera back at that look's pose by
  the runtime's own look move at the session's derived speed, with correctives (witness_pose.go_to), settles, and
  the look runs; its post hook adds compared_with (the compare with that look, at the device points acted on
  since it and the located points in view) and the move's control periods and binding bound, at no extra call.
- the automatic compare: every look (a look call, a standard look, a look_after look) whose pose is the same view
  as a look taken before the last device act gets compared_with against the newest such look.
- verify: witness_verify's FILTER wraps the merged verify handler (the device package's runs first).
- ex.state["witness"]: go_and_look(q, note="", intent=None) (for act), compare(before, after, at=None),
  same_view(q_a, q_b),
  reading(record or id, part=None, points=(), reach_mm=None) (witness_device.reading), can_return(look) (dry:
  would look like=look be refused now), records, acts ({seq, name, t0, t1, point_mm, contact_id}).

A device act (near the finish package's definition, kept here so neither imports the other): an operator call
whose result has a contact or gave true, or stopped in gave, force, contact, part_hit; moved_part too, except
from a plain move (move_tip, move_joints, park) that met no contact (a free-air move that reports moved_part is
no act); a retreat never; or a push or lift whose felt list is not empty. Nothing here reads ground truth or names a robot or a device. Record ids are
SV (same view), not the spec's W: the device package's watch already numbers its watches W1, W2, ...
"""

import json
import math
import os

import numpy as np
from PIL import Image

from rcpx import colour_words, sheet_words
from rcpx import r3_limits as L
from rcpx import surfaces as S
from rcpx.context import refusal
from rcpx.control_base import primitive
from rcpx.r2 import witness_bounds as WB
from rcpx.r2 import witness_device as wd
from rcpx.r2 import witness_diff as diff
from rcpx.r2 import witness_pose as pose
from rcpx.r2 import witness_verify as wv

ID = "I35"
PKG = "witness"
MM = 1000.0
ACT_STOPS = ("gave", "force", "contact", "moved_part", "part_hit")
R3_VERBS = ("push", "lift")
LIGHT_KINDS = ("lit", "went_dark", "colour_changed")   # a change of light or colour in place: no moved_mm
FREE_MOVES = ("move_tip", "move_joints", "park")   # their moved_part with no contact is the arm's own load
LOOK_NAMES = ("look", "standard_look")
AT_KEYS = ("id", "label", "part", "changed", "solid", "share", "ring_share", "far_share", "colour_before",
           "colour_after", "glowing", "moved_mm", "moved_px", "why", "point_mm", "kind")   # a record's at items
SAID_PARTS_MAX = 3          # targets named in said (changed ones first)
NOT_IN_VIEW_MAX = 6         # out-of-view points named in a brief result (the rest counted)
T_EPS = 1e-6                # sim seconds: a look at the very time an act starts was taken before it
UNKNOWN_HINT = "give two look ids a look returned, for example L4 and L7"

_AT_ITEM = {"anyOf": [{"type": "string"},
                      S._obj({"px": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2},
                              "radius_px": S._num(1, 320, "radius of the disc to judge, pixels")}, ["px"]),
                      S._obj({"box_px": {"type": "array", "items": {"type": "number"}, "minItems": 4,
                                         "maxItems": 4}}, ["box_px"])],
            "description": "what to judge: a located or contact id, a part name, a survey candidate id, "
                           "{\"px\": [u, v], \"radius_px\": r} or {\"box_px\": [u0, v0, u1, v1]}"}
_AT = {"anyOf": _AT_ITEM["anyOf"] + [{"type": "array", "items": _AT_ITEM, "minItems": 1, "maxItems": 8}],
       "description": _AT_ITEM["description"] + "; or a list of them; leave it out for every located device point "
                                                "in view"}

TOOLS = [
    S._tool("compare",
            "Free. Says what changed between two pictures taken from the same place, part by part (CHANGED, no "
            "change, or unclear; dim frames and small parts never read no; look like=BEFORE first puts the camera "
            "back). Example: compare {\"before\": \"L4\", \"after\": \"L7\"}.",
            S._obj({"before": S._ID, "after": S._ID, "at": _AT,
                    "detail": S._str("brief (default) or full (adds the alignment, light gain, threshold and "
                                     "every change)", ["brief", "full"])}, ["before", "after"]),
            {"before": "L4", "after": "L7"}, "what changed between L4 and L7 at every located device point in view",
            free=True),
    {"extends": "look", "args": {"like": {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}],
                                          "description": "a look id: put the camera back where it was for that "
                                                         "look first"}, "clear": WB.CLEAR_ARG},
     "note": "With like (a look id) it puts the camera back where it was for that look first and the answer gains "
             "compared_with: what changed since that look, part by part."},
]

PRACTICES = ("To know a state, compare two looks from the same place: look like=Lk puts the camera back; a change "
             "of viewpoint is not a change of state.")


def state(ex):
    st = ex.state.get(PKG)
    if st is None:
        st = ex.state[PKG] = {"records": [], "acts": [], "n": 0, "call_t0": None, "pending": None,
                              "candidates": [], "auto": 0}
    return st


# ---------------------------------------------------------------- reading looks
def _frame(ex, rec):
    img = rec.get("img")
    if img is not None:
        return diff.rgb(img)
    fr = rec.get("frame")
    if isinstance(fr, str) and os.path.exists(fr):
        return np.asarray(Image.open(fr).convert("RGB"))
    if isinstance(fr, np.ndarray):
        return diff.rgb(fr)
    return None


# ---------------------------------------------------------------- compare
def _kind_words(c):
    k = c.get("kind")
    if k == "moved":
        return "it moved"
    if k in ("lit", "went_dark", "colour_changed"):
        g = ", glowing" if c.get("glowing") else ""
        return f"a patch went from {c['colour_before']} to {c['colour_after']}{g}"
    return {"appeared": "something appeared there", "gone": "something went away"}.get(k, "it changed")


def _overlap(box, spec):
    if "disc" in spec:
        u, v, r = spec["disc"]
        return box[0] - r <= u <= box[2] + r and box[1] - r <= v <= box[3] + r
    b = spec["box"]
    return not (box[2] < b[0] or box[0] > b[2] or box[3] < b[1] or box[1] > b[3])


def _blob_spec(t, described, lab, comps):
    """For a point whose part has no size on the sheet: the bounding box of the compact change that holds the point and is
    larger than the point's region (a lamp seen close, wider than the 12 px floor), else None."""
    if t.get("part_px") is not None or "disc" not in t["spec"]:
        return None
    u, v, r = t["spec"]["disc"]
    h, w = lab.shape
    iu, iv = int(round(u)), int(round(v))
    if not (0 <= iu < w and 0 <= iv < h) or lab[iv, iu] == 0:
        return None
    for c, d in zip(comps, described):
        b = c["box"]
        if c["label"] == lab[iv, iu] and d["compact"] and min(b[2] - b[0], b[3] - b[1]) > 2 * r:
            return {"box": b}
    return None


def _judge_target(t, ia, ib, valid, count, m, thr, dim, light, aligned, dev, described, shape, f_px, gains,
                  lab=None, comps=(), cam=None):
    blob = _blob_spec(t, described, lab, comps) if lab is not None and aligned else None
    reg, ring = diff.region(shape, blob or t["spec"])
    if not aligned:
        j = {"changed": "unclear", "solid": False, "share": None, "ring_share": None,
             "why": "could not line the frames up"}
    else:
        j = diff.judge(reg, ring, count, valid, m, t.get("part_px"), light, dim, cam=cam)
    if dev is not None and _overlap(dev["box"], t["spec"]):     # on a device that moved nothing reads yes or no
        j = {**j, "changed": "unclear", "solid": False,
             "why": "the whole device moved" if dev["answer"] == "yes" else "the whole device may have moved"}
    item = {"id": t.get("id"), "label": t.get("label"), "part": t.get("part"), "point_mm": t.get("point_mm"), **j}
    if blob is not None:
        item["region"] = "the change around the point"
    over = [c for c in described if _overlap(c["box_px"], t["spec"])]
    if over:
        c = over[0]
        item.update(colour_before=c["colour_before"], colour_after=c["colour_after"], glowing=c["glowing"],
                    kind=c["kind"])
    else:
        rv = reg & valid
        if rv.any():
            item.update(colour_before=colour_words.name(ia[rv].reshape(-1, 1, 3), gains[0]),
                        colour_after=colour_words.name(ib[rv].reshape(-1, 1, 3), gains[1]),
                        glowing=bool(colour_words.glowing(ib[rv].reshape(-1, 1, 3), ib)))
    if aligned and item["changed"] == "yes" and item.get("kind") not in LIGHT_KINDS:   # a light is not a move
        off = diff.moved_shift(ia, ib, count & reg & valid, thr)
        if off is not None and off[2] >= diff.MOVED_EXPLAIN_FRAC:
            item["moved_px"] = [round(off[0], 1), round(off[1], 1)]
            if t.get("depth_mm"):
                item["moved_mm"] = round(math.hypot(off[0], off[1]) * t["depth_mm"] / f_px, 1)
    return item


def _said(sv, items, extra, light, dev, aligned, not_in_view, exposure=None):
    head = f"same view ({sv['mm']:g} mm, {sv['deg']:g} deg apart)"
    if not aligned:
        return head + ": could not line the frames up, so every part reads unclear"
    bits = []
    order = sorted(items, key=lambda i: {"yes": 0, "no": 1}.get(i["changed"], 2))
    for it in order[:SAID_PARTS_MAX]:
        if it["changed"] == "yes":
            mv = f" {it['moved_mm']:g} mm" if it.get("moved_mm") else ""
            words = _kind_words(it) + (mv if it.get("kind") == "moved" else "")
            bits.append(f"at the {it['label']} CHANGED, {words}")
        elif it["changed"] == "no":
            bits.append(f"at the {it['label']} no change")
        else:
            bits.append(f"at the {it['label']} unclear ({it.get('why') or 'not decided'})")
    if light:
        bits.insert(0, "the light changed, so only a coloured light can count")
    elif exposure:
        bits.insert(0, f"the camera's exposure changed (x{exposure:g}) and was evened out")
    if dev is not None:
        how = "moved" if dev["answer"] == "yes" else "may have moved"
        bits = [f"the whole device {how}" + (f" {dev['mm']:g} mm" if dev.get("mm") else "")
                + ", so its parts read unclear: locate the part again"] + [b for b in bits if "unclear" not in b]
    if extra:
        bits.append(f"{extra} other change{'s' if extra > 1 else ''} elsewhere")
    elif items:
        bits.append("nothing else changed")
    elif not bits:
        bits.append("no change seen in the frame, and no part was judged" if not not_in_view else
                    "no located point is in view")
    text = head + ": " + "; ".join(bits)
    words = text.split()
    return " ".join(words[:L.SAID_MAX_WORDS]) if len(words) > L.SAID_MAX_WORDS else text


def _save_pic(ex, rec_after, ka, kb, rid, img):
    fr = rec_after.get("frame")
    d = ex._frames_dir() or (os.path.dirname(fr) if isinstance(fr, str) else None)   # the session's own folder
    if not d:
        return None
    path = os.path.join(d, f"{rid}_{kb}_{ka}.png")
    Image.fromarray(img).save(path)
    return path


def compare_looks(ex, before, after, at=None, detail="brief", origin="tool"):
    """The compare (module text): a result dict, or a refusal. Appends a record on success."""
    st = state(ex)
    kb, rb = ex._look_rec(before)
    ka, ra = ex._look_rec(after)
    for k, r in ((kb, rb), (ka, ra)):
        if r is None:
            return refusal("unknown_look", f"no look {str(k)[:20]!r} in this session", UNKNOWN_HINT)
    for k, r in ((kb, rb), (ka, ra)):
        if r.get("can_see") == "dark":
            return refusal("dark", f"look {k} is dark (brightness {float(r.get('brightness') or 0):.1f} of 255): "
                           "a dark frame shows no change", "compare two looks that can see")
    model = ex.model
    sv = pose.same_view(model, rb["q"], ra["q"], pose.tolerance(ex, rb["q"]))
    if not sv["same"]:
        return refusal("not_same_view", f"{kb} and {ka} were taken {sv['mm']:g} mm and {sv['deg']:g} deg apart "
                       f"(roll {sv['roll_deg']:g} deg; the same view is within {sv['tol']['mm']:g} mm, "
                       f"{sv['tol']['deg']:g} deg): a change of viewpoint is not a change of state",
                       f"look like={kb} puts the camera back at that look's pose; then compare {kb} with it")
    ia, ib0 = _frame(ex, rb), _frame(ex, ra)
    if ia is None or ib0 is None:
        return refusal("frame", "a look's frame was not kept as an image", "take the looks again and compare those")
    tg = pose.targets(ex, at, rb["q"])
    if isinstance(tg, dict):
        return tg
    pu = WB.inputs(ex)
    margin = pose.OWN_MARGIN_PX + float(model.f_px) * math.radians(pu["pu_deg"])
    own = pose.own_mask(model, rb["q"], margin) | pose.own_mask(model, ra["q"], margin)
    al = diff.align(ia, ib0, ~own)
    ib, cover = diff.warp(ib0, al) if al["ok"] else (diff.rgb(ib0), np.ones(own.shape, dtype=bool))
    valid = ~own & cover
    g = diff.gain(ia, ib, valid)
    spread = diff.gain_spread(ia, ib, valid, g)
    exposure = g is not None and abs(g - 1.0) > L.LIGHT_CHANGED_FRAC and spread is not None \
        and spread <= diff.GAIN_UNIFORM_IQR and rb.get("can_see") == "ok" and ra.get("can_see") == "ok"
    light = g is not None and abs(g - 1.0) > L.LIGHT_CHANGED_FRAC and not exposure
    ibn = diff.scaled(ib, None if not g else 1.0 / g)
    t, m = diff.threshold(ia, ibn, valid)
    shape = ia.shape
    changed = diff.change_mask(ia, ibn, t, valid) if al["ok"] else np.zeros(own.shape, dtype=bool)
    comps, lab = diff.components(changed)
    gains = (None, None)        # raw colour names: grey-world gains off a coloured floor turn a grey lamp blue
    described = [diff.describe(c, lab, ia, ibn, gains[0], gains[1], shape, t, find_move=i < L.CHANGES_MAX)
                 for i, c in enumerate(comps[:diff.DESCRIBE_MAX])]
    count = changed
    if light:
        keep = [c for c, d in zip(comps, described) if d["compact"] and d["colour_before"] != d["colour_after"]
                and not colour_words.near(d["colour_before"], d["colour_after"])]
        count = np.isin(lab, [c["label"] for c in keep]) if keep else np.zeros(own.shape, dtype=bool)
    dim = "dim" in (rb.get("can_see"), ra.get("can_see"))
    dev, dev_check = None, None
    seen = [x for x in tg if "spec" in x]
    dbox = pose.device_box(ex, rb["q"], float(ra.get("t") or 0.0),
                       [x["point_mm"] for x in seen if x.get("point_mm") is not None])
    if dbox is not None and al["ok"]:
        off = diff.device_offset(ia, ibn, [int(x) for x in dbox[0]], dbox[2])
        if off is not None:
            dev_check = {"px": [round(off[0], 1), round(off[1], 1)], "ncc": round(off[2], 3), "turn_deg": off[3],
                         "stayed_ncc": round(off[4], 3)}
        dev = wd.moved(off, dbox[0], dbox[1], float(model.f_px), sv["mm"])
    not_in_view = [x.get("id") or x.get("label") for x in tg if "spec" not in x]
    items = [_judge_target(x, ia, ibn, valid, count, m, t, dim, light, al["ok"], dev, described, shape,
                           float(model.f_px), gains, lab, comps[:diff.DESCRIBE_MAX], cam=WB.camera(ex)) for x in seen]
    extra = sum(1 for d in described if not any(_overlap(d["box_px"], x["spec"]) for x in seen))
    other = wd.elsewhere(described, None if dbox is None else dbox[0], [x["spec"] for x in seen], _overlap)
    st["n"] += 1
    rid = f"SV{st['n']}"
    acts = [a for a in st["acts"] if a["t0"] < float(ra.get("t") or 0.0)]
    rec = {"id": rid, "seq": int(getattr(ex, "_calls", 0)), "before": kb, "after": ka,
           "before_t": float(rb.get("t") or 0.0), "after_t": float(ra.get("t") or 0.0),
           "same_view": {k: sv[k] for k in ("mm", "deg", "roll_deg")},
           "at": [{k: it.get(k) for k in AT_KEYS} for it in items],
           "changes": described, "light_changed": bool(light),
           "device_moved": None if dev is None else {k: dev[k] for k in ("answer", "px", "mm", "turn_deg", "ncc",
                                                                         "stayed_ncc", "why")},
           "aligned": bool(al["ok"]), "elsewhere": other, "exposure_gain": round(g, 3) if exposure else None,
           "after_act_seq": acts[-1]["seq"] if acts else None, "origin": origin}
    rec["reading"] = wd.record_reading(rec)
    st["records"].append(rec)
    marks = [x["spec"] for x in seen]
    pic = _save_pic(ex, ra, ka, kb, rid, diff.triptych(ia, ibn, count, marks))
    said = _said(sv, items, extra, light, dev, al["ok"], not_in_view, rec["exposure_gain"])
    out = {"said": said, "same_view": {"mm": sv["mm"], "deg": sv["deg"]},
           "at": [_brief_item(it) for it in items], "changes": [_brief_change(d) for d in described[:L.CHANGES_MAX]],
           "light_changed": bool(light), "device_moved": rec["device_moved"] and
           {k: rec["device_moved"][k] for k in ("answer", "mm", "turn_deg")}, "reading": rec["reading"],
           "not_in_view": not_in_view, "record": rid, "pictures": [pic] if pic else [],
           "next": wd.fork(ex, items, rid, dev)}
    if detail == "full":
        out["full"] = {"align": {k: al.get(k) for k in ("ok", "shift_px", "turn_deg")},
                       "gain": None if g is None else round(g, 3), "gain_spread": spread, "exposure": exposure,
                       "threshold": round(t, 2), "median_grey": round(m, 1),
                       "roll_deg": sv["roll_deg"], "tol": sv["tol"], "at": rec["at"], "changes": described,
                       "device_check": dev_check}
    else:
        out = _fit(out)
    return out


def _brief_item(it):
    keep = ("id", "label", "changed", "colour_before", "colour_after", "glowing", "moved_mm")
    out = {k: it[k] for k in keep if it.get(k) is not None}
    if it["changed"] == "unclear" and it.get("why"):
        out["why"] = it["why"]
    return out


def _brief_change(d):
    keys = ("kind", "box_px", "colour_before", "colour_after", "glowing", "moved_px")
    return {k: d[k] for k in keys if d.get(k) is not None}


def _fit(out):
    """The brief result at most BRIEF_MAX_BYTES besides pictures: changes are dropped first, then the colour words
    of each part, then the parts beyond the said sentence's."""
    def size(o):
        return len(json.dumps({k: v for k, v in o.items() if k != "pictures"}, default=str))
    if len(out["not_in_view"]) > NOT_IN_VIEW_MAX:
        more = len(out["not_in_view"]) - NOT_IN_VIEW_MAX
        out["not_in_view"] = out["not_in_view"][:NOT_IN_VIEW_MAX] + [f"and {more} more"]
    while size(out) > L.BRIEF_MAX_BYTES and out["changes"]:
        out["changes"] = out["changes"][:-1]
    if size(out) > L.BRIEF_MAX_BYTES:
        out["at"] = [{k: v for k, v in it.items() if k in ("id", "label", "changed", "why")} for it in out["at"]]
    while size(out) > L.BRIEF_MAX_BYTES and len(out["at"]) > 1:
        out["at"] = out["at"][:-1]
        out["at_cut"] = True
    if size(out) > L.BRIEF_MAX_BYTES and isinstance(out.get("next"), dict):
        out["next"] = {**out["next"], "if_seen_shows": str(out["next"]["if_seen_shows"])[:120]}
    return out


# ---------------------------------------------------------------- device acts and the hooks
def is_device_act(name, result):
    if not isinstance(result, dict) or "refused" in result:
        return False
    if name == "retreat":                       # leaving a device is not an act on it
        return False
    c = result.get("contact")
    touched = c is True or (isinstance(c, dict) and bool(c))
    if touched or result.get("gave") is True:
        return True
    stop = result.get("stopped")
    if stop in ACT_STOPS and stop != "moved_part":
        return True
    if result.get("moved_part") is True or stop == "moved_part":
        return name not in FREE_MOVES           # a move through the air with no contact moved no part
    return name in R3_VERBS and bool(result.get("felt"))


def _act_point(result):
    c = result.get("contact")
    if isinstance(c, dict) and c.get("point_mm") is not None:
        return [float(x) for x in c["point_mm"]], c.get("id")
    if result.get("point_mm") is not None:
        return [float(x) for x in result["point_mm"]], result.get("id")
    if result.get("tip_mm") is not None:
        return [float(x) for x in result["tip_mm"]], None
    return None, None


def _operator_pre(ex):
    """True in the pre hooks of an operator's own call (not inside ex.gates or a nested primitive)."""
    return getattr(ex, "_depth", 0) == 0 and getattr(ex, "_hook_depth", 0) == 1


def _hook_at(ex, since_t):
    """The device points acted on since since_t (a located entry within reach of the act's point, else the act's
    own point), then every fresh located point (compare drops those out of view)."""
    plist, out = pose.parts_of(ex), []
    located = [e for e in ex.world.query(kind="located", include_stale=False) if e.get("point_mm") is not None]
    for a in state(ex)["acts"]:
        if a["t0"] < since_t or a.get("point_mm") is None:
            continue
        p = np.asarray(a["point_mm"], dtype=float)
        near = []
        for e in located:
            part = sheet_words.label_part(e["label"], plist)
            reach = max(L.ACT_NEAR_MIN_MM, L.ACT_NEAR_SIZE_FRAC * (pose.part_size_mm(part) or 0.0))
            if float(np.linalg.norm(np.asarray(e["point_mm"], dtype=float) - p)) <= reach:
                near.append(e["id"])
        out.extend(near or ([a["contact_id"]] if a.get("contact_id") in ex.world.entries else
                            [{"point_mm": a["point_mm"], "label": f"point the {a['name']} met"}]))
    named = [e["id"] for e in located if sheet_words.label_part(e["label"], plist) is not None]
    out = named + out + [e["id"] for e in located]     # the sheet's parts first: a brief result cuts from the end
    return out or None


def _compact(cmp):
    if "refused" in cmp:
        return {"said": f"no comparison: {cmp['refused']['why']}", "refused": cmp["refused"]["check"],
                "reading": "not_judged"}
    keep = ("said", "record", "reading", "same_view", "at", "at_cut", "light_changed", "device_moved", "pictures",
            "next")
    return {k: cmp[k] for k in keep if k in cmp}


def _auto(ex, lk, origin):
    """compared_with for a new look whose pose is the same view as a look taken before the last device act."""
    st = state(ex)
    if not st["acts"]:
        return None
    key, rec = ex._look_rec(lk.get("look_id"))
    if rec is None or rec.get("can_see") == "dark":
        return None
    t_act = st["acts"][-1]["t0"]
    tol = pose.tolerance(ex, rec["q"])
    here = pose.cam_pose(ex.model, rec["q"])
    cams = st.setdefault("cams", {})            # a look's camera pose never changes: computed once per look
    best = None
    for k, r in ex._looks.items():
        if k == key or r.get("t") is None or float(r["t"]) > t_act + T_EPS or r.get("can_see") == "dark":
            continue
        if k not in cams:
            cams[k] = pose.cam_pose(ex.model, r["q"])
        n = pose.pose_numbers(cams[k], here)
        if n["mm"] <= tol["mm"] and n["deg"] <= tol["deg"] and abs(n["roll_deg"]) <= tol["roll_deg"]:
            if best is None or float(r["t"]) >= float(ex._looks[best]["t"]):
                best = k
    if best is None:
        return None
    st["auto"] += 1
    cmp = compare_looks(ex, best, key, at=_hook_at(ex, float(ex._looks[best]["t"])), origin=origin)
    out = _compact(cmp)
    out["with"] = best
    return out


def _pre_any(ex, name, kwargs):
    if _operator_pre(ex):
        st = state(ex)
        st["call_t0"] = float(ex.sim_time())
    return None


def _pre_look(ex, name, kwargs):
    if "like" not in kwargs:
        if "clear" in kwargs and not WB.look_takes_clear(ex):
            kwargs.pop("clear")                 # a look that does not move takes no word
        return None
    like = kwargs.pop("like")
    intent, why = WB.clear_word(kwargs.pop("clear", None))
    if why:
        return refusal("args", why, "leave clear out, or say true after a look")
    if not _operator_pre(ex):
        return None
    key, rec = ex._look_rec(like)
    if rec is None:
        return refusal("unknown_look", f"no look {str(like)[:20]!r} in this session", "like takes a look id a look "
                       "returned, for example L4")
    ex._act_name = name                         # the move belongs to this look (the world's last act)
    try:
        ex.world.tick(int(ex._calls) + 1, ex.sim_time())
    except Exception:                           # a world without a clock still gets the move
        pass
    g = pose.go_to(ex, rec["q"], tool="look", intent=intent)
    if "refused" in g:
        return {k: v for k, v in g.items() if k in ("refused", "periods", "bounds")}
    s = pose.settle(ex, tool="look")
    state(ex)["pending"] = {"like": key, "go": g, "settle": s}
    return None


def _post_look(ex, name, kwargs, result):
    st = state(ex)
    pending, st["pending"] = st["pending"], None
    if not isinstance(result, dict) or "refused" in result or result.get("look_id") is None:
        return None
    if getattr(ex, "_act_name", None) != name:
        return None                              # a look the runtime took inside another call (look_after, act)
    out = dict(result)
    if pending is not None:
        g, like = pending["go"], pending["like"]
        out["like"] = {"look": like, "stopped": g["stopped"], "mm": g["view"]["mm"], "deg": g["view"]["deg"],
                       "correctives": g["correctives"], "settled": pending["settle"]["settled"]}
        for key in ("periods", "bounds"):
            if key in g:
                out["like"][key] = g[key]
        if not g["view"]["same"]:
            stop = "" if g["stopped"] == "goal" else f"the move back stopped ({g['stopped']}), "
            out["compared_with"] = {"with": like, "said": f"{stop}could not return the camera to {like}'s pose (off "
                                    f"{g['view']['mm']:g} mm, {g['view']['deg']:g} deg): no comparison",
                                    "reading": "not_judged"}
            return out
        cmp = compare_looks(ex, like, result["look_id"], at=_hook_at(ex, float(ex._looks[like]["t"])),
                            origin="like")
        out["compared_with"] = dict(_compact(cmp), **{"with": like})
        return out
    cw = _auto(ex, result, "auto")
    if cw is not None:
        out["compared_with"] = cw
        return out
    return None


def _post_any(ex, name, kwargs, result):
    if not isinstance(result, dict) or "refused" in result:
        return None
    st = state(ex)
    if name == "survey" and isinstance(result.get("candidates"), list):
        st["candidates"] = [c for c in result["candidates"] if isinstance(c, dict)]
    out = None
    if is_device_act(name, result):
        t0 = st["call_t0"] if st["call_t0"] is not None else float(ex.sim_time())
        wit = result.get("witness") if isinstance(result.get("witness"), dict) else {}
        _, before = ex._look_rec(wit.get("before")) if wit.get("before") else (None, None)
        if before is not None and before.get("t") is not None:
            t0 = float(before["t"]) + 1e-6       # an R3 verb's own before look comes ahead of its push
        p, cid = _act_point(result)
        seq = int(getattr(ex, "_calls", 0))
        nested = result.get("look") if isinstance(result.get("look"), dict) else None
        t1 = float(ex.sim_time())
        if nested is not None and ex._look_rec(nested.get("look_id"))[1] is not None:
            t1 = min(t1, float(ex._look_rec(nested["look_id"])[1]["t"]))
        st["acts"].append({"seq": seq, "name": name, "t0": t0, "t1": t1, "point_mm": p, "contact_id": cid})
        for r in st["records"]:                  # records made inside this call witness this act
            if r["seq"] == seq and r["after_t"] > t0:
                r["after_act_seq"] = seq
    nested = result.get("look") if isinstance(result.get("look"), dict) else None
    if name not in LOOK_NAMES and nested is not None and nested.get("look_id") is not None \
            and "compared_with" not in nested:
        cw = _auto(ex, nested, "auto")
        if cw is not None:
            out = dict(result)
            out["look"] = dict(nested, compared_with=cw)
    return out


def _record(ex, record):
    """A record by its id (or the record itself)."""
    if isinstance(record, dict):
        return record
    return next((r for r in state(ex)["records"] if r["id"] == str(record)), None)


def can_return(ex, look):
    """Dry: could look like=look go back now? {"ok", "why"}: the look exists, is not dark, its pose does not put
    the tool at a stored point and the gates an operator's move would meet let the move start. Nothing moves."""
    key, rec = ex._look_rec(look)
    if rec is None or rec.get("can_see") == "dark":
        return {"ok": False, "why": "no such look" if rec is None else f"look {key} is dark"}
    r = pose._goal_at_point(ex, rec["q"]) or pose._gate(ex, rec["q"])
    return {"ok": r is None, "why": None if r is None else (r.get("refused") or {}).get("why")}


def setup(ex):
    st = state(ex)
    st["go_and_look"] = lambda q, note="", intent=None: pose.go_and_look(ex, q, note, intent)
    st["compare"] = lambda before, after, at=None: compare_looks(ex, before, after, at, origin="runtime")
    st["reading"] = lambda record, part=None, points=(), reach_mm=None: wd.reading(
        _record(ex, record), part, points, reach_mm)
    st["can_return"] = lambda look: can_return(ex, look)
    st["same_view"] = lambda q_a, q_b: {k: v for k, v in pose.same_view(ex.model, q_a, q_b,
                                                                         pose.tolerance(ex, q_a)).items()
                                        if k != "tol"}
    ex.add_pre(lambda name, kw: _pre_any(ex, name, kw))
    ex.add_pre(lambda name, kw: _pre_look(ex, name, kw), names=["look"])
    ex.add_post(lambda name, kw, res: _post_look(ex, name, kw, res), names=list(LOOK_NAMES))
    ex.add_post(lambda name, kw, res: _post_any(ex, name, kw, res))
    ex.add_post(lambda name, kw, res: wv.post_verify(ex, name, kw, res), names=["verify"])


def FILTER(surface, resources, handlers, ex=None):
    return wv.filter_tables(surface, resources, handlers, ex)


class Mixin:
    @primitive
    def compare(self, before, after, at=None, detail="brief"):
        if detail not in ("brief", "full"):
            return refusal("detail", f"detail is brief or full, not {str(detail)[:20]!r}", "leave it out")
        return compare_looks(self, before, after, at, detail, origin="tool")
