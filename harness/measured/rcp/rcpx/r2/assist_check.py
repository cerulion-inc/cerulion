# SPDX-License-Identifier: AGPL-3.0-only
"""Pointing tests with a known answer, and what the robot's own checks said (package assist).

Every pointing act is a test the robot can mark without ground truth: it knows where its own body is in that look
(the body model at the look's joint pose) and what the manual says the parts look like. All of it is sensing: it
changes no tool, moves nothing and adds no text; the gauge reads it.

    passive(ex, request, response)      the extra dict of Gauge.feed for this answered request (answer path):
                                        on_body ("static" | "moving" | None), mislabel, parts, roles, confirmed,
                                        check, by_robot, unconfirmed
    pointing(request, response)         {look, px [u, v], label, tool} of a pointing act, else None
    colour_mislabel(image, px, label, doc)   None or {label, colour, other, state}: the pointed pixel has the
                                        manual's colour of ANOTHER part of the same device and none of the named
                                        part's own (pure but for the image; the replay gate runs it on recorded frames)
    sense_job(ex, request, response, call=None)   None or the off-path job {call, what, look, px, part, frame_path}
    located_rows(ex, guard, flags)      the World's located entries for the offer engine, each with a dry reach
                                        check and the guard's target verdict
    manual(ex)                          {plist, names, roles, kinds} of the served robot://devices (cached)

On the answer path a pointing act costs one ray test and one colour read of a 9 px square (at a picture 640 px
wide; it scales with the width) of the frame the look already wrote. Nothing here reads a scene, names a robot or
a device, or imports another package: the manual is read through rcpx.sheet_words, colours through
rcpx.colour_words, the body through ex.state["selfsee"] when that package is loaded, else the model's own static
collision capsule and mount.
"""
import json
import os

import numpy as np

from rcpx import colour_words, sheet_words
from rcpx import r3_limits as L

MM = 1000.0
STATIC_GROUPS = ("own_base",)            # the base and the mount: what the body event reads
MOVING_GROUPS = ("own_arm", "own_hand")  # a hit here is a note, not an event
COLOUR_KEYS = ("where", "how_to_tell", "notes", "moves", "direction")
ACT_STOPS_KEYS = ("contact", "outcome_class", "moved_part")
LOOK_KEYS = ("look_id", "look")
WIDTH_REF = 640.0
SENSE_WHAT = "mislabel_find"


def _args(request):
    """The call's arguments as a dict ({} when they cannot be read)."""
    if not isinstance(request, dict):
        return {}
    args = request.get("args")
    if args is None and isinstance(request.get("args_text"), str):
        try:
            args = json.loads(request["args_text"])
        except ValueError:
            args = None
    return args if isinstance(args, dict) else {}


def _result(response):
    r = response.get("result") if isinstance(response, dict) else None
    return r if isinstance(r, dict) else {}


def _num(v):
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _pair(v):
    if isinstance(v, (list, tuple)) and len(v) == 2 and _num(v[0]) is not None and _num(v[1]) is not None:
        return [float(v[0]), float(v[1])]
    return None


# ---------------------------------------------------------------- the manual
def doc(ex):
    d = (getattr(ex, "context", None) or {}).get("devices")
    if callable(d):
        try:
            d = d()
        except Exception:
            d = None
    return d if isinstance(d, dict) else {}


def manual(ex):
    """{doc, plist, names, roles, kinds} of the served manual, parsed once per document object."""
    d = doc(ex)
    cache = _check_state(ex)["manual"] if isinstance(getattr(ex, "state", None), dict) else {}
    if cache.get("doc") is d and "plist" in cache:
        return cache
    plist = sheet_words.parts(d) if d else []
    roles = {}
    for p in plist:
        if p["name"]:
            roles.setdefault(p["name"], p["role"])
    kinds = {"control": [], "cue": [], "forbidden": []}
    for name, role in roles.items():
        if role in kinds:
            kinds[role].append(name)
    cache.update(doc=d, plist=plist, names=list(roles), roles=roles, kinds=kinds)
    return cache


# ---------------------------------------------------------------- the pointing act
def pointing(request, response):
    """{look, px, label, tool, refused} when the request points at a pixel of a look, else None. The pixel is the
    result's own (u, v) when it states one (a tool that turned another pixel form into u and v), else the
    arguments' u and v, px or center_px; a refusal that names the pixel it refused is read too."""
    if not (isinstance(request, dict) and request.get("op") == "call"):
        return None
    args, res = _args(request), _result(response)
    body = res.get("own_body") if isinstance(res.get("own_body"), dict) else {}
    px = None
    if _num(res.get("u")) is not None and _num(res.get("v")) is not None:
        px = [float(res["u"]), float(res["v"])]
    px = px or _pair(body.get("px"))
    if px is None and _num(args.get("u")) is not None and _num(args.get("v")) is not None:
        px = [float(args["u"]), float(args["v"])]
    px = px or _pair(args.get("px")) or _pair(args.get("center_px"))
    look = next((args[k] for k in LOOK_KEYS if isinstance(args.get(k), (str, int))), None)
    look = look if look is not None else (body.get("look_id") or res.get("look_id"))
    if px is None or look is None:
        return None
    label = args.get("label") if args.get("label") is not None else args.get("part")
    return {"look": str(look), "px": px, "label": None if label is None else str(label),
            "tool": str(request.get("name")), "refused": "refused" in res}


def _look_rec(ex, look):
    fn = getattr(ex, "_look_rec", None)
    if callable(fn):
        try:
            return fn(look)
        except Exception:
            pass
    looks = getattr(ex, "_looks", None) or {}
    key = str(look)
    if key not in looks and key.isdigit():
        key = f"L{key}"
    return key, looks.get(key)


def _frame_path(ex, rec, base=None):
    path = (rec or {}).get("frame")
    if not isinstance(path, str):
        return None
    if os.path.isabs(path):
        return path
    for root in (base, getattr(getattr(ex, "journal", None), "dir", None)):
        if root is not None and os.path.exists(os.path.join(str(root), path)):
            return os.path.join(str(root), path)
    return path


# ---------------------------------------------------------------- own body
def _ray_capsule(o, d, p0, p1, r):
    """The distance along the ray o + t d (d a unit vector) at which it first comes within r of the segment p0 p1,
    or None. Sampled along the segment: the capsules are short and the answer is a yes or no."""
    best = None
    for s in np.linspace(0.0, 1.0, 9):
        c = p0 + s * (p1 - p0)
        t = float((c - o) @ d)
        if t <= 0.0:
            continue
        miss = float(np.linalg.norm(o + t * d - c))
        if miss <= r and (best is None or t < best):
            best = t
    return best


def _ray_box(o, d, lo, hi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - o) / d, (hi - o) / d
    tmin = np.nanmax(np.minimum(t1, t2))
    tmax = np.nanmin(np.maximum(t1, t2))
    return float(max(tmin, 0.0)) if tmax >= max(tmin, 0.0) else None


def _model_hit(ex, q, u, v):
    """Without the selfsee package: the first of the model's collision capsules the pixel's ray meets (the first
    capsule is the base) and the mount under it (robot://body's standing surface), zero margin."""
    model = ex.model
    o, d = model.pixel_ray(np.asarray(q, dtype=float), float(u), float(v))
    o, d = np.asarray(o, dtype=float), np.asarray(d, dtype=float)
    hits = []
    for i, (_name, p0, p1, r) in enumerate(model.collision_capsules(np.asarray(q, dtype=float))):
        t = _ray_capsule(o, d, np.asarray(p0, dtype=float), np.asarray(p1, dtype=float), float(r))
        if t is not None:
            hits.append((t, "static" if i == 0 else "moving"))
    surf = getattr(ex, "standing_surface", None)
    s = surf() if callable(surf) else {}
    half, top = s.get("mount_half_side_mm"), s.get("mount_top_z_mm")
    if half is not None:
        h, z = float(half) / MM, float(top or 0.0) / MM
        floor = float(s.get("floor_z_mm") or 0.0) / MM
        t = _ray_box(o, d, np.array([-h, -h, min(floor, z) - 1e-6]), np.array([h, h, z]))
        if t is not None:
            hits.append((t, "static"))
    return min(hits)[1] if hits else None


def on_body(ex, look, u, v):
    """"static" when the pixel's ray first meets the robot's own base or mount at that look's joint pose, "moving"
    for its arm or hand, else None. Zero pixel margin."""
    _key, rec = _look_rec(ex, look)
    if rec is None or rec.get("q") is None:
        return None
    q = np.asarray(rec["q"], dtype=float)
    first = (getattr(ex, "state", None) or {}).get("selfsee", {}).get("first_hit")
    try:
        if callable(first):
            what = (first(q, float(u), float(v)) or {}).get("what")
            return "static" if what in STATIC_GROUPS else "moving" if what in MOVING_GROUPS else None
        return _model_hit(ex, q, u, v)
    except Exception:
        return None


def _refused_body(res):
    """A call another package refused with check own_body: its group decides (the base and the mount are static)."""
    body = res.get("own_body")
    if not (isinstance(res.get("refused"), dict) and res["refused"].get("check") == "own_body"
            and isinstance(body, dict)):
        return None
    if body.get("group") in STATIC_GROUPS or body.get("how") == "mount" or body.get("source") == "region_b":
        return "static"
    return "moving" if body.get("group") in MOVING_GROUPS else None


# ---------------------------------------------------------------- the manual's colours
def colours_of(part):
    """Every colour the part's own entry names (its states' colours and any colour word of its own text)."""
    out = set((part or {}).get("colours", {}).values())
    for key in COLOUR_KEYS:
        for c, _ in colour_words.words(str((part or {}).get(key) or "")):
            out.add(c)
    return out


def _image(image):
    if isinstance(image, (str, os.PathLike)):
        from PIL import Image
        with Image.open(image) as im:
            return np.asarray(im.convert("RGB"))
    a = np.asarray(image)
    return a[..., :3] if a.ndim == 3 else a


def spot_colour(image, px):
    """The colour word of the square round the pixel: 9 px at a picture 640 px wide, scaled with the width."""
    a = _image(image)
    size = max(3, int(round(L.PART_CHECK_PX * a.shape[1] / WIDTH_REF)))
    r = size // 2
    u, v = int(round(float(px[0]))), int(round(float(px[1])))
    patch = a[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
    if patch.size == 0:
        return None
    return colour_words.name(patch)


GROUND_GRID = (6, 8)          # rows and columns of the squares the ground colour is read on
GROUND_SHARE = 1.0 / 3.0      # a colour word on this share of them is the picture's ground


def ground_colour(image):
    """The colour word most of the picture has (the standing surface, a wall), read on a coarse grid of the same
    squares the pointed pixel is read with; None when no word holds GROUND_SHARE of the grid."""
    a = _image(image)
    h, w = a.shape[:2]
    words = {}
    rows, cols = GROUND_GRID
    for i in range(rows):
        for j in range(cols):
            c = spot_colour(a, ((j + 0.5) * w / cols, (i + 0.5) * h / rows))
            if c is not None:
                words[c] = words.get(c, 0) + 1
    if not words:
        return None
    best = max(sorted(words), key=lambda k: words[k])
    return best if words[best] >= GROUND_SHARE * rows * cols else None


def colour_mislabel(image, px, label, doc, can_see=None):
    """None, or {"label", "colour", "other", "state"}: the label names a part of the manual, the pixel's colour is
    near a colour the manual gives ANOTHER part of the same device (in the state named), and near none of the
    named part's own colours. image: a path, a picture or an array (H x W x 3). can_see: the look's own word when
    the caller has it; a look that does not read ok gives None (a colour read in dim light names the light, not
    the part). doc: the robot://devices document, or sheet_words.parts of it."""
    if can_see is not None and can_see != "ok":
        return None
    plist = sheet_words.parts(doc) if isinstance(doc, dict) else list(doc or [])
    part = sheet_words.label_part(label, plist) if plist and label is not None else None
    if part is None:
        return None
    image = _image(image)
    colour = spot_colour(image, px)
    if colour is None or any(colour_words.near(colour, c) for c in colours_of(part)):
        return None
    for other in plist:
        if other is part or other["device"] != part["device"]:
            continue
        for state, c in other.get("colours", {}).items():
            if colour_words.near(c, colour):
                # the colour of the ground the thing stands on is no sign of another part (seen:
                # a mark a few pixels off a small part, on a table of the colour the manual gives a lit lamp,
                # read as that lamp on a correct operator)
                ground = ground_colour(image)
                if ground is not None and colour_words.near(ground, colour):
                    return None
                return {"label": str(label), "colour": colour, "other": other["name"], "state": state}
    return None


# ---------------------------------------------------------------- what the robot's own checks said
def _check_state(ex):
    return ex.state.setdefault("assist_check", {"witness_seen": 0, "cue": None, "cue_at_act": None, "acted": False,
                                                "manual": {}})


def _is_act(res):
    return "refused" not in res and "stopped" in res and any(k in res for k in ACT_STOPS_KEYS)


def _cue_word(name, res):
    """The cue's reading in a result, as lit or dark, when the robot itself read it (never the model's word)."""
    cue = res.get("cue")
    if name == "device_state" and isinstance(cue, dict) and cue.get("reading") is not None:
        return "lit" if str(res.get("cue_suggests") or "").startswith("lit") else "dark"
    ev = res.get("evidence_checked")
    if isinstance(ev, dict) and ev.get("tested") == "cue_read" and isinstance(ev.get("cue"), dict) \
            and "lit" in ev["cue"]:
        return "lit" if ev["cue"]["lit"] else "dark"
    return None


def _seen(res):
    """(confirmed, check) from a result's seen rows ({changed: yes | no | unclear}): the robot's own compare."""
    rows = [r for r in (res.get("seen") or []) if isinstance(r, dict)] if isinstance(res.get("seen"), list) else []
    marks = [str(r.get("changed")) for r in rows]
    if not marks or res.get("moved_whole"):
        return None, None
    if "yes" in marks:
        return True, None
    return (None, "no_change") if all(m == "no" for m in marks) else (None, None)


def _robot_checks(ex, request, res):
    """(confirmed, check): True, or "no_change", from the newest same-view record or cue read that belongs to this
    call; (None, None) when the result itself must decide."""
    st = _check_state(ex)
    name = str(request.get("name")) if isinstance(request, dict) else ""
    confirmed = check = None
    records = ((getattr(ex, "state", None) or {}).get("witness") or {}).get("records") or []
    new, st["witness_seen"] = records[st["witness_seen"]:], len(records)
    for rec in new:
        if not isinstance(rec, dict):
            continue
        if rec.get("reading") == "changed" and not rec.get("device_moved"):
            confirmed = True
        elif rec.get("reading") == "unchanged" and confirmed is None:
            check = "no_change"
    word = _cue_word(name, res)
    if _is_act(res):
        st["cue_at_act"], st["acted"] = st["cue"], True
    if word is not None:
        if st["acted"] and st["cue_at_act"] is not None:
            if word != st["cue_at_act"]:
                confirmed = True
            elif confirmed is None:
                check = "no_change"
        st["cue"] = word
    return confirmed, (None if confirmed else check)


def passive(ex, request, response, base=None):
    """The extra dict of Gauge.feed for one answered request (module text). base: the episode folder, for a look
    whose frame path is relative."""
    man = manual(ex)
    extra = {"parts": list(man["names"]), "roles": dict(man["roles"])}
    if not (isinstance(request, dict) and request.get("op") == "call"):
        return extra
    res = _result(response)
    point = pointing(request, response)
    if point is not None:
        hit = _refused_body(res)
        if hit is None and "refused" not in res:
            hit = on_body(ex, point["look"], point["px"][0], point["px"][1])
        if hit is not None:
            extra["on_body"] = hit
        if hit is None and point["label"] is not None and "refused" not in res and man["plist"] \
                and sheet_words.label_part(point["label"], man["plist"]) is not None:
            _key, rec = _look_rec(ex, point["look"])
            path = _frame_path(ex, rec, base) if rec is not None and rec.get("can_see") == "ok" else None
            if path is not None:
                try:
                    found = colour_mislabel(path, point["px"], point["label"], man["plist"], rec.get("can_see"))
                except Exception:
                    found = None
                if found is not None:
                    extra["mislabel"] = found
    if res.get("by") == "robot":
        extra["by_robot"] = True
        shows = res.get("shows_change") is True
        extra["unconfirmed"] = not shows
        confirmed, check = _seen(res)
        if shows and (res.get("met") is True or res.get("stopped") == "met" or confirmed):
            extra["confirmed"] = True
        elif "shows_change" in res:
            extra["confirmed"] = False          # inside a robot-run step only the cue confirms (2.4)
        elif confirmed:
            extra["confirmed"] = True
        if check and not extra.get("confirmed"):
            extra["check"] = check
        _robot_checks(ex, request, res)         # keep the record counter and the cue memory in step
        return extra
    confirmed, check = _robot_checks(ex, request, res)
    if confirmed:
        extra["confirmed"] = True
    elif check:
        extra["check"] = check
    return extra


# ---------------------------------------------------------------- off the answer path
def sense_job(ex, request, response, call=None, base=None):
    """The off-path job for this answered request, or None: only a pointing act that was not refused, whose label
    names a part of the manual, in a session whose vision equipment finds by words."""
    eyes = (getattr(ex, "state", None) or {}).get("eyes")
    if not (isinstance(eyes, dict) and eyes.get("level") == "words" and callable(eyes.get("find_at"))):
        return None
    point = pointing(request, response)
    if point is None or point["refused"] or point["label"] is None:
        return None
    man = manual(ex)
    part = sheet_words.label_part(point["label"], man["plist"]) if man["plist"] else None
    if part is None:
        return None
    key, rec = _look_rec(ex, point["look"])
    if rec is None:
        return None
    return {"call": call, "what": SENSE_WHAT, "look": key, "px": [point["px"][0], point["px"][1]],
            "part": part["name"], "frame_path": _frame_path(ex, rec, base)}


def run_job(find_at):
    """fn(job) for assist_sense.start: the eyes package's find_at on the job's own plain values."""
    def fn(job):
        return find_at(job["look"], job["px"][0], job["px"][1], job["part"])
    return fn


# ---------------------------------------------------------------- the located entries, for the offer engine
def _dry_reach(ex, point_mm):
    """True | False | None: reach_check on the point, run bare (no hook, no journal record, no call counted)."""
    fn = getattr(ex, "reach_check", None)
    if not callable(fn) or point_mm is None:
        return None
    ex._depth = getattr(ex, "_depth", 0) + 1
    try:
        out = fn(point_mm=list(point_mm))
        return bool(out.get("reachable")) if isinstance(out, dict) and "reachable" in out else None
    except Exception:
        return None
    finally:
        ex._depth -= 1


def located_rows(ex, guard=None, flags=None, limit=6):
    """[{id, label, stale, in_reach, target_ok, uncertain_mm}] of the newest located entries, newest last.
    target_ok is None when a written act call may name the entry, else the reason; with no guard no entry passes."""
    world = getattr(ex, "world", None)
    entries = [e for e in getattr(world, "entries", {}).values() if e.get("kind") == "located"] if world else []
    rows = []
    for e in entries[-int(limit):]:
        stale = bool(e.get("stale"))
        reach = None if stale else _dry_reach(ex, e.get("point_mm"))
        if guard is None:
            ok = "no guard"
        else:
            try:
                ok = guard["target_ok"](e["id"], dict((flags or {}).get(e["id"]) or {}))
            except Exception:
                ok = "guard failed"
        if ok is None and reach is False:
            ok = "out of reach"
        rows.append({"id": e["id"], "label": e.get("label"), "stale": stale, "in_reach": reach, "target_ok": ok,
                     "uncertain_mm": _num((e.get("data") or {}).get("uncertain_mm"))})
    return rows


# ---------------------------------------------------------------- the standing preferences
# One word each through assist {"set": WORD}; only the operator sets one. Each answer says what will be different,
# what the robot will do on its own under it, and what it costs. The tables stand in this file (not in assist.py)
# only for that file's length; assist.py applies them.
PREFS = {"seeing": "plain", "acting": "own", "checking": "own_checks", "offers": "on", "tools": "all"}
WORDS = {"numbers": ("seeing", "numbers"), "plain": ("seeing", "plain"), "steps": ("acting", "steps"),
         "routine": ("acting", "routine"), "own": ("acting", "own"), "robot_checks": ("checking", "robot_checks"),
         "own_checks": ("checking", "own_checks"), "quiet": ("offers", "quiet"), "offers": ("offers", "on"),
         "few": ("tools", "few"), "all": ("tools", "all")}
SET_SAID = {
    "numbers": "every look now comes with the robot's numbered list and the numbered picture; the robot asks its "
               "vision equipment once per look on its own; cost: a longer look result",
    "plain": "looks are as they were, with no numbers; the robot adds nothing on its own; no cost",
    "steps": "a result that stores a located part now carries next = the whole step on it; nothing moves until you "
             "make that call; cost: one more key on those results",
    "routine": "the robot numbers what it sees, tries the candidates and checks the result; you confirm each step "
               "with assist take N; nothing moves until you do",
    "own": "next is left as each tool writes it; the robot proposes no step; no cost",
    "robot_checks": "after each of your acts that met something the robot MOVES THE CAMERA back to the view before "
                    "the act and compares, on its own; cost: sim seconds, no call; skipped while holding, after a "
                    "refusal, or when the view cannot be returned to",
    "own_checks": "the robot compares nothing by itself; you look and verify; no cost",
    "quiet": "no offers for the rest of the session; claim answers keep their next; the robot does nothing on its "
             "own; assist {\"set\": \"offers\"} turns them back on",
    "offers": "offers are on again: at most one on a result, only when the robot noticed something; no cost",
    "few": "your tool list is now the help steps only; every tool still runs by name; list again to see it; cost: "
           "your tool definitions are rebuilt once",
    "all": "your tool list is whole again; list again to see it; cost: your tool definitions are rebuilt once",
}

__all__ = ["PREFS", "SET_SAID", "WORDS", "colour_mislabel", "colours_of", "doc", "located_rows", "manual", "on_body",
           "passive", "pointing", "run_job", "sense_job", "spot_colour"]
