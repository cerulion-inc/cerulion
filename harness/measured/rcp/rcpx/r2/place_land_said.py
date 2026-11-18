# SPDX-License-Identifier: AGPL-3.0-only
"""The words of a set down, what is known of the arrival, what a stated limit costs the landing and what letting go
can make (split out of rcpx/r2/place_land.py). Nothing here moves anything, reads ground truth or names
a robot."""

import math

import numpy as np

from rcpx import bounds_ratios as R
from rcpx import control_hold
from rcpx.control_base import _mm
from rcpx.r2.place_hand import finite

MM = 1000.0
DOWN = np.array([0.0, 0.0, -1.0])


def cost(ex, B, q, land, bnd, z_end, pads, depth=None):
    """C12: what a stated limit costs the landing, {"periods": with less without, "without_periods", "where"}, or
    None when the landing runs under no limit or has no band: the band's depth at the limit's arrival speed
    against the same depth at the speed the ending force would have run (the hold's squeeze per pad; the body's
    own speed where no squeeze stands)."""
    if land is None or land["is"] != "limit" or bnd is None or z_end is None:
        return None
    held = control_hold.held(ex)
    depth = max(0.0, float(bnd["top_z"]) - float(z_end)) if depth is None else float(depth)
    v_free = float(B.v_free(q, DOWN))
    arr = B.arrival(land["n"], land["source"], q, DOWN, held)
    v_lim = v_free if arr.get("v_m_s") is None else min(v_free, float(arr["v_m_s"]))
    sq = None if pads is None else pads.get("squeeze_pad_n")
    v_ends = v_free
    if finite(sq) and float(sq) > 0.0:
        e = B.arrival(float(sq), "intent", q, DOWN, held, (None, None))
        v_ends = v_free if e.get("v_m_s") is None else min(v_free, float(e["v_m_s"]))
    dt = float(ex.dt)
    n_lim = int(math.ceil(depth / (max(v_lim, np.finfo(float).tiny) * dt)))
    n_ends = int(math.ceil(depth / (max(v_ends, np.finfo(float).tiny) * dt)))
    return {"periods": max(0, n_lim - n_ends), "without_periods": n_ends, "with_periods": n_lim,
            "where": f"the arrival stretch of the set down ({depth * MM:.1f} mm at {v_lim * MM:.1f} mm/s; with "
                     f"nothing stated {v_ends * MM:.1f} mm/s)"}


def let_go(ex, pads, facts, drop_m, after_n=None):
    """The strike block of a release (the jaws open on a thing that stands or hangs): each pad's load on the thing
    only falls; what the thing lands on gains at most what the pads carried of it, its weight (as the hold read it
    at rest; not read: the squeeze each pad pressed it with stands in, said), and, where it was let go above what
    stands there (drop_m: how far its bottom stood over the surface, or how far the place's height may be off),
    the stop of that fall: its weight over g, times the speed of a fall from drop_m, times the rate of the contact
    model the body states for its own shapes (the momentum given up over the contact's own time). after_n: the
    set down's own figure after its stop: letting go is the end of the landing, whose ONE figure bounds what the
    landing makes on every pair, so the release is never stated under it (measured on the bridge's arm:
    a light thing let go 3 mm up made 3 to 11 N on what it landed on, where its weight as the arm read it and
    the fall by the model gave 0.5 to 4 N)."""
    w, src = facts.get("weight_n") if isinstance(facts, dict) else None, "the held thing's weight as read at rest"
    if not finite(w) or not float(w) > 0.0:
        w = None if not isinstance(pads, dict) else pads.get("squeeze_pad_n")
        src = "the squeeze each pad pressed it with (its weight was not read)"
    if not finite(w):
        return None
    g = float(np.linalg.norm(np.asarray(ex.model.mj.opt.gravity, dtype=float))) if hasattr(ex.model, "mj") else 0.0
    F = control_hold.facts(ex)
    b = None if F is None else F.get("b_per_s")
    fall = 0.0
    if finite(drop_m, b) and float(drop_m) > 0.0 and g > 0.0:
        fall = float(w) * float(b) * math.sqrt(2.0 * float(drop_m) / g)
    peak = (float(w) + fall) * (1.0 + R.REL)
    if finite(after_n):
        peak = max(peak, float(after_n))
    return {"peak_n": round(peak, 1), "v_m_s": None if not (finite(drop_m) and g > 0.0) else
            math.sqrt(2.0 * max(0.0, float(drop_m)) * g), "z_source": f"{src}"
            + ("" if not fall > 0.0 else f" and the stop of a fall of {float(drop_m) * MM:.1f} mm by the contact "
               "model's own rate (the body's model, not measured)") + "; the pads' own load on it only falls",
            "at_least_n": None, "model_peak_n": round(peak, 1), "link": None, "gentle": None}


def read_moved(rel):
    """What the way down's own readings showed of a landing that no guard told (the design rulings: a reading that moved is
    said beside what the arm read), as words, or "" when none moved. rel["landing_read"]: the set down's own figure
    after the stop (strike["ran"]): a load that rose on the arm's own reading at a height with no guard telling
    it, and the jaws' gap that closed while it went down (the thing moved between the pads)."""
    got = rel.get("landing_read") if isinstance(rel.get("landing_read"), dict) else {}
    words = []
    if finite(got.get("load_rose_at_z_mm")):
        words.append(f"a load rose on the arm's own reading at z {float(got['load_rose_at_z_mm']):.0f} mm with no "
                     "guard telling it")
    if finite(got.get("gap_closed_mm_s")) and float(got["gap_closed_mm_s"]) > 0.0:
        words.append("the jaws' own reading showed their gap close at up to "
                     f"{float(got['gap_closed_mm_s']):.1f} mm/s on the way down (the thing moved between the pads)")
    return "; ".join(words)


def _level(rel):
    got = rel.get("landing_read") if isinstance(rel.get("landing_read"), dict) else {}
    lvl = got.get("f_guard_n")
    return float(lvl) if finite(lvl) and float(lvl) > 0.0 else None


def felt_over(rel):
    """The level under which this arm's guard tells no contact on that way down, as words (" over N N"), or ""
    where the set down states none."""
    lvl = _level(rel)
    return "" if lvl is None else f" over {lvl:.3g} N"


def read_none(rel, where):
    """A way down no guard told, in the words of what the arm READ: the arm read no load
    over the level it tells, and a contact lighter than that level is not told there; its silence is never said
    as an absence of contact."""
    lvl = _level(rel)
    return (f"the arm read no load{felt_over(rel)} {where}"
            + (f" (a contact lighter than {lvl:.3g} N is not told there)" if lvl is not None else
               " (no level is stated for what this arm tells there: a light contact is not told)"))


class SaidMixin:
    # ---------------------------------------------------------------- the words, the arrival evidence
    def _pl_set_said(self, landed, lost, met, plan, rel, r):
        """The first sentence of a set down: where the thing was let go, in the words of what was measured (a
        release over the rim or over the spot is said as that, with the height it fell from)."""
        if lost:
            return ("Lost the thing over the place: the hold went while it was lowered, its bottom at z "
                    f"{rel['bottom_z_mm']:.0f} mm. Look before the next step.")
        d = rel["drop"]
        inside = (d or {}).get("under_rim_mm") is not None and d["under_rim_mm"] >= 0
        if landed and plan["mode"] == "into" and inside:
            how = (f"a contact ended the way down with its bottom {rel['rim']} (it landed inside, or the hand met "
                   "the rim; the floor inside is not seen) and it was let go there")
        elif landed and plan["mode"] == "into":
            how = (f"a contact ended the way down with its bottom {rel['rim']} (the thing or the hand met "
                   "something there; the floor inside is not seen) and it was let go there")
        elif landed:
            how = f"it landed at z {rel['bottom_z_mm']:.0f} mm"
        elif met:
            how = (f"the way down met something at z {rel['bottom_z_mm']:.0f} mm ({r.get('why') or r.get('stopped')}) "
                   "and it was let go there")
        elif plan["mode"] == "into" and inside:
            how = (read_none(rel, "on the way down") + f" and it was let go into it, its bottom {rel['rim']}" + (
                f", {d['height_mm']:.0f} mm over the floor the look shows inside" if d.get("height_mm") is not None
                else " (the floor inside is not seen)") + f" ({d['why']})")
        elif plan["mode"] == "into":
            how = (f"let go with its bottom {rel['rim']} ({d['why']}); it fell into the opening from there" + (
                f", {d['height_mm']:.0f} mm over the floor the look shows inside" if d.get("height_mm") is not None
                else " (the floor inside is not seen)"))
        elif plan["mode"] == "release_above":
            how = ("let go above the place on your word" + (f", a drop of {d['height_mm']:.0f} mm"
                                                              if d["height_mm"] is not None else ""))
        elif d["height_mm"] is not None and d["height_mm"] > rel["track_mm"]:
            how = (f"let go {d['height_mm']:.0f} mm over the spot, its bottom at z {rel['bottom_z_mm']:.0f} mm "
                   f"({d['why']})")
        elif rel.get("bottom_over_surface_mm") is not None and rel["bottom_over_surface_mm"] < -rel["track_mm"]:
            how = (read_none(rel, f"down to z {rel['bottom_z_mm']:.0f} mm, "
                             f"{-rel['bottom_over_surface_mm']:.0f} mm under the height the spot gives (as far as that "
                             "height and the thing's own bottom can be off)") + ", and it was let go there")
        else:
            how = (read_none(rel, f"down to z {rel['bottom_z_mm']:.0f} mm, the height the spot gives")
                   + ", and it was let go there")
        moved = read_moved(rel) if not (landed or met) else ""
        how += "" if not moved else f" ({moved})"
        return f"Set the thing down: {how}; the jaws opened and the hand rose."

    # ---------------------------------------------------------------- arrival evidence
    def _pl_arrival(self, target, rel, before_look=None, picked=None):
        """What is known of the arrival: (arrived true | false | None, evidence). Release facts first; then the
        same view before and after where a camera can say (witness compare when loaded and a before look of a
        fixed camera exists)."""
        ev = {"release": {k: rel.get(k) for k in ("opened_at_mm", "bottom_over_surface_mm", "touchdown_felt",
                                                   "drop")}, "view": None}
        spots = [("place", [float(x) for x in target["centre"]])]
        if picked is not None:
            spots.append(("pick_spot", [float(x) for x in picked]))
        seen_change = {}
        wit = self.state.get("witness")
        cmp_fn = getattr(wit, "compare", None) if wit is not None else None
        if callable(cmp_fn) and before_look and "scene_camera" in self.state:
            try:
                from rcpx.r2 import scene_camera as SC
                rec = SC._state(self)["looks"].get(before_look)
                if rec is not None:
                    after = SC.scene_look(self, rec["camera"], "after the set-down")
                    res = cmp_fn(before_look, after.get("look_id"), at=[_mm(p) for _, p in spots])
                    rows = res.get("at") if isinstance(res, dict) else None
                    for (name, _), row in zip(spots, rows or []):
                        seen_change[name] = row.get("changed")
                    ev["view"] = {"by": "the same view before and after", "before": before_look,
                                  "after": after.get("look_id"), "changed": seen_change}
            except Exception as exc:
                ev["view"] = {"by": "the same view before and after", "failed": type(exc).__name__}
        yes = [v for v in seen_change.values() if v == "yes"]
        no = [v for v in seen_change.values() if v == "no"]
        if rel.get("touchdown_felt") and not no:
            return True, ev
        if seen_change and len(yes) == len(seen_change):
            return True, ev
        if seen_change and len(no) == len(seen_change):
            return False, ev
        return None, ev
