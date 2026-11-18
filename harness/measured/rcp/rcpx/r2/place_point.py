# SPDX-License-Identifier: AGPL-3.0-only
"""Package place_point (round six, the design notes C11): the one door from a spot to what the hands take. No tools
of its own: the whole-step tools (pick, place, put, work_part) call resolve(ex, spot, role) for every spot that is
not already a Seen.

A spot is {"look": id, "px": [u, v]} (a scene camera look or a hand camera look), a located label, or [x, y, z]
mm. resolve asks rcpx/r2/see_spot.py for the point, its error and what stands there, with no motion and no control
period, and never refuses for doubt: its only refusals are see_spot's malformed arguments. The way in with
its two looks, the closer pair, the look on arrival and the carved things of round four are gone.

resolve(ex, spot, role="pick", intent=None) -> Seen (SI: m, rad, base frame) with the spot under "spot":
- a thing stands under the point: kind "thing" with "outline" {centre, narrow, wide, bearing}, "top_z", "height"
  (over the level it stands on, or over the stated support where that is the higher: a drawn surface under the
  plane the hand meets adds nothing to a height), "sigma_xy", "sigma_z", "part_chord" {width, bearing, top_z,
  centre}, "hollow" (True when higher points of the same thing surround the pointed point), "depth_inside",
  "wall" (the wall's thickness; the wall itself is spot.extent.wall), "pointed_xy";
- role part: the same Seen with top_z the POINTED point's own height (a handle on a thing's front is worked where
  it was pointed at, not at the thing's top; the thing's top is spot.extent.top_z);
- role pick or part with no thing under the point: kind "thing" with outline None, height None, sigma_z None and
  top_z the point's own height; never kind surface_spot;
- role place on bare support: kind "surface_spot"; its sigma_z is the depth's own error at the point, and
  where no depth is seen the stated support's own error (as round four's support spot);
- a point with no thing that lies under the stated support takes the support's height (and "doubt" says so): the
  hand meets the stated plane first.
sigma_xy is the spot's error on the level: its error across the ray, and the share of its error along the ray that
lies on the level (a ray that comes in slanted carries its depth error into the plan; a point of two marks a few
millimetres apart is known along the view to metres). It is over 0 wherever an error was seen or stored, and None
for a point that came with none (a typed point, a label stored without one): "doubt" then says that no error is
stated, and no figure stands in for it. "periods" is 0, "looks_in_motion" false, "coarse" false. "said" says what
was taken and names the source; "doubt" is the spot's own list. Nothing here reads truth or names an object.
"answer" (roles pick and part; the design notes C11, the design rulings) is the spot's own: the look shows the pointed place
empty; a tool that gets it returns stopped "answered" at once, with no motion and no control period.
"""

import math

import numpy as np

from rcpx.r2 import place_near, see_cloud, see_spot

ID = "R4-point"
MM = 1000.0          # unit
TOOLS = []
PRACTICES = ""


def _state(ex):
    return ex.state.setdefault("place_point", {"n": 0, "resolved": {}})


def _level(sp, up):
    """The spot's error on the level (module docstring), or None when no error is stated."""
    sig = sp["sigma_m"]
    if sig.get("across") is None:
        return None
    if sig.get("along_ray") is None or sp.get("ray") is None:
        return float(sig["across"])
    rise = float(np.asarray(sp["ray"], dtype=float) @ up)
    return float(math.hypot(sig["across"], sig["along_ray"] * math.sqrt(max(0.0, 1.0 - rise * rise))))


def _across(seen):
    return ("no error is stated for it" if seen["sigma_xy"] is None else
            f"known to {seen['sigma_xy'] * MM:.1f} mm across")


def _said(seen, sp):
    e = sp.get("extent") or {}
    if sp.get("answer"):
        text = "Took nothing: the look shows the pointed place empty, and what it shows near it is listed"
    elif e.get("kind") == "thing":
        n = sp.get("names") or {}
        which = "under your point" if n.get("by") != place_near.WITHIN else \
            f"{n['distance_m'] * MM:.1f} mm beside your pixel (within the look's own error of {n['error_m'] * MM:.1f} mm)"
        text = (f"Took the thing {which}: {e['narrow_m'] * MM:.0f} by {e['wide_m'] * MM:.0f} mm across, its "
                f"top {e['top_z'] * MM:.0f} mm up (base frame), {_across(seen)}")
        if e.get("hollow_m") is not None:
            text += f"; your point lies {e['hollow_m'] * MM:.0f} mm down inside it"
        if sp.get("entry_m") is not None:
            text += "; a place there is entered from above the rim over your point"
    elif seen["kind"] == "surface_spot":
        text = f"Took a bare place on the surface, {_across(seen)}"
    else:
        text = f"Took the point alone ({_across(seen)}): no thing is made out there"
    return text + f". Source: {sp['source']}. No motion, 0 control periods."


def resolve(ex, spot, role="pick", intent=None):
    """A spot to a Seen (module docstring), or see_spot's refusal for a malformed argument."""
    if isinstance(spot, dict) and spot.get("kind") in ("thing", "surface_spot"):
        return spot
    sp = see_spot.spot(ex, spot, role)
    if "refused" in sp:
        return sp
    st = _state(ex)
    st["n"] += 1
    up, sup_z, sup_err = see_spot.support_of(ex)
    e1, e2 = see_cloud.axes_of(up)
    p = np.asarray(sp["point_m"], dtype=float)
    pxy = [float(p @ e1), float(p @ e2)]
    e = sp.get("extent") or {}
    sig, level = sp["sigma_m"], _level(sp, up)
    out = {"id": None, "kind": "thing", "centre_xy": pxy, "top_z": float(p @ up), "sigma_xy": level,
           "sigma_z": None, "outline": None, "height": None, "hollow": None, "depth_inside": None, "wall": None,
           "handle": None, "part_chord": None, "neighbours": [], "pointed_xy": pxy, "looks": list(sp["looks"]),
           "source": sp["source"], "doubt": list(sp["doubt"]), "periods": 0, "looks_in_motion": False,
           "coarse": False, "spot": sp}
    if e.get("kind") == "thing":
        part, wall = e.get("part"), e.get("wall")
        out.update(id=f"S{st['n']}", centre_xy=list(e["centre_xy"]), top_z=float(e["top_z"]),
                   sigma_xy=float(max(level, e["sigma_m"])), sigma_z=float(e["sigma_m"]),
                   outline={"centre": list(e["centre_xy"]), "narrow": e["narrow_m"], "wide": e["wide_m"],
                            "bearing": e["bearing_rad"]},
                   height=float(e["top_z"] - max(e["base_z"], sup_z)), hollow=e.get("hollow_m") is not None,
                   depth_inside=e.get("hollow_m"), wall=None if not wall else wall["thick_m"],
                   part_chord=None if not part else {"width": part["narrow_m"], "bearing": part["bearing_rad"],
                                                     "top_z": part["top_z"], "centre": list(part["centre_xy"])})
        if role == "part":             # a part is worked where it was pointed at: on a thing's side as on its top
            out["top_z"] = float(p @ up)
    else:
        if float(p @ up) < sup_z:      # a drawn surface under the plane the hand meets: the stated support stands in
            out["top_z"] = sup_z
            seen_to = sig.get("along_ray")     # said when the gap is one the spot can tell (a point cut ON the
            if sup_z - float(p @ up) > (math.sqrt(np.finfo(float).eps) if seen_to is None else seen_to):   # support
                out["doubt"].append(f"the point lies {(sup_z - float(p @ up)) * MM:.0f} mm under the stated support: "
                                    "the stated support is taken as its height")
        if role == "place":       # its height error: the depth's own; where no depth is seen, the stated support's
            along = sig.get("along_ray")
            out.update(kind="surface_spot", height=0.0, hollow=False,
                       sigma_z=float(sup_err or 0.0) if along is None else float(along))
    if sp.get("answer"):          # the look shows the pointed place empty: the tool that gets this moves nothing
        out["answer"] = sp["answer"]
    out["said"] = _said(out, sp)
    st["resolved"][st["n"]] = {"look": (sp["looks"] or [None])[0], "role": role, "id": out["id"], "kind": out["kind"]}
    return out


def setup(ex):
    _state(ex)
