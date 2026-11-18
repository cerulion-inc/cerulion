# SPDX-License-Identifier: AGPL-3.0-only
"""What locate decides from its views and from the sheet (package sight), apart from
the moves themselves (rcpx/r2/sight_locate.py):

    find(geo, Fk, camk, matches)          one extra view's match: the clear peak, a wider patch, or a weak or
                                          rivalled peak that the way back confirms
    another(ex, plan, tried, matches)     one more view when the planned ones did not give two matches
    second_scale(...)                     the same point measured again with the next larger patch
    band_px(ex, depth_m)                  the epipolar band, widened by the stated camera pose uncertainty
    identity(part, parts, colour, pc)     what the picture shows about whether the spot IS the named part
    next_call(ex, part, id, parts, d, ident)   push or lift only for a control part whose do line holds the verb
                                          and whose identity is verified; else None and the reason

Nothing here moves the arm, reads simulator ground truth or names a robot or a device.
"""

import math

import numpy as np

from rcpx import colour_words, r3_limits as L, sheet_words
from rcpx.r2 import sight_match as M
from rcpx.r2 import sight_parts as SP
from rcpx.r2 import sight_plan as P
from rcpx.r2 import sight_bounds as SB

MM = 1000.0


def find(geo, Fk, camk, matches):
    """One extra view's match {ok, px, size, ncc, runner, why, weak_ok, wider}. With no match yet: along the naming
    ray's band. After one: in the window where that match's line crosses the naming ray, and when the window holds
    no clear match, along the whole band again, on its own. When the pointed patch gives no clear match (it repeats
    along this view's line, or its peak is under NCC_PEAK) the next larger patches of PATCH_SIZES_PX are tried on
    the same frames, round the same pixel. A peak under NCC_PEAK that stands alone on the line (weak), or one over
    it with a rival elsewhere on the line, counts only when the way back (sight_match.back_check) finds the pointed
    pixel again from it and nothing else; the three-ray miss gate of step 8 then has to pass as for any match."""
    F0, cam0, px, size, o, d, t_near, t_far, band, normals, radial = geo
    a = matches[0] if matches else None

    def lines(m):
        if a is None or not m["ok"]:
            return m
        da = M.line_dist(camk, o, d, float((a["P"] - o) @ d), m["px"])
        oa, dda = a["cam"].ray(*a["px"])
        db = M.line_dist(camk, oa, dda, float((a["P"] - oa) @ dda), m["px"])
        if max(da, db) > band:
            return {**m, "ok": False, "weak_ok": False,
                    "why": f"the match lies {max(da, db):.0f} px off the lines of sight"}
        return m

    if a is not None:
        m = M.window_search([(a["F"], a["cam"], a["px"]), (F0, cam0, np.array(px, dtype=float))], Fk, camk,
                            a["size"], a["P"], half=int(round(M.WINDOW_PX + band - L.EPI_BAND_PX)), normals=normals)
        m = lines({**m, "size": a["size"]})
        if m["ok"]:
            return m
    first = None
    for s in [size] + [x for x in L.PATCH_SIZES_PX if x > size]:
        m = M.band_search(F0, Fk, cam0, camk, px, s, o, d, t_near, t_far, band=int(round(band)), normals=normals)
        m["size"] = s
        if m["ok"]:                  # a clear match of the pointed patch, or of a wider one round the same pixel
            return lines({**m, "wider": s > size})
        if m.get("weak") or (m.get("ncc") or 0.0) >= L.NCC_PEAK:
            # a weak peak, or a peak with a rival elsewhere on this view's line: it counts only when the way back
            # finds the pointed pixel, and only that
            nk = M.surface_normals(camk, camk.ray(*m["px"])[1], radial)
            good, off, _ = M.back_check(F0, Fk, cam0, camk, px, m["px"], s, t_near, t_far, band=int(round(band)),
                                        normals=nk)
            if good:
                return lines({**m, "ok": True, "weak_ok": True, "wider": s > size, "why": ""})
            m = {**m, "ok": False, "why": m.get("why", "no clear match") + (
                "" if off is None else f"; the way back lands {off:g} px from your pixel")}
        first = first or m
    return first


def another(ex, plan, tried, matches):
    """One more view when the planned ones did not give two matches, or None. With one match: the ring about that
    match's point (sight_plan.b_about), a view not tried yet, on the matched view's own side first (the part was
    found there; the other side lost it). With none: a ring view not tried yet, the other turn
    (about the vertical against up and down) than the tried ones first, so the new view's line crosses the patch
    another way. Its paths (from here, and back to the start) pass the same checks as the planned ones."""
    ctx = plan["ctx"]
    if matches:
        alt = P.b_about(ex, matches[0]["view"], matches[0]["P"], ctx, exclude=tried, same_side=True)
        cands = [] if alt is None else [alt]
    else:
        turns = {bool(v["az_deg"]) for v in tried}
        rest = [v for v in plan["ranked"] if not any(v is t for t in tried)]
        cands = sorted(rest, key=lambda v: bool(v["az_deg"]) in turns)       # stable: the other turn first
    q_here = np.asarray(ex._sense().q, dtype=float)
    for c in cands:
        if P.path_check(ex, q_here, c["q"], ctx["pts"], ctx["boxes"], ctx["line"], refs=ctx.get("refs")) is None and \
           P.path_check(ex, c["q"], ctx["q_now"], ctx["pts"], ctx["boxes"], ctx["line"], refs=ctx.get("refs")) is None:
            return c
    return None


def second_scale(F0, cam0, matches, size, o, d, t_near, t_far, normals, u, v):
    """The same point measured again from the frames already taken with the next larger patch (no motion): a
    match on an occluding edge or a shading pattern moves with the patch's size, a match on the part does not.
    The point, or None when the larger patch finds no clear match."""
    sizes = [s for s in L.PATCH_SIZES_PX if s > size]
    if not sizes or len(matches) < 2:
        return None
    s2 = sizes[0]
    a, b = matches[0], matches[1]
    ma = M.band_search(F0, a["F"], cam0, a["cam"], (u, v), s2, o, d, t_near, t_far, normals=normals)
    if not ma["ok"]:
        return None
    pa = M.on_ray(o, d, [a["cam"].ray(*ma["px"])])[0]
    if pa is None:
        return None
    mb = M.window_search([(a["F"], a["cam"], ma["px"]), (F0, cam0, np.array([u, v]))], b["F"], b["cam"], s2, pa,
                         normals=normals)
    if not mb["ok"]:
        return None
    return M.on_ray(o, d, [a["cam"].ray(*ma["px"]), b["cam"].ray(*mb["px"])])[0]


def band_px(ex, depth_m):
    """The epipolar band (px each side): EPI_BAND_PX, widened by the camera pose uncertainty robot://body states
    (camera.pose_uncertainty: its turn, and its shift at the bracket's middle depth), so a camera that is placed only
    to within that still finds the patch inside the band. On the nominal sim it is EPI_BAND_PX."""
    facts = SB.inputs(ex)
    f = float(ex.model.f_px)
    extra = f * math.radians(facts["pu_deg"]) + (f * facts["pu_mm"] / MM / max(depth_m, 0.05))
    return float(L.EPI_BAND_PX) + extra


def identity(part, parts, colour, pc_text):
    """What the picture shows about whether the pointed spot IS the part the label names: {"verified": bool,
    "why": text}. The only evidence sight holds is the sheet's own words against the spot's colour:
    - a part_check warning (the spot has another part's colour, or the label names no part): not verified;
    - a device that holds a part the sheet forbids: verified only when the spot shows a colour the sheet gives the
      named part and no colour it gives a forbidden part (a label is the operator's word, not evidence, and an act
      on a wrong point there would be an act on a forbidden part);
    - else (no forbidden part on the device): not contradicted, which is all the sheet lets the picture say."""
    if part is None:
        return {"verified": False, "why": "the label names no part of robot://devices"}
    if pc_text:
        return {"verified": False, "why": "the spot's colour is not this part's (part_check)"}
    banned = [p for p in parts if p["device"] == part["device"] and p["role"] == "forbidden" and p is not part]
    if part["role"] == "forbidden":
        return {"verified": False, "why": f"robot://devices forbids the {part['name']}"}
    if not banned:
        return {"verified": True, "why": "nothing on this device is forbidden and the spot's colour does not "
                "contradict the label"}
    names = " and the ".join(p["name"] for p in banned)
    mine = SP.colours_of(part)
    theirs = set()
    for p in banned:
        theirs |= SP.colours_of(p)
    if colour is not None and any(colour_words.near(colour, c) for c in mine) and not any(
            colour_words.near(colour, c) for c in theirs):
        return {"verified": True, "why": f"the spot is {colour}, a colour robot://devices gives the {part['name']} "
                f"and not the {names}"}
    return {"verified": False, "why": f"robot://devices forbids the {names} on this device and gives no colour "
            f"that tells the {part['name']} from it in a picture: the label alone is not evidence"}


def next_call(ex, part, entry_id, parts, d, ident=None):
    """(next, why not): next is push {target} or lift {target} only when the part is a control part,
    its do line's verbs hold that verb (sheet_words.do_line with the tools listed here), and the spot's identity is
    verified (identity). Otherwise None with the reason: sight never offers an act on a cue, fixed or forbidden
    part, on a part whose do line names no such act, or on a spot it cannot tell from a forbidden part."""
    if part is None:
        return None, "the label names no part"
    if part["role"] != "control":
        return None, f"the {part['name']} is not a part to act on ({part['role']})"
    if ident is not None and not ident.get("verified"):
        return None, ident.get("why") or "the spot's identity is not verified"
    have = set(ex.primitive_names()) if hasattr(ex, "primitive_names") else set()
    declared = ex.cfg.get("r3_lift_declared")          # the run's own setting; no other package is read for it
    if declared is None:
        declared = (getattr(ex, "context", None) or {}).get("r3_lift_declared")
    lift_ok = "lift" in have and bool(declared)
    tools = {t for t in ("push", "lift") if t in have and (t != "lift" or lift_ok)}
    line = sheet_words.do_line(part, d, tools) if d else None
    verbs = (line or {}).get("verbs") or []
    if "lift" in verbs and "lift" in tools:
        return {"tool": "lift", "args": {"target": entry_id}, "why": "the sheet asks for it to go up"}, ""
    if "push" in verbs and "push" in tools:
        return {"tool": "push", "args": {"target": entry_id}, "why": "the sheet's do line for this part is push"}, ""
    return None, "the do line of this part names no push or lift here (robot://devices; tool_help lists the do line)"


__all__ = ["another", "band_px", "find", "identity", "next_call", "second_scale"]
