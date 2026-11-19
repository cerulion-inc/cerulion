# SPDX-License-Identifier: AGPL-3.0-only
"""After a take that did not hold, of the pick of package place: what the look shows.

A pick takes ONCE. A second take by itself, where a fresh look read the spot anew, held once in 25 tries and
struck in 4, so it is off the default path. What stays is the seeing. A
take that did not hold comes back with what ONE fresh look of the fixed camera the spot was pointed in shows there
(no motion, no control period), and the operator decides:

    shows(ex, seen, take) -> {"look": id or None, "thing": True | False | None, "same": True | False | None,
                              "said": one sentence, "ready": the ONE call that takes it again there, or None}

- thing True: a thing is made out at the spot in the fresh look; "same" says whether its point lies inside what
  the first reading showed (its outline grown by both errors, its top no higher than the first by more than both
  readings resolve); "ready" is the same pick pointed at that look's pixel.
- thing False: the look shows the spot and makes out nothing standing there.
- thing None: the spot could not be read again (it came with no look of a fixed camera, the look was refused, the
  robot's own body hides it): "said" tells which.

Nothing here moves the robot, reads ground truth or names a robot. Lengths m.
"""

import math

import numpy as np

from rcpx.r2 import place_grasp
from rcpx.r2.place_grasp import HALF, MM


def _ratios():
    from rcpx import bounds_ratios as R
    return R


def inside(first, new):
    """None when the new reading (place_grasp.read_seen) lies inside what the first showed, else why not: its
    pointed point within the first outline grown by both errors, its top no higher than the first top by more
    than BAND times the largest error of the two readings."""
    R = _ratios()
    if new["narrow"] is None:
        return "the fresh look makes out no thing where the first did"
    at = new["pointed"] if new["pointed"] is not None else new["centre"]
    d = np.asarray(at, dtype=float) - np.asarray(first["centre"], dtype=float)
    u = np.array([math.cos(first["bearing"]), math.sin(first["bearing"])])
    err = first["sigma_xy"] + new["sigma_xy"]
    if abs(float(d @ u)) > HALF * first["narrow"] + err or \
            abs(float(d @ np.array([-u[1], u[0]]))) > HALF * first["wide"] + err:
        return (f"the fresh look reads the spot {float(np.linalg.norm(d)) * MM:.0f} mm from the middle of what the "
                "first showed, outside it")
    over = new["top_z"] - first["top_z"]
    if over > R.BAND * max(first["sigma_z"] or 0.0, new["sigma_z"] or 0.0, err):
        return f"the fresh look reads a top {over * MM:.0f} mm over the first: not the thing the first take was for"
    return None


def fresh(ex, seen):
    """(a Seen read again at the first reading's own point in a fresh look of the fixed camera it was pointed in,
    how), or (None, why none): no motion, no control period. The Seen carries the pixel it was read at ("at":
    {"look", "px"}). None when the spot came with no look of a fixed camera, when the look cannot be taken, or
    when the pixel shows the robot's own body."""
    sp = seen.get("spot") if isinstance(seen, dict) else None
    if not isinstance(sp, dict) or sp.get("point_m") is None:
        return None, "the spot came with no look of a fixed camera to read it again in"
    try:
        from rcpx.r2 import place_point, scene_camera, see_spot
        for key in sp.get("looks") or []:
            _, rec, fixed = see_spot.look_of(ex, key)
            if rec is None or not fixed:
                continue
            got = scene_camera.scene_look(ex, rec["camera"], "pick: the spot again, after a take that did not hold")
            if "refused" in got:
                return None, f"the fresh look was refused ({got['refused'].get('check')})"
            now = see_spot.look_of(ex, got["look_id"])[1]
            view = see_spot.view_of(now)
            x, y, depth = view.project(np.asarray(sp["point_m"], dtype=float))
            if not (depth > 0.0 and view.inside(x, y)):
                return None, "the fresh look does not show the spot"
            cloud = see_spot.cloud_of(ex, now)
            if cloud is not None and bool(cloud[3][int(math.floor(y)), int(math.floor(x))]):
                return None, f"the robot's own body hides the spot in the fresh look ({got['look_id']})"
            at = {"look": got["look_id"], "px": [float(x), float(y)]}
            again = place_point.resolve(ex, at, "pick")
            if "refused" in again:
                return None, f"the fresh look could not be read ({again['refused'].get('check')})"
            return dict(again, at=at), (f"read the spot again in a fresh look ({got['look_id']}, camera "
                                        f"{rec['camera']}, 0 control periods): {str(again.get('said'))[:200]}")
    except Exception as exc:        # no seeing package in this build, or a look that cannot be read: said
        return None, f"the spot could not be read again ({type(exc).__name__})"
    return None, "the spot came with no look of a fixed camera to read it again in"


def shows(ex, seen, take):
    """What the look shows at the spot after a take that did not hold (module text). take: the take's own record
    (its first reading under "S")."""
    again, how = fresh(ex, seen)
    if again is None:
        return {"look": None, "thing": None, "same": None, "ready": None,
                "said": f"what stands at the spot now is not seen ({how})"}
    look, S = again["at"]["look"], place_grasp.read_seen(again)
    if S["narrow"] is None:
        return {"look": look, "thing": False, "same": None, "ready": None,
                "said": f"look {look} (taken now, 0 control periods) makes out no thing at the spot"}
    first = take.get("S") or {}
    why = inside(first, S) if first.get("narrow") is not None else None
    same = None if first.get("narrow") is None else why is None
    planned = take.get("planned") if isinstance(take.get("planned"), dict) else {}
    ready = {"tool": "pick", "args": {"pick": again["at"]},
             "periods": planned.get("periods")}   # the first take's own count (the same take, pointed there)
    return {"look": look, "thing": True, "same": same, "ready": ready,
            "said": (f"look {look} (taken now, 0 control periods) shows a thing at the spot, "
                     f"{S['narrow'] * MM:.0f} by {S['wide'] * MM:.0f} mm across, its top at z {S['top_z'] * MM:.0f} mm"
                     + ("" if same is None else ", where the first look showed it" if same else f" ({why})")
                     + ": the same pick pointed there takes it again")}
