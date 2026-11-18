# SPDX-License-Identifier: AGPL-3.0-only
"""Package sight (I37; SPEC-R3 4.5): point once and the robot measures where it is; look at a thing from a named
side; survey candidates that fit the device.

Tools (the vnext surface; each result leads with said and ends with next, SPEC-R3 3.1):
- locate(look, px, part, note="", on="space", detail="brief"): rcpx/r2/sight_locate.py. The operator points at a
  part in a look (or a survey_N or around_N sheet); the robot takes two more views itself (rcpx/r2/sight_plan.py),
  finds the patch again (rcpx/r2/sight_match.py) and stores a located entry (SPEC-R3 3.2) with its uncertainty, or
  the line of sight alone (depth ray_only) when the pictures cannot fix the depth. on "table" or a contact id meets
  that plane instead, with no motion.
- view(target, side="robot_side", distance_mm=None, elevation_deg=None, detail="brief"): rcpx/r2/sight_view.py.
- the survey post hook (rcpx/r2/sight_parts.py survey_post): every candidate gains fits (its measured size beside
  the device's rough size; the verdict is withdrawn, value is always null: a survey box did not tell the device
  from other things where it was measured), the result best_fit (where to look first, sure false, the others in
  order) and a said, ex.state["sight"]["candidates"] keeps the boxes, and the survey sheet's tile map is kept.

No act on an unverified point (rcpx/r2/sight_find.py identity, next_call): locate's next is push or lift only for a
control part whose do line holds that verb and whose identity the picture verifies. On a device that holds a part
the sheet forbids, the operator's label is not evidence: without a colour the sheet gives the named part (and not a
forbidden one) at the spot, next is null, no_act says why, said offers no act, and the located entry carries
data.identity {verified false, why} for whoever acts on it later. A spot with another part's colour gets no act.

ex.state["sight"] (SPEC-R3 3.3): candidates, plan_camera(target_mm, side, distance_mm=None, elevation_deg=None) ->
{q or None, got_side, why} (dry: act's witness view), sheets {id: {size, cols, tile_px, tiles, path, known}}.

Safety: every motion is the runtime's own (a nested primitive runs bare), so each one first passes ex.gates (the
dark gate, the person gates, the path gates: SPEC-R3 3.8), then runs as a guarded, governed joint move whose
guard is the core's own for a look move; the links and the tool keep the bounds' clearance for a path, at the
speed the path check solves, from every point robot://world holds and every survey box, and the tool clear of the
line of sight being
measured. A view that reads dark ends locate; the camera returns by retreat. locate and view are moves toward the
world: the waiting package's dark gate covers them (setup adds their names).

Depends on (each checked at call time; without it the tool says so): selfsee (first_hit, on_body: "own body not
checked"), act (push: next names push only when loaded), witness (look like: view around's next). Lift is named
only when the run's own setting r3_lift_declared is true (ex.cfg, else ex.context); no profile is read. C6 World "box" entries are written when rcpx.context.KINDS has "box"; until then the boxes live in
ex.state["sight"]["candidates"] and sight's own clearance checks read them there.

Part B (the core lane) calls nothing here; it registers: FREE_TOOLS unchanged (locate and view are charged calls),
the brief layer's KEEP_BLOCKS (fits, best_fit, part_check, said, next, no_act).
Nothing under this package reads simulator ground truth or names a robot or a device.
"""

from rcpx import surfaces as S
from rcpx.control_base import primitive
from rcpx.r2 import sight_bounds as SB
from rcpx.r2 import sight_locate as SL
from rcpx.r2 import sight_parts as SP
from rcpx.r2 import sight_view as SV

ID = "I37"


def _state(ex):
    return ex.state.setdefault("sight", {"candidates": [], "sheets": {}, "locates": 0, "surveys": 0, "arounds": 0})


class Mixin:
    @primitive
    def locate(self, look, px, part, note="", on="space", detail="brief", clear=None):
        return SL.run(self, look, px, part, note=note, on=on, detail=detail, clear=clear)

    @primitive
    def view(self, target, side="robot_side", distance_mm=None, elevation_deg=None, detail="brief", clear=None):
        return SV.run(self, target, side=side, distance_mm=distance_mm, elevation_deg=elevation_deg, detail=detail,
                      clear=clear)


def setup(ex):
    st = _state(ex)
    st["plan_camera"] = lambda target_mm, side, distance_mm=None, elevation_deg=None: SV.plan_camera(
        ex, target_mm, side, distance_mm, elevation_deg)
    ex.add_post(lambda name, kwargs, result: SP.survey_post(ex, result), names=["survey"])
    w = ex.state.get("waiting")
    if isinstance(w, dict) and isinstance(w.get("toward_world"), set):
        w["toward_world"].update(("locate", "view"))      # the dark gate covers the runtime's camera moves


_LOOK = {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}],
         "description": "the look id that shows the part (L3), or a sheet id a survey or view around returned "
                        "(survey_1, around_1)"}

TOOLS = [
    S._tool(
        "locate",
        "Point at a part in a picture: the look, the pixel and the part's name from robot://devices. The robot takes "
        "two more pictures itself and measures where the part is (guarded camera moves, back where it was after), "
        "and stores it as a located entry with its uncertainty; when the pictures cannot fix the depth it stores the "
        "line of sight and says so (push then feels along it). It warns when the spot looks like another part. "
        "Example: locate {\"look\": \"L3\", \"px\": [412, 233], \"part\": \"button\"}.",
        S._obj({"look": _LOOK,
                "px": {"type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 2,
                       "description": "[u, v] pixel of the part in that picture, from the top-left"},
                "part": S._str("the part's name as robot://devices gives it (button, lamp, lever ...), or other "
                               "with a note"),
                "note": S._str("what the point is, when part is other"),
                "on": S._str("space (default: measured from three views), table (where the line meets the table) "
                             "or a contact id (the level face that contact touched)"),
                "clear": SB.CLEAR_ARG,
                "detail": S._str("brief (default) or full: full adds the views, matches and misses",
                                 ["brief", "full"])}, required=["look", "px", "part"]),
        {"look": "L3", "px": [412, 233], "part": "button"},
        "measure where the button in look L3 at pixel (412, 233) is",
        brief="Point at a part in a look; the robot measures where it is from two more views."),
    S._tool(
        "view",
        "Moves the camera to see a thing from one side: robot_side, left, right, far_side, above, front (the side a "
        "located part faces) or around (every side it can reach, one picture). The target is a located or contact "
        "id, a part name or a survey candidate (c2). The moves are guarded and keep clear of what the robot knows "
        "stands there. Use it when the part you need is in no picture. Example: view {\"target\": \"c2\", "
        "\"side\": \"around\"}.",
        S._obj({"target": {"anyOf": [{"type": "string"}], "description": "a located or contact id, a part name, or "
                           "a survey candidate id (c1, c2, ...)"},
                "side": S._str("robot_side (default), left, right, far_side, above, front or around",
                               list(SV.SIDES)),
                "distance_mm": {"type": "number", "description": "camera distance from the target in mm (default: "
                                "the nearest at which the whole thing is in the picture; a distance the arm cannot "
                                "reach is answered with what it can)"},
                "elevation_deg": {"type": "number", "description": "how far the camera looks down, in degrees "
                                  "(default 25, above 75)"},
                "clear": SB.CLEAR_ARG,
                "detail": S._str("brief (default) or full", ["brief", "full"])}, required=["target"]),
        {"target": "c2", "side": "around"}, "look at candidate c2 from every side the arm reaches, on one sheet",
        brief="Put the camera on a named side of a thing, or around it."),
]

PRACTICES = ("locate: point at the part in a look that shows it and give its name; the robot measures where it is.\n"
             "view puts the camera on a named side; around shows every side it can reach.")

__all__ = ["ID", "Mixin", "PRACTICES", "TOOLS", "setup"]
