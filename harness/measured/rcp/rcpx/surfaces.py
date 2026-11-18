# SPDX-License-Identifier: AGPL-3.0-only
"""The operator surface (SPEC section 3): the tool tables. Argument checking and output filters live in
rcpx/surface_args.py, the list answers in rcpx/surface_list.py, what a session adds in rcpx/surface_session.py;
every name of the three is re-exported here, so `from rcpx import surfaces` is the one import.

Two surfaces:
- VNEXT: every primitive of the control section (rcpx/control.py) plus verify. Each entry has a
  description written for a model that has never seen this robot, a "brief" (the one line the compact list shows),
  an argument schema and one worked example.
- V0: the plain-manifest baseline: read_state, move_joints (degrees by joint name), gripper, snapshot.

NO NUMBER OF THE ROBOT LIVES HERE (SPEC-R4 A5, A6). The tables are written in WORDS: where the runtime knows a
number (the contact threshold, the gentle first-touch bound, the robot's full speed ...) the text holds one of the
slot phrases of rcpx/surface_session.py SLOTS, and a session puts its own derived value and where it came from
behind the phrase when it serves a list (for_session; rcpx/server.py Session._list). A listing built with no
executor holds the words alone. Nothing is read from the executor's defaults when this module loads.

Schema language and the constructors of a schema (_num, _pos, _obj, _tool ...): rcpx/surface_args.py. A schema
carries no derived end: a speed, force, travel, time or count argument has its type and its sign ("x-positive")
and no range; a value the robot cannot do is refused by the primitive, with the derived end in the refusal.
Ranges that remain are geometry or shares (an angle to turn, the gripper's opening as a share, a pixel index).
guarded is retired (SPEC-R4 A6): the move tools still accept it ("x-retired": no listing shows it) and it changes
no bound. clear, expect and max_n are the operating model's words ("x-intent"). JOINT stands for a joint name in
an example; for_robot() puts the robot's first joint name there once a session knows it.

What a session adds when it opens (rcpx/server_edge.py): the session argument detail every call takes
(DETAIL_VALUES; not in any schema), the dark rule its refusal hints carry (dark_rule over its own tool list), no
gripper tool listed on a robot whose tool is a fixed finger. FREE_TOOLS are not charged against the server's call
count: the about text says which calls are charged without naming a tool, and each free tool's
description says Free.
"""
import copy

from rcpx import surface_session as _session
from rcpx.surface_args import (DIRECTION_NAMES, IMAGE_EXT, TRUTH_MARKER, _bool, _direction, _fmt,  # noqa: F401
                               _int, _is_type, _jsonish, _num, _obj, _pos, _show, _str, _tool, _type_fits, _vec3,
                               check, check_args, coerce, describe, frame_paths, normalize_args, public_schema,
                               scrub_truth)
from rcpx.surface_list import (CLAIM_HOW, COMPACT_BYTES, brief, compact, named, printed_bytes,  # noqa: F401
                               takes, type_text)
from rcpx.surface_session import (DETAIL_VALUES, FREE_TOOLS, SLOTS, WAIT_TOOL, WORLD_WAIT_KEYS,  # noqa: F401
                                  dark_rule, is_free)


# ---------------------------------------------------------------------------------------------------------------
# What the text says about the runtime: rules in words. A slot phrase (SLOTS) stands where a session knows a number.

THRESHOLD, ONE_READING, HAND = SLOTS["f_thr"], SLOTS["f_one"], SLOTS["f_hand"]
TOOL_RATING, RIGID, FULL_SPEED = SLOTS["f_tool"], SLOTS["rigid"], SLOTS["v_full"]
LIMIT_COST = SLOTS["limit_cost"]
# the design rulings as amended (the maintainer's answer to question 29): a limit only a model typed is a request until its record
# earns it more; the words say so where max_n and gentle are offered
TYPED_LIMIT = "A limit you type is a request, not a guarantee: the result says kept or PASSED with its figure."

CONTACT_STOP = (
    f"The contact stop is armed on every move: it ends with stopped contact when the force estimate at the tip "
    f"passes {THRESHOLD} on two readings (one reading over {ONE_READING}), or when a joint feels a push over its own "
    f"threshold (any link, the arm touching itself included); a push past what a joint or {TOOL_RATING} allows "
    "ends it with stopped cap; after either the arm eases. A part that gives under the threshold is pushed, not "
    "felt. The result says what the move met below the threshold: peak_force_n, resisted_mm, first_resist_mm, "
    f"brushed and moved_part (the tip went on against resistance by more than {RIGID}).")
SPEED_RULE = (
    f"Ask nothing for speed: every move runs at {FULL_SPEED} with the contact stop armed; clear: true is recorded "
    "and changes no speed. A speed you state slows the whole move. A stated limit (max_n on the call, the sheet of a "
    "thing you name, the site) slows it where a contact can begin: near what a look shows and in space that no look "
    "of the scene as it stands, no passage of the arm and no sheet shows free (with none, the whole way). The "
    "result's limit block says kept or "
    "PASSED with the figure, and limit.cost the control periods the limit added. A result that moved carries "
    "bounds (the bound that set the pace and what would lift it) and periods. expect: \"contact\" says that "
    "contact is meant at the end of the move.")
JOINT_PATH = ("move_joints turns every joint at once in joint space, so the tip's path between the two poses bows "
              "a little (a few mm) off a straight line.")
JOINT_SPEED = ("speed is a share of full joint speed (every joint at most at its rated speed, robot://body "
               "joints[].speed_limit, scaled by the joint that has the furthest to turn): leave it out for full "
               "speed, state a lower share for a slower move.")
# The dark rule, one text in one place. DARK_NOTE is what every surface with a dark gate says in its
# look, move and touch descriptions; dark_rule() builds the hint a dark refusal carries from the session's own tools.
DARK_NOTE = ("A dark look decides nothing: while the newest look reads dark, every move toward the world (move_tip, "
             "move_joints, touch, press, orient_tool and the like) is refused with check dark, because a push "
             "nobody can see is not safe; any new look reads the light again, and park, home, retreat and ease "
             "still run. When the light does not come back, claim success false and say that it is dark: in a room "
             "that stays dark that is the right ending, and working blind is not.")
DARK_SHORT = "While the newest look reads dark this move is refused (check dark): see look."
PRESS_STOPS = {  # every value press can return in stopped (the test reads them back out of the code)
    "force": "the push reached the force asked or offered (a nudge ends there, firm holds it a moment)",
    "travel": "the tip went the whole travel under the ceiling",
    "lead": "the command ran as far ahead of the tip as it may under the ceiling: the part gives way, or the arm "
            "cannot build that force here",
    "stall": "rounds mode: the tip stopped advancing and the force stopped rising",
    "slipped": "the tip slid sideways off the push line under load, past the tool tip's half width",
    "cap": f"a tip force past what the arm or {TOOL_RATING} allows, or a joint at its rating (why names it)",
    "time": "the push passed its planned time with none of the above: a soft part still moving",
    "gave": "nudge mode: the part let go (a give) and the stroke ended there",
    "off_axis": "the tip left its line while the push was short of the force: the part takes the push across its "
                "line (a hinge, a stop or an edge of the tool)",
}
GIVE_AFTER = ("what the tool does after a give is the session's setting give_lift: none, the tool stands still "
              "where the part let go (the part moves on away along the push, so after a lever or a lid the finger "
              "may still be in what the part sweeps back through: move it clear before the part comes back); back, "
              "the tool goes back along its stroke to where the part first resisted and clear of it by the tool's "
              "own width (lifted says so)")
JOINT = "JOINT_NAME"  # stands for a joint name in an example until for_robot() knows the robot's
HALF_TURN_DEG = 180  # an angle to turn about a tool axis: geometry, not a limit of the robot (ledger CR-023, kept)


def _turn(text):
    return _num(-HALF_TURN_DEG, HALF_TURN_DEG, text)


FREE_NOTE = (" Free: it computes on stored looks or the model, moves nothing and is not charged against the "
             "server's call count; a harness that limits your replies still counts the reply.")
DIM_RULE = ("A dim look can locate large things; it decides a small state only from close (the part large in the "
            "frame) or by a before and after pair taken from the same pose: say so in the claim.")
_FRAME_MOVE = _str("frame the offset is in: base (x forward, y left, z up), tool (x along the finger, z the "
                   "hand's up) or camera (x toward image right, y toward image bottom, z along the view)",
                   ["base", "tool", "camera"])
_FRAME_PUSH = _str("frame the direction is in: base or tool", ["base", "tool"])
_ID = {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}], "description": "the look_id a look returned"}
_SPEED = _pos("a share of the robot's full joint speed, more than 0 and at most 1; leave it out for full speed")
_GRIP = {"anyOf": [{"type": "number", "minimum": 0.0, "maximum": 1.0},
                   {"type": "string", "enum": ["open", "close"]}],
         "description": "opening as a fraction (0 closed, 1 fully open) or \"open\" or \"close\""}
# The operating model's words (SPEC-R4 A6) and the retired argument. One line of text each.
_CLEAR = {"anyOf": [{"type": "boolean"},
                    {"type": "object", "properties": {"look": {"anyOf": [{"type": "string"}, {"type": "integer"}]}},
                     "required": ["look"], "additionalProperties": False}],
          "x-intent": True,
          "description": "your word that the way is clear, after a look: true (your newest look) or {\"look\": id} "
                         "(that look). It is recorded in the result and changes no speed and no guard: with it or "
                         "without it the move runs at the robot's own speed with the contact stop armed. Refused "
                         "only for a look id this session does not hold"}
_EXPECT = {"type": "string", "enum": ["contact"], "x-intent": True,
           "description": "\"contact\": contact is meant at the end of this move"}
_TOL = {**_pos("what you need of the end pose, mm: stated, the end is waited for and corrected to it"),
        "x-intent": True}
_MAX_N = {**_pos("a stated limit, newtons: the most any contact may reach. " + TYPED_LIMIT), "x-intent": True}
_MAX_N_PRICED = {**_pos(f"a stated limit, newtons: the most any contact of the call may reach; it slows every "
                        f"stretch where a contact can begin, the carry too where no look shows its way free: "
                        f"{LIMIT_COST} {TYPED_LIMIT}"), "x-intent": True}
_RETIRED = {"type": "boolean", "x-retired": True,
            "description": "retired: the contact stop is always armed; look, then say clear: true"}

VNEXT_TOOLS = [
    _tool("move_joints",
          "Move the joints to target angles in degrees: by joint name (names and limits in robot://body; joints "
          "you leave out stay put), as a full list in joint order, or as a named pose (home, park or standard). "
          + JOINT_PATH + " " + JOINT_SPEED + " " + SPEED_RULE + " " + CONTACT_STOP + " " + DARK_SHORT
          + " Returns asked, achieved (measured), stopped (goal, short, contact, cap or refused), contact and "
          "tip_mm.",
          _obj({"q_deg": {"anyOf": [{"type": "object", "additionalProperties": {"type": "number"}},
                                    {"type": "array", "items": {"type": "number"}},
                                    {"type": "string"}],
                          "description": "targets in degrees by joint name, a full list in joint order, or a "
                                         "named pose"},
                "speed": _SPEED, "clear": _CLEAR, "expect": _EXPECT, "tolerance_mm": _TOL, "guarded": _RETIRED},
               ["q_deg"]),
          {"q_deg": "home"}, "go to the named home pose at full speed where the way is free",
          brief="Move joints to target angles in degrees (by name, full list or a named pose)."),
    _tool("move_tip",
          "Move the tool tip in a straight line by a relative offset in mm in frame base, tool or camera, using "
          "every joint. " + SPEED_RULE + " " + CONTACT_STOP + " Returns asked_mm, achieved_mm (measured), stopped "
          "(goal, short, contact, limit, cap or refused), contact and tip_mm; short: the arm stood short of the goal by "
          "more than its tolerance (told_mm says where). stopped limit means a joint limit or the "
          "edge of reach ended the line early (why names it): try keep_orientation false, or a shorter line. "
          + DARK_SHORT,
          _obj({"dx": _num(text="offset along the frame's x, mm"),
                "dy": _num(text="offset along the frame's y, mm"),
                "dz": _num(text="offset along the frame's z, mm"),
                "frame": _FRAME_MOVE,
                "speed_mm_s": _pos("a tip speed in mm per second, only when you want a lower one than the "
                                   "robot's own: it stands; one over the robot's own speed changes nothing. "
                                   "Leave it out for full speed"),
                "keep_orientation": _bool("hold the hand's orientation while moving (default true)"),
                "clear": _CLEAR, "expect": _EXPECT, "max_n": _MAX_N_PRICED, "tolerance_mm": _TOL,
                "guarded": _RETIRED}),
          {"dx": 0, "dy": 0, "dz": 150, "frame": "base", "clear": True},
          "after a look that showed the way up clear: raise the tip 150 mm at full speed, the contact stop armed",
          brief="Move the tool tip in a straight line by a relative offset in mm; fast where the way is free."),
    _tool("orient_tool",
          "Turn the hand about the tool tip (the tip stays where it is; tip_moved_mm reports any drift) by yaw, "
          "pitch and roll in degrees. The turns are made about the tool's own axes in this order: yaw about tool z "
          "(the hand's up), then pitch about the new tool y, then roll about the new tool x (the finger): "
          "R_new = R_start Rz(yaw) Ry(pitch) Rx(roll). Positive yaw swings the finger toward tool y (its left); "
          "positive pitch tips the finger toward tool -z (a level finger dips down); positive roll leans the "
          "hand's up toward tool -y (its right). relative true starts from the present orientation; relative "
          "false starts from the tool's orientation at the home pose. Speed, the contact stop, clear and expect "
          "are as move_tip's. Returns asked_deg, turned_deg, error_deg, tip_moved_mm, stopped (goal, contact, "
          "limit, cap or refused) and tip_mm. An arm with fewer than six joints cannot reach every orientation: "
          "stopped limit with a small turned_deg means this arm cannot turn that way from here. " + DARK_SHORT,
          _obj({"pitch_deg": _turn("turn about the tool's y axis, degrees; applied after yaw"),
                "roll_deg": _turn("turn about the tool's x axis (the finger), degrees; applied last"),
                "yaw_deg": _turn("turn about the tool's z axis, degrees; applied first"),
                "relative": _bool("angles start from the present orientation (default true); false: from the "
                                  "tool's orientation at the home pose"),
                "clear": _CLEAR, "expect": _EXPECT, "tolerance_mm": _TOL}),
          {"pitch_deg": 15}, "tip the finger 15 degrees toward the hand's underside, about the tool's y axis",
          brief="Turn the hand about the tool tip by yaw, pitch, roll in degrees (tool axes)."),
    _tool("touch",
          "Feel for a surface: move the tip along a direction until the force on it reaches force_n (one reading "
          f"is enough) or max_mm runs out; with no force_n it ends at {HAND}, never under {THRESHOLD}. "
          "The way in runs at the robot's own speed through space known free (a look of the scene as it stands, "
          "the arm's own passage) and, from where a contact can begin, at the speed solved for the force the touch "
          "ends at (never slower than this arm can tell a contact); where nothing is known free the whole way runs "
          "at that speed, and the band block prices it. The result says the ending force, the speed of the way in "
          "and the peak the arrival made: a part that gives under that peak can be worked by the touch (a look "
          "after it shows it, 0 control periods). Returns contact, point_mm (base frame "
          "mm: where the touching tool surface was when the force passed the threshold), surface (the tool "
          "surface that took it, a face robot://body lists), at_tip, fit_residual (the share of the joint torques "
          "a single push there leaves unexplained: 0 is a clean fit), source (tool; arm or self when a link of "
          "the arm or the arm itself made the stop: then point_mm is not measured and nothing is stored), "
          "normal_est (the way the surface pushed back, friction included), force_n, travelled_mm and stopped: "
          "contact, cap, moved_part (a part gave way under the touch, which then stopped pushing), goal (max_mm "
          "ran out, contact false: felt nothing), limit or slipped. It also says what it met below the threshold: "
          "peak_force_n, resisted_mm and moved_part; a part lighter than the threshold may have been pushed along "
          "unfelt. Where the peak of the arrival cannot be resolved the result says so. A contact the tool made is "
          "stored in robot://world with what the thing bore and its position error. "
          + DARK_SHORT,
          _obj({"direction": _direction("which way to move"),
                "max_mm": _pos("give up after this many mm"),
                "frame": _FRAME_PUSH,
                "force_n": _pos(f"the force the touch ends at, newtons. Default: {HAND}. "
                                "A smaller number makes the way in slower, never slower than this arm can tell a "
                                "contact; a larger one faster. To bound the strike itself state max_n"),
                "max_n": _MAX_N, "clear": _CLEAR},
               ["direction", "max_mm"]),
          {"direction": "down", "max_mm": 60, "frame": "base"},
          "feel straight down for up to 60 mm and report where the tip met a surface",
          brief="Feel along a direction until the tip meets a surface. A light push, not a look.",
          pass_detail=True),
    _tool("press",
          "Push on something you have touched, along direction. The force, in one line: it rises in one continuous "
          "ramp from the force the finding touch already put on the part, never in a jump, and ends when the part "
          "moves, when it stands still under the rising force (a stop or a stiff point: one press cannot tell "
          "which), or at the ceiling: force_n when you state it, else the sheet's force "
          "for the part, else the most this part or another part of the same device has been seen to need; "
          "max_n bounds the ramp from above; nothing ever passes what the arm or the tool is rated for here. With "
          "no force stated, no sheet and nothing this part or its device has shown, the ramp rises until the part "
          "moves or stands still, up to what the arm can push here, and the result says which ceiling it ran to; "
          "state max_n to bound it. The travel: travel_mm as you "
          "state it, else the sheet's travel, else until the part gives or stands still. The base check is due "
          "when the push, times what the base gives per newton (measure_base_give), could shift the stored points "
          "by more than their own error, and after a cap stop; with a standard look reference set the arm then "
          "leaves the part the way it came, takes a standard look, compares it and comes back (base_moved says "
          "what it found). Never on a base known to be fixed: robot://body says base.kind "
          "fixed or measure_base_give read rigid. base_check defer leaves the check for later "
          "(the result says it is due); skip leaves it out. Mode nudge (for buttons and spring levers; it ends at "
          "a give, and " + GIVE_AFTER + "), firm (one steady push, held a moment at the force) or rounds (load "
          "with the joints near the base, deliver with the far ones; rounds go on while a round still gains travel "
          "or force, or up to max_rounds). peak_force_n is the force along the push, "
          "peak_force_total_n the whole force, over every leg. Returns travel_mm (along_push_mm and "
          "across_mm split it), peak_force_n, peak_force_total_n, reached_force, gave, slipped, rounds, slack_mm, "
          "stopped, base_moved, resisted_mm, first_resist_mm, moved_part (the tip travelled against resistance: a "
          "part gave way or slid), sprang_back (the part followed the tool back as the push eased), "
          "tip_mm_at_stop, still_touching, held_s and ref (for verify by_sense press). gave is true when the force fell below half its peak while the tip went on by more than "
          f"{RIGID}: a snap. A spring button can click with no give: a look shows it (0 control periods). stopped is one "
          "of: " + "; ".join(f"{k} ({v})" for k, v in PRESS_STOPS.items()) + ". Every press ends with a release: "
          f"the tool backs off along minus the push until it reads under {THRESHOLD}; eased is true only when "
          "that holds, and eased false (with eased_why) means something still holds against the tool. "
          "force_after_n is what the tip feels at the end. " + DARK_SHORT,
          _obj({"direction": _direction("which way to push"),
                "travel_mm": {**_pos("how far to push, mm, when you want to state it"), "x-offered": True},
                "force_n": {**_pos("the force to reach, newtons, when you want to state it"), "x-offered": True},
                "max_n": {**_pos("a stated limit, newtons: the most the ramp and any contact may reach. "
                                 + TYPED_LIMIT),
                          "x-intent": True},
                "mode": _str("nudge, firm or rounds (default nudge)", ["nudge", "firm", "rounds"]),
                "max_rounds": _pos("rounds mode only: a count of load and deliver rounds, when you want one",
                                   "integer"),
                "frame": _FRAME_PUSH,
                "base_check": _str("auto (default): the base check runs when it is due; defer: not now, the "
                                   "result says it is due; skip: not for this push", ["auto", "defer", "skip"]),
                "clear": _CLEAR},
               ["direction"]),
          {"direction": "up", "mode": "nudge", "frame": "base"},
          "nudge upward, as for a spring lever, until the part gives or stands still; RCP offers the force from "
          "the part's sheet or from what the part has shown, and the result names it",
          brief="Push along a direction in one force ramp until the part moves (nudge, firm or rounds).",
          pass_detail=True),
    _tool("reach_check",
          "Say whether the tip can reach a point (base frame, mm), optionally arriving along an "
          "approach direction (the way the tip travels as it arrives). Returns reachable, margin_mm (how much "
          "further along that direction the tip could still go, searched to the edge of reach), short_by_mm and "
          "direction when it is out of reach, leading (the tool face that would lead along the approach: when "
          "it is not the tip, the push would be made with that face), force_n (how hard the arm can push there "
          "by its own ratings; the tool's own rating stands beside it) and q_deg, one arm posture that reaches "
          "the point. Another call may come back on a different posture: a straight stroke is planned from "
          "the posture the arm stands in." + FREE_NOTE,
          _obj({"point_mm": _vec3(text="the point [x, y, z] in base frame, mm"),
                "approach": _direction("the direction the tip would travel when it arrives (optional)")},
               ["point_mm"]),
          {"point_mm": [250, 0, 80], "approach": "down"},
          "can the tip arrive at x 250, y 0, z 80 mm moving downward, and how hard can it push there",
          brief="Before a stroke: can the tip reach a point, and how hard can it push there. Free."),
    _tool("look",
          "Take a wrist camera picture. Returns look_id, frame and annotated (image paths; annotated marks the "
          "tool tip, a scale bar at the tip's depth and the image centre), brightness (0 to 255), can_see (dark, "
          "dim or ok), camera_pose (position_mm, looks_along, image_right and image_down in the base frame), "
          "tip_px and mm_per_px_at_tip. One frame has no depth: the scale holds only at the tip's distance. "
          "can_see reads the light of what is lit in the frame: a lit scene seen mostly as dark surfaces reads "
          "ok, a lamp glowing in a dark room reads dark. A look is also what the word clear rests on: after it, "
          "clear: true in a move says that you judge the way clear in this picture. " + DIM_RULE + " " + DARK_NOTE
          + " A look moves nothing and takes no world time: a part that is still moving (a head closing) shows "
          "mid-move.",
          _obj({"note": _str("a short note stored with the look")}),
          {"note": "the shelf from above"}, "take an annotated wrist picture labelled for the journal",
          brief="Wrist camera picture: annotated frame, brightness, can_see (dark, dim, ok), camera pose."),
    _tool("standard_look",
          "Go to the standard viewing pose (the home pose unless the robot sets another) by a joint move, stay "
          "there and look. The way the arm came is free; the rest of the way is entered as its space kind says "
          "(clear: true after a look frees it). set_reference true stores this look as the reference; "
          "later standard looks are compared with it and report under "
          "relocalize whether the base or the world moved. It sees the floor patch the camera faces from that "
          "pose, and the square plate the robot stands on (robot://body base.standing_surface) is its own mount, "
          "not a device: a device outside this view needs looks from other poses (robot://practices says how to "
          "search). Refused when the scene is dark here (check can_see). " + DARK_NOTE,
          _obj({"set_reference": _bool("store this look as the reference (default false)"), "clear": _CLEAR}),
          {"set_reference": True}, "take the standard look and keep it as the reference for later comparisons",
          brief="Go to the standard pose and look; with a reference it says if the base or world moved."),
    _tool("relocalize",
          "Compare the latest standard look with its reference; returns yaw_deg, shift_mm_est, scale, moved "
          "(did the world or base move) and confidence.",
          _obj({}), {}, "check whether anything moved since the reference look",
          brief="Compare the latest standard look with its reference: did the base or the world move."),
    _tool("mark",
          "Name a pixel (u right, v down, from the top-left) in a past look; triangulate takes one label "
          "marked in two looks taken from different poses." + FREE_NOTE,
          _obj({"look_id": _ID,
                "u": _num(0, None, "pixel column, from the left"),
                "v": _num(0, None, "pixel row, from the top"),
                "label": _str("a name for the point, such as \"lever\"")}, ["look_id", "u", "v", "label"]),
          {"look_id": "L1", "u": 322, "v": 241, "label": "lever"},
          "say that pixel (322, 241) of look L1 shows the point named lever",
          brief="Name a pixel of a past look; the same label in two looks can be triangulated."),
    _tool("triangulate",
          "Return the 3D point in base frame mm for a label marked in two or more looks, with the rays used. "
          "error_mm is how far the rays miss each other, not the depth error: when ray_angle_deg is under about "
          "15 the point can be off along the view by many times error_mm. depth_uncertainty_mm is how far off "
          "along the view the point may be when each mark is one pixel off; warning is set when the rays are too "
          "close to fix depth or the point lies under the surface the robot stands on. For depth you rely on, "
          "move the camera further sideways between the two looks, or touch." + FREE_NOTE,
          _obj({"label": _str("the label given to mark")}, ["label"]),
          {"label": "lever"}, "where is the point labelled lever, from its two marks",
          brief="The 3D point (base frame mm) of a label marked in two or more looks."),
    _tool("picture",
          "Draw the robot at its present pose from outside, side or top view, with every contact point (a "
          "contact stop that no touch has confirmed is drawn hollow) and located object, to plan a stroke before "
          "making it; returns the image path, grid_mm and px_per_mm." + FREE_NOTE,
          _obj({"view": _str("side or top (default side)", ["side", "top"])}),
          {"view": "side"}, "draw the arm from the side with what it has touched so far",
          brief="Draw the arm from outside (side or top) with contacts and located objects."),
    _tool("gripper",
          "Open, close, or set the gripper opening as a fraction (0 closed, 1 fully open). The jaws move at "
          "their own measured speed. Refused on a robot whose tool is a fixed finger.",
          _obj({"width_frac": _GRIP, "hold": _bool("keep squeezing after closing (default true)"),
                "max_n": _MAX_N}, ["width_frac"]),
          {"width_frac": "open"}, "open the gripper fully",
          brief="Open, close or set the gripper opening (0 closed to 1 open)."),
    _tool("retreat",
          "Back out of a tight place by retracing the most recent moves in reverse: until every link is clear of every stored contact by its stopping distance and that contact's "
          "own error, or by steps moves when you state a count; the way the arm came is free, so it runs at full "
          "speed. The contact stop is armed against what opposes the way out; the force of leaving a touched "
          "thing does not stop it. Returns retreated (how many moves were undone), left_in_history, stopped, "
          "moved_mm and tip_mm.",
          _obj({"steps": _pos("a count of recent moves to undo, when you want one", "integer"),
                "clear": _CLEAR, "tolerance_mm": _TOL}),
          {}, "back out the way the arm came until every link is clear of what it touched",
          brief="Back out the way the arm came: retrace the most recent moves in reverse."),
    _tool("park",
          "Go to the rest pose (the home pose unless the robot sets another) by a joint move: the way the arm "
          "came is free and runs at full speed, the rest is entered as its space kind says (clear: true after a "
          "look frees it); if its planned path is refused (it would meet the surface the robot stands on or the "
          "arm itself) it first retraces the recent moves and tries again; a refusal that stays names what blocks "
          "it and what to do. It rests the arm; a claim does not need it, and a state "
          "confirmed before it stays confirmed. A session left idle past its bound parks by itself before it "
          "closes. " + CONTACT_STOP +
          " Returns pose, stopped, contact, tip_mm and eased; already_there when the arm stood at the pose (it "
          "did not move).",
          _obj({"clear": _CLEAR, "tolerance_mm": _TOL}), {}, "move to the rest pose",
          brief="Go to the rest pose (the home pose unless the robot sets another) by a joint move."),
    _tool("home",
          "Go to the home pose by a joint move; if its planned path is refused it first retraces the recent moves "
          "and tries again, as park does, and its speed is park's. " + CONTACT_STOP + " Returns pose, stopped, "
          "contact, tip_mm and eased; already_there when the arm stood at the pose (it did not move).",
          _obj({"clear": _CLEAR, "tolerance_mm": _TOL}), {}, "move to the home pose",
          brief="Go to the home pose by a joint move."),
    _tool("ease",
          f"Relax the arm where it is: the command follows the arm until nothing pushes (the tip reads under "
          f"{THRESHOLD}). touch, press and every move that stops on contact or cap already end eased, and look "
          "eases first when a joint is still pressing; ease ends the push a result says eased false of. "
          "Returns eased, force_n (what the tip still feels) and tip_mm.",
          _obj({}), {}, "stop pushing and hold where the arm is",
          brief="Stop pushing and hold the present pose (touch, press and contact stops end eased)."),
    _tool("measure_base_give",
          "With the tip resting on a touched surface, make a test push in a direction and measure how "
          f"much the robot's own base tilts, turns and shifts. The push ramps up from {THRESHOLD} until the shift "
          "can be read, never past what that surface already bore, nor past force_n when you state it. Returns "
          "tilt_deg, yaw_deg, shift_mm, mm_per_n, verdict (rigid, gives or not_measurable) and push_bound_n, the "
          "force later presses are held to on a base that gives.",
          _obj({"direction": _direction("which way to push (default up)"),
                "force_n": _pos("the most force this test may use, newtons, when you want to bound it"),
                "clear": _CLEAR}),
          {"direction": "up"}, "test how much an upward push moves the base",
          brief="Test push on a touched surface: how much the robot's own base gives; bounds pushes."),
    _tool("verify",
          "Record a device state you claim. Name the claim by device and state (device is the device's id in "
          "robot://devices; your own name if none is listed) or by claim id, and give the evidence: a look "
          "(look_id) or a second sense (by_sense and ref), with what it shows. A look taken before the arm last "
          "moved shows the old state (check stale: a new look costs 0 control periods). confirmed decides the record; "
          "basis says what it rests on (your_evidence: your words alone) and evidence_checked what the robot itself "
          "tested; a check key in an answer that is not refused (stale, no_evidence) names why it stayed "
          "unconfirmed. Returns claim_id, "
          "confirmed, accepted, checked, basis, why and, when not confirmed, a hint." + FREE_NOTE,
          _obj({"device": _str("the device's id in robot://devices (not a part's name), or your own name for it "
                               "if none is listed"),
                "state": _str("the state to confirm, such as open or closed (robot://devices may name them)"),
                "id": _str("instead of device and state: the claim_id an earlier verify returned"),
                "look_id": _ID,
                "by_sense": _str("instead of a look: the other sense that agrees, such as touch or press"),
                "ref": {"anyOf": [{"type": "string"}, {"type": "integer", "minimum": 0}],
                        "description": "with by_sense: the id that sensing call returned"},
                "evidence": _str("what the look or the sense shows that decides the state (needed with a look or "
                                 "a second sense)")}),
          {"device": "machine", "state": "open", "look_id": "L3",
           "evidence": "the front edge stands clear of the body"},
          "confirm the machine is open from look L3, taken after the push that opened it",
          brief="Confirm a device state by a look (or second sense) newer than the arm's last move. Free."),
]

V0_TOOLS = [
    _tool("read_state",
          "Read the joint angles in degrees by joint name and the gripper state (empty on a robot whose tool is "
          "a fixed finger).",
          _obj({}), {}, "read where every joint is now"),
    _tool("move_joints",
          "Move joints to target angles in degrees given by joint name (names and limits in robot://body); joints "
          "you leave out keep their angle. " + JOINT_PATH + " " + JOINT_SPEED + " The robot does not slow by "
          "itself: every move runs at that speed with the contact stop armed, also where it expects contact or "
          "has not seen; only a limit the site states slows it, where a contact can begin. clear: true says that "
          "you judge the way clear, is recorded and changes no speed. "
          + CONTACT_STOP + " Returns asked, achieved (measured), stopped (goal, "
          "contact, cap or refused), contact and tip_mm. tip_mm is the tool tip in the base frame, in mm (origin "
          "on the mounting plane under the first joint axis, x forward, y left, z up). contact, after a stop on "
          "one, holds point_mm, force_n, force_vec_n (the force the tip puts on the world, base frame) and "
          "normal_est (the opposite direction: the way the world pushed back, friction included).",
          _obj({"targets": {"type": "object", "additionalProperties": {"type": "number"},
                            "description": "target angle in degrees for each joint you name"},
                "speed": _SPEED, "clear": _CLEAR}, ["targets"]),
          {"targets": {JOINT: 10.0}}, f"turn the joint named {JOINT} to 10 degrees",
          rename={"targets": "q_deg"},
          brief="Move joints to target angles in degrees by joint name; stops on contact."),
    _tool("gripper",
          "Open, close, or set the gripper opening as a fraction (0 closed, 1 fully open).",
          _obj({"width_frac": _GRIP, "hold": _bool("keep squeezing after closing (default true)")},
               ["width_frac"]),
          {"width_frac": "close"}, "close the gripper"),
    _tool("snapshot",
          "Take a raw wrist camera picture; returns the image path, brightness (the mean luma, 0 to 255) and "
          "can_see (dark, dim or ok, read from what is lit in the frame: a lamp glowing in a dark room reads "
          "dark); a frame that is not ok also lists bright_spots (where something glows or shines). A dark frame "
          "shows nothing: do not move into contact with anything while the newest picture is dark. When the light "
          "does not come back, claim success false and say that it is dark: in a room that stays dark that is "
          "the right ending, and working blind is not.",
          _obj({}), {}, "take a picture from the wrist camera", light_facts=True),
]

CLAIM = {
    "description": "End the episode with your own statement of what you achieved; nothing else runs after it.",
    "args": _obj({"statement": _str("what you did and what you saw, in plain words"),
                  "success": _bool("true if you believe the task is done")}, ["statement", "success"]),
    "example": {"args": {"statement": "did the two steps the task names; a look after each act showed the state",
                         "success": True},
                "means": "end the episode claiming success"},
}

JOURNAL_TEXT = ("Every call, result, frame and world event so far, in order: the newest records one line each; "
                "?last=N, ?kind=K, ?from=SEQ, ?seq=SEQ (one whole record) and ?whole=true read more.")
RESULT_TEXT = ("The whole result of call SEQ (the seq of its answer; a brief answer names it), as it was journaled: "
               "free, nothing runs again. robot://result/last is the newest.")


COUNTS = ("calls_left, sim_seconds_left: what is left of a stated budget (absent when none is stated: the runtime "
          "sets none); calls_used: every call; calls_free: those not charged; motion_calls: those in which the arm "
          "moved or the world ran; world_wait_s: waiting, not work.")
COSTS_VNEXT = ("Each call is charged one; calls whose descriptions say Free (they compute on stored looks or the "
               "model) and list, read and claim are not charged against the server's call count; a harness that "
               "limits your replies still counts the reply. " + COUNTS)
COSTS_V0 = ("Each call is charged one against the server's call count; list, read and claim are not; a harness "
            "that limits your replies still counts the reply. " + COUNTS)
BRIEF_TEXT = ("A long answer is brief and names what it left out; robot://result/SEQ (free) or detail \"full\" gives "
              "it whole.")
BOUNDS_TEXT = ("No limit of speed or force is typed here: each is derived from the robot and from what it has seen "
               "and felt, and a result that moved names the bound that held it and what would lift it (bounds).")

_CONVENTIONS = ("Units: mm, degrees, newtons, seconds. Base frame: origin on the mounting plane under the first "
                "joint axis, x forward, y left, z up. Tool frame: origin at the tool tip, x along the finger's "
                "pointing direction, z the hand's up. Pixels: u right, v down, from the top-left. Every call "
                "returns a JSON object; one that cannot run returns refused with check, why and hint and changes "
                "nothing. " + BOUNDS_TEXT + " " + COSTS_VNEXT + " " + BRIEF_TEXT)

VNEXT = {
    "name": "vnext",
    "about": _CONVENTIONS,
    "tools": VNEXT_TOOLS,
    "resources": {
        "robot://practices": "What holds on any robot here: speed and stated limits, what a result's words mean, "
                             "looks and the dark, depth, states, pushing a device's part, the claim.",
        "robot://task": "The task for this episode.",
        "robot://body": "The robot, derived from its model: joints (order, units, limits, rated speed, sign), "
                        "links, the tool (tip, surfaces, what stands out behind the tip), wrist camera, reach, "
                        "tip force, base; self (what the robot measured on itself, each with its source); session "
                        "(the site limit if one is stated, the largest tip speed, how much of the reach is known "
                        "free).",
        "robot://world": "What is known so far in base frame mm: contact points (a contact stop stays unconfirmed "
                         "until a touch agrees) with what each thing bore, located objects, device states "
                         "(confirmed or unconfirmed) and events.",
        "robot://devices": "Each device (its id is what verify takes) and its parts, as hypotheses with a source: "
                           "kind, moves, direction, approach_allowed, approach_forbidden, how_to_tell, where (in "
                           "words) and rough sizes (size_mm); position, travel and force only when the source "
                           "gives them (not_given lists what it leaves out).",
        "robot://devices/{id}": "One device entry by id.",
        "robot://journal": JOURNAL_TEXT,
        "robot://result/{seq}": RESULT_TEXT,
    },
}

V0 = {
    "name": "v0",
    "about": "Angles in degrees, lengths in mm; base frame: origin on the mounting plane under the first joint "
             "axis, x forward, y left, z up. Every call returns a JSON object; one that cannot run returns refused "
             "with check, why and hint and changes nothing. " + COSTS_V0 + " " + BRIEF_TEXT,
    "tools": V0_TOOLS,
    "resources": {
        "robot://task": "The task for this episode.",
        "robot://body": "Joint table: joint names in order and limits in degrees, and the installation: how the "
                        "base is mounted and the surface the robot stands on.",
        "robot://devices": "Each device and its parts as its manual describes them, hypotheses with a source: "
                           "kind, moves, direction, approach_allowed, approach_forbidden, how_to_tell, where (in "
                           "words) and rough sizes (size_mm); position, travel and force only when the source "
                           "gives them (not_given lists what it leaves out).",
        "robot://devices/{id}": "One device entry by id.",
        "robot://result/{seq}": RESULT_TEXT,
    },
}

SURFACES = {"vnext": VNEXT, "v0": V0}


def get(name, ex=None):
    """The surface table by name ("vnext" or "v0"): in words; with an executor, rendered for its session."""
    if name not in SURFACES:
        raise KeyError(f"unknown surface {name!r}; known: {', '.join(SURFACES)}")
    return SURFACES[name] if ex is None else for_session(ex, SURFACES[name])


def for_session(ex, surface=None, B=None):
    """The surface (default vnext) with this session's derived values, each with its source, behind the slot
    phrases of its texts (rcpx/surface_session.py). With no bounds for ex the surface comes back in words."""
    return _session.for_session(ex, VNEXT if surface is None else surface, B)


def _swap(obj, old, new):
    if isinstance(obj, str):
        return obj.replace(old, new)
    if isinstance(obj, dict):
        return {_swap(k, old, new): _swap(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_swap(v, old, new) for v in obj]
    return obj


def for_robot(surface, joint_names):
    """The surface with JOINT in its worked examples replaced by the robot's first joint name, so an example
    copied as written is a call this robot accepts. Entries that hold no JOINT are the same objects; with no
    joint names the surface comes back unchanged."""
    if not joint_names or not any(JOINT in repr(t.get("example")) for t in surface.get("tools", [])):
        return surface
    out = copy.copy(surface)
    out["tools"] = [{**t, "example": _swap(t["example"], JOINT, str(joint_names[0]))}
                    if JOINT in repr(t.get("example")) else t for t in surface["tools"]]
    return out
