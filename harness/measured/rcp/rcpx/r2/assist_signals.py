# SPDX-License-Identifier: AGPL-3.0-only
"""Pure result shapes and counted evidence."""

import math

# The early window is this share of a call budget.
EARLY_SHARE = 1.0 / 6.0
# The early window has this length without a call budget.
EARLY_CALLS = 15
# The initial time window is this share of a time budget.
SLOW_TIME_SHARE = 1.0 / 4.0
# This share defines remaining call capacity.
PLENTY_SHARE = 1.0 / 3.0
# Ray disagreement above this distance is counted early.
RAYS_ERR_MM = 10.0
# This many refusals opens the session refusal event.
REFUSALS = 5
# This many identical refusal checks opens the refusal event.
SAME_REFUSAL = 4
# This distance is the minimum repeat radius.
SAME_SPOT_FLOOR_MM = 10.0
# This share of reach scales the repeat radius.
SAME_SPOT_REACH_SHARE = 1.0 / 100.0
# This distance is the minimum nearby radius.
NEAR_FLOOR_MM = 20.0
# This share of reach scales the nearby radius.
NEAR_REACH_SHARE = 1.0 / 50.0
# This many repeated acts opens the repeat event.
REPEATS = 4
# This many calls without an act or location opens idle.
IDLE_CALLS = 20
# Open evidence expires after this many later calls.
DECAY_CALLS = 20
# This pixel distance allows matching coordinates.
SAMEPX_PX = 3.0
# Matching pixels require more than this camera displacement.
SAMEPX_CAM_MM = 20.0
# Movement below this distance counts as no give.
NO_GIVE_MM = 0.5
# This look run length opens a run note.
LOOKS_ROW = 3
# An initial look run must end within this many calls.
LOOKS3_WITHIN = 5
# These are the evidence categories in display order.
SKILLS = ("seeing", "acting", "checking")
# These are the stages in increasing order.
STAGES = ("no_point", "located", "ran_free", "met", "no_give", "moved_back",
          "gave", "gave_unchanged", "confirmed")
# These names are act fallbacks.
ACT_NAMES = ("touch", "press", "push", "lift", "probe_motion", "impulse", "grasp", "act_on")
# These names are feel fallbacks.
FEEL_NAMES = ("touch",)
# These names are end fallbacks.
END_NAMES = ("park", "ask_person")
# These names are verify fallbacks.
VERIFY_NAMES = ("verify",)
# These names are plain movement fallbacks.
MOVE_NAMES = ("move_tip", "move_joints", "orient_tool", "retreat", "park", "home",
              "approach", "standard_look", "ease", "view")
# These names can point at a pixel (used only to say which rules a tool list can feed).
POINT_NAMES = ("mark", "crop", "locate", "choose")
# These names can store a located point (used only to say which rules a tool list can feed).
LOCATE_NAMES = ("triangulate", "locate", "choose")
# These names are look fallbacks.
LOOK_NAMES = ("look", "standard_look", "picture", "crop", "snapshot", "watch",
              "state", "read_state", "device_state", "wait_for_light")
# These checks describe malformed requests.
MALFORMED_CHECKS = ("args", "pixel", "unknown_tool", "marks", "target", "device",
                    "sense", "evidence", "along", "joint", "label", "duration", "cue_point")
# These stops indicate something was met.
MET_STOPS = ("contact", "cap", "part_hit", "force", "stall")
# These stops indicate a bump.
BUMP_STOPS = ("contact", "cap", "part_hit")
# These stops indicate free travel unless something was met.
FREE_STOPS = ("goal", "limit", "travel", "not_found", "no_contact")
# These stops indicate give.
GAVE_STOPS = ("gave", "changed", "reached_end", "unconfirmed")
# These stops indicate movement back.
BACK_STOPS = ("ceiling",)
# These stops indicate no give after meeting something.
RIGID_STOPS = ("rigid",)
# These outcome words indicate free travel.
FREE_CLASSES = ("ran_free_beyond_tip",)
# These outcome words indicate something was met.
MET_CLASSES = ("touched_no_give", "moved_under_load", "slid_off", "gave")


def mapping(value):
    return value if isinstance(value, dict) else {}


def number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value, size=3):
    return value if isinstance(value, list) and len(value) == size and all(number(x) for x in value) else None


def label(value):
    return value.strip().lower() if isinstance(value, str) else None


def word(args, result, result_part=True):
    values = [args.get("label"), args.get("part"), result.get("label")]
    if result_part:
        values.append(result.get("part"))
    for value in values:
        if isinstance(value, str) and value.strip():
            return label(value)
    return None


def copy(value):
    if isinstance(value, dict):
        return {k: copy(value[k]) for k in sorted(value, key=lambda k: (k is not None, str(k)))}
    if isinstance(value, (list, tuple)):
        return [copy(x) for x in value]
    if isinstance(value, float):
        return round(value, 1) if math.isfinite(value) else None
    return value


def distance(a, b):
    if vector(a) is None or vector(b) is None:
        return None
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def radii(reach_mm=None):
    reach = reach_mm if number(reach_mm) else 0.0
    return (max(SAME_SPOT_FLOOR_MM, reach * SAME_SPOT_REACH_SHARE),
            max(NEAR_FLOOR_MM, reach * NEAR_REACH_SHARE))


def read(result):
    result = mapping(result)
    refusal = result.get("refused")
    refused = bool(refusal)
    check = refusal.get("check") if isinstance(refusal, dict) else "?" if refused else None
    out = {"refused": refused, "check": check, "touched": False, "gave": False,
           "met": False, "free": False, "load_mm": None, "no_give": False,
           "back": False, "ran_mm": None, "tip_mm": None, "point_mm": None, "bump": False}
    if refused:
        return out
    contact = result.get("contact")
    stopped = result.get("stopped")
    outcome = result.get("outcome_class")
    touched = contact is True or isinstance(contact, dict)
    gave = result.get("moved_part") is True or result.get("gave") is True or outcome == "gave" or stopped in GAVE_STOPS
    met = touched or gave or stopped in MET_STOPS or outcome in MET_CLASSES
    load = result.get("moved_under_load_mm")
    load = load if number(load) else None
    no_give = met and not gave and (outcome == "touched_no_give" or
                                   (load is not None and load < NO_GIVE_MM) or stopped in RIGID_STOPS)
    back = met and not gave and not no_give and ((load is not None and load >= NO_GIVE_MM) or
                                                outcome == "moved_under_load" or stopped in BACK_STOPS)
    ran = result.get("travelled_mm")
    if not number(ran):
        ran = result.get("travel_mm")
    tip = vector(result.get("tip_mm"))
    point = vector(result.get("point_mm"))
    if point is None:
        point = vector(mapping(contact).get("point_mm"))
    if point is None:
        point = tip
    out.update(touched=touched, gave=gave, met=met,
               free=not met and (outcome in FREE_CLASSES or stopped in FREE_STOPS),
               load_mm=load, no_give=no_give, back=back,
               ran_mm=round(ran, 1) if number(ran) else None,
               tip_mm=tip, point_mm=point,
               bump=touched or stopped in BUMP_STOPS)
    return out


def pixel(args, result):
    args, result = mapping(args), mapping(result)
    look = args.get("look_id") or args.get("look") or result.get("look_id")
    px = None
    for data in (result, args):
        if number(data.get("u")) and number(data.get("v")):
            px = [data["u"], data["v"]]
            break
    if px is None:
        px = vector(args.get("px"), 2)
    if px is None:
        px = vector(args.get("center_px"), 2)
    if not isinstance(look, str) or px is None:
        return None
    return look, list(px), word(args, result, result_part=False)


def classify(name, args, result, classes=None):
    args, result, classes = mapping(args), mapping(result), mapping(classes)
    kind = classes.get(name) if isinstance(name, str) else None
    out = set()
    if ("look_id" in result and "frame" in result) or name in LOOK_NAMES or kind == "look":
        out.add("look")
    has_px = ("u" in args and "v" in args) or isinstance(args.get("px"), list) and len(args["px"]) == 2 or isinstance(args.get("center_px"), list) and len(args["center_px"]) == 2
    if has_px and ("look_id" in args or "look" in args) or kind == "point":
        out.add("point")
    if isinstance(result.get("id"), str) and "point_mm" in result and "stopped" not in result or kind == "locate":
        out.add("locate")
    elif isinstance(result.get("located"), str) and result.get("point_mm") is not None:
        out.add("locate")      # a tool that stores its point under "located" (a chosen candidate, felt or measured)
    act = False
    if kind == "move":
        pass
    elif kind in ("act", "feel"):
        act = True
    elif name in MOVE_NAMES:
        pass
    elif name in ACT_NAMES:
        act = True
    elif "stopped" in result and any(k in result for k in ("contact", "outcome_class", "moved_part")):
        act = any(k in result for k in ("outcome_class", "travelled_mm", "travel_mm")) or read(result)["touched"] or result.get("stopped") in MET_STOPS
    if act:
        out.add("act")
        if name in FEEL_NAMES or kind == "feel":
            out.add("feel")
    if name in END_NAMES or kind == "end":
        out.add("end")
    if name in VERIFY_NAMES or kind == "verify":
        out.add("verify")
    return out


def may_be(name, classes=None):
    """The class words a tool of this name can produce, from the name sets and a package's classes alone."""
    out = classify(name, {}, {}, classes)
    kind = mapping(classes).get(name) if isinstance(name, str) else None
    if kind != "move":
        if name in POINT_NAMES:
            out.add("point")
        if name in LOCATE_NAMES:
            out.add("locate")
    return out


def _entry(skill, kind, says, classes=(), constants=None):
    # enabled means: the rule raises an event; signs and notes never do
    return {"skill": skill, "enabled": kind == "event", "kind": kind,
            "classes": classes, "constants": constants or {}, "says": says}


SIGNALS = {
    "body": _entry("seeing", "event", "the {tool} at call {call} points at the robot's own base or mount", ("point",)),
    "air": _entry("seeing", "event", "the {tool} at call {call} ran {ran_mm} mm and felt nothing{more}", ("act",)),
    "rays15": _entry("seeing", "event", "the rays of {label} at call {call} miss each other by {error_mm} mm", ("locate",), {"RAYS_ERR_MM": RAYS_ERR_MM, "EARLY_SHARE": EARLY_SHARE, "EARLY_CALLS": EARLY_CALLS}),
    "mislabel": _entry("seeing", "event", "the {tool} at call {call} on {label} is {colour}: the manual gives {colour} to the {other} when {state}, not to the {label}", ("point",)),
    "samepx": _entry("seeing", "event", "the marks of {label} at calls {first_call} and {call} are the same pixel ({u}, {v}) in {first_look} and {look}, taken from places {cam_mm} mm apart", ("point",), {"SAMEPX_PX": SAMEPX_PX, "SAMEPX_CAM_MM": SAMEPX_CAM_MM}),
    "refuse": _entry("acting", "event", "{count} calls refused so far (calls {calls})", constants={"REFUSALS": REFUSALS, "SAME_REFUSAL": SAME_REFUSAL}),
    "repeat": _entry("acting", "event", "{count} repeats of an act within {radius_mm} mm with no more force and no checked change (calls {calls})", ("act",), {"REPEATS": REPEATS, "SAME_SPOT_FLOOR_MM": SAME_SPOT_FLOOR_MM, "SAME_SPOT_REACH_SHARE": SAME_SPOT_REACH_SHARE}),
    "idle": _entry("acting", "event", "{idle_calls} calls since the last act or located point (call {since}); a contact was made at call {contact_call} and nothing gave", ("act",), {"IDLE_CALLS": IDLE_CALLS}),
    "wind_down": _entry("checking", "event", "{tool} at call {call} with no accepted verify, nothing that gave and no checked change{more}", ("end",), {"PLENTY_SHARE": PLENTY_SHARE}),
    "stop": _entry("checking", "event", "a claim of no success after call {call}{more}", constants={"PLENTY_SHARE": PLENTY_SHARE}),
    "no_contact": _entry("checking", "event", "a claim of success after call {call}: no act of this session met anything and nothing gave"),
    "touch_contradicts": _entry("checking", "event", "a claim of success after call {call}: the {air_tool} at call {air_call} ran {ran_mm} mm and felt nothing, and no touch has met anything since", ("act",)),
    "no_check": _entry("checking", "event", "a claim of success after call {call}: no verify was accepted and nothing gave"),
    "flip": _entry("checking", "event", "a claim of success after call {call}: a claim of no success was answered after call {answered_call} and no checked change has come since"),
}
for _name, _skill, _says, _constants in (
    ("mislabel_find", "seeing", "the finding from call {call} was out", {}),
    ("mislabel_met", "seeing", "the mark at call {call} was followed by contact", {}),
    ("body_moving", "seeing", "the {tool} at call {call} points at a moving surface", {}),
    ("apart", "seeing", "the locations at call {call} are apart", {}),
    ("contradicted", "checking", "the observations at call {call} differ", {}),
    ("looks3", "seeing", "a run of looks ended at call {call}", {"LOOKS_ROW": LOOKS_ROW, "LOOKS3_WITHIN": LOOKS3_WITHIN}),
    ("slow_start", "seeing", "the elapsed simulation time at call {call} is {used_s} s with no located point", {"SLOW_TIME_SHARE": SLOW_TIME_SHARE}),
):
    SIGNALS[_name] = _entry(_skill, "candidate", _says, constants=_constants)
for _name in ("surface_met", "gave", "confirmed"):
    SIGNALS[_name] = _entry(None, "sign", "{sign} at call {call}")
for _name in ("malformed", "same_refusal", "looks_row", "ray_miss_late", "duplicate",
              "bump", "no_give", "routine_gave", "mislabel_find_in", "mislabel_find_out",
              "mislabel_find_unknown", "meta_no_call_replies", "meta_malformed_json",
              "picked", "taken", "pref", "claim_answered"):
    _skill = "seeing" if _name in ("looks_row", "ray_miss_late", "mislabel_find_in", "mislabel_find_out", "mislabel_find_unknown", "picked") else "checking" if _name == "claim_answered" else "acting"
    SIGNALS[_name] = _entry(_skill, "note", "{tool} at call {call}")
SIGNALS["malformed"]["constants"] = {"MALFORMED_CHECKS": MALFORMED_CHECKS}
SIGNALS["looks_row"]["constants"] = {"LOOKS_ROW": LOOKS_ROW}
SIGNALS["no_give"]["constants"] = {"NO_GIVE_MM": NO_GIVE_MM, "NEAR_FLOOR_MM": NEAR_FLOOR_MM, "NEAR_REACH_SHARE": NEAR_REACH_SHARE}
SIGNALS["surface_met"]["constants"] = {"NEAR_FLOOR_MM": NEAR_FLOOR_MM, "NEAR_REACH_SHARE": NEAR_REACH_SHARE}

# This is the required explanation of counted evidence.
CAVEAT = "this is the robot's guess from {n} counted events of this session; it can be wrong; nothing changes unless you call; the thresholds were measured on recorded sessions of 90 calls on one bench"


class _Fill(dict):
    def __missing__(self, key):
        return "?"


def evidence(signal, data, signals=None):
    """The evidence phrase of an event; a missing or empty value is written as a question mark, never raised."""
    table = SIGNALS if signals is None else signals
    values = copy(mapping(data))
    template = str(mapping(mapping(table).get(signal)).get("says") or signal)
    if signal == "air":
        values["more"] = "; the nearest located point lay {} mm from where it ended".format(values["to_point_mm"]) if values.get("to_point_mm") is not None else ""
    elif signal == "rays15" and values.get("error_mm") is None:
        template = "the views of {label} at call {call} did not agree"
    elif signal == "refuse" and values.get("why") == "same":
        template = "{tool} refused {count} times by the check {check} (calls {calls})"
    elif signal in ("wind_down", "stop"):
        values["more"] = "; {} calls left".format(values["calls_left"]) if values.get("calls_left") is not None else "; before any act met something" if signal == "stop" else ""
    if isinstance(values.get("calls"), list):
        values["calls"] = ", ".join(str(x) for x in values["calls"])
    return template.format_map(_Fill({k: "?" if v is None else v for k, v in values.items()}))
