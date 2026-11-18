# SPDX-License-Identifier: AGPL-3.0-only
"""The texts of the carry profile (SPEC-R3.md section 6), rendered from what a session loads and lists. Pure: names
in, text out. Nothing here names a robot, a device or a part.

    practices(tools, packages, calls, lift_listed=False) -> str     6.1, one numbered line each
    lines(tools, schemas=None) -> {tool name: description}          6.2, for the declared tools that have one
    named_tools(text, universe) -> [tool names the text asks to run]  the per-arm name check (gap M1)

tools: the names the session lists or reaches through use (a set). packages: the loaded package names (finish and
witness change what a claim and a push can say). calls: the session's call budget (line 7). lift_listed: lift is
declared (cfg r3_lift_declared): only then does a text name lift.

Rendering rule (6.4): a line whose tools are not loaded is replaced by its fallback; every alternative below keeps
the line's intent with the tools that are there. Where the spec's fallback names a field the arm does not have,
the nearest wording that names only what is there is used (SPEC-CHANGES.md, carry lines):
- line 2 without view: the spec's "the candidate whose fits says it matches" needs sight's fits, and sight is the
  package that brings view; without sight the line names the candidate's size against the device's size_mm;
- line 3 and line 5 name press (through use) instead of push when act is not loaded;
- line 5 says "look from another side" without view and "mark another part" without locate, and reads stopped
  alone without witness (no seen);
- line 6 is built from its two halves: the same-view sentence needs witness, the "gets an answer" sentence needs
  finish.
"""

import re

PUSH, LIFT, VIEW, LOCATE, SURVEY, COMPARE = "push", "lift", "view", "locate", "survey", "compare"
EXAMPLE = "Example:"

# 6.2, one line of what, one of when, one example. look's like sentence needs witness (look gains like).
LOOK_LIKE = ("Takes a picture from where the camera is. With like it first puts the camera back where it was for that "
             "look, so two pictures can be compared. Example: look {} or look {\"like\": \"L4\"}.")
LOOK_PLAIN = ("Takes a picture from where the camera is. Use it before and after every act. Example: look {}.")
LINES = {
    SURVEY: ("Turns the camera all round and lists the things standing on the table (c1, c2, ...) with where they "
             "are, their size, and whether that fits the device. Use it when the device is not in the picture. "
             "Example: survey {}."),
    "look": LOOK_LIKE,
    VIEW: ("Moves the camera to see a thing from one side: robot_side, left, right, far_side, above, front (the side a "
           "located part faces) or around (every side it can reach, one picture). Use it when the part you need is "
           "in no picture. Example: view {\"target\": \"c2\", \"side\": \"around\"}."),
    LOCATE: ("Point at a part in a picture: the look, the pixel and the part's name from robot://devices. The robot "
             "takes two more pictures itself and measures where the part is. Example: locate {\"look\": \"L3\", "
             "\"px\": [412, 233], \"part\": \"button\"}."),
    PUSH: ("Pushes a located part the way it moves: the finger comes straight onto its face, feels it, pushes a "
           "little harder each time until it gives (a drawer: until it stops), lets go and shows the same view "
           "before and after. Use it on a button, a switch half, a drawer face, or a raised lever lip to close it; "
           "never on a lamp. Example: push {\"target\": \"located_3\"}."),
    LIFT: ("Lifts a lever or lip from under its edge: the fingertip goes under it from outside, pushes up a little "
           "harder each time until it gives, then comes out. Use it to open a lid by its lever. Example: lift "
           "{\"target\": \"located_2\"}."),
    COMPARE: ("Free. Says what changed between two pictures taken from the same place, part by part. Example: "
              "compare {\"before\": \"L4\", \"after\": \"L7\"}."),
}
# a survey without sight's fits: the size words the candidates do carry
SURVEY_NO_FITS = ("Turns the camera all round and lists the things standing on the table (c1, c2, ...) with where "
                  "they are and their size. Use it when the device is not in the picture. Example: survey {}.")

# 6.1. The lines whose wording does not depend on what is loaded.
P1 = ("Each result starts with said, one sentence about what happened, and ends with next, the call that fits what "
      "happened. Send next unless the picture shows it does not fit.")
P2_FULL = ("Find the device: look. If it is not in the picture, survey and take the candidate whose fits says it "
           "matches; view it from the robot's side, or around.")
P2_FITS = ("Find the device: look. If it is not in the picture, survey and look at the candidate whose fits says it "
           "matches.")
P2_SIZE = ("Find the device: look. If it is not in the picture, survey and look at the candidate whose size is "
           "nearest the device's size_mm in robot://devices.")
P2_LOOK = "Find the device: look; move the camera with move_tip until the device is in the picture."
P3_FULL = ("Point at the part you need with locate, in a look that shows it, using the part's name from "
           "robot://devices. The robot measures where it is. A lamp only shows the state: never {verb} it.")
P3_MARK = "Mark the part in two looks and triangulate (use); a lamp only shows the state: never {verb} it."
P4_FULL = ("Act with the verb its do line names: push{lift}. The robot offers a force and a speed from what it has "
           "measured and felt and says where each came from; state force_n, max_n or a lower speed to set your own.")
P4_LIFT = ", or lift for a lever or lip that must go up"
P4_USE = "Act as the do line says; press and touch run through use."
P5 = ("Read {read}. changed: the part moved. came_back or ceiling: it springs back and needs more than the robot may "
      "push there. rigid: this spot does not move, so it is not the moving part: {side} or {point}.")
P6_SEEN = ("Claim success only when seen, in the same view before and after, shows the state robot://devices gives "
           "in how_to_tell.")
P6_LOOK = "Claim success only when a look after the act shows the state robot://devices gives in how_to_tell."
P6_GATE = ("A claim before that, or a give-up with most of the budget left, gets an answer with what is left to "
           "try.")
P7 = "Most of the {n} calls are spare. A step that did not work is information, not the end."
P7_NO_COUNT = "A step that did not work is information, not the end."      # no call budget is stated


def practices(tools, packages=(), calls=None, lift_listed=False):
    """6.1 as served under carry: seven numbered lines (line 5 is left out when push is not loaded, it has no
    fallback), each worded with the tools this session has (module text)."""
    t, pk = set(tools or ()), set(packages or ())
    out = [P1]
    if SURVEY in t and VIEW in t:
        out.append(P2_FULL)
    elif SURVEY in t:
        out.append(P2_FITS if "sight" in pk else P2_SIZE)
    else:
        out.append(P2_LOOK)
    verb = PUSH if PUSH in t else "press"
    out.append((P3_FULL if LOCATE in t else P3_MARK).format(verb=verb))
    if PUSH in t:
        out.append(P4_FULL.format(lift=P4_LIFT if (lift_listed and LIFT in t) else ""))
        out.append(P5.format(read="stopped and seen" if "witness" in pk else "stopped",
                             side="view another side" if VIEW in t else "look from another side",
                             point="point at another part" if LOCATE in t else "mark another part"))
    else:
        out.append(P4_USE)
    six = P6_SEEN if "witness" in pk else P6_LOOK
    out.append(six + (" " + P6_GATE if "finish" in pk else ""))
    out.append(P7.format(n=int(calls)) if calls is not None else P7_NO_COUNT)
    return "\n".join(f"{i}. {line}" for i, line in enumerate(out, start=1))


def lines(tools, schemas=None):
    """6.2 for the declared tools that have a line here: {name: description}. schemas: {tool name: its argument
    schema} as the session serves it; look is described with like only when its schema has like (witness), and
    survey with fits only when sight is there to add them (schemas holds view or locate)."""
    t = set(tools or ())
    sch = schemas or {}
    out = {}
    for name in (SURVEY, "look", VIEW, LOCATE, PUSH, LIFT, COMPARE):
        if name not in t:
            continue
        text = LINES[name]
        if name == "look":
            props = ((sch.get("look") or {}).get("properties") or {})
            text = LOOK_LIKE if "like" in props else LOOK_PLAIN
        elif name == SURVEY and not ({VIEW, LOCATE} & t):
            text = SURVEY_NO_FITS
        out[name] = text
    return out


def sentences(text):
    """The sentences of a description (a full stop, then a space or the end; a dot inside braces, quotes or
    brackets, as in an example's argument or "(c1, c2, ...)", does not end one)."""
    plain = re.sub(r'\{[^{}]*\}|"[^"]*"|\([^()]*\)', "X", str(text))
    return [s for s in re.split(r"(?<=\.)\s+", plain.strip()) if s]


_NOT_A_CALL_BEFORE = re.compile(r"\b(?:do not|never|a|an|the|same|each)\s+$")
_NOT_A_CALL_AFTER = re.compile(r"^\s+is not\b")


def named_tools(text, universe):
    """The names of `universe` (tool names) that a text names as something to run: every word outside double
    quotes (a quote is the device sheet's own words) that is a tool name, except a prohibition ("do not push",
    "never push"), a noun ("a push", "the same view", "each push") and a statement that a tool is absent ("lift is
    not enabled"): none of those asks for the tool. In order of first mention."""
    names = set(universe or ())
    outside = re.sub(r'"[^"]*"', '"', str(text or ""))
    out = []
    for m in re.finditer(r"[a-z_]+", outside):
        w = m.group(0)
        if w not in names or w in out:
            continue
        if _NOT_A_CALL_BEFORE.search(outside[max(0, m.start() - 12):m.start()]) or \
                _NOT_A_CALL_AFTER.search(outside[m.end():m.end() + 12]):
            continue
        out.append(w)
    return out


__all__ = ["EXAMPLE", "LINES", "LOOK_LIKE", "LOOK_PLAIN", "SURVEY_NO_FITS", "lines", "named_tools", "practices",
           "sentences"]
