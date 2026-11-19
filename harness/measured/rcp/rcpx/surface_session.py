# SPDX-License-Identifier: AGPL-3.0-only
"""What a session adds to the surface tables when it opens (rcpx/surfaces.py re-exports every name here; the
session side is rcpx/server_edge.py).

- FREE_TOOLS: calls that compute on stored looks or the model and move nothing do not spend the call
  budget. is_free(name, entry) also honours an entry marked "free": true.
- WORLD_WAIT_KEYS: a tool that reports the seconds it waited on the world under its own key; those
  seconds are world time, not work time.
- DETAIL_VALUES: every call takes detail ("brief" or "full"), a session argument (rcpx/server_edge.py).
- dark_rule(tool names, cannot): the hint a dark refusal carries, naming only tools the session has.

Numbers a text shows are the session's, never typed.
- SLOTS: the words a tool table holds where the runtime knows a number. facts(ex) reads this session's values
  from its bounds (rcpx.bounds.of on the real executor: bounds_of), each with the source word of the getter;
  render(obj, facts) puts "(about VALUE UNIT ..., SOURCE)" behind the first use of each phrase in every text;
  for_session(ex, surface) is the two together. A value that cannot be read, or is not a finite positive number,
  is left out: the words stand alone, as they do with no executor or no bounds module.
- first_statement(ex): what a session says once about its bounds (no site limit stated or the owner's, the
  largest tip speed at this pose, how much of the reach is known free, how to say that the way is clear).
- GUARDED_RETIRED, REFUSALS, look_hint: the sentence for the retired argument, the refusals the derived bounds
  add, and the look call a clear_needs_look hint holds.
This module imports nothing of rcpx at load (rcpx/surfaces.py and rcpx/context_devices.py import it).
"""

WAIT_TOOL = "wait_for_light"


def dark_rule(tool_names, cannot=False):
    """The hint of a dark refusal on a surface with these tools: what this surface offers in the dark, naming only
    tools it has. cannot: the claim takes a cannot statement (round two's claim)."""
    names = set(tool_names or ())
    end = ("claim success false" + (" with cannot" if cannot else "") + " and say that it is dark, with every "
           "state you could not see reported as unconfirmed: when the light does not come back that is the right "
           "ending")
    head = ("A dark picture cannot guide a push: moves toward the world stay refused while the newest look reads "
            "dark, and any new look reads the light again. ")
    if WAIT_TOOL in names:
        return (head + f"{WAIT_TOOL} parks and waits in world time, which does not count against "
                "sim_seconds_left; it cannot know whether the light will come back. When it ends still dark, " + end
                + ". Do not work by touch in the dark.")
    stay = [n for n in ("park", "home") if n in names]
    rest = (f"{' and '.join(stay)} do{'es' if len(stay) == 1 else ''} not wait (from the rest pose each takes a "
            "fraction of a second). " if stay else "")
    look = "one more look" if "look" in names else "one more picture"
    return (head + "No tool here adds light or waits for it: " + rest + f"If {look} is still dark, " + end
            + ". Do not work by touch in the dark.")


# Ruling 3: calls that only compute on stored looks or the model and move nothing do not spend the call budget.
# compare: it reads two stored looks and moves nothing.
FREE_TOOLS = ("crop", "mark", "triangulate", "verify", "reach_check", "picture", "device_state", "compare")
# A tool that reports the seconds it waited on the world under its own key (waiting is world time).
WORLD_WAIT_KEYS = {"wait_for_light": "waited_s"}


def is_free(name, entry=None):
    """True for a call that does not spend the call budget (a FREE_TOOLS name, or an entry marked free)."""
    return name in FREE_TOOLS or bool((entry or {}).get("free"))


# The argument every call takes. It is the session's, not a tool's: the session takes it out of the
# arguments before the tool's own schema is checked (so no table changes), and a tool entry marked pass_detail gets
# detail=True when it is "full" (touch and press then report more of what they met).
DETAIL_VALUES = ("brief", "full")

# The listing holds these words until a session can report its own bounds.
SLOTS = {
    "f_thr": "the contact threshold",
    "f_one": "the one-reading threshold",
    "f_hand": "the hand's own weight",
    "f_tool": "the tool's own rating",
    "rigid": "the travel the robot can tell from none",
    "v_full": "the robot's full speed",
    "limit_cost": "what a stated limit costs here",
}
SOURCE_WORDS = {"rating": "from its ratings", "model": "from its model, not yet measured",
                "self_test": "from its own self-test", "contact": "from a contact", "seen": "from a look",
                "swept": "from its own sweep", "word": "on your word", "sheet": "from the device sheet",
                "intent": "as you stated", "stated": "as the owner stated", "prior": "a prior: nothing is known yet",
                "site": "the site's limit"}
GUARDED_RETIRED = ("guarded is retired: the contact stop is always armed; look, then say clear: true to state "
                   "that the way is clear")   # said once per session to a call that passes guarded false
# The refusals round four adds to the list an operator can meet: check -> what it means and
# what its hint holds. The session makes sure a clear_needs_look hint holds the look call (look_hint).
REFUSALS = {
    "clear_needs_look": "the word clear (or a speed above RCP's offer) with no look you were shown that is newer "
                        "than the last contact or act that moved a thing; the hint holds the look call",
    "force_unknown": "a push with no force stated, no sheet and nothing this part or its device has shown; the "
                     "hint offers the ramp with a max_n for you to state",
    "selfknow_missing": "the robot has not measured itself and may not do so here; the hint names the owner's "
                        "one setting",
}


def core_executor(ex):
    """The executor behind a transparent wrapper: a harness may hand the session a wrapper that keeps the real
    executor under _ex (its calls pass through the harness's own bookkeeping). The bounds, the pose and a
    primitive's signature are the real executor's."""
    try:
        inner = getattr(ex, "_ex", None)
    except Exception:
        inner = None
    return ex if inner is None else inner


def bounds_of(ex):
    """The session's bounds (rcpx.bounds.of on the real executor), or None: no executor, no bounds module in the
    tree, or an executor it cannot be built for (a stub). This is the one seam: every reader calls it through the
    module at call time, and a test may replace it. A words-only session needs no bounds module."""
    if ex is None:
        return None
    try:
        from rcpx import bounds
        return bounds.of(core_executor(ex))
    except Exception:
        return None


def _pose(ex):
    ex = core_executor(ex)
    try:
        return ex._sense().q
    except Exception:
        try:
            return ex.model.q_home
        except Exception:
            return None


def _positive(getter):
    import math
    try:
        value = float(getter())
        return value if math.isfinite(value) and value > 0 else None
    except Exception:
        return None


def _plain(value):
    from decimal import Decimal
    return format(Decimal(f"{value:.2g}"), "f")


def _source(B, name):
    try:
        return SOURCE_WORDS.get(B.src(name)[0])
    except Exception:
        return None


def _source_word(B, name):
    try:
        return B.src(name)[0]
    except Exception:
        return None


def _full_speed(B, q):
    """The largest tip speed along the six base axes at pose q, in mm/s; None when it cannot be read, and None
    while the joint speeds are neither rated nor measured (source model): that value only sizes the robot's own
    self-test, no move of an operator runs at it, so it is not shown."""
    values = []
    for u in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]):
        value = _positive(lambda: B.v_free(q, u))
        if value is not None:
            values.append(value)
    if not values or _source_word(B, "v_free") == "model":
        return None
    return _positive(lambda: max(values) * 1000)


def facts(ex, B=None):
    """The numbers a listing can substantiate now, each getter isolated from the others."""
    import sys
    if B is None:
        surface_session = sys.modules[__name__]
        B = surface_session.bounds_of(ex)
    if B is None:
        return {}
    q, u = _pose(ex), [0, 0, -1]
    rows = (
        ("f_thr", "f_thr", lambda: B.f_thr(0.0, 0.0, 2), "N on this robot at rest", 1),
        ("f_one", "f_thr", lambda: B.f_thr(0.0, 0.0, 1), "N on this robot at rest", 1),
        ("f_hand", "f_hand", lambda: B.f_hand(), "N on this robot", 1),
        ("f_tool", "f_tool", lambda: B.f_tool(), "N on this robot", 1),
        ("rigid", "rigid", lambda: B.rigid(q, u), "mm at this pose", 1000),
        ("v_full", "v_free", lambda: _full_speed(B, q), "mm/s at this pose", 1),
    )
    out = {}
    for slot, name, getter, words, scale in rows:
        value = _positive(getter)
        if value is not None:
            src = _source(B, name)
            scaled = _positive(lambda: value * scale)
            if scaled is not None:
                out[slot] = f"about {_plain(scaled)} {words}" + (f", {src}" if src else "")
    if ex is not None:      # what a call costs here in control periods (rcpx/price_words.py), for the opening
        try:
            from rcpx import price_words
            P = price_words.prices(ex)
        except Exception:
            P = {}
        if P:
            out["_prices"] = P
            cost = price_words.limit_words(P)
            if cost:
                out["limit_cost"] = cost
    return out


def render(obj, facts):
    """Render string values, retaining executable examples and leaving keys alone. A full listing (its tools with
    their schemas) also gets the opening generated from its own tools, once (rcpx/surface_words.py)."""
    from rcpx import surface_words
    if surface_words.is_full_listing(obj):
        obj = surface_words.with_opening(obj, (facts or {}).get("_prices"))
    if not facts:
        return obj
    if isinstance(obj, str):
        for slot, phrase in SLOTS.items():
            at = obj.find(phrase)
            if slot in facts and isinstance(facts[slot], str) and at >= 0:
                end = at + len(phrase)
                if not obj[end:].startswith(" (about "):
                    obj = obj[:end] + " (" + facts[slot] + ")" + obj[end:]
        return obj
    if isinstance(obj, list):
        return [render(value, facts) for value in obj]
    if isinstance(obj, dict):
        out = {}
        for key, value in obj.items():
            if key == "example":
                out[key] = {k: render(v, facts) if k == "means" else v for k, v in value.items()} \
                    if isinstance(value, dict) else value
            else:
                out[key] = render(value, facts)
        return out
    return obj


def for_session(ex, surface, B=None):
    return render(surface, facts(ex, B))


def _site_words(site):
    words = []
    for key, unit in (("speed_mm_s", "mm/s"), ("force_n", "N")):
        value = _positive(lambda: site[key])
        if value is not None:
            words.append(f"{_plain(value)} {unit}")
    try:
        if "zones" in site:
            count = len(site["zones"])
            words.append(f"{count} zone" + ("" if count == 1 else "s"))
    except Exception:
        pass
    return ", ".join(words) or "as stated in the site record"


def first_statement(ex, B=None):
    """The opening motion's statement of site, speed, free reach and the operator's word."""
    import math
    import sys
    if B is None:
        surface_session = sys.modules[__name__]
        B = surface_session.bounds_of(ex)
    if B is None:
        return None
    try:
        site = B.site or None
    except Exception:
        site = None
    speed = _full_speed(B, _pose(ex))
    try:
        share = float(B.space.share("free"))
        if not math.isfinite(share) or not 0 <= share <= 1:
            share = None
    except Exception:
        share = None
    try:
        people = core_executor(ex).cfg.get("people") is True
    except Exception:
        people = False
    sentences = ["No site limit is stated." if site is None else
                 f"The owner states a site limit: {_site_words(site)}."]
    if speed is not None:
        src = _source(B, "v_free")
        sentences.append("The largest tip speed the bounds allow on this robot at this pose is about "
                         f"{_plain(speed)} mm/s" + (f" ({src})" if src else "") + ".")
    if share is not None:
        sentences.append(f"About {round(100 * share)} percent of the reach is known free so far (swept by the "
                         "robot's own body or seen by a look); the rest is unseen.")
    sentences.append("clear: true is recorded and changes no speed.")
    if people:
        sentences.append("The owner states that people or machines may enter the reach: the word clear then "
                         "needs a look taken after the last call that moved.")
    out = {"said": " ".join(sentences), "site": site, "tip_speed_mm_s": speed, "free_share": share}
    if people:
        out["people"] = True
    return out


def look_hint(tool_names):
    names = set(tool_names or ())
    look = "call look '{}'" if "look" in names else "call snapshot '{}'" if "snapshot" in names else "a new look"
    return f"a look shows the way ({look}, 0 control periods); clear: true is recorded and changes no speed"
