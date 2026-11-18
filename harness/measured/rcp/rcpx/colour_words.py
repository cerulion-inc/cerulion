# SPDX-License-Identifier: AGPL-3.0-only
"""Colour words (SPEC-R3.md 3.5): one table that names a patch's colour the same way in every package, and reads
the colour words of a device sheet. Pure: images (H x W x 3 uint8 RGB) and text in, names and booleans out.

    balance(frame, mask)                        grey-world gains (r, g, b) from the masked pixels (the floor)
    hsv(patch, gains=None)                      (H, S, V) of the patch's per-channel median after gains (OpenCV)
    name(patch, gains=None)                     the colour name of the patch (the table below, in order)
    name_hsv(h, s, v)                           the same table on one HSV triple
    near(a, b)                                  True when a and b are the same name or a NEAR pair
    words(text)                                 [(colour, state or None)] from a sheet's words
    glowing(patch, frame, compact=True)         the patch glows by its own light (a lit lamp or LED)
    compact(mask)                               a region compact enough to be one light
    matches(observed, glowing, goal, lit_state, v=None)   the observed colour shows the goal colour

The table (H 0 to 180, S and V 0 to 255), tested in order: V under 45 black; S under 45: dark grey (V under 100),
grey (V under 170), white; H under 25 with S 60 or more and V under 130 brown; H under 8 or 170 or more red; under
20 orange; under 34 yellow; under 85: bright green when V 180 or more and S 120 or more, else green; under 130 blue;
under 160 purple; else pink. Every threshold is in rcpx.r3_limits with its reason.
"""

import re

import cv2
import numpy as np

from rcpx import r3_limits as L

NAMES = ("black", "dark grey", "grey", "white", "brown", "red", "orange", "yellow", "green", "bright green", "blue",
         "purple", "pink")
NEAR = (("black", "dark grey"), ("dark grey", "grey"), ("grey", "white"), ("red", "orange"), ("red", "pink"),
        ("orange", "yellow"), ("orange", "brown"), ("green", "bright green"), ("blue", "purple"),
        ("purple", "pink"))
_NEAR = {frozenset(p) for p in NEAR}
# words a sheet may use for a colour, longest first so "dark grey" wins over "grey"
_SPELLINGS = (("bright green", "bright green"), ("dark grey", "dark grey"), ("dark gray", "dark grey"),
              ("black", "black"), ("grey", "grey"), ("gray", "grey"), ("white", "white"), ("brown", "brown"),
              ("red", "red"), ("orange", "orange"), ("yellow", "yellow"), ("green", "green"), ("blue", "blue"),
              ("purple", "purple"), ("violet", "purple"), ("pink", "pink"))
_COLOUR_RE = re.compile(r"\b(" + "|".join(re.escape(s) for s, _ in _SPELLINGS) + r")\b")
_CANON = dict(_SPELLINGS)
_STATE_RE = re.compile(r"[a-z_]+")


def _rgb(img):
    a = np.asarray(img)
    if a.ndim == 1:
        a = a.reshape(1, 1, -1)
    if a.ndim == 2:
        a = np.stack([a, a, a], axis=-1)
    return a[..., :3]


def balance(frame, mask=None):
    """Grey-world gains (r, g, b) from the masked pixels: each channel scaled so the masked mean is grey. An empty
    or missing mask uses the whole frame; a channel with no light keeps gain 1."""
    f = _rgb(frame).astype(np.float64)
    px = f.reshape(-1, 3) if mask is None else f[np.asarray(mask, dtype=bool)]
    if px.size == 0:
        px = f.reshape(-1, 3)
    mean = px.mean(axis=0)
    grey = float(mean.mean())
    return tuple(float(grey / m) if m > 1e-6 else 1.0 for m in mean)


def hsv(patch, gains=None):
    """(H, S, V) in OpenCV units of the per-channel median of the patch after gains (clipped to 0..255)."""
    p = _rgb(patch).astype(np.float64).reshape(-1, 3)
    if gains is not None:
        p = p * np.asarray(gains, dtype=np.float64).reshape(1, 3)
    med = np.clip(np.median(p, axis=0), 0, 255).astype(np.uint8).reshape(1, 1, 3)
    h, s, v = cv2.cvtColor(med, cv2.COLOR_RGB2HSV).reshape(3)
    return int(h), int(s), int(v)


def name_hsv(h, s, v):
    """The colour name of one HSV triple (the module table)."""
    if v < L.V_BLACK:
        return "black"
    if s < L.S_GREY:
        return "dark grey" if v < L.V_DARK_GREY else "grey" if v < L.V_GREY else "white"
    if h < L.BROWN_H and s >= L.BROWN_S and v < L.BROWN_V:
        return "brown"
    if h < L.RED_H_LO or h >= L.RED_H_HI:
        return "red"
    if h < L.ORANGE_H:
        return "orange"
    if h < L.YELLOW_H:
        return "yellow"
    if h < L.GREEN_H:
        return "bright green" if v >= L.BRIGHT_GREEN_V and s >= L.BRIGHT_GREEN_S else "green"
    if h < L.BLUE_H:
        return "blue"
    if h < L.PURPLE_H:
        return "purple"
    return "pink"


def name(patch, gains=None):
    """The colour name of a patch (its per-channel median after gains)."""
    return name_hsv(*hsv(patch, gains))


def near(a, b):
    """True when a and b are the same colour name or one of the NEAR pairs."""
    return a == b or frozenset((a, b)) in _NEAR


def _state_after(text):
    m = _STATE_RE.search(text)
    return m.group(0) if m else None


def _state_before(text):
    found = _STATE_RE.findall(text)
    return found[-1] if found else None


def words(text):
    """[(colour, state or None)] in the order the colours appear: "X means Y", "X when Y" and "X: Y" give (X, Y);
    "Y: X" gives (X, Y); "X-marked" and any other mention of a colour give (X, None). Colours are named as the module
    table names them (gray reads grey, violet reads purple)."""
    if isinstance(text, (list, tuple)):
        out = []
        for t in text:
            out.extend(w for w in words(t) if w not in out)
        return out
    if not isinstance(text, str) or not text.strip():
        return []
    out = []
    for clause in re.split(r"[,;.]|\band\b|\bor\b(?=\s+(?:" + "|".join(s for s, _ in _SPELLINGS) + r")\b)",
                           text.lower()):
        for m in _COLOUR_RE.finditer(clause):
            colour = _CANON[m.group(1)]
            rest, before = clause[m.end():], clause[:m.start()]
            state = None
            mm = re.match(r"\s+means\s+(.*)", rest)
            mw = re.match(r"\s+when\s+(.*)", rest)
            mc = re.match(r"\s*:\s*(.*)", rest)
            if mm:
                state = _state_after(mm.group(1))
            elif mw:
                state = _state_after(mw.group(1))
            elif mc:                                   # "X: Y" (a full sheet's "dark grey: off")
                state = _state_after(mc.group(1))
            elif ":" in before:
                state = _state_before(before.rsplit(":", 1)[0])
            item = (colour, state)
            if item not in out:
                out.append(item)
    return out


def compact(mask, frame_shape=None):
    """True when a region is compact enough to be one light: it fills at least COMPACT_FILL of its bounding box and
    that box covers at most COMPACT_MAX_SHARE of the frame (frame_shape: (H, W), default the mask's own)."""
    m = np.asarray(mask, dtype=bool)
    if not m.any():
        return False
    ys, xs = np.nonzero(m)
    box = (ys.max() - ys.min() + 1) * (xs.max() - xs.min() + 1)
    hh, ww = (frame_shape or m.shape)[:2]
    return float(m.sum()) / box >= L.COMPACT_FILL and box <= L.COMPACT_MAX_SHARE * hh * ww


def glowing(patch, frame, compact=True):
    """True when the patch glows by its own light: V at least GLOW_REL x the frame's median V, and either at least
    GLOW_V bright with S GLOW_S or more (a coloured light), or SATURATED_V or more with S under SATURATED_S on a
    compact region (a saturated light, as a real LED often reads white). compact: the caller's region test (a
    witness component; see compact()); a bare patch is taken as compact."""
    h, s, v = hsv(patch)
    fv = cv2.cvtColor(_rgb(frame).astype(np.uint8), cv2.COLOR_RGB2HSV)[..., 2]
    if v < L.GLOW_REL * float(np.median(fv)):
        return False
    if v >= L.GLOW_V and s >= L.GLOW_S:
        return True
    return bool(compact) and v >= L.SATURATED_V and s < L.SATURATED_S


def matches(observed, glowing_now, goal_colour, lit_state, v=None):
    """True when the observed colour shows the goal colour: the same name or a NEAR one; or, for a lit goal state
    (the state whose colour is not the off colour), a glowing patch at SATURATED_V or more (v given), or one that
    reads white (v not given: only a saturated light reads white and glowing at once)."""
    if observed is None or goal_colour is None:
        return False
    if near(observed, goal_colour):
        return True
    if not (lit_state and glowing_now):
        return False
    return (v is not None and v >= L.SATURATED_V) or (v is None and observed == "white")


__all__ = ["NAMES", "NEAR", "balance", "compact", "glowing", "hsv", "matches", "name", "name_hsv", "near", "words"]
