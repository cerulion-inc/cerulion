# SPDX-License-Identifier: AGPL-3.0-only
"""Outside-unit reports of derived SI bounds, A5.

A bound that lowers a speed, a force or an acceleration names one of BINDING as its
source. The words of RETIRED were the sources of round four's doubt rules; item() still takes them from packages
that have not been rewritten, and no plan of rcpx/bounds_plan.py makes one.
"""

import math

MM = 1000.0  # unit: metres to millimetres
SOURCES = ("rating", "model", "self_test", "contact", "seen", "swept", "word", "sheet", "intent", "stated", "prior", "site")
WORDS = dict(zip(SOURCES, ("maker's rating", "body model", "measured self-test", "measured contact", "seen in a look", "swept this session", "operator's word", "device sheet", "stated intent", "stated fact", "gentle prior", "site limit")))
BINDING = ("model", "rating", "self_test", "site", "intent", "sheet", "contact")
RETIRED = ("prior", "word", "swept", "earlier", "unseen")
WHATS = ("speed", "force", "torque", "travel", "time")
_UNITS = dict(zip(WHATS, ("mm/s", "N", "N m", "mm", "s")))


def _rounded(value):
    return float(format(float(value), ".3g")) if value is not None and math.isfinite(value) else None


def item(what, value_si, source, evidence, lift, over_m=None):
    """One bound in outside units, rounded to significant figures, A5."""
    if what not in WHATS or source not in SOURCES:
        raise ValueError("unknown bound kind or source")
    scale = MM if what in ("speed", "travel") else 1.0
    return {"what": what, "value": _rounded(None if value_si is None else value_si * scale), "unit": _UNITS[what],
            "over_mm": None if over_m is None else [_rounded(v * MM) for v in over_m],
            "source": source, "evidence": str(evidence), "lift": str(lift)}


def brief(items):
    return {"binding": items[0], "more": len(items) - 1} if items else None


def full(items):
    return items


def attach(result, items, detail="brief"):
    result["bounds"] = full(items) if detail == "full" else brief(items)
    return result


def text(item):
    value = "unresolved" if item["value"] is None else format(item["value"], "g")
    over = item.get("over_mm")
    where = f" over {over[0]:g} to {over[-1]:g} mm" if over is not None else ""
    return f"{item['what']} held to {value} {item['unit']}{where}: {item['evidence']}; to raise it: {item['lift']}"


def first_statement(v_max_m_s, shares, site):
    """What a session says once about its bounds. shares is taken and not read: no free reach is counted."""
    stated = "no site limit is stated" if not site else "site limits stated: " + str(site)
    speed = "unresolved" if not math.isfinite(v_max_m_s) else format(_rounded(v_max_m_s * MM), "g")
    return (f"{stated}; the largest tip speed the bounds allow is {speed} mm/s; only a stated limit lowers it "
            "(gentle, max_n, force_n, a speed, the site, a device sheet)")


def phase_record(phase, did, moving, waiting, jaw, seconds, free_share, binding):
    return {"phase": phase, "did": did, "periods": moving + waiting + jaw, "moving": moving, "waiting": waiting,
            "jaw": jaw, "seconds": seconds, "free_share": free_share, "bounds": binding}
