# SPDX-License-Identifier: AGPL-3.0-only
"""What a move met, below any stop threshold too (Felt), with the two small helpers the core's results share. Split
from rcpx/control_base.py in round six (that file's size); control_base re-exports every name. No robot is named
here and no limit is stated here: Felt's levels are asked of the session's bounds (ex._felt_levels).
"""

import math

MM = 1000.0   # unit: mm per m


class Felt:
    """What a move met, below any stop threshold too: the largest force, how far the tip advanced against
    resistance, where resistance first rose, each stretch of it, and whether a part gave way. A reading resists when
    its force is over the level the estimate can hide at rest plus its noise (the session's own, from the bounds);
    the advance counted is the tip's step along the force it pushes with, half of it for the step in which
    resistance began. A part moved (moved_part) when the tip advanced, against resistance, further than the arm
    itself gives under that force (its resolution plus the force over its servo stiffness), or when the resistance
    fell under the give share of its peak while the tip went on that far along the force it had at the peak: the
    force shape of a part that gives way, which a rigid surface never shows."""

    def __init__(self, ex):
        lv = getattr(ex, "_felt_levels", None)
        lv = lv() if lv is not None else {}
        self.floor = float(lv.get("floor_n", 0.0))      # resistance begins over this
        self.rigid_m = float(lv.get("rigid_m", 0.0))    # the tip's own resolution
        self.k = float(lv.get("k_n_m", math.inf))       # the servo's stiffness along the push
        self.give = float(lv.get("give") or _ratio("GIVE"))   # a give: the force under this share of its peak
        self.peak = 0.0
        self.first = None       # tip (m) at the first resisting reading
        self.spans = []         # [t0, tip (m), peak N, t1] per stretch of resistance
        self.gave = None        # {"peak_n", "fell_to_n", "went_on_mm", "t_s"} once a part gave way
        self._along = 0.0       # m advanced along the push while resisting (signed sum)
        self._open = False      # the last reading resisted
        self._tip = None        # tip at the last reading
        self._pk = None         # [peak N, tip at it, unit force at it, readings] of the newest stretch

    def rigid_at(self, force_n):
        """The travel the arm itself gives under force_n: less than this is no motion of a part (m)."""
        return self.rigid_m + (float(force_n) / self.k if self.k > 0.0 else 0.0)

    def add(self, s):
        self.peak = max(self.peak, s.Fmag, float(getattr(s, "Fend", None) or 0.0))
        last, self._tip = self._tip, s.tip.copy()
        if self._pk is not None and self.gave is None:
            pk_n, pk_tip, pk_u, n = self._pk
            on = float((s.tip - pk_tip) @ pk_u)
            if pk_n > self.floor and s.Fmag < self.give * pk_n and on > 0.0 and on >= self.rigid_at(pk_n):
                self.gave = {"peak_n": round(pk_n, 2), "fell_to_n": round(s.Fmag, 2), "went_on_mm": round(on * MM, 2),
                             "t_s": round(s.t, 3)}
        if s.Fmag <= self.floor or s.Fmag <= 0.0:
            self._open = False
            return
        if last is not None:
            self._along += float((s.F / s.Fmag) @ (s.tip - last)) * (1.0 if self._open else HALF_STEP)
        if not self._open:
            self.spans.append([s.t, s.tip.copy(), s.Fmag, s.t])
            if self.first is None:
                self.first = s.tip.copy()
            self._pk = [s.Fmag, s.tip.copy(), s.F / s.Fmag, 1]
        else:
            span = self.spans[-1]
            span[2], span[3] = max(span[2], s.Fmag), s.t
            self._pk[3] += 1
            if s.Fmag > self._pk[0]:
                self._pk[:3] = [s.Fmag, s.tip.copy(), s.F / s.Fmag]
        self._open = True

    @property
    def resisted(self):
        """Tip advance against resistance (m)."""
        return max(0.0, self._along)

    @property
    def moved(self):
        """A part moved: more advance against resistance than the arm gives under that force, or a give."""
        return (self.resisted > 0.0 and self.resisted >= self.rigid_at(self.peak)) or self.gave is not None

    def merge(self, other):
        """Fold in a later leg of the same move."""
        self.peak = max(self.peak, other.peak)
        self._along += other._along
        if self.first is None:
            self.first = other.first
        self.spans += other.spans
        self.gave = self.gave or other.gave
        return self

    def report(self, stop_n=None, detail=False):
        """The result keys: peak_force_n, resisted_mm, first_resist_mm, moved_part (with moved_part_why and, after
        a give, part_gave), brushed (stretches of resistance: the first 3, all of them up to 8 with detail).
        stop_n: the force the move would have stopped on, for the wording."""
        resisted = self.resisted
        out = {"peak_force_n": round(self.peak, 2), "resisted_mm": round(resisted * MM, 2),
               "first_resist_mm": None if self.first is None else _mm(self.first)}
        out["moved_part"] = bool(self.moved)
        under = "" if stop_n is None or self.peak >= stop_n else f", under the {stop_n:.2f} N stop threshold"
        if self.gave is not None:
            g = self.gave
            out["part_gave"] = dict(g)
            out["moved_part_why"] = (f"the resistance rose to {g['peak_n']:.1f} N and fell to {g['fell_to_n']:.1f} N "
                                     f"while the tip went on {g['went_on_mm']:.1f} mm{under}: a part gave way (or the "
                                     "tip slid past an edge); look before the next push")
        elif self.moved:
            out["moved_part_why"] = (f"the tip travelled {resisted * MM:.1f} mm against up to {self.peak:.1f} N"
                                     f"{under}: a part gave way or slid (the arm itself gives under "
                                     f"{self.rigid_at(self.peak) * MM:.2f} mm at that force); look before the next "
                                     "push")
        if self.spans:
            shown = self.spans[:len(self.spans) if detail else BRUSH_BRIEF]
            out["brushed"] = [{"t_s": round(sp[0], 3), "tip_mm": _mm(sp[1]), "peak_n": round(sp[2], 2),
                               "for_s": round(sp[3] - sp[0], 3)} for sp in shown]
            if len(self.spans) > len(out["brushed"]):
                out["brushed_more"] = len(self.spans) - len(out["brushed"])
        return out


BRUSH_BRIEF = 3   # stretches of resistance a brief result lists (text size, not a limit)
HALF_STEP = 0.5   # numeric: the mean share of the step in which resistance began


def _ratio(name):
    """One of the named ratios (rcpx/bounds_ratios.py: the only place a bare multiple lives)."""
    from rcpx import bounds_ratios
    return getattr(bounds_ratios, name)


def _mm(p):
    return [round(float(x) * MM, 2) for x in p]
