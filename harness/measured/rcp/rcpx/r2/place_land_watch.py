# SPDX-License-Identifier: AGPL-3.0-only
"""What the robot itself reads while a held thing is lowered, and how far its own stops run on (round six, second
pass, package p2_land's fix round: the design rulings; contract C15 (4)).

Two readers of the control periods (tick hooks of the session: they move nothing and read no truth):

- RunOn (one per session, run_on(ex)): whether this body's hand runs on PAST the end of its own command when a
  command comes to stand, and how far: the largest excursion of the measured tip beyond the standing command
  along the travel it had, over the body's own settle time. Only stops with no load on the arm are read (a stop
  on a contact is eased onto the reading and says nothing of the free hand). A landing ends its fast stretch
  ABOVE the band where a contact can begin by the body's own lag at that stretch's speed (the stored record),
  so that the HAND, not its command, has come to the arrival speed at the band's upper edge; an arm whose free
  stops all stood within its still band of their command's end trails its command and gets no margin; a way
  down of this session that ran on further than the lag raises it.
- LandWatch (one per way down): the hand's own height and speed, the arm's force reading and the jaws' reading in
  every control period of both stretches of a way down: the speed the hand had in the period before a load first
  rose on the arm's own reading over the hold's own level, whether a guard told that load or not; the fastest the hand ran at or under the band's upper edge before
  it; the largest reading; the fastest the gap closed (a thing caught again between the pads). The set down's ONE
  figure after the stop is stated for these measured speeds (place_land.after).

SI inside (m, s, N). Nothing here names a robot.
"""

import math

import numpy as np

from rcpx import bounds_ratios as R
from rcpx import control_hold

DOWN = np.array([0.0, 0.0, -1.0])
KEY = "land_run_on"


def _loaded(ex, s):
    """The arm reads a load over the contact level in force (the hold's own level while it holds something)."""
    return control_hold.rest_force(ex, s) > control_hold.level(ex)


class RunOn:
    """How far the hand runs on past the end of its own command (module text). tau_s: the largest excursion over
    the command's fastest speed of that motion (s), n: the stops it was read on."""

    def __init__(self, ex):
        self.ex, self.tau_s, self.over_m, self.n, self.stops = ex, 0.0, 0.0, 0, []
        self.p, self.u, self.v_peak, self.moved, self.dirty, self.stand = None, None, 0.0, 0, False, None
        ex.add_tick(self._tick)

    def _settle(self):
        return max(int(R.QUANT), int(math.ceil(float(self.ex.bounds.t_settle()) / float(self.ex.dt))))

    def _close(self):
        st, self.stand = self.stand, None
        if st is None or not st["clean"] or st["n"] < int(R.QUANT) or not st["v"] > 0.0:
            return
        self.n += 1
        self.over_m = max(self.over_m, st["over"])
        self.tau_s = max(self.tau_s, max(0.0, st["over"]) / st["v"])
        self.stops.append({"v_m_s": st["v"], "over_m": st["over"], "down": float(-st["u"][2]), "periods": st["n"]})

    def _tick(self, s):
        ex = self.ex
        if ex._q_des is None:
            return None
        p = np.asarray(ex.model.fk_tip(np.asarray(ex._q_des, dtype=float))[0], dtype=float)
        if self.p is None:
            self.p = p
            return None
        step = p - self.p
        d = float(np.linalg.norm(step))
        self.p = p
        loaded = _loaded(ex, s)
        if self.stand is not None and (d > 0.0 or getattr(ex, "_period_kind", None) == "jaw"):
            self._close()                             # the command moves again, or the jaws do: the stand ends
        if d > 0.0:                                   # the command moves
            if self.moved == 0:
                self.dirty = False
            if self.u is not None and float((step / d) @ self.u) < 1.0 - R.REL:
                self.v_peak = 0.0                     # a corner: the speed of the stretch that ends in the stand
            self.u, self.moved = step / d, self.moved + 1
            self.v_peak = max(self.v_peak, d / float(ex.dt))
            self.dirty = self.dirty or loaded
            return None
        if getattr(ex, "_period_kind", None) == "jaw":
            self.v_peak, self.moved = 0.0, 0
            return None
        if self.stand is None and self.moved >= int(R.QUANT):      # the command came to stand after a motion
            self.stand = {"p": p.copy(), "u": self.u.copy(), "v": self.v_peak, "over": 0.0, "n": 0,
                          "clean": not self.dirty}
        self.v_peak, self.moved = 0.0, 0
        if self.stand is not None:
            st = self.stand
            st["over"] = max(st["over"], float((np.asarray(s.tip, dtype=float) - st["p"]) @ st["u"]))
            st["n"] += 1
            st["clean"] = st["clean"] and not loaded
            if st["n"] >= self._settle():
                self._close()
        return None

    def margin(self, v_m_s, lag_m, still_m, landed=None):
        """(how far above a height its hand must not pass at speed a command that ran at v_m_s is to end, m; the
        words of its source). The first that applies:
        1. landed {"tau_s", "n", "cut"}: this session's own ways down ran on past their fast stretch's end by at
           most tau_s per unit of that stretch's speed, read with no load on the arm (cut false): that, with the
           accepted relative error of a measured quantity;
        2. an arm whose free stops of this session (QUANT or more) all stood within its still band still_m of
           their command's end does not run on: no margin. This reads every free stop, of any direction and
           speed: a hand that stood still after a level or a slower stop can run on after a faster one along
           gravity (and then reaches the band faster than the arrival); read only on stops along the way down at
           the stretch's speed or faster, a thing held on its corner lands that much higher and the jaws of a hand
           with one hinged jaw may then not open on it (tests/r6_p2_land_ab.py);
        3. lag_m, the body's own lag at that speed (the stored record: how far the hand trails its command
           there): a hand that runs on past a command that comes to stand has that much to make up and can
           carry it past the end; never under what a way down of this session showed before a load cut it. (Not
           floored by the furthest a FASTER free stop ran on: on the bridge's arm that raised the middle
           stretch's end 134 mm over the band and the set down to 82 control periods per object.)"""
        v = max(0.0, float(v_m_s))
        seen = 0.0 if not landed else float(landed["tau_s"]) * v
        if landed and landed.get("n") and not landed.get("cut"):
            return seen * (1.0 + R.REL), (
                f"measured: this session's {int(landed['n'])} way(s) down ran on past the end of their fast "
                f"stretch by at most {float(landed['tau_s']) * 1000.0:.1f} mm per m/s of its speed")
        if self.n >= int(R.QUANT) and self.over_m <= float(still_m) and not seen > 0.0:
            return 0.0, (f"the hand does not run on past a command that comes to stand: this session's {self.n} "
                         "free stops stood within its still band of their command's end")
        said = (f"the body's own lag at {v * 1000.0:.0f} mm/s, {float(lag_m) * 1000.0:.1f} mm (the stored record: "
                "how far the hand trails its command there, which a hand that runs on past a standing command "
                "can carry past its end)")
        said += ("; no free stop of this session was read" if not self.n else
                 f"; this session's {self.n} free stops ran on by up to {self.over_m * 1000.0:.1f} mm")
        if seen > float(lag_m):
            said += (f"; a way down of this session ran on at least {seen * 1000.0:.1f} mm at this speed before "
                     "a load ended it")
        return max(float(lag_m), seen), said


def run_on(ex):
    """The session's RunOn, made at the first call (a core with no tick hooks: None)."""
    if not callable(getattr(ex, "add_tick", None)) or not isinstance(getattr(ex, "state", None), dict):
        return None
    st = ex.state.setdefault("place", {})
    if st.get(KEY) is None:
        st[KEY] = RunOn(ex)
    return st[KEY]


class LandWatch:
    """One way down of a held thing, read per control period (module text). top_z: the tip height of the band's
    upper edge (None: no band). done() ends the reading and returns what it read."""

    def __init__(self, ex, top_z=None):
        self.ex, self.top_z = ex, None if top_z is None else float(top_z)
        N = control_hold.numbers(ex) if control_hold.held(ex) is not None else None
        self.per = None if N is None else float(N["gap_per_span_m"])
        self.still = None if N is None else float(N["still_band"])
        self.frac, self.gap_fast = None, 0.0
        self.v_last, self.z_last = None, None
        self.f0, self.read_n = None, 0.0
        self.met = None             # {"v_m_s", "z", "by"}: the hand as the load first rose
        self.v_band, self.z_in, self.v_fast, self.z_low, self.n = 0.0, None, 0.0, None, 0
        self.fast = None            # mark(): (the lowest the hand came on the fast stretch, a load rose there)
        self.live = callable(getattr(ex, "add_tick", None)) and isinstance(getattr(ex, "_ticks", None), list)
        if self.live:
            ex._ticks.append(self._tick)

    def _tick(self, s):
        ex = self.ex
        z = float(s.tip[2])
        v = float((ex.model.jac(s.q)[:3] @ s.qd) @ DOWN)
        self.n += 1
        if self.f0 is None:
            self.f0 = float(s.Fmag)
        rise = float(s.Fmag) - self.f0
        self.read_n = max(self.read_n, rise)
        before = v if self.v_last is None else max(v, self.v_last)      # the period before this reading
        by = None
        if self.per is not None:
            frac, _ = control_hold.read(ex, s)
            if frac is not None and self.frac is not None and self.frac - frac > self.still:
                self.gap_fast = max(self.gap_fast, (self.frac - frac) * self.per / float(ex.dt))
            self.frac = frac
        if rise > control_hold.level(ex):
            by = "the arm's own reading"
        if by is not None and self.met is None:
            self.met = {"v_m_s": max(0.0, before), "z": z if self.z_last is None else max(z, self.z_last), "by": by}
        if self.met is None:
            self.v_fast = max(self.v_fast, v)
            if self.top_z is not None and z <= self.top_z:
                if self.z_in is None:
                    self.z_in = z
                self.v_band = max(self.v_band, before)
        self.z_low = z if self.z_low is None else min(self.z_low, z)
        self.v_last, self.z_last = v, z
        return None

    def mark(self):
        """The fast stretch and its hand-over end here: (the lowest the hand came on them, whether a load rose
        on them: its run-on was then cut short by what it met)."""
        self.fast = (self.z_low, self.met is not None)
        return self.fast

    def loaded_now(self):
        """True when the arm stands with a load over the hold's level that this way down made."""
        s = self.ex._sense()
        return self.met is not None and self.f0 is not None and float(s.Fmag) - self.f0 > control_hold.level(self.ex)

    def done(self):
        if self.live and self._tick in self.ex._ticks:
            self.ex._ticks.remove(self._tick)
        self.live = False
        return {"met": self.met, "v_band_m_s": self.v_band, "entered_z": self.z_in, "v_fast_m_s": self.v_fast,
                "read_n": self.read_n, "gap_m_s": self.gap_fast, "z_low": self.z_low, "fast": self.fast,
                "periods": self.n}
