# SPDX-License-Identifier: AGPL-3.0-only
"""A surface push through the shared derived push law."""
from rcpx.control_base import primitive
from rcpx.r2.shove_bounds import whole
from rcpx.r2.shove_args import push_args
from rcpx.r2.shove_law import MM, PushLawMixin, install
from rcpx.r2.shove_tools import PRACTICES, TOOLS

ID = "N6"


class Mixin(PushLawMixin):
    @primitive
    @whole
    def shove(self, point_mm=None, label=None, toward_mm=None, toward_label=None, direction_deg=None,
             distance_mm=None, contact=None, height_mm=None, max_force_n=None, standoff_mm=None, clear=None):
        a, bad = push_args(self, point_mm, label, toward_mm, toward_label, direction_deg, distance_mm, contact,
                           height_mm, max_force_n, standoff_mm, clear=clear)
        if bad:
            return bad
        f, bad = self._sv_push_run(a["point"], a["d"], a["distance"], a["height"], a["frac"], a["force"],
                                   a["standoff"], None, True, "push", intent=a["intent"])
        if a["label"] and f.get("met_at_mm") is not None:
            ids = [e["id"] for e in self.world.query(label=a["label"]) if not e.get("stale")]
            self.world.mark_stale(ids, "pushed: it no longer stands where it was located", source="shove")
        return report(f, bad, a)


def report(f, bad, a):
    """The push result: a sentence first, then the numbers; a refusal before any move comes back as it is."""
    if bad is not None and not f:
        return bad
    stop = f.get("stopped", "refused" if bad is not None else None)
    pushed = round(float(f.get("pushed_m", 0.0)) * MM, 1)
    peak = f.get("peak_n", 0.0)
    look = f.get("look") or {}
    lk = f" Look {look['look_id']} shows where it ended." if look.get("look_id") else ""
    want = a["distance"] * MM
    said = {
        "done": f"Pushed the thing {pushed:.0f} mm along {f.get('direction_deg', 0):.0f} deg"
                + (" with a steady load the whole way" if f.get("followed") else ", the load not steady")
                + f" (peak {peak:.1f} N).",
        "force": f"Stopped after {pushed:.0f} of {want:.0f} mm: the push force reached its limit; the thing does "
                 "not slide this way (held, blocked or too heavy).",
        "lost_contact": f"Lost contact after {pushed:.0f} of {want:.0f} mm: the thing no longer pushed back (it "
                        "turned away, tipped or slid off the hand's line).",
        "no_contact": f"Felt nothing within {round(float(f.get('free_m', 0.0)) * MM):.0f} mm along the push: the "
                      "thing is not where the point says, or too light to feel.",
        "limit": f"Stopped at the arm's limit after {pushed:.0f} of {want:.0f} mm of push.",
        "on_top": "Did not push: the hand came down on the thing itself each time, not behind it.",
        "too_tall": "Did not push: " + str(f.get("why", "the thing is too tall for this hand to push")) + ".",
    }.get(stop, f"The push stopped ({stop}) after {pushed:.0f} mm.")
    out = {"said": said + lk, "pushed_mm": pushed, "contact_force_n_peak": peak, "followed": bool(f.get("followed")),
           "stopped": stop, "look": look, "contact": a["contact"], "max_force_n": a["force"], "next_stroke_mm": f.get("next_stroke_mm")}
    for k in ("met_at_mm", "hand_end_mm", "push_height_mm", "height_note", "top_z_mm", "top", "support_z_mm",
              "support", "stepped_back_mm", "hand_tilt_deg", "contact_width_mm", "direction_deg", "steady_share",
              "meet_n", "world_s", "why", "periods", "bounds", "phases", "next_stroke_mm"):
        if f.get(k) is not None:
            out[k] = f[k]
    if bad is not None:
        out["refused_after_moving"] = bad.get("refused")
        out["said"] += " A later move was refused: " + str((bad.get("refused") or {}).get("why", ""))
    if stop == "on_top":
        out["hint"] = "give a point on the thing's side nearest the hand, or a larger standoff_mm"
    return out


def setup(ex):
    install(ex)
    ex.state.setdefault("shove", {})


__all__ = ["ID", "Mixin", "PRACTICES", "TOOLS", "setup"]
