# SPDX-License-Identifier: AGPL-3.0-only
"""robot://world obstacles and robot://body mounted_parts for the obstacles package (see obstacles.py): the
obstacle list built from world entries (contacts, triangulated and located points, located boxes) and swept
keep-outs, the moving part entries a device act triggers, and the two resource documents. A mixin of the package's
state (obstacles._State): it reads self.ex, self.cfg, self.body, self.mounted, self.keepouts, self.new_keepouts,
self.problems and self.counts. Nothing here reads simulator ground truth or names a robot.
"""

import math
from rcpx.r2 import obstacles_bounds as derived

import mujoco
import numpy as np

MM = 1000.0
CHECKED = ("move_joints", "home", "park", "standard_look")   # plus move_tip with guarded false

def _r(v, nd=1):
    return [round(float(x), nd) for x in v]


def _vec(v, n=3):
    try:
        a = np.asarray(v, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    return a if a.size == n and np.all(np.isfinite(a)) else None


def _num(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _rz(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


class WorldMixin:
    # ------------------------------------------------------------------------------------------ obstacles
    def obstacles(self):
        """[(public dict in mm, shape dict in m)] for every active obstacle, in world insertion order."""
        cfg, out = self.cfg, []
        for e in list(self.ex.world.entries.values()):
            if e.get("stale") or e["kind"] not in ("contact", "located", "device"):
                continue
            p = _vec(e.get("point_mm"))
            if p is None:
                continue
            data = e.get("data") or {}
            pub = {"id": e["id"], "label": e.get("label"), "source": e.get("source"), "from": e["id"]}
            if e["kind"] == "contact":
                c = _vec(data.get("part_point_mm"))
                c = p if c is None else c
                n = None
                for key in ("approach_normal", "part_normal", "fit_normal", "normal_est"):
                    n = _vec(data.get(key)) if data.get(key) is not None else None
                    if n is not None:
                        break
                surfaces = self.ex.model.spec.get("tool_surfaces") or []
                half_size = max((max(x.get("size_mm") or [0]) for x in surfaces), default=0) / MM
                R = derived.contact_radius_m(derived.of(self.ex), e, half_size) * MM
                pub.update(kind="contact", centre_mm=_r(c), radius_mm=round(R, 1))
                if data.get("touched_by"):
                    pub["touched_by"] = data["touched_by"]
                if n is not None and np.linalg.norm(n) > 0:
                    n = n / np.linalg.norm(n)
                    pub.update(shape="half_sphere", normal=_r(n, 3), solid="behind the touched surface")
                    out.append((pub, {"shape": "half_ball", "c": c / MM, "n": n, "R": R / MM}))
                else:
                    pub.update(shape="sphere")
                    out.append((pub, {"shape": "sphere", "c": c / MM, "R": R / MM}))
                continue
            size = _vec(data.get("size_mm"))
            if size is not None and np.all(size > 0):
                yaw = _num(data.get("yaw_deg")) or 0.0
                pub.update(kind=f"{e['kind']}_box", shape="box", centre_mm=_r(p), size_mm=_r(size), yaw_deg=yaw)
                out.append((pub, {"shape": "box", "c": p / MM, "Rb": _rz(yaw), "half": size / (2 * MM)}))
                continue
            err = _num(data.get("error_mm")) or 0.0
            R = derived.uncertainty_m(derived.of(self.ex), e) * MM
            kind = "triangulated" if e.get("source") == "triangulate" else e["kind"]
            pub.update(kind=kind, shape="sphere", centre_mm=_r(p), radius_mm=round(R, 1))
            out.append((pub, {"shape": "sphere", "c": p / MM, "R": R / MM}))
        now = self.ex.sim_time()
        for key, ko in list(self.keepouts.items()):
            if ko["until_s"] is not None and now >= ko["until_s"]:
                continue
            mp = ko["part"]
            pub = {"id": key, "kind": "swept", "shape": "sector", "label": mp["label"], "from": mp["from"],
                   "hinge_mm": _r(mp["h"] * MM), "axis": _r(mp["a"], 3), "start_dir": _r(mp["u"], 3),
                   "radius_mm": round(mp["r"] * MM, 1), "arc_deg": round(math.degrees(mp["A"]), 1),
                   "width_mm": round(mp["w"] * MM, 1), "since_s": round(ko["from_s"], 3),
                   "until_s": None if ko["until_s"] is None else round(ko["until_s"], 3),
                   "triggered_by": ko["act"]}
            if mp["width_assumed"]:
                pub["width_note"] = "no width_mm in the entry: the seen extent is used; zero where none was seen"
            if ko["until_s"] is None:
                pub["until_note"] = "no within_s in the entry: the keep-out holds until the entry changes"
            out.append((pub, {"shape": "sector", "h": mp["h"], "a": mp["a"], "u": mp["u"], "A": mp["A"],
                              "r": mp["r"], "w": mp["w"]}))
        return out

    def moving_parts(self):
        """Every moving part entry with complete geometry (world entries first, then robot://devices parts)."""
        found = []
        for e in list(self.ex.world.entries.values()):
            if e["kind"] in ("device", "located") and not e.get("stale"):
                data = e.get("data") or {}
                items = data.get("moving_parts") or ([data["moving_part"]] if data.get("moving_part") else [])
                for i, mp in enumerate(items):
                    found.append((f"swept:{e['id']}:{i}", e.get("label"), e["id"], mp))
        devs = self.ex.context.get("devices")
        if isinstance(devs, dict):
            for dev in devs.get("devices") or []:
                for i, part in enumerate((dev or {}).get("parts") or []):
                    if isinstance(part, dict) and "hinge_mm" in part:
                        found.append((f"swept:{dev.get('id')}:{part.get('part', i)}",
                                      f"{dev.get('id')} {part.get('part', '')}".strip(), f"device {dev.get('id')}",
                                      part))
        out = []
        for key, label, src, mp in found:
            g = self._sector(dict(mp, extent_mm=(mp.get("extent_mm") or ((self.ex.world.get(src) or {}).get("data") or {}).get("size_mm"))))
            if g is None:
                why = f"{key}: the moving part entry is incomplete or malformed; it is not used"
                if why not in self.problems:
                    self.problems = (self.problems + [why])[-20:]
                continue
            out.append(dict(g, key=key, label=label, **{"from": src}))
        return out

    @staticmethod
    def _sector(mp):
        if not isinstance(mp, dict):
            return None
        h, a, u = _vec(mp.get("hinge_mm")), _vec(mp.get("axis")), _vec(mp.get("start_dir"))
        r, arc = _num(mp.get("radius_mm")), _num(mp.get("arc_deg"))
        if h is None or a is None or u is None or r is None or arc is None or r <= 0 or not 0 < arc <= 360:
            return None
        if np.linalg.norm(a) < 1e-9:
            return None
        a = a / np.linalg.norm(a)
        u = u - (u @ a) * a
        if np.linalg.norm(u) < 1e-6:
            return None
        u = u / np.linalg.norm(u)
        w = _num(mp.get("width_mm"))
        within = _num(mp.get("within_s", mp.get("time_s")))
        return {"h": h / MM, "a": a, "u": u, "r": r / MM, "A": math.radians(arc),
                "w": (w if w is not None and w > 0 else max(mp.get("extent_mm") or [0])) / MM, "width_assumed": not (w is not None and w > 0),
                "within_s": within if within is not None and within >= 0 else None}

    def on_act(self, primitive=None, args=None, result=None, **_):
        p = _vec((result or {}).get("point_mm")) if (result or {}).get("point_mm") is not None else None
        if p is None:
            p = _vec((result or {}).get("tip_mm"))
        if p is None:
            return
        p = p / MM
        now = self.ex.sim_time()
        for mp in self.moving_parts():
            x = p - mp["h"]
            s = float(x @ mp["a"])
            rho = float(np.linalg.norm(x - s * mp["a"]))
            pad = derived.trigger_pad_m(derived.of(self.ex), self.ex.world.get(mp["from"]) or {},
                                         self.ex.world.get((result or {}).get("id")) or {})
            if rho > mp["r"] + pad or abs(s) > mp["w"] / 2.0 + pad:
                continue
            until = None if mp["within_s"] is None else now + mp["within_s"]
            self.keepouts[mp["key"]] = {"part": mp, "from_s": now, "until_s": until, "act": primitive}
            self.counts["keep_outs"] += 1
            shape = {"shape": "sector", "h": mp["h"], "a": mp["a"], "u": mp["u"], "A": mp["A"], "r": mp["r"],
                     "w": mp["w"]}
            C = self.body.clearances(self.ex._sense().q, [shape])[:, 0]
            inside = [self.body.names[i] for i in np.nonzero(C < 0.0)[0]]
            note = {"id": mp["key"], "label": mp["label"], "until_s": None if until is None else round(until, 3),
                    "parts_inside": inside}
            if inside:
                note["hint"] = ("part of the arm is inside the path of a moving part: retreat now, "
                                "before it moves")
            self.new_keepouts.append(note)

    # ------------------------------------------------------------------------------------------ documents
    def world_doc(self, content):
        if not isinstance(content, dict):
            return content
        obs = [pub for pub, _ in self.obstacles()]
        return {**content, "obstacles": obs,
                "obstacle_check": {"margin_rule": "the clearance margin grows with the move speed", "margin_mm": self.full_speed_margin_mm(),
                                   "checked": list(CHECKED) + ["move_tip with "
                                   "guarded false"], "rule": "a checked move is refused (seen_obstacle) when a part "
                                   "of the arm would come within margin_mm of an obstacle it was not already that "
                                   "close to (a part already inside a touched contact's volume, after a press moved "
                                   "the part, may go margin_mm deeper than it stands); "
                                   "moves expecting contact may enter one and report entered",
                                   "problems": list(dict.fromkeys(self.problems))[-5:]}}

    def body_doc(self, content):
        if not isinstance(content, dict):
            return content
        model = self.ex.model
        m = model.mj
        cam = model._cam
        Rcb = np.zeros(9)
        mujoco.mju_quat2Mat(Rcb, m.cam_quat[cam])
        Rcb = Rcb.reshape(3, 3)
        pt, Rt = model.fk_tip(model.q_home)
        pc, Rc = model.fk(model.q_home)["camera"]
        parts = []
        for mp in self.mounted:
            centre_w = pc + Rc @ mp["c"]
            parts.append({
                "name": mp["name"], "shape": mp.get("shape", "box"), "on_body": m.body(int(m.cam_bodyid[cam])).name,
                "size_mm": _r(mp["size_m"] * MM), "centre_in_body_mm": _r((m.cam_pos[cam] + Rcb @ mp["c"]) * MM),
                "centre_in_tool_mm": _r(Rt.T @ (centre_w - pt) * MM),
                "axes_in_tool": {"width": _r(Rt.T @ Rc[:, 0], 3), "height": _r(Rt.T @ Rc[:, 1], 3),
                                 "depth": _r(Rt.T @ Rc[:, 2], 3)},
                "hypothesis": True,
                "source": mp.get("source", "model"), "evidence": mp.get("evidence"),
                "used_for": "the obstacle path check (seen_obstacle) and touched_by"})
        return {**content, "mounted_parts": parts}
