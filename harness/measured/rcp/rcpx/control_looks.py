# SPDX-License-Identifier: AGPL-3.0-only
"""Look primitives of the Executor: look, standard_look and relocalize, mark and triangulate,
picture, snapshot (the v0 raw frame).

Every look is also noted in the space book (rcpx/bounds.py): the camera's pose, intrinsics and
size, the frame and the time, so the operator's word "clear" can name it. A look
record keeps the camera's pose of that instant, the joint speeds and the jaws as read, and the depth picture when
the body's camera gives one (Backend.depth); rcpx/r2/see_spot.py reads a pointed pixel from it. Nothing is carved.
triangulate gives the point of two marks with its error and never refuses a small baseline; what the point is
worth along the view and across (see_spot.marks_error) is in the result ("sigma_mm") and is stored with the label.

Every look saves the raw frame and its annotated twin under the episode's frames dir and carries brightness and
can_see; a look-dependent primitive refuses on a dark look and says so. The look's help:
pointed_pictures() draws a look that a model pointed in with the pixel marked and the things near it numbered,
and a crop around the pixel, on new files beside its frame, with no motion; a receipt that answers a pixel carries
them (rcpx/r2/place_near.py). Camera poses come from the body model at
the joint pose stored with each look; nothing here reads simulator ground truth.
"""

import math
import os

import numpy as np
from PIL import Image

from rcpx import bounds, perception
from rcpx.context import refusal
from rcpx.control_base import _mm, primitive
from rcpx.control_view import ViewMixin

MM = 1000.0


NO_LIGHT = ("No tool here adds light. A dark look cannot decide a state: any later look reads the light again; while it "
            "stays dark, claim the state unconfirmed (and say it is dark) unless a second sense confirms it")
MOVE_HINT = ("park, home, retreat and ease still run, and any look reads the light again; no tool here adds light. "
             "If the light does not come back, claim the task not done and say that it is dark")


def _move_price(ex):
    """What a move of the tip costs here, in the opening's words (rcpx/price_words.py), for a hint that asks for one."""
    from rcpx import price_words
    line = (price_words.prices(ex) or {}).get("line")
    if not line:
        return "a move, its control periods said in its result"
    return f"about {price_words.figure(line['periods'])} control periods for {line['mm']:g} mm of free way here"


def _dark_refusal(what, look_id, brightness=None):
    b = "" if brightness is None else f" (brightness {brightness:.1f} of 255)"
    return refusal("can_see", f"look {look_id} is dark{b}: {what}", NO_LIGHT)


# ---------------------------------------------------------------------- the look's help
POINTED_DRAWN = ("your pixel: an upright cross; each thing listed near it: a tilted cross with its number in the "
                 "list")


def pointed_pictures(ex, key, rec, u, v, near=(), reach_px=0.0):
    """The look's help for a pixel a model pointed in look `key` (rec: its record, wrist or scene camera): the SAME
    picture with the pointed pixel (u, v) marked and each entry of `near` ({"px": [u, v]}, in its order) marked and
    numbered, and a crop around the pixel enlarged to the picture's own size. The crop holds every marked pixel and
    reach_px pixels on each side (the caller's reach: the hand's own opening at the pixel's range). No motion, no
    control period, no new picture: the look's own frame, drawn on new files beside it.
    Returns {"marked": path, "crop": path, "crop_box_px": [u0, v0, u1, v1], "crop_scale", "drawn", "periods": 0},
    or {"pictures_why": text, "periods": 0} when the look's picture is not a saved image here."""
    import cv2
    from rcpx.r2 import frames_draw as draw
    from rcpx.r2.see_cloud import MID
    img = draw.load_rgb(rec.get("img")) if rec.get("img") is not None else draw.load_rgb(rec.get("frame"))
    frame = rec.get("frame")
    where = os.path.dirname(frame) if isinstance(frame, str) else None
    if where is None and callable(getattr(ex, "_frames_dir", None)):
        where = ex._frames_dir()
    if img is None or not where:
        return {"pictures_why": f"look {key}'s picture is not a saved image here: the pixels above still hold",
                "periods": 0}
    st = ex.state.setdefault("control_looks", {})
    st["pointed"] = n = int(st.get("pointed", 0)) + 1
    h, w = img.shape[:2]
    pts = [(int(u), int(v))] + [(int(t["px"][0]), int(t["px"][1])) for t in near]

    def marks(out, at):         # drawn with OpenCV's own marker size; the numbers as the frames package labels
        lab = draw._Labels(out)
        for i, (x, y) in enumerate(at):
            if i:
                cv2.drawMarker(out, (x, y), draw.COLOR_WORLD, cv2.MARKER_TILTED_CROSS)
                lab.put(str(i), (x, y), draw.COLOR_WORLD)
            else:
                cv2.drawMarker(out, (x, y), draw.COLOR_MARK, cv2.MARKER_CROSS)
        return out

    stem = "".join(c if c.isalnum() or c in "-_" else "_" for c in f"{key}_pointed{n}")
    marked = draw.save_rgb(marks(np.ascontiguousarray(img).copy(), pts), os.path.join(where, stem + ".png"))
    half = int(math.ceil(max([float(reach_px)] + [max(abs(x - pts[0][0]), abs(y - pts[0][1])) for x, y in pts[1:]])))
    u0, v0, side = draw.crop_box(pts[0][0] + MID, pts[0][1] + MID, half + half + 1, w, h)   # both sides, the pixel
    big = np.ascontiguousarray(draw.enlarge(img, u0, v0, side, max(1.0, min(w, h) / float(side))))
    f = big.shape[1] / float(side)
    inside = [(int(round((x - u0 + MID) * f - MID)), int(round((y - v0 + MID) * f - MID))) for x, y in pts]
    crop = draw.save_rgb(marks(big, inside), os.path.join(where, stem + "_crop.png"))
    return {"marked": marked, "crop": crop, "crop_box_px": [u0, v0, u0 + side, v0 + side], "crop_scale": f,
            "drawn": POINTED_DRAWN, "periods": 0}


class LooksMixin(ViewMixin):
    # ------------------------------------------------------------------ frames
    def _save(self, img, stem):
        if self.journal is not None:
            rel = self.journal.save_frame(img, stem)
            return os.path.join(str(self.journal.dir), rel)
        fd = self.cfg["frames_dir"]
        if fd is None:
            return img  # the server saves arrays it finds in a result
        os.makedirs(fd, exist_ok=True)
        path = os.path.join(fd, stem + ".png")
        Image.fromarray(np.ascontiguousarray(img[:, :, :3])).save(path)
        return path

    def _frames_dir(self):
        if self.journal is not None:
            d = str(self.journal.frames_dir)
        else:
            d = self.cfg["frames_dir"]
        if d is not None:
            os.makedirs(d, exist_ok=True)
        return d

    def _camera_pose(self, q):
        pc, Rc = self.model.fk(q)["camera"]
        return {"position_mm": _mm(pc), "looks_along": [round(float(x), 4) for x in -Rc[:, 2]],
                "image_right": [round(float(x), 4) for x in Rc[:, 0]],
                "image_down": [round(float(x), 4) for x in -Rc[:, 1]]}

    # ------------------------------------------------------------------ the space book
    def _note_seen(self, look_id, q, img, t_s, shown=True):
        """Note a wrist look in the space book (the camera's pose from the body model at the look's joint pose,
        its intrinsics and size, the frame, the time). shown: its picture went to the operator (only such a look
        can carry the operator's word). Nothing here may cost the operator its picture: a failure is journalled."""
        try:
            m, book = self.model, bounds.of(self).space
            pc, Rc = m.fk(q)["camera"]
            T = np.eye(len(pc) + 1)
            T[:len(pc), :len(pc)], T[:len(pc), len(pc)] = Rc, pc
            cu, cv, _ = m.project(q, pc - Rc[:, -1])   # the pixel the optical axis meets: the model's own centre
            K = np.array([[m.f_px, 0.0, cu], [0.0, m.f_px, cv], [0.0, 0.0, 1.0]])
            book.note_look(look_id, {"T_base_cam": T, "K": K, "size": [int(m.cam_w), int(m.cam_h)]}, frame=img,
                           at=t_s, shown=bool(shown))
        except Exception as exc:
            self._journal("bounds_problem", where="note_look", error=f"{type(exc).__name__}: {exc}")

    def _look_keeps(self, s, camera="wrist"):
        """What a look record keeps for the pointing service (rcpx/r2/see_spot.py): the camera of that instant, the
        joint speeds, the jaws as read and the depth picture where the camera gives one. A failure costs the
        operator nothing: the look then carries no depth, and a spot in it says so."""
        keeps = {"qd": np.asarray(s.qd, dtype=float).copy(), "depth": None, "depth_stated": None, "cam": None,
                 "jaws": None}
        if isinstance(s.gripper, dict) and s.gripper.get("q") is not None:
            keeps["jaws"] = np.asarray(s.gripper["q"], dtype=float).copy()
        try:
            from rcpx.r2 import see_spot
            if camera == "wrist":
                keeps["cam"] = see_spot.camera_of_model(self, s.q)
            keeps.update(see_spot.take_depth(self, camera))
        except Exception as exc:
            self._journal("bounds_problem", where="look_keeps", error=f"{type(exc).__name__}: {exc}")
        return keeps

    # ------------------------------------------------------------------ the light
    def _note_light(self, brightness, can_see, source, look_id=None):
        """Keep the newest light reading; the dark gate (MotionMixin._dark_refusal_now) follows it."""
        self._light = {"brightness": round(float(brightness), 1), "can_see": can_see, "source": source,
                       "look_id": look_id, "t_s": round(float(self.sim_time()), 3)}
        self._dark_look = ((look_id or f"{source} at t {self._light['t_s']:.1f} s"), float(brightness)) \
            if can_see == "dark" else None
        return self._light

    def _read_light(self, why):
        """One frame read for its light only (no look id, no world time): the newest light reading."""
        img = np.asarray(self.backend.render("wrist"))
        return self._note_light(perception.brightness(img), perception.can_see(img), f"a light reading {why}")

    def _no_light_hint(self):
        return MOVE_HINT

    # ------------------------------------------------------------------ look
    def _look_impl(self, note="", keep_image=False, shown=True):
        s, eased = self._ease_if_pressing()
        img = np.asarray(self.backend.render("wrist"))
        ann, tip_px, mmpp = perception.annotate_model(img, self.model, s.q)
        light = perception.light(img)
        b, cs = light["brightness"], light["can_see"]
        self._n_look += 1
        look_id = f"L{self._n_look}"
        frame = self._save(img, f"{look_id}_raw")
        annotated = self._save(ann, f"{look_id}_annotated")
        self._looks[look_id] = {"q": s.q.copy(), "can_see": cs, "brightness": b, "frame": frame,
                                "img": img if keep_image else None, "mm_per_px": mmpp, "t": s.t,
                                **self._look_keeps(s)}
        # the dark gate follows the newest light reading: this look's
        self._note_light(b, cs, "look", look_id)
        self.world.note_look(look_id, cs, frame=frame if isinstance(frame, str) else None, t_s=s.t)
        self._note_seen(look_id, s.q, img, s.t, shown)
        out = {"look_id": look_id, "frame": frame, "annotated": annotated, "brightness": round(b, 1),
               "can_see": cs, "camera_pose": self._camera_pose(s.q),
               "tip_px": None if tip_px is None else [round(tip_px[0], 1), round(tip_px[1], 1)],
               "mm_per_px_at_tip": None if mmpp is None else round(mmpp, 4), "note": str(note or ""),
               "floor_view_mm": self.look_footprint(s.q), "hidden_by_hand_px": self.hidden_by_hand(s.q)}
        if light.get("bright_spots"):
            out["bright_spots"] = light["bright_spots"]   # a lamp that glows in a dark frame
        if cs == "dark":
            out["warning"] = ("too dark to see: look-dependent tools refuse on this look, and moves toward the "
                              "world (move_tip, touch, press, orient_tool, move_joints) are refused with check dark "
                              "while the newest light reading is dark (any look reads it again); park, home, retreat "
                              "and ease still run")
        elif cs == "dim":
            from rcpx.surfaces import DIM_RULE   # the one dim rule; imported here: surfaces reads DEFAULTS
            out["warning"] = "dim. " + DIM_RULE
        if eased:
            out["eased"] = "a joint was still pressing; eased before the picture"
        if getattr(self.backend, "render_retried", False):
            out["render_retried"] = "the camera failed once and was read again (the harness, not the scene)"
        return out

    @primitive
    def look(self, note=""):
        return self._look_impl(note)

    @primitive
    def snapshot(self):
        """The raw wrist frame with its light (brightness, can_see: facts the plain surface is given too). It does
        not arm the dark gate (the plain surface has none)."""
        s, _ = self._ease_if_pressing()
        img = np.asarray(self.backend.render("wrist"))
        self._n_look += 1
        out = {"image": self._save(img, f"S{self._n_look}_raw"), **perception.light(img)}
        self._note_seen(f"S{self._n_look}", s.q, img, s.t)
        hid = self.hidden_by_hand(s.q)
        if hid is not None and hid["share"] >= 0.5:
            out["warning"] = (f"the robot's own hand and tool cover about {round(hid['share'] * 100)} percent of "
                              "this picture; move the joints so the camera looks past them")
        return out

    # ------------------------------------------------------------------ standard look and relocalize
    def _standard_look_impl(self, set_reference=False, record_contact=True, move_kw=None):
        """The standard look. move_kw: keyword arguments for the joint move to the standard pose (the base check's
        motion kind, what it leaves and the call's intent); with them the move is not retried after a retreat,
        the look is the runtime's own (not shown to the operator), and the result carries the move under "_move"
        (the caller pops it). The joint moves run at the speed the bounds derive."""
        pre = np.asarray(self.backend.render("wrist"))
        self._note_light(perception.brightness(pre), perception.can_see(pre), "standard_look")
        if perception.can_see(pre) == "dark":
            return refusal("can_see", f"the scene is dark here (brightness {perception.brightness(pre):.1f} of 255): "
                           "a standard look would see nothing", self._no_light_hint())
        q_std = self._standard_q()
        # the scene is lit at this pose (checked above), so the move runs even when the last look was dark
        r = self._joint_move(q_std, None, dark_ok=True, record_contact=record_contact, **(move_kw or {}))
        if "refused" in r and self._history and move_kw is None:
            self._retreat_impl()
            r = self._joint_move(q_std, None, dark_ok=True, record_contact=record_contact)
        if "refused" in r:
            return {**r, "_move": r} if move_kw is not None else r
        if r["stop"] != "goal":
            bad = refusal("standard_pose", f"the move to the standard pose stopped ({r['stop']})"
                          + (f": {r['why']}" if r.get("why") else ""),
                          "retreat or move clear, then take the standard look again")
            if r.get("contact"):
                bad["contact"] = r["contact"]
            return {**bad, "_move": r} if move_kw is not None else bad
        self._ease_impl()
        lk = self._look_impl("standard look", keep_image=True, shown=move_kw is None)
        if move_kw is not None:
            lk["_move"] = r
        if lk["can_see"] == "dark":
            lk["relocalize"] = _dark_refusal("it cannot be compared with the reference", lk["look_id"])
            return lk
        rec = self._looks[lk["look_id"]]
        if set_reference:
            self._std_ref = rec
            self._std_last = None
            lk["reference"] = "set"
            return lk
        self._std_last = rec
        if self._std_ref is None:
            lk["reference"] = "none yet: standard_look(set_reference=true) keeps one"
            return lk
        lk["relocalize"] = self._relocalize_impl()
        return lk

    @primitive
    def standard_look(self, set_reference=False):
        return self._standard_look_impl(bool(set_reference))

    def _relocalize_impl(self):
        ref, cur = self._std_ref, self._std_last
        if ref is None:
            return refusal("reference", "no standard look reference yet", "standard_look(set_reference=true) first")
        if cur is None:
            return refusal("standard_look", "no standard look since the reference", "take standard_look() first")
        for rec, name in ((ref, "reference"), (cur, "latest")):
            if rec["can_see"] == "dark":
                return refusal("can_see", f"the {name} standard look is dark: it cannot be compared", NO_LIGHT)
        r = perception.relocalize(ref["img"], cur["img"])
        pc, Rc = self.model.fk(ref["q"])["camera"]
        cam_up_z = float(Rc[2, 2])  # camera z (toward the viewer) along base z: looking down when positive
        yaw = float(r["yaw_deg"]) * (1.0 if cam_up_z >= 0 else -1.0)
        view = -Rc[:, 2]
        if view[2] < -0.1 and pc[2] > 0:
            depth = float(pc[2] / -view[2])  # along the optical axis to the mounting plane
            mmpp = depth * MM / self.model.f_px
        else:
            mmpp = ref["mm_per_px"] or 1.0
        su, sv = r["shift_px"]
        shift = (Rc[:, 0] * su + (-Rc[:, 1]) * sv) * mmpp
        shift[2] = 0.0
        conf = float(r["confidence"]) * min(1.0, abs(cam_up_z) / 0.5)
        out = {"yaw_deg": round(yaw, 3), "shift_mm_est": [round(float(x), 1) for x in shift],
               "scale": r["scale"], "moved": r["moved"], "confidence": round(conf, 3), "inliers": r["inliers"],
               "why": r.get("why", "")}
        if r["moved"]:
            ev = self.world.world_moved({"yaw_deg": out["yaw_deg"], "shift_mm_est": out["shift_mm_est"],
                                         "confidence": out["confidence"]})
            out["stale"] = ev["stale"]
        return out

    @primitive
    def relocalize(self):
        return self._relocalize_impl()

    # ------------------------------------------------------------------ mark and triangulate
    def _look_rec(self, look_id):
        key = str(look_id)
        if key not in self._looks and key.isdigit():
            key = f"L{key}"
        return key, self._looks.get(key)

    @primitive
    def mark(self, look_id, u, v, label):
        key, rec = self._look_rec(look_id)
        if rec is None:
            return refusal("look_id", f"no look {look_id!r} in this session", "pass a look_id a look returned")
        if rec["can_see"] == "dark":
            return _dark_refusal("a mark on it locates nothing", key, rec["brightness"])
        u, v = float(u), float(v)
        if not (0 <= u < self.model.cam_w and 0 <= v < self.model.cam_h):
            return refusal("pixel", f"pixel ({u:g}, {v:g}) is outside the {self.model.cam_w} x {self.model.cam_h} "
                           "frame", "u from 0 to width, v from 0 to height, from the top-left")
        o, dvec = self.model.pixel_ray(rec["q"], u, v)
        marks = self._marks.setdefault(str(label), {})
        marks[key] = {"u": u, "v": v, "origin": o, "dir": dvec}
        out = {"label": str(label), "look_id": key, "u": u, "v": v, "marks": len(marks),
               "ray": {"origin_mm": _mm(o), "dir": [round(float(x), 4) for x in dvec]}}
        if len(marks) < 2:
            out["next"] = ("the same label marked in a second look from another pose gives triangulate a point (the "
                           "move to that pose: " + _move_price(self) + "; the look, the mark and triangulate: 0 "
                           "control periods)")
        return out

    @primitive
    def triangulate(self, label):
        marks = self._marks.get(str(label)) or {}
        if len(marks) < 2:
            return refusal("marks", f"{label!r} is marked in {len(marks)} look(s); two are needed",
                           "look from a second pose a few cm to the side and mark the same point")
        for key in marks:
            if self._looks[key]["can_see"] == "dark":
                return _dark_refusal("its mark cannot be used", key)
        A, b = np.zeros((3, 3)), np.zeros(3)
        dirs = []
        for mk in marks.values():
            dv = mk["dir"]
            P = np.eye(3) - np.outer(dv, dv)
            A += P
            b += P @ mk["origin"]
            dirs.append(dv)
        widest = max(math.degrees(math.acos(min(1.0, abs(float(a @ c))))) for a in dirs for c in dirs)
        try:
            p = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            p = None
        if p is None or not np.all(np.isfinite(p)):   # the marks are one ray: no point lies on both (no plan exists)
            return refusal("baseline", f"the rays differ by {widest:.2f} deg: they are one ray and fix no point",
                           "mark the same point in a look taken from another place")
        miss, rays = [], []
        for key, mk in marks.items():
            w = p - mk["origin"]
            if float(w @ mk["dir"]) <= 0:
                return refusal("behind", f"the point falls behind the camera of look {key}",
                               "check that the marks show the same point")
            miss.append(float(np.linalg.norm(w - (w @ mk["dir"]) * mk["dir"])))
            rays.append({"look_id": key, "origin_mm": _mm(mk["origin"]),
                         "dir": [round(float(x), 4) for x in mk["dir"]], "miss_mm": round(miss[-1] * MM, 2)})
        err = max(miss) * MM
        pm = _mm(p)
        found = self.world.query(kind="located", label=str(label))
        # what the point is worth (error_mm is only how far the rays miss each other): kept WITH the label, so a
        # spot named by it states this error and not the miss (rcpx/r2/see_spot.py)
        from rcpx.r2 import see_spot
        worth = see_spot.marks_error([(mk["origin"], mk["dir"]) for mk in marks.values()], p, self.model.f_px)
        data = {"error_mm": round(err, 2), "looks": list(marks), **see_spot.marks_data(worth)}
        if found:
            e = self.world.update(found[-1]["id"], point_mm=pm, source="triangulate", data=data)
        else:
            e = self.world.add("located", str(label), point_mm=pm, source="triangulate", data=data)
        out = {"label": str(label), "point_mm": pm, "error_mm": round(err, 2), "rays": rays, "id": e.get("id"),
               "ray_angle_deg": round(widest, 2), "sigma_mm": data["sigma_mm"]}
        q = perception.triangulation_quality([(mk["origin"], mk["dir"]) for mk in marks.values()], p,
                                             self.model.f_px, floor_z_m=self.cfg["floor_z_m"])
        out["depth_uncertainty_mm"] = q["depth_uncertainty_mm"]
        warn = list(q["warnings"])
        # two rays no further apart than the camera resolves (QUANT of its own pixels) are one view: the point is
        # given, and that its depth is not fixed is said first
        if worth["along_m"] is None:
            warn.insert(0, see_spot.no_depth_text(worth))
        if warn:
            out["warning"] = "; ".join(warn)
        return out

    # ------------------------------------------------------------------ picture
    @primitive
    def picture(self, view="side"):
        if view not in ("side", "top"):
            return refusal("view", f"unknown view {view!r}", "use side or top")
        fd = self._frames_dir()
        if fd is None:
            return refusal("frames", "no episode dir to draw into", "run inside an episode")
        s = self._sense()
        fk = self.model.fk(s.q)
        contacts = self.world.query(kind="contact")
        objects = self.world.query(kind="located") + [e for e in self.world.query(kind="device") if e["point_mm"]]
        self._n_picture += 1
        out_path = os.path.join(fd, f"picture_{self._n_picture}_{view}.png")
        r = perception.draw_schematic(view, fk["links"], contacts, objects, out_path, tip_mm=np.asarray(fk["tip"][0]) * MM)
        if "refused" in r:
            return r
        return {"image": r["image"], "view": view, "grid_mm": r["grid_mm"], "px_per_mm": r["px_per_mm"],
                "drawn": r["drawn"], "tip_mm": _mm(fk["tip"][0])}
