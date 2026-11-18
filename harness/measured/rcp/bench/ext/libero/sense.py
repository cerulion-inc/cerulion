# SPDX-License-Identifier: AGPL-3.0-only
"""The bridge's cameras, and what its arm senses beyond one reading per control period (round six, the design notes
C5 and C11): depth pictures of its two cameras and the wrist force sensor at the physics rate. A mixin of
LiberoBackend (bench/ext/libero/backend.py); the env host's side is bench/ext/libero/hostsense.py.

- render(camera) and camera_info(camera): the wrist camera at the spec's size, and the fixed scene camera(s) with
  the installation pose in frame B (rcpx.r2.scene_camera works the looks from them).

- depth(camera): metres along the optical axis (h x w) of the frame render(camera) gave at this control step (the
  host renders colour and depth in ONE call, so they are one picture and no control step runs), else of a frame
  taken now. None when the host gives no depth.
- with_depth(name, info): camera_info's answer with "depth": {"sigma_m_at_1m", "grows": "square", "source"}: the
  simulator states its depth buffer's own step as the error; and, where the host states it,
  "sample_px": [du, dv] with "sample_source": where inside a pixel the depth picture samples its ray (the depth
  at [v, u] is along the ray through (u + du, v + dv)), so that a deprojection need not assume the middle.
- fast_block(st): read()["fast"] = {"dt_s": the physics step, "qd": k x n joint speeds after every physics step of
  the last control period, "wrist_n": k x 3}. wrist_n is what the robot puts on the world at the wrist sensor after
  every physics step, frame B, tared as the period reading wrist_force_n is (the weight of what hangs below the
  sensor, at that step's own pose) and, because a force at this rate is read while the hand speeds up and brakes,
  with the inertia of that same mass at its measured acceleration taken off as well (m a of the part below the
  sensor, from the arm's own joint speeds; without it every start and stop of a free move would read as a push).
  Absent once the host runs no more steps (the last period's samples are not a new reading).

Nothing here is truth: the sensor's raw values, the arm's joint speeds and the body model's own kinematics.
"""

import math
import os
import zlib

import numpy as np


def stated_sigma():
    """A depth noise stated for this run (the design notes C18, the design rulings): env R6_DEPTH_SIGMA_M, metres
    at 1 m; 0 when none is stated. Gate G1's rows noise1 and noise3 set it; nothing else does."""
    return float(os.environ.get("R6_DEPTH_SIGMA_M", 0.0) or 0.0)


def noise_seed(run, sigma, camera, step, frame):
    """The seed of one frame's noise picture: the run's own tag (a gate row states the cell and the draw: the
    backend's depth_seed; "" when none is stated), the stated sigma, the camera, the control step and the frame.
    So a frame read twice gives one picture, and two cells, two draws or two sigmas never share one."""
    return zlib.crc32(f"{run}:{sigma:g}:{camera}:{step}:{frame}".encode())


class SenseMixin:
    # ------------------------------------------------------------------ cameras
    def _camera(self, camera):
        if camera == "wrist":
            c = self.model.spec["camera"]
            return "wrist", int(c["width"]), int(c["height"])
        if camera in self.scene:
            c = self.scene[camera]
            return camera, int(c["width"]), int(c["height"])
        raise ValueError(f"camera is wrist or one of {sorted(self.scene)}")

    def render(self, camera="wrist", scale=None):
        from PIL import Image

        name, w, h = self._camera(camera)
        if scale is not None:
            w, h = int(round(w * float(scale))), int(round(h * float(scale)))
        for attempt in (0, 1):
            try:
                res = self.link.call("render", camera=name, width=w, height=h, depth=True)
                with Image.open(res["path"]) as im:
                    img = np.asarray(im.convert("RGB"), dtype=np.uint8).copy()
                self.keep_depth(name, res)
                self.render_retried = attempt > 0
                return img
            except Exception as exc:   # one retry: a failed render is the harness, not the robot
                self.render_failures.append({"camera": name, "attempt": attempt, "error": f"{exc}"[:300]})
                if attempt:
                    raise

    def camera_info(self, camera="wrist"):
        name, w, h = self._camera(camera)
        if name == "wrist":
            fovy = float(self.model.spec["camera"]["fovy"])
            f = (h / 2.0) / math.tan(math.radians(fovy) / 2.0)
            return self.with_depth(name, {
                "name": "wrist", "width": w, "height": h, "fovy_deg": fovy, "fx": f, "fy": f, "cx": w / 2.0,
                "cy": h / 2.0, "model": "pinhole, no distortion; u right, v down, origin top left"})
        c = self.scene[name]
        return self.with_depth(name, {
            "name": name, "width": w, "height": h, "fovy_deg": c["fovy"], "fx": c["fx"], "fy": c["fy"],
            "cx": c["cx"], "cy": c["cy"], "model": "pinhole, no distortion; u right, v down, origin top left",
            "fixed": True, "installation": {k: c[k] for k in ("position_m", "R", "looks_along", "image_right",
                                                              "image_down")}})

    def scene_cameras(self):
        """{name: camera_info(name)} of the fixed cameras (rcpx.r2.scene_camera reads this)."""
        return {name: self.camera_info(name) for name in sorted(self.scene)}

    # ------------------------------------------------------------------ depth
    def keep_depth(self, name, res):
        """Remember the depth picture a render answer carries (render() calls this)."""
        kept = self.__dict__.setdefault("_depth_kept", {})
        d = res.get("depth") if isinstance(res, dict) else None
        if isinstance(d, dict) and d.get("path"):
            kept[name] = {"path": d["path"], "step": res.get("step"), "size": (res.get("width"), res.get("height")),
                          "frozen": self._t_frozen}
            self.__dict__.setdefault("_depth_facts", {})[name] = {k: v for k, v in d.items() if k != "path"}
        else:
            kept.pop(name, None)

    def depth(self, camera="wrist"):
        name, w, h = self._camera(camera)
        kept = self.__dict__.setdefault("_depth_kept", {}).get(name)
        now = self._st.get("step")
        if kept is None or kept["step"] != now or kept["size"] != (w, h) or kept["frozen"] != self._t_frozen:
            self.render(camera)                       # a frame of now: colour and depth in one call
            kept = self._depth_kept.get(name)
        if kept is None:
            return None
        z = np.load(kept["path"]).astype(float)
        s1 = stated_sigma()
        if s1:   # a depth camera's noise as stated: z + N(0, 1) x s x z x z, seeded as noise_seed says
            rng = np.random.default_rng(noise_seed(self.__dict__.get("depth_seed", ""), s1, name, kept["step"],
                                                   kept["frozen"]))
            z = z + rng.normal(0.0, 1.0, z.shape) * s1 * z * z
        return z

    def depth_facts(self, name):
        """The camera's depth buffer facts as the host states them (asked once), or None."""
        facts = self.__dict__.setdefault("_depth_facts", {})
        if name not in facts:
            try:
                _, w, h = self._camera(name)
                facts[name] = self.link.call("cam", camera=name, width=w, height=h).get("depth")
            except Exception:
                facts[name] = None
        return facts[name]

    def with_depth(self, name, info):
        d = self.depth_facts(name)
        if isinstance(d, dict) and d.get("step_m_at_1m") is not None:
            s1 = stated_sigma()
            info["depth"] = {"sigma_m_at_1m": max(float(d["step_m_at_1m"]), s1), "grows": d.get("grows", "square"),
                             "source": "stated for this run" if s1 > float(d["step_m_at_1m"]) else
                             d.get("source", "the simulator's depth buffer: its own step")}
            if d.get("sample_px") is not None:   # where inside a pixel the depth is sampled, as the host states it
                info["depth"].update(sample_px=[float(v) for v in d["sample_px"]],
                                     sample_source=d.get("sample_source"))
        return info

    # ------------------------------------------------------------------ the wrist force at the physics rate
    def fast_block(self, st):
        blk = st.get("fast") if isinstance(st, dict) else None
        if not isinstance(blk, dict) or not blk.get("qd") or self.spent or self.lost:
            return None
        memo = self.__dict__.get("_fast_memo")
        if memo is not None and memo[0] is st:
            return memo[1]
        dt = float(blk["dt"])
        qd = np.asarray(blk["qd"], float).reshape(-1, self.n)
        out = {"dt_s": dt, "qd": qd}
        raw = blk.get("wrist_force_raw")
        if raw is not None and len(raw) == len(qd) and blk.get("qd0") is not None:
            out["wrist_n"] = self._wrist_series(np.asarray(raw, float).reshape(-1, 3), qd,
                                                np.asarray(blk["qd0"], float).reshape(self.n),
                                                np.asarray(st["q"], float).reshape(self.n), dt)
        self.__dict__["_fast_memo"] = (st, out)
        return out

    def _wrist_series(self, raw, qd, qd0, q_end, dt):
        """The sensor's samples as the force on the world (module docstring). Sample k is read at the pose and
        speed the arm had BEFORE physics step k and at that step's own acceleration, so the poses are stepped
        back from the period's end by the joint speeds (q before a step = q after it less dt x its speed)."""
        n = len(qd)
        speeds = np.vstack([qd0[None, :], qd])          # speeds[i]: after step i (0: before the first)
        q = np.empty((n + 2, self.n))                   # q[i + 1]: after step i; q[0]: one step before the period
        q[n + 1] = q_end
        for i in range(n, -1, -1):
            q[i] = q[i + 1] - dt * speeds[i]
        com, rot = np.empty((n + 2, 3)), np.empty((n + 2, 3, 3))
        for i in range(n + 2):
            self._set_kin(q[i])
            com[i] = self._d.subtree_com[self._ft_body]
            rot[i] = self._d.site_xmat[self._ft_site].reshape(3, 3)
        acc = (com[2:] - 2.0 * com[1:-1] + com[:-2]) / (dt * dt)       # at the pose before each step
        weight = self._ft_mass * self.g_B
        return np.einsum("kij,kj->ki", rot[1:n + 1], raw) + weight - self._ft_mass * acc
