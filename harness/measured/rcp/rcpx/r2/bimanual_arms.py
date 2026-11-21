# SPDX-License-Identifier: AGPL-3.0-only
"""The seam of the bimanual package (rcpx/r2/bimanual.py): a robot with several arms on one base, each arm an rcpx
Backend view over ONE world.

MultiArmBackend is what a robot implements (the robot's own arm names in arms, one control period dt, one world that
step(n) advances with every arm on its held command). ArmView is the per-arm rcpx Backend (rcpx/backend.py) an
Executor runs on: read and command go to that arm, step goes through the Maintainer, the wrist camera is the camera
mounted on that arm, the fixed cameras come with their installation pose in that arm's frame B, and any other
attribute is the arm's own extra (MultiArmBackend.arm_extra).

The Maintainer advances the world. Alone (solo) an arm's step(n) advances it n periods at once. In lockstep
(run_lockstep: one job per arm, each in its own thread) exactly one thread runs at a time, in the order of the arms
tuple: a job runs until it asks for periods (step) or ends; when every live job waits or has ended the world
advances min(asked) periods, the jobs whose periods are all run go on, again in arms order. So the interleaving,
the commands each period and every journal record are the same on every run, whatever the thread timing. Before
the first period runs, a job that ended with a refusal stops every other job (their first command is withdrawn):
a refusal of one arm moves no arm. A world step that raises stops every waiting job (world_error says why), so a
failing robot never leaves the session waiting. ArmView forwards spent, lost and steps_run from the one world.
Nothing here names a robot.
"""

import threading
import types

import numpy as np

from rcpx.backend import Backend


class MultiArmBackend:
    """What a robot with several arms on one base implements (PLAN section 8). Positions SI, frames 4 x 4."""

    arms = ()                 # the robot's own arm names, in a fixed order
    dt = None                 # the world must state its control period, s
    spent = False             # the world runs no more control periods (a budget or the episode is over)
    lost = False              # the robot stopped answering
    steps_run = 0             # control periods run so far

    def read(self, arm):
        """The rcpx Backend.read() shape for that arm: t, q, qd, tau, load, torque_frac, q_target,
        gripper {q, qd, target, open_frac} or None, steps_left (or None)."""
        raise NotImplementedError

    def command(self, arm, q_target, torque_frac=1.0, gripper=None):
        """Backend.command semantics for one arm; held until the next command of that arm."""
        raise NotImplementedError

    def step(self, n=1):
        """n control periods of the one world, every arm on its held command."""
        raise NotImplementedError

    def render(self, camera):
        """H x W x 3 uint8 frame of a camera named in cameras()."""
        raise NotImplementedError

    def camera_info(self, camera):
        """Backend.camera_info shape: name, width, height, fovy_deg, fx, fy, cx, cy, model."""
        raise NotImplementedError

    def cameras(self):
        """{name: {"mount": "fixed" | an arm name, "width", "height"}}."""
        raise NotImplementedError

    def T_R_B(self, arm):
        """4 x 4: the arm's frame B in the robot frame R (an installation fact)."""
        raise NotImplementedError

    def T_R_cam(self, name):
        """4 x 4: a FIXED camera's pose in frame R; columns image right, image up, toward the viewer."""
        raise NotImplementedError

    def arm_extra(self, arm):
        """An object whose attributes the arm's ArmView exposes (servo_law, servo_goal, ...), or None."""
        return None


def inv(T):
    """Inverse of a rigid 4 x 4."""
    T = np.asarray(T, dtype=float)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


class _Abort(BaseException):
    """Raised inside a waiting lockstep job to stop it before the world moved (not an Exception: the primitive's
    own catch-all does not turn it into a result)."""


class Coordinator:
    """Advances the one world for every arm (module docstring)."""

    def __init__(self, multi):
        self.multi = multi
        self.arms = tuple(multi.arms)
        self.periods = 0                 # world periods advanced through the maintainer
        self.lockstep = False
        self._cv = threading.Condition()
        self._turn = None
        self._want = {}
        self._done = {}
        self._abort = set()
        self._moved = False
        self.world_error = None          # text of a world step that failed during the last lockstep move

    # ------------------------------------------------------------------ the world
    def _advance(self, n):
        n = int(n)
        if n <= 0:
            return
        self.multi.step(n)
        self.periods += n

    def step(self, arm, n=1):
        """An arm asks for n periods (ArmView.step)."""
        n = int(n)
        if not self.lockstep or arm not in self._want:
            self._advance(n)
            return
        with self._cv:
            if arm in self._abort:
                raise _Abort()
            self._want[arm] = n
            self._pass()
            self._cv.wait_for(lambda: self._turn == arm)
            if arm in self._abort:
                raise _Abort()
            self._want[arm] = None

    # ------------------------------------------------------------------ lockstep
    def _runnable(self):
        return [a for a in self.arms if a in self._want and not self._done[a] and self._want[a] is None]

    def _pass(self):
        """Hand the turn on (lock held): the next arm that runs, or the world, then that arm."""
        while True:
            ready = self._runnable()
            if ready:
                self._turn = ready[0]
                self._cv.notify_all()
                return
            waiting = [a for a in self.arms if a in self._want and not self._done[a]]
            if not waiting:
                self._turn = "done"
                self._cv.notify_all()
                return
            if not self._moved and self._refused_before_motion():
                self._abort.update(waiting)
                self._turn = waiting[0]
                for a in waiting:
                    self._want[a] = None
                self._cv.notify_all()
                return
            k = min(self._want[a] for a in waiting)
            if k > 0:
                try:
                    self._advance(k)
                except Exception as exc:   # the world failed: every waiting job stops, the move ends
                    self.world_error = f"{type(exc).__name__}: {exc}"
                    self._abort.update(waiting)
                    self._turn = waiting[0]
                    for a in waiting:
                        self._want[a] = None
                    self._cv.notify_all()
                    return
                self._moved = True
            for a in waiting:
                self._want[a] -= k
                if self._want[a] <= 0:
                    self._want[a] = None

    def _refused_before_motion(self):
        return any(isinstance(r, dict) and "refused" in r for a, r in self._results.items() if self._done.get(a))

    def run_lockstep(self, jobs):
        """jobs {arm: fn() -> result dict}; runs them time aligned (module docstring). Returns (results {arm:
        result}, started_period, periods, aborted arms). A job that raises is a refusal of its arm."""
        order = [a for a in self.arms if a in jobs]
        self._results = {}
        started = self.periods
        self.world_error = None
        with self._cv:
            self.lockstep, self._moved = True, False
            self._want = {a: None for a in order}
            self._done = {a: False for a in order}
            self._abort = set()
            self._turn = None
        threads = []
        for a in order:
            t = threading.Thread(target=self._job, args=(a, jobs[a]), name=f"arm-{a}", daemon=True)
            threads.append(t)
            t.start()
        with self._cv:
            self._pass()
            self._cv.wait_for(lambda: self._turn == "done")
        for t in threads:
            t.join()
        aborted = sorted(self._abort, key=order.index)
        with self._cv:
            self.lockstep = False
            self._want, self._done, self._abort = {}, {}, set()
            self._turn = None
        return dict(self._results), started, self.periods - started, aborted

    def _job(self, arm, fn):
        with self._cv:
            self._cv.wait_for(lambda: self._turn == arm)
        try:
            out = fn()
        except _Abort:
            out = None
        except Exception as exc:   # a job that fails is that arm's refusal, never the session's end
            out = {"refused": {"check": "internal", "why": f"the {arm} job failed: {type(exc).__name__}: {exc}",
                               "hint": "look or read robot://world before the next move"}}
        with self._cv:
            self._results[arm] = out
            self._done[arm] = True
            self._pass()


class ArmView(Backend):
    """One arm of a MultiArmBackend as an rcpx Backend (module docstring)."""

    def __init__(self, multi, arm, coordinator, frames):
        self.multi, self.arm, self.coordinator = multi, arm, coordinator
        self.dt = float(multi.dt)
        self._frames = frames            # bimanual_world.Frames: T_R_B of every arm
        cams = dict(multi.cameras() or {})
        mine = [n for n, c in sorted(cams.items()) if c.get("mount") == arm]
        self.wrist_name = mine[0] if mine else None
        self.fixed = [n for n, c in sorted(cams.items()) if c.get("mount") == "fixed"]
        self._extra = multi.arm_extra(arm) or types.SimpleNamespace()

    # ------------------------------------------------------------------ Backend
    def read(self):
        return self.multi.read(self.arm)

    def command(self, q_target, torque_frac=1.0, gripper=None):
        self.multi.command(self.arm, q_target, torque_frac=torque_frac, gripper=gripper)

    def step(self, n=1):
        self.coordinator.step(self.arm, n)

    def _camera(self, camera):
        if camera in (None, "wrist"):
            if self.wrist_name is None:
                raise ValueError(f"arm {self.arm} carries no camera")
            return self.wrist_name
        if camera in self.fixed:
            return camera
        raise ValueError(f"camera is wrist or one of {self.fixed}")

    def render(self, camera="wrist", scale=None):
        name = self._camera(camera)
        if scale is not None:
            try:
                return self.multi.render(name, scale=scale)
            except TypeError:
                pass
        return self.multi.render(name)

    def camera_info(self, camera="wrist"):
        name = self._camera(camera)
        info = dict(self.multi.camera_info(name))
        if name != self.wrist_name:
            T = inv(self._frames.T_R_B(self.arm)) @ np.asarray(self.multi.T_R_cam(name), dtype=float)
            R = T[:3, :3]
            info["fixed"] = True
            info["installation"] = {"position_m": [float(x) for x in T[:3, 3]], "R": R.tolist(),
                                    "looks_along": [float(x) for x in -R[:, 2]],
                                    "image_right": [float(x) for x in R[:, 0]],
                                    "image_down": [float(x) for x in -R[:, 1]]}
        info["name"] = "wrist" if name == self.wrist_name else name
        return info

    def scene_cameras(self):
        """{name: camera_info(name)} of the fixed cameras, posed in this arm's frame B (rcpx.r2.scene_camera)."""
        return {name: self.camera_info(name) for name in self.fixed}

    def sim_time(self):
        return float(self.read()["t"])

    @property
    def spent(self):
        return bool(getattr(self.multi, "spent", False))

    @property
    def lost(self):
        return bool(getattr(self.multi, "lost", False))

    @property
    def steps_run(self):
        return int(getattr(self.multi, "steps_run", 0))

    def __getattr__(self, name):
        if name.startswith("__") or name in ("multi", "arm", "coordinator", "_extra", "_frames"):
            raise AttributeError(name)
        try:
            return getattr(self.__dict__["_extra"], name)
        except (KeyError, AttributeError):
            raise AttributeError(name) from None
