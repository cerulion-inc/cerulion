# SPDX-License-Identifier: AGPL-3.0-only
"""Backend: the one seam between the executor (rcpx/control.py) and a robot, simulated or real (SPEC section 3).

SI inside. read() returns numpy arrays; nothing here is ground truth about the world, only what a real arm reports
(joint positions, velocities, the torque each servo applies and that torque as a signed fraction of its limit) and
what its wrist camera sees. Scoring-only facts live on the simulated backend (MujocoBackend.truth) and never pass
through this interface.

What a backend states about itself:
- dt_s: the measured period of its control loop, s. There is no default: a backend that states none raises on the
  read, and a session cannot open on it. dt is the older spelling of the same number (a backend may set either).
- is_hardware: False on a simulator. On hardware the robot measures itself only when its owner asked for that.
- read()["fast"] (optional): {"dt_s", "qd": k x n joint speeds at the backend's fastest rate over the last control
  period, "wrist_n": k x 3 where it has a wrist force (what the robot puts on the world after each of those
  samples, tared as the period's own reading is)} or absent (then a stop inside one period is unresolved).
- depth(camera) (optional): the camera's depth picture, metres along the optical axis (h x w), or None when the
  camera gives none; camera_info(camera)["depth"] = {"sigma_m_at_1m", "grows": "square", "source"} states its
  error (absent: no depth, or an error nobody stated).
- contact_model() (optional): {"kind": "damped", "b_per_s": ...} where the backend has a model of its tool's contact,
  else None (the robot then measures its stops).
- servo_law() (optional): {"kind": "joint_pd" or the backend's own word, "kp", "kd", ...} or None (then the body
  model's joint servo is assumed, and said). A backend that keeps its law as a plain dict attribute of that name is
  read the same way: servo_law_of(backend) below takes both.
"""

from abc import ABC, abstractmethod
from numbers import Real

import numpy as np


class StatedLaw(dict):
    """A servo law a backend keeps as a plain dict attribute: it still reads as that dict, and called it gives the
    form of servo_law() (the same entries, with kind "joint_pd" where the dict names none)."""

    def __call__(self):
        out = dict(self)
        out.setdefault("kind", "joint_pd")
        return out


class Backend(ABC):
    is_hardware = False

    def __setattr__(self, name, value):
        if name == "servo_law" and isinstance(value, dict) and not isinstance(value, StatedLaw):
            value = StatedLaw(value)
        object.__setattr__(self, name, value)

    @property
    def dt_s(self):
        """The measured period of the control loop, s."""
        value = self.__dict__.get("_dt_s")
        if value is None:   # a subclass that states dt as a plain number of its own (on the instance or the class)
            typed = self.__dict__.get("dt", getattr(type(self), "dt", None))
            value = typed if isinstance(typed, Real) and not isinstance(typed, bool) else None
        if value is None:
            raise AttributeError("this backend states no loop period: set dt_s (s), the measured period of its "
                                 "control loop; nothing stands in for it")
        return float(value)

    @dt_s.setter
    def dt_s(self, value):
        self.__dict__["_dt_s"] = float(value)

    dt = dt_s   # the older spelling: reads and writes the same number

    @abstractmethod
    def read(self) -> dict:
        """{"t": s since start, "q": rad, "qd": rad/s, "tau": N m applied, "load": tau / torque_limit (signed),
        "gripper": None | {"q", "target", "open_frac"}} and, where the backend has a faster signal, "fast"."""

    @abstractmethod
    def command(self, q_target, torque_frac=1.0, gripper=None) -> None:
        """Servo goal per arm joint (rad, clipped to the joint limits), the torque cap as a fraction of each joint's
        limit (0 to 1), and the gripper: None keeps it, "open" | "close" | a fraction 0 (closed) to 1 (open)."""

    @abstractmethod
    def step(self, n=1) -> None:
        """Advance n control periods with the goal held."""

    @abstractmethod
    def render(self, camera="wrist") -> np.ndarray:
        """H x W x 3 uint8 frame."""

    @abstractmethod
    def camera_info(self, camera="wrist") -> dict:
        """Intrinsics only: {"name", "width", "height", "fovy_deg", "fx", "fy", "cx", "cy", "model"}. The camera
        pose comes from the body model (RobotModel.fk), as on a real arm."""

    def depth(self, camera="wrist"):
        """The depth picture of that camera: metres along the optical axis, h x w, the same instant as the frame
        render(camera) last gave where the body takes both at once. None: this camera gives no depth."""
        return None

    def contact_model(self):
        """The backend's own model of its tool's contact, or None when it has none."""
        return None

    def servo_law(self):
        """The servo law the backend runs, or None when it states none."""
        return None

    def sim_time(self) -> float:
        return float(self.read()["t"])


def period_of(backend):
    """The backend's loop period (s): dt_s, else the older dt; raises AttributeError when it states neither (a
    backend that is not a Backend subclass is read the same way)."""
    for name in ("dt_s", "dt"):
        value = getattr(backend, name, None)
        if isinstance(value, Real) and not isinstance(value, bool):
            return float(value)
    raise AttributeError("this backend states no loop period (dt_s)")


def _stated(backend, name):
    got = getattr(backend, name, None)
    if got is None:
        return None
    if callable(got):
        try:
            got = got()
        except TypeError:
            return None
    return dict(got) if isinstance(got, dict) else None


def servo_law_of(backend):
    """servo_law() of any backend: the method's answer, a plain dict attribute of that name, or None."""
    law = _stated(backend, "servo_law")
    if law is not None:
        law.setdefault("kind", "joint_pd")
    return law


def contact_model_of(backend):
    """contact_model() of any backend, or None."""
    return _stated(backend, "contact_model")


def fast_of(reading):
    """The fast signal of one read(): {"dt_s", "qd", "wrist_n" or absent}, or None when the backend gave none."""
    fast = reading.get("fast") if isinstance(reading, dict) else None
    if not isinstance(fast, dict) or fast.get("qd") is None or fast.get("dt_s") is None:
        return None
    return fast


def depth_of(backend, camera):
    """(depth picture m or None, its stated error {"sigma_m_at_1m", "grows", "source"} or None) of any backend's
    camera: a backend with no depth method, or one that fails, gives (None, None)."""
    fn = getattr(backend, "depth", None)
    if not callable(fn):
        return None, None
    try:
        z = fn(camera)
        info = backend.camera_info(camera) if z is not None else None
    except Exception:
        return None, None
    if z is None:
        return None, None
    stated = info.get("depth") if isinstance(info, dict) else None
    return np.asarray(z, dtype=float), (dict(stated) if isinstance(stated, dict) else None)
