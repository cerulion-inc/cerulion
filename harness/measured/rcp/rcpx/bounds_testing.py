# SPDX-License-Identifier: AGPL-3.0-only
"""Test-only injection of derived values.

Nothing under rcpx, bench/ext or bench/operators may import this module.
"""

from rcpx.bounds import of
import numpy as np


def _save(B):
    if not hasattr(B, "_testing_saved"):
        B._testing_saved = B._override.copy(), B.know, B.model_know, B.space


def override(ex, **values):
    B = of(ex)
    _save(B)
    B._override.update(values)
    return B


class _Know:
    """Test knowledge in front of the session's own: named values (a callable is called with the getter's own
    arguments), never persisted and never written into the session's SelfKnow."""

    def __init__(self, base):
        self._base, self.values, self.ready = base, {}, True

    def set(self, name, value, source="self_test", at=None):
        self.values[name] = value, source

    def get(self, name, *args):
        if name in self.values:
            value, source = self.values[name]
            return (value(*args) if callable(value) else value), source, None
        getter = getattr(self._base, "get", None)
        return getter(name, *args) if callable(getter) else (None, None, None)

    def note_false_stop(self, stop):
        old = self.get("f_hidden", stop.get("v", 0.0), 0.0, stop.get("held"))[0] or 0.0
        self.set("f_hidden", max(old, stop["peak_n"]), "self_test")

    def __getattr__(self, name):
        return getattr(self._base, name)


def know(ex, **values):
    B = of(ex)
    _save(B)
    ready = values.pop("ready", True)
    if not isinstance(B.know, _Know):
        B.know = _Know(B.know)
    for name, value in values.items():
        B.know.set(name, value, "self_test")
    B.know.ready = ready
    return B


class _Overlay:
    def __init__(self, space, region):
        self.space, self.region = space, region

    def __getattr__(self, name):
        return getattr(self.space, name)

    def kind(self, p0, p1, radius):
        inside = self.region == "all" if isinstance(self.region, str) else False
        if isinstance(self.region, dict):
            lo, hi = (np.asarray(v, dtype=float) for v in self.region["box_m"])
            inside = all(np.all(np.asarray(p) - radius >= lo) and np.all(np.asarray(p) + radius <= hi) for p in (p0, p1))
        return ("free", None, float("inf"), "swept") if inside else self.space.kind(p0, p1, radius)


def free(ex, region):
    B = of(ex)
    _save(B)
    B.space = _Overlay(B.space, region)
    return B


def book(ex, space):
    B = of(ex)
    _save(B)
    B.space = space
    return B


def restore(ex):
    B = of(ex)
    if hasattr(B, "_testing_saved"):
        B._override, B.know, B.model_know, B.space = B._testing_saved
        del B._testing_saved
    B._sources.clear()
    return B
