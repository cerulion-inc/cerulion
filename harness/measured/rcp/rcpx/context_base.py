# SPDX-License-Identifier: AGPL-3.0-only
"""Shared pieces of the session context (rcpx/context.py re-exports every name here): the one refusal shape,
plain JSON conversion, and the ground truth guard. Nothing here reads simulator ground truth."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

TRUTH_MARKER = "truth"  # any record key containing this is ground truth and never journaled
FRAME_KEYS = ("frame", "annotated", "image", "picture")


def refusal(check: str, why: str, hint: str) -> dict:
    """The one refusal shape (SPEC section 0)."""
    return {"refused": {"check": check, "why": why, "hint": hint}}


def jsonable(x):
    """Plain JSON types: numpy to lists and scalars, tuples to lists, sets sorted, NaN to None."""
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if isinstance(x, (set, frozenset)):
        return sorted(jsonable(v) for v in x)
    if isinstance(x, np.ndarray):
        return jsonable(x.tolist())
    if isinstance(x, np.generic):
        return jsonable(x.item())
    if isinstance(x, bool) or x is None or isinstance(x, (int, str)):
        return x
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, Path):
        return str(x)
    return str(x)


class GroundTruthLeak(ValueError):
    """A record handed to the Journal carries simulator ground truth."""


def _check_no_truth(obj, where="record"):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if TRUTH_MARKER in str(k).lower():
                raise GroundTruthLeak(f"{where}.{k} names ground truth; the journal never stores it")
            _check_no_truth(v, f"{where}.{k}")
    elif isinstance(obj, (list, tuple)):
        for i, v in enumerate(obj):
            _check_no_truth(v, f"{where}[{i}]")


def _is_image(a) -> bool:
    return isinstance(a, np.ndarray) and a.dtype == np.uint8 and a.ndim == 3 and a.shape[2] in (3, 4)
