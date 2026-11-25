# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for the measured code's frozen cell lists: the ids in FAKE_SEALED (comma-separated) are sealed."""
import os


def sealed(cid, lists_dir=None):
    return cid in os.environ.get("FAKE_SEALED", "").split(",")
