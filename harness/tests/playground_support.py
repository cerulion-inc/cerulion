# SPDX-License-Identifier: AGPL-3.0-only
"""Shared by the sim server's tests: the import path, a settings file under a temporary folder, and an
in-process simulator that runs nothing."""
import json
import os
import shutil
import sys
import tempfile

HARNESS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURE = os.path.join(HARNESS, "tests", "fixtures", "fake_snapshot")
if HARNESS not in sys.path:
    sys.path.insert(0, HARNESS)

from playground import settings as settings_mod  # noqa: E402

with open(settings_mod.DEFAULT_SETTINGS, encoding="utf-8") as _f:
    SHIPPED = json.load(_f)


class Folder:
    """A temporary folder; work_root is a second, short one under /tmp (socket paths must stay short)."""

    def __init__(self):
        self.path = tempfile.mkdtemp(prefix="pgtest")
        self.work = tempfile.mkdtemp(prefix="pgw", dir="/tmp")

    def at(self, *parts):
        return os.path.join(self.path, *parts)

    def remove(self):
        shutil.rmtree(self.path, ignore_errors=True)
        shutil.rmtree(self.work, ignore_errors=True)


def write_settings(folder, **changes):
    """A settings file of the shipped keys with every path under folder; changes replace keys (None removes)."""
    doc = dict(SHIPPED)
    os.makedirs(folder.at("bx"), exist_ok=True)
    os.makedirs(folder.at("records"), exist_ok=True)
    doc.update({"snapshot_root": FIXTURE, "bench_root": folder.at("bx"), "body_records": folder.at("records"),
                "data_dir": folder.at("data"), "work_root": folder.work, "python": sys.executable,
                "body_record_module": "fake_body_record",
                "macos_envhost": [os.path.join(FIXTURE, "bench", "ext", "episode_ext.py"), "{runtime}"]})
    for key, value in changes.items():
        if value is None:
            doc.pop(key, None)
        else:
            doc[key] = value
    path = folder.at("settings.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f)
    return path


class FakeBridge:
    listing = {"tools": [{"name": "look"}]}


class FakeSim:
    """The pool's simulator in-process: start sets a bridge (or raises fail_start), score answers verdict, and
    every call is kept in calls."""

    def __init__(self, verdict=None, fail_start=None):
        self.verdict, self.fail_start, self.calls = verdict or {}, fail_start, []

    async def start(self, ep):
        self.calls.append(("start", ep.eid))
        if self.fail_start:
            raise RuntimeError(self.fail_start)
        os.makedirs(ep.ep_dir, exist_ok=True)
        ep.bridge, ep.practices = FakeBridge(), "practices"

    async def score(self, ep_dir):
        self.calls.append(("score", ep_dir))
        return dict(self.verdict)

    async def stop(self, ep_dir):
        self.calls.append(("stop", ep_dir))
