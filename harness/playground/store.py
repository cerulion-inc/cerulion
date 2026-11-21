# SPDX-License-Identifier: AGPL-3.0-only
"""What is kept of an episode: its transcript and its pictures, on disk, for the viewer page."""
import contextlib
import json
import os
import shutil
import time

from .common import EID_RE, ENDED, read_json, utc


class Store:
    """KEEP_ROOT/ID/: meta.json (the head of the viewer page), events.jsonl (the transcript: every call, result,
    claim and status line, appended as it happens), frames/ (the pictures the results named). Everything the
    viewer shows comes from here, so a page outlives the episode, the simulator's work folder and the server.

    The sweep: an episode that ended more than keep_full_pictures_s ago keeps its transcript and its last
    keep_last_pictures pictures; one that ended more than keep_viewer_s ago is removed whole."""

    def __init__(self, settings):
        self.s, self.root = settings, settings.keep_root
        os.makedirs(self.root, exist_ok=True)

    def dir(self, eid):
        """The folder of an episode id, or None for a string that is not an episode id (a path, a dot)."""
        return os.path.join(self.root, eid) if isinstance(eid, str) and EID_RE.match(eid) else None

    def begin(self, ep):
        os.makedirs(os.path.join(self.dir(ep.eid), "frames"), exist_ok=True)
        self.write_meta(ep)

    def write_meta(self, ep):
        tmp = os.path.join(self.dir(ep.eid), "meta.json.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(ep.meta(), f)
        os.replace(tmp, os.path.join(self.dir(ep.eid), "meta.json"))

    def append(self, eid, event):
        with open(os.path.join(self.dir(eid), "events.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(event) + "\n")

    def add_frame(self, eid, source):
        target = os.path.join(self.dir(eid), "frames", os.path.basename(source))
        with contextlib.suppress(OSError):
            shutil.copyfile(source, target)

    def frame_path(self, eid, name):
        """The kept picture of that name, or None (also for a name that tries to leave the frames folder)."""
        folder = self.dir(eid)
        if not folder or not isinstance(name, str) or name != os.path.basename(name) or name in ("", ".", ".."):
            return None
        path = os.path.join(folder, "frames", name)
        return path if os.path.isfile(path) else None

    def events(self, eid):
        out = []
        folder = self.dir(eid)
        if not folder:
            return out
        try:
            with open(os.path.join(folder, "events.jsonl"), encoding="utf-8") as f:
                for line in f:
                    with contextlib.suppress(ValueError):
                        out.append(json.loads(line))
        except OSError:
            pass
        return out

    def ended_epoch(self, eid, meta, now=None):
        """When the episode ended. One with no recorded end (the server was lost mid-episode) counts as ended at
        its last transcript line, once that is older than any episode can run."""
        if meta.get("ended_epoch"):
            return float(meta["ended_epoch"])
        try:
            last = os.path.getmtime(os.path.join(self.dir(eid), "events.jsonl"))
        except OSError:
            last = os.path.getmtime(self.dir(eid))
        now = time.time() if now is None else now
        return last if now - last > self.s.clock_s + 600 else None

    def load(self, eid, since=0):
        """The viewer's document for an episode that is not running in this server, or None."""
        folder = self.dir(eid)
        if not folder or not os.path.isdir(folder):
            return None
        meta = read_json(os.path.join(folder, "meta.json"))
        if not meta:
            return None
        events = self.events(eid)
        if meta.get("status") not in ENDED:
            if self.ended_epoch(eid, meta) is None:
                meta.update(status="away", why="This episode is not running in this server right now. If the "
                                               "server was restarted, the episode was stopped and this page "
                                               "shows what was recorded up to then.")
            else:
                meta.update(status="ended", reason="server_lost",
                            why="The sim server stopped without closing this episode. This page shows what was "
                                "recorded up to then.")
        return {**meta, "events": events[since:], "events_total": len(events)}

    def last_pictures(self, events):
        """(the last keep_last_pictures pictures a transcript named, newest first; how many it named in all)."""
        named = [f for e in events for f in e.get("frames") or []]
        keep = []
        for name in reversed(named):
            if name not in keep and len(keep) < self.s.keep_last_pictures:
                keep.append(name)
        return keep, len(set(named))

    def sweep(self, now=None, live=()):
        """Apply the keep rules once. now: seconds since the epoch (a test passes a later time). live: the ids of
        the episodes running now, never touched."""
        now = time.time() if now is None else now
        done = {"trimmed": 0, "pictures_removed": 0, "removed": 0, "kept": 0}
        for eid in sorted(os.listdir(self.root)):
            folder = self.dir(eid)
            if not folder or not os.path.isdir(folder) or eid in live:
                continue
            meta = read_json(os.path.join(folder, "meta.json"))
            ended = self.ended_epoch(eid, meta, now)
            if ended is None:
                continue
            age = now - ended
            if age > self.s.keep_viewer_s:
                shutil.rmtree(folder, ignore_errors=True)
                done["removed"] += 1
                continue
            done["kept"] += 1
            if age > self.s.keep_full_pictures_s and not meta.get("pictures_trimmed"):
                keep, _ = self.last_pictures(self.events(eid))
                removed = 0
                frames = os.path.join(folder, "frames")
                for name in os.listdir(frames) if os.path.isdir(frames) else []:
                    if name not in keep:
                        with contextlib.suppress(OSError):
                            os.remove(os.path.join(frames, name))
                            removed += 1
                meta["pictures_trimmed"] = {"utc": utc(now), "removed": removed, "kept": sorted(keep)}
                with open(os.path.join(folder, "meta.json"), "w", encoding="utf-8") as f:
                    json.dump(meta, f)
                done["trimmed"] += 1
                done["pictures_removed"] += removed
        return done
