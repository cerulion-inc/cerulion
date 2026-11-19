# SPDX-License-Identifier: AGPL-3.0-only
"""The record of ended episodes: one SQLite file in the data folder."""
import json
import logging
import math
import os
import sqlite3
import threading

log = logging.getLogger("cerulion.sim")


class History:
    """DATA_DIR/episodes.sqlite: one row for every ended episode, and every gap between calls of those episodes
    (what the idle limit is computed from once there are more of them than the settings' stated source).

    record never raises: a row that cannot be written is reported on stderr and the episode ends as it would.
    gap_count and gap_percentile answer what was read last when the file cannot be read."""

    KEYS = ("id", "state", "started", "ended", "reason", "sim_wall_s", "sim_cpu_s", "peak_rss_mb", "calls",
            "verdict")
    TABLES = ("CREATE TABLE IF NOT EXISTS episodes (id TEXT PRIMARY KEY, state TEXT, started TEXT, ended TEXT, "
              "reason TEXT, sim_wall_s REAL, sim_cpu_s REAL, peak_rss_mb REAL, calls INTEGER, verdict TEXT)",
              "CREATE TABLE IF NOT EXISTS gaps (episode TEXT, n INTEGER, gap_s REAL)",
              "CREATE INDEX IF NOT EXISTS gaps_by_length ON gaps (gap_s)")

    def __init__(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path, self.lock = path, threading.Lock()
        self.db = sqlite3.connect(path, isolation_level=None, check_same_thread=False)
        for table in self.TABLES:
            self.db.execute(table)
        self.failed = 0
        self._gap_count, self._gap_at = 0, {}

    def record(self, row, gaps=()):
        """Write an ended episode (row: KEYS; verdict a dict or None) and its gaps between calls, once per id: a
        second record of the same id changes nothing. True when the row is in the file."""
        values = dict(row, verdict=json.dumps(row["verdict"]) if row.get("verdict") else None)
        try:
            with self.lock:
                self.db.execute("BEGIN IMMEDIATE")
                try:
                    cursor = self.db.execute(
                        f"INSERT INTO episodes ({', '.join(self.KEYS)}) VALUES ({','.join('?' * len(self.KEYS))}) "
                        "ON CONFLICT (id) DO NOTHING", tuple(values.get(k) for k in self.KEYS))
                    if cursor.rowcount == 1:
                        self.db.executemany("INSERT INTO gaps (episode, n, gap_s) VALUES (?, ?, ?)",
                                            [(row["id"], n, float(g)) for n, g in enumerate(gaps)])
                    self.db.execute("COMMIT")
                except BaseException:
                    self.db.execute("ROLLBACK")
                    raise
        except sqlite3.Error as exc:
            self.failed += 1
            log.warning("episode %s not written to %s: %s", row.get("id"), self.path, exc)
            return False
        return True

    def gap_count(self):
        try:
            with self.lock:
                self._gap_count = int(self.db.execute("SELECT COUNT(*) FROM gaps").fetchone()[0])
        except sqlite3.Error:
            pass
        return self._gap_count

    def gap_percentile(self, percentile):
        """The nearest-rank percentile of the recorded gaps (the smallest gap with at least that share of the gaps
        at or below it), or None when there are none."""
        count = self.gap_count()
        if not count:
            return None
        rank = max(1, math.ceil(float(percentile) / 100.0 * count))
        try:
            with self.lock:
                found = self.db.execute("SELECT gap_s FROM gaps ORDER BY gap_s LIMIT 1 OFFSET ?",
                                        (rank - 1,)).fetchone()
            if found is not None:
                self._gap_at[float(percentile)] = float(found[0])
        except sqlite3.Error:
            pass
        return self._gap_at.get(float(percentile))

    def episodes(self):
        """Every ended episode, in the order they ended."""
        with self.lock:
            rows = self.db.execute(f"SELECT {', '.join(self.KEYS)} FROM episodes ORDER BY rowid").fetchall()
        out = []
        for r in rows:
            doc = dict(zip(self.KEYS, r))
            doc["verdict"] = json.loads(doc["verdict"]) if doc["verdict"] else None
            out.append(doc)
        return out

    def state(self):
        """For /health: the file, and how many rows could not be written to it."""
        return {"file": self.path, "not_written": self.failed}
