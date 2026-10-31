# SPDX-License-Identifier: AGPL-3.0-only
import json, os


class CbLog:
    """Per-node callback log: one JSON line per callback, in callback order."""

    def __init__(self, node_name):
        d = os.environ.get("PROBE_OUT", ".")
        os.makedirs(d, exist_ok=True)
        self.f = open(os.path.join(d, f"{node_name}.jsonl"), "w", buffering=1)
        self.i = 0

    def log(self, **kw):
        kw["i"] = self.i
        self.i += 1
        self.f.write(json.dumps(kw, sort_keys=True) + "\n")
