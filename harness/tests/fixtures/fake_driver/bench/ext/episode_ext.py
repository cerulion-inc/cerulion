# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for the episode stop command, for tests/test_run_cell_episode.py: `stop --ep EP` ends the stand-in
episode server whose pid EP/episode.json holds and appends EP to the file FAKE_STOP_MARK names. With
FAKE_STOP_HANG set it is a stop that cannot end its episode: it waits, and leaves the server running. With
FAKE_STOP_EXIT set it exits with that code, as the real one exits 1 when something it stops is still running."""
import json
import os
import signal
import sys
import time


def main(argv):
    if argv[:2] != ["stop", "--ep"] or len(argv) != 3:
        return 2
    ep = argv[2]
    if os.environ.get("FAKE_STOP_HANG"):
        time.sleep(600)
    with open(os.path.join(ep, "episode.json"), encoding="utf-8") as f:
        pid = json.load(f)["pid"]
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    with open(os.environ["FAKE_STOP_MARK"], "a", encoding="utf-8") as f:
        f.write(ep + "\n")
    return int(os.environ.get("FAKE_STOP_EXIT", "0"))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
