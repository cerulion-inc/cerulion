# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for python -m cerulion_harness.body_record, for tests/test_run_cell_episode.py. By
FAKE_BODY_MODE:

    ok      prints a found record, with whether this process writes bytecode, and exits 0
    fail    prints a row with its problem and exits 1
    garbage prints what is not its JSON and exits 1
    hang    writes FAKE_BODY_READY and waits; on SIGINT writes FAKE_BODY_CLEANED, as the real one's own clean-up
            stops its simulator, and exits 130
    deaf    as hang, with SIGINT ignored: only SIGKILL ends it
"""
import json
import os
import signal
import sys
import time


def main():
    mode = os.environ["FAKE_BODY_MODE"]
    if mode == "ok":
        print(json.dumps({"rows": [{"cell": "c", "record": "found"}], "measured": 0, "found": 1, "problems": 0,
                          "dont_write_bytecode": sys.dont_write_bytecode}))
        return 0
    if mode == "garbage":
        print("Traceback (most recent call last):")
        return 1
    if mode == "fail":
        print(json.dumps({"rows": [{"cell": "c", "problem": "the simulator did not start"}], "problems": 1}))
        return 1
    if mode == "deaf":
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    ready = os.environ["FAKE_BODY_READY"]
    with open(ready + ".tmp", "w", encoding="utf-8") as f:
        f.write(str(os.getpid()))
    try:
        # FAKE_BODY_READY appears only here, inside the try: a SIGINT sent once it exists runs the clean-up.
        os.replace(ready + ".tmp", ready)
        time.sleep(600)
    except KeyboardInterrupt:
        with open(os.environ["FAKE_BODY_CLEANED"], "w", encoding="utf-8") as f:
            f.write("cleaned")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
