# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for the body record step (python -m BODY_RECORD_MODULE --for-cell CELL --records DIR --work DIR),
for the sim server's tests: the first call for a cell measures (it writes CELL.json into the records folder), a
later one finds that file. Its status lines go to stderr, its one JSON document to stdout."""
import json
import os
import sys


def main(argv):
    cell, records = argv[argv.index("--for-cell") + 1], argv[argv.index("--records") + 1]
    path = os.path.join(records, cell + ".json")
    found = os.path.exists(path)
    if not found:
        print(f"measuring the robot body of {cell} once", file=sys.stderr)
        print("  step 1 of 1", file=sys.stderr)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"cell": cell}, f)
    row = {"record": path, "measured_now": not found, "checked": True, "key": "fixture",
           "self_test_wall_seconds": 0.0 if found else 1.5, "wall_seconds": 0.0 if found else 2.5}
    print(json.dumps({"rows": [row]}))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
