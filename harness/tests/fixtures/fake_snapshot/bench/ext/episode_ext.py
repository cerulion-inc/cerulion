# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for the measured launcher (python -m bench.ext.episode_ext start | score | stop), for the sim
server's tests: no simulator runs. Every call is appended to the JSON lines file FAKE_SIM_LOG with its arguments
and the environment the server gave it. FAKE_SIM_FAIL=start makes start fail."""
import json
import os
import sys

ENV_KEYS = ("PYTHONPATH", "LIBERO_BX", "RCPX_SELFKNOW_DIR", "MUJOCO_GL", "PYOPENGL_PLATFORM", "RCPX_EXT_ENVHOST",
            "PLAYGROUND_SNAPSHOT", "OMP_NUM_THREADS")


def arg(argv, name):
    return argv[argv.index(name) + 1]


def main(argv):
    verb, ep = argv[0], arg(argv, "--ep")
    with open(os.environ["FAKE_SIM_LOG"], "a", encoding="utf-8") as f:
        f.write(json.dumps({"verb": verb, "argv": argv, "cwd": os.getcwd(),
                            "env": {k: os.environ.get(k) for k in ENV_KEYS}}) + "\n")
    if verb == "start":
        if os.environ.get("FAKE_SIM_FAIL") == "start":
            print("the fixture refuses to start\nscene not found", file=sys.stderr)
            return 3
        os.makedirs(os.path.join(ep, "frames"), exist_ok=True)
        with open(os.path.join(ep, "episode.json"), "w", encoding="utf-8") as f:
            json.dump({"pid": None, "cell": arg(argv, "--cell")}, f)
        with open(os.path.join(ep, "r2_problems.json"), "w", encoding="utf-8") as f:
            json.dump({"absent": []}, f)
        return 0
    if verb == "score":
        claimed = os.path.exists(os.path.join(ep, "episode_end.json"))
        print("scoring\n" + json.dumps({"success": claimed, "claimed_success": claimed, "false_claim": False,
                                        "steps_used": 7, "first_success_step": 5 if claimed else None,
                                        "budget_steps": 900, "success_within_ref": claimed, "ref_max_steps": 220,
                                        "end_reason": "claim" if claimed else None}))
        return 0
    if verb == "stop":
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
