# SPDX-License-Identifier: AGPL-3.0-only
"""The measured operator's own code, run for tests/test_picture_mode.py in a Python process of its own, so the
measured bench package never meets the stand-in bench packages of the other tests and writes no bytecode into the
measured tree (the test starts it with python -I -B). Standard library only: rcpx.server, the episode server module
the operator imports for two file names, is stood in for by those two names.

    python -I -B measured_operator.py MEASURED_RCP history < SCENARIO
    python -I -B measured_operator.py MEASURED_RCP flags < WORDS

history   SCENARIO is {"history": [the arguments of History], "steps": [{"add_turn": [message, ...]} or
          {"build": limit_tokens}, ...]}. Prints {"builds": [[messages, trimmed, estimate] of each build], "counts":
          History's counts at the end}, or {"error": text} when History refuses its arguments.
flags     WORDS is a list of command-line words. Prints what the operator's own parser made of the picture flags.
"""
import json
import sys
import types


def stand_in_episode_server():
    server = types.ModuleType("rcpx.server")
    server.END_NAME, server.SOCK_NAME = "episode_end.json", "rcp.sock"
    package = types.ModuleType("rcpx")
    package.server = server
    sys.modules["rcpx"], sys.modules["rcpx.server"] = package, server


def history(scenario):
    from bench.operators.chat_support import History
    try:
        h = History(*scenario["history"])
    except ValueError as exc:
        return {"error": str(exc)}
    builds = []
    for step in scenario["steps"]:
        if "add_turn" in step:
            h.add_turn(step["add_turn"])
        else:
            # Copied at once: a later build edits the history the messages came from.
            builds.append(json.loads(json.dumps(list(h.build(step["build"])))))
    return {"builds": builds, "counts": h.counts}


def flags(words):
    from bench.operators.chat_operator import build_parser
    ns = build_parser().parse_args(words)
    return {"image_mode": ns.image_mode, "cache_prefix_mark": ns.cache_prefix_mark,
            "image_high_water": ns.image_high_water, "keep_images": ns.keep_images}


def main():
    measured, what = sys.argv[1], sys.argv[2]
    stand_in_episode_server()
    sys.path.insert(0, measured)
    print(json.dumps({"history": history, "flags": flags}[what](json.load(sys.stdin))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
