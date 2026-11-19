# SPDX-License-Identifier: AGPL-3.0-only
"""A stand-in for the episode driver run-cell starts, for tests/test_run_cell_episode.py: no simulator and no
model. It takes the flags run-cell passes and, by FAKE_DRIVER_MODE:

    episode   plays one episode of two model replies through --base-url as the measured operator does: each
              body is json.dumps of the model, the history, the tools and the fixed fields, with --extra-body
              merged in; the history carries each tool call (and its extra_content) back; a log record per reply
              and a truth log line per control step, in the attempt folder the real driver uses; then the scored
              result and the kept log.
    hang      starts a stand-in episode server in a session of its own, notes its pid where the episode stop
              command reads it, writes its own pid to FAKE_DRIVER_READY and waits to be stopped.
"""
import argparse
import json
import os
import shlex
import shutil
import subprocess
import sys
import time
import urllib.request

FLAGS = ("--model", "--base-url", "--key-env", "--cells", "--only", "--label", "--results", "--op-args", "--clock",
         "--max-attempts", "--retry-wait", "--egl-slots", "--budget-requests", "--budget-steps", "--arm",
         "--packages", "--work", "-j")
SYSTEM = {"role": "system", "content": "You operate a robot."}
USER = {"role": "user", "content": "Task: pick the bowl."}
TOOLS = [{"type": "function", "function": {"name": "move_to", "parameters": {"type": "object"}}}]
STEPS = (30, 12)


def extra_body(op_args):
    words = shlex.split(op_args)
    return json.loads(words[words.index("--extra-body") + 1])


def post(url, key, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode("utf-8"), method="POST",
                                 headers={"Content-Type": "application/json", "Accept": "application/json",
                                          "Authorization": "Bearer " + key})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))   # the relay is on the loopback address
    with opener.open(req, timeout=60) as resp:
        return json.loads(resp.read())


def append(path, rec):
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(rec) + "\n")


def episode(ns, group, ep, scoring):
    log, truth = os.path.join(ep, "operator.jsonl"), os.path.join(scoring, "truth_log.jsonl")
    append(truth, {"event": "reset", "ep": 0})
    url = ns.base_url.rstrip("/") + "/chat/completions"
    key = os.environ[ns.key_env]
    messages, step = [SYSTEM, USER], 0
    for turn, steps in enumerate(STEPS, start=1):
        body = {"model": ns.model, "messages": messages, "tools": TOOLS, "tool_choice": "auto", "temperature": 1.0,
                "max_tokens": 32000, "seed": 1, "parallel_tool_calls": False}
        body.update(extra_body(ns.op_args))
        append(log, {"kind": "request", "turn": turn - 1})
        reply = post(url, key, body)
        message = reply["choices"][0]["message"]
        call = message["tool_calls"][0]
        append(log, {"kind": "reply", "turn": turn, "usage": reply.get("usage")})
        item = {"id": call["id"], "type": "function", "function": call["function"]}
        if "extra_content" in call:
            item["extra_content"] = call["extra_content"]
        messages = messages + [{"role": "assistant", "content": message.get("content") or "", "tool_calls": [item]},
                               {"role": "tool", "tool_call_id": call["id"], "content": "moved"}]
        for _ in range(steps):
            step += 1
            append(truth, {"step": step, "success": False, "ep": 0})
    bench = open(ns.cells, encoding="utf-8").read().split()[1]
    results = os.path.join(os.path.abspath(ns.results), bench, f"{ns.label}_{ns.arm}")
    kept = os.path.join(results, "_episodes", ns.only, "a1")
    os.makedirs(kept)
    shutil.copy2(log, kept)
    with open(os.path.join(results, f"{ns.only}.json"), "w", encoding="utf-8") as f:
        json.dump({"success": True, "first_success_step": step, "success_within_ref": True, "claimed_success": True,
                   "false_claim": False, "model_requests": len(STEPS), "steps_used": step, "end_reason": "done",
                   "grid": {"attempt": 1, "operator_wall_s": 1.5}}, f)


def hang(ns, group, ep, scoring):
    server = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"], start_new_session=True,
                              stdin=subprocess.DEVNULL)
    with open(os.path.join(ep, "episode.json"), "w", encoding="utf-8") as f:
        json.dump({"pid": server.pid}, f)
    with open(os.environ["FAKE_DRIVER_READY"] + ".tmp", "w", encoding="utf-8") as f:
        json.dump({"driver": os.getpid(), "server": server.pid, "ep": ep}, f)
    os.replace(os.environ["FAKE_DRIVER_READY"] + ".tmp", os.environ["FAKE_DRIVER_READY"])
    time.sleep(600)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("cmd")
    for flag in FLAGS:
        p.add_argument(flag)
    p.add_argument("--retry-failed", action="store_true")
    ns = p.parse_args()
    group = os.path.join(os.path.abspath(ns.work), f"{ns.label}_{ns.arm}")
    ep, scoring = os.path.join(group, f"{ns.only}_a1"), os.path.join(group, "_scoring", f"{ns.only}_a1")
    os.makedirs(ep)
    os.makedirs(scoring)
    {"episode": episode, "hang": hang}[os.environ["FAKE_DRIVER_MODE"]](ns, group, ep, scoring)
    return 0


if __name__ == "__main__":
    sys.exit(main())
