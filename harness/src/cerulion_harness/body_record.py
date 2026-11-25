# SPDX-License-Identifier: AGPL-3.0-only
"""Measure this machine's own body record, once.

    python -m cerulion_harness.body_record [--for-cell ID | --cell ID | --all] [--check] [--records DIR]
                                           [--work DIR] [--wait SECONDS]

The Cerulion harness setup reads a record of the robot's own calibration from body_records/: its tracked joint
speeds, its lag, its braking, the noise of its force estimate, its jaws. The robot measures these about itself in
a self-test, outside every task, and a record is stored under a key that is a hash of the compiled robot body's
numbers. Those numbers come out of the physics library's model compiler, and their last digits differ between
processor families and operating systems. The three shipped records were measured on Linux x86-64. On another
platform (Linux arm64, macOS on Apple silicon) the same robot gets another key, the shipped record is not found,
and every move is refused.

This command starts the simulator, computes the key this machine gives the robot body with the same code an
episode uses, and looks for a record under it. If one is there it stops: nothing moves and nothing is written. If
none is there it runs the self-test of the Cerulion harness setup once, as the shipped records were made (one
look, then the still hold, the jaws, the sweep of every joint, the ramps and the taps onto the table), and stores
the record under this machine's key beside the shipped ones. That is a one-time cost of one to two minutes for each
robot body; every later episode reads the stored record and pays nothing.

The self-test runs outside every task, in a scene of its own. A record belongs to the robot body, not to a scene,
but the taps need free table under the hand, so the scene decides how much of the record the taps fill in. By
default the scene is the one the shipped record of that body was measured in.

A record is measured lazily: once for each robot body, the first time a start state needs it. The install
measures the body of the default start state only. cerulion rcp bench and cerulion rcp --sim ask this command
before an episode of the Cerulion harness setup starts; when the body of that start state has no record yet it is
measured then, with a message that says so and how long it took. What was checked on this machine is noted in
body_records/_checked_on_this_machine.json, so a start state that was checked once costs nothing afterwards.

(no option)    the robot body of the default start state of run-cell (the spatial and goal suites)
--for-cell ID  the robot body of this start state (a cell id as run-cell takes it): found, or measured
               now in the scene the shipped record of that body was measured in
--all          each of the three robot bodies the shipped records cover
--cell ID      run in this start state's own scene instead
--check        print the key and whether a record is stored; never measure
--records DIR  the folder of records (default: body_records/ of the harness folder)

The other way, an opt-in that measures nothing: python -m cerulion_harness.port_records writes a COPY of the
x86-64 record under this machine's key.
"""
import argparse
import json
import os
import platform
import re
import shutil
import sys
import tempfile
import time

from cerulion_harness import layout
from cerulion_harness.cli import DEFAULT_CELL, cell_line

# The three robot bodies the shipped records cover: the scene the self-test runs in (a cell id as run-cell
# takes it), the shipped record of that body (measured on Linux x86-64), and the cells that body serves.
# The first two scenes are the ones those shipped records were measured in; the third body runs in a start
# state of its own suite.
BODIES = (
    ("l10_t02_i00", "726d4e8cc7d8227186ce25893edfbeb2d71458ce", "the spatial and goal suites, and LIBERO-10 task 9"),
    ("l10_t00_i00", "91cdd2f0a4eed53d722840fad5741e8c3eb2500f", "LIBERO-10 tasks 0 and 6"),
    ("pro_object_swap_t00_i00", "60ea8887142e00f9336bcb46319d0b6a605f5780", "the object suite"),
)
ENV = {"PYTHONDONTWRITEBYTECODE": "1", "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
       "MKL_NUM_THREADS": "1", "LP_NUM_THREADS": "2"}
STEPS = 200000   # the self-test's own step budget, far above an episode's 900: it runs outside every task
PACKAGES = "all,place"   # the tool packages an episode of run-cell loads


CHECKED = "_checked_on_this_machine.json"   # not a record: the snapshot reads KEY.json and KEY.N.json only


def scene_of(cell_id):
    """The scene the self-test runs in for the robot body of this start state: the scene the shipped record of
    that body was measured in where the body is one of the three, else the start state's own scene."""
    if re.match(r"pro_(spatial|goal)_", cell_id) or re.match(r"l10_t0*9_", cell_id):
        return BODIES[0][0]
    if re.match(r"l10_t0*[06]_", cell_id):
        return BODIES[1][0]
    if re.match(r"pro_object_", cell_id):
        return BODIES[2][0]
    return cell_id


def _machine():
    return f"{platform.system()} {platform.machine()}"


def _checked(records):
    try:
        with open(os.path.join(records, CHECKED), encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return {}
    return doc if isinstance(doc, dict) and doc.get("machine") == _machine() \
        and isinstance(doc.get("cells"), dict) else {}


def checked(cell_id, records):
    """With no simulator: the note that this start state's robot body has a record on this machine, or None. A
    note counts only on the kind of machine that wrote it and while the record file it names is still there."""
    row = _checked(records).get("cells", {}).get(cell_id)
    if isinstance(row, dict) and row.get("record") and os.path.isfile(os.path.join(records, row["record"])):
        return row
    return None


def _note(cell_id, records, row):
    """Keep the note for one start state. A folder that cannot be written (a read-only image) keeps none: the
    next run then checks again with one simulator start."""
    if not (row.get("key") and row.get("path") and os.path.isfile(row["path"])):
        return
    doc = _checked(records) or {"machine": _machine(), "cells": {}}
    doc["about"] = ("Not a record. Which start states were checked on this machine and which record their robot "
                    "body reads. Written by python -m cerulion_harness.body_record; safe to delete.")
    doc["cells"][cell_id] = {"key": row["key"], "record": os.path.basename(row["path"]),
                             "checked_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    try:
        tmp = os.path.join(records, CHECKED + f".{os.getpid()}.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=2, sort_keys=True)
        os.replace(tmp, os.path.join(records, CHECKED))
    except OSError:
        pass


def for_cell(cell_id, records, work, wait_s, check, say):
    """The lazy way: the record of the robot body one start state needs. Noted earlier: nothing starts. Else one
    simulator start in that start state's own scene reads the key; a record that is there is noted; one that is
    not is measured now, once, in the scene the shipped record of that body was measured in."""
    t0 = time.time()
    seen = checked(cell_id, records)
    if seen:
        return {"cell": cell_id, "record": "found", "measured_now": False, "key": seen["key"],
                "path": os.path.join(records, seen["record"]), "checked": "earlier", "wall_seconds": 0.0}
    scene = scene_of(cell_id)
    say(f"first use of start state {cell_id} on this machine: checking the robot body's record (one simulator "
        f"start)")
    row = one(cell_id, records, work, wait_s, check or scene != cell_id, say)
    if row.get("record") == "none" and not check:
        need = row["key"]
        row = one(scene, records, work, wait_s, False, say)
        if not row.get("problem") and row.get("key") != need:
            say(f"the scene {scene} holds another robot body (key {str(row.get('key'))[:8]}): measuring in "
                f"{cell_id} itself")
            row = one(cell_id, records, work, wait_s, False, say)
        row["measured_in"], row["cell"] = row.get("cell"), cell_id
    row["checked"] = "now"
    row["wall_seconds"] = round(time.time() - t0, 1)
    if not row.get("problem") and row.get("record") in ("found", "measured"):
        _note(cell_id, records, row)
        if row.get("measured_now"):
            _note(row.get("measured_in") or cell_id, records, row)
            say(f"measured once and stored: {row.get('self_test_wall_seconds')} s of self-test, "
                f"{row['wall_seconds']} s in all. No later run on this machine pays this for this robot body.")
        else:
            say(f"found: the record of this robot body is stored ({row['wall_seconds']} s to check, once)")
    return row


def one(cell_id, records, work, wait_s, check, say):
    """The record of one robot body on this machine, read in the scene of one start state: found, or measured now
    and stored."""
    from bench.ext import episode_ext as E
    from bench.ext.libero import backend as B
    from bench.ext.libero import envproto
    from rcpx import bounds_self, bounds_selfstart, bounds_selftest

    row = {"cell": cell_id}
    t0 = time.time()
    line, cell = E.parse_cell(f"{cell_line(cell_id)} vnext {PACKAGES}")
    ep_dir = tempfile.mkdtemp(prefix="b", dir=work)
    scoring = E.scoring_dir(ep_dir)
    os.makedirs(scoring, exist_ok=True)
    env_sock = os.path.join(scoring, E.ENV_SOCK)
    cell = dict(cell, budget_steps=STEPS)
    budget = {"requests": 80, "steps": STEPS}
    E._write_json(os.path.join(scoring, E.EPISODE_NAME), {
        **line, "cell_id": cell["id"], "runtime": cell["runtime"], "base_suite": cell["base_suite"],
        "seed": cell["env_seed"], "language": cell["language"], "budget": budget, "budget_steps": STEPS,
        "max_steps_ref": cell["max_steps_ref"], "workspace": cell["workspace"], "env_sock": env_sock})
    gl = os.environ.get("MUJOCO_GL")
    pid, why = E._launch_host(cell, scoring, env_sock, gl if gl in ("egl", "osmesa") else "osmesa", wait_s)
    if pid is None:
        raise RuntimeError(f"the simulator did not start: {why}")
    try:
        with envproto.Client(env_sock, timeout_s=wait_s) as c:
            c.call("reset", cell=cell)
        env = envproto.Client(env_sock, timeout_s=wait_s)
        try:
            session = B.build_session(env, ep_dir, E.serve_facts(cell), budget, PACKAGES, selfknow="measure",
                                      selfknow_dir=records)
            ex = session.executor
            row["simulator_start_seconds"] = round(time.time() - t0, 1)
            have, loaded = bounds_selfstart.know_of(ex)
            row["key"] = bounds_self.key_of(bounds_selfstart.identity_of(ex))
            row["path"] = have.path
            if loaded and have.ready:
                row.update(record="found", measured_now=False, measured_by=have.measured_by,
                           saved_at=have.stored, wall_seconds=round(time.time() - t0, 1))
                return row
            if check:
                row.update(record="none", measured_now=False, wall_seconds=round(time.time() - t0, 1))
                return row
            say(f"no record of this robot body on this machine (key {row['key'][:8]}): the robot measures itself "
                f"now, once, in {cell_id}. It takes one to two minutes; this happens once for each robot body")
            ex.cfg["selftest_taps"] = "free"   # as the shipped records were measured: the taps climb to the free speed
            look = session.dispatch({"op": "call", "name": "look", "args": {}}).get("result") or {}
            t1 = time.time()
            out = bounds_selftest.run(ex, intent={"clear": look.get("look_id")}, directory=records,
                                      log=lambda text: say("  " + " ".join(str(text).split())[:160]))
            row.update(record="measured", measured_now=True, path=out.get("path"), measured_by=out.get("measured_by"),
                       robot_seconds=round(float(out["took_s"]), 1), control_periods=out.get("periods"),
                       self_test_wall_seconds=round(time.time() - t1, 1),
                       wall_seconds=round(time.time() - t0, 1))
            if not (out.get("path") and os.path.isfile(out["path"])):
                row["problem"] = "the self-test ended and no record file was written"
            return row
        finally:
            env.close()
    finally:
        E._stop_pid(pid, [env_sock])
        for made in (ep_dir, scoring):               # only the two folders this run made in the scratch folder
            shutil.rmtree(made, ignore_errors=True)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m cerulion_harness.body_record", description=__doc__.split("\n\n")[0])
    p.add_argument("--for-cell", default=None, help="the robot body of this start state: found, or measured now, "
                                                    f"once (default: {DEFAULT_CELL})")
    p.add_argument("--cell", default=None, help="run in this start state's own scene")
    p.add_argument("--records", default=None,
                   help="the folder of records (default: body_records/ of the harness folder)")
    p.add_argument("--all", action="store_true", help="each of the three robot bodies the shipped records cover")
    p.add_argument("--check", action="store_true", help="print the key and whether a record is stored; never measure")
    p.add_argument("--work", default="/tmp/rbb", help="scratch folder; keep it short (unix socket paths). Each run "
                                                      "makes and removes its own folder inside it")
    p.add_argument("--wait", type=float, default=1800.0, help="seconds to wait for a simulator answer")
    ns = p.parse_args(argv)
    if sum(bool(x) for x in (ns.all, ns.cell, ns.for_cell)) > 1:
        p.error("--for-cell, --cell and --all exclude each other")
    lazy = not (ns.all or ns.cell)
    records = os.path.abspath(ns.records) if ns.records else layout.RECORDS
    if lazy:
        try:
            cell_line(ns.for_cell or DEFAULT_CELL)
        except ValueError as exc:
            print(str(exc), file=sys.stderr)
            return 2
        seen = checked(ns.for_cell or DEFAULT_CELL, records)
        if seen:   # noted earlier on this machine: no simulator, nothing to load
            row = for_cell(ns.for_cell or DEFAULT_CELL, records, None, None, ns.check, None)
            print(json.dumps({"rows": [row], "measured": 0, "found": 1, "problems": 0}, indent=2))
            return 0
    if not os.path.isfile(layout.SIM_PYTHON):
        print(f"the simulator is not installed in {layout.BX}: run {os.path.join(layout.HARNESS, 'install.sh')}",
              file=sys.stderr)
        return 2
    if not os.path.isdir(layout.RCP_TREE):
        print(f"no measured code tree at {layout.RCP_TREE}", file=sys.stderr)
        return 2
    try:
        cells = [b[0] for b in BODIES] if ns.all else [ns.cell or ns.for_cell or DEFAULT_CELL]
        for c in cells:
            cell_line(c)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    os.environ.update(layout.simulator_env(os.environ))   # macOS: Apple's OpenGL (macos/envhost_macos.sh)
    for k, v in ENV.items():
        os.environ.setdefault(k, v)
    os.chdir(layout.RCP_TREE)
    sys.path.insert(0, layout.RCP_TREE)
    os.makedirs(ns.work, exist_ok=True)

    def say(text):
        print(text, file=sys.stderr, flush=True)

    rows, problems = [], 0
    try:
        for c in cells:
            try:
                row = (for_cell if lazy else one)(c, records, ns.work, ns.wait, ns.check, say)
                if not lazy and not row.get("problem") and row.get("record") in ("found", "measured"):
                    _note(c, records, row)
            except Exception as exc:   # one body's failure is reported; the others still run
                row = {"cell": c, "problem": f"{type(exc).__name__}: {exc}"[:400]}
            problems += 1 if row.get("problem") else 0
            row["serves"] = next((b[2] for b in BODIES if b[0] == scene_of(c)),
                                 "the robot body of this start state")
            rows.append(row)
    finally:
        for folder in (os.path.join(ns.work, "_scoring"), ns.work):
            try:
                os.rmdir(folder)   # the scratch folder itself goes only when nothing else is in it
            except OSError:
                pass
    print(json.dumps({"rows": rows, "measured": sum(1 for r in rows if r.get("measured_now")),
                      "found": sum(1 for r in rows if r.get("record") == "found"), "problems": problems}, indent=2))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
