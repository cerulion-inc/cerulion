# SPDX-License-Identifier: AGPL-3.0-only
"""An opt-in: COPY the shipped x86-64 body records under this machine's keys. A copy, not a measurement.

The install does not run this. On a platform whose keys differ it measures this machine's own record instead
(python -m cerulion_harness.body_record). Use this command only if you want the x86-64 calibration itself read on this
machine, for instance to compare it with the record measured here. It never writes over a record that is stored.

    python -m cerulion_harness.port_records [--probe] [--work DIR] [--wait SECONDS]

The Cerulion harness setup reads a record of the robot's own calibration from body_records/. A record is stored
under a key that is a hash of the compiled robot body's numbers. Those numbers come out of the physics library's model
compiler, and their last digits differ between processor families and operating systems. The three shipped
records were measured on Linux x86-64, so on that platform they are found as they are and this command does
nothing. On another platform (Linux arm64, macOS on Apple silicon) the same robot gets another key, the record is
not found, and every move is refused.

This command starts the simulator once for one start state of each robot mounting, computes the key this machine
gives that body with the same code an episode uses, and writes a copy of the measured record under that key
beside the original. The copy holds every measured number unchanged: only the key and the body hash differ. It is
the x86-64 calibration read by a body whose compiled numbers differ in their last digits; it is not a new
measurement. The copies are written once; a second run finds them and does nothing.

--probe prints the key of each mounting on this machine and writes nothing.
"""
import argparse
import json
import os
import shutil
import sys
import tempfile

from cerulion_harness import layout

# One start state for each robot mounting the benchmark uses, and the shipped record (measured on Linux x86-64)
# that belongs to it. Lines: BENCH SUITE TASK INIT PERTURB.
PROBES = (
    # the body of the spatial and goal suites (and of LIBERO-10 task 9)
    ("libero_pro libero_spatial_swap 0 0 swap", "726d4e8cc7d8227186ce25893edfbeb2d71458ce"),
    # the body of the object suite
    ("libero_pro libero_object_swap 0 0 swap", "60ea8887142e00f9336bcb46319d0b6a605f5780"),
    # the body of LIBERO-10 tasks 0 and 6
    ("libero libero_10 0 0 none", "91cdd2f0a4eed53d722840fad5741e8c3eb2500f"),
)
ENV = {"PYTHONDONTWRITEBYTECODE": "1", "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
       "MKL_NUM_THREADS": "1", "LP_NUM_THREADS": "2"}


def identity_here(probe, work, wait_s):
    """(identity, key) of the robot body of one start state on this machine: the simulator is started and reset
    as an episode starts it, the session is built with no record (it moves nothing), and the simulator is stopped."""
    from bench.ext import episode_ext as E
    from bench.ext.libero import backend as B
    from bench.ext.libero import envproto
    from rcpx import bounds_self, bounds_selfstart

    line, cell = E.parse_cell(f"probe {probe} vnext all,place")
    ep_dir = tempfile.mkdtemp(prefix="p", dir=work)
    scoring = E.scoring_dir(ep_dir)
    os.makedirs(scoring, exist_ok=True)
    env_sock = os.path.join(scoring, E.ENV_SOCK)
    cell = dict(cell, budget_steps=900)
    budget = {"requests": 80, "steps": 900}
    E._write_json(os.path.join(scoring, E.EPISODE_NAME), {
        **line, "cell_id": cell["id"], "runtime": cell["runtime"], "base_suite": cell["base_suite"],
        "seed": cell["env_seed"], "language": cell["language"], "budget": budget, "budget_steps": 900,
        "max_steps_ref": cell["max_steps_ref"], "workspace": cell["workspace"], "env_sock": env_sock})
    gl = os.environ.get("MUJOCO_GL")
    pid, why = E._launch_host(cell, scoring, env_sock, gl if gl in ("egl", "osmesa") else "osmesa", wait_s)
    if pid is None:
        raise RuntimeError(f"the simulator did not start: {why}")
    try:
        with envproto.Client(env_sock, timeout_s=wait_s) as c:
            c.call("reset", cell=cell)
        env = envproto.Client(env_sock)
        try:
            session = B.build_session(env, ep_dir, E.serve_facts(cell), budget, "all,place", selfknow=None,
                                      selfknow_dir=os.path.join(work, "none"))
            ident = bounds_selfstart.identity_of(session.executor)
        finally:
            env.close()
    finally:
        E._stop_pid(pid, [env_sock])
    return ident, bounds_self.key_of(ident)


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m cerulion_harness.port_records", description=__doc__.split("\n\n")[0])
    p.add_argument("--probe", action="store_true", help="print this machine's keys and write nothing")
    p.add_argument("--work", default="/tmp/rbp", help="scratch folder; keep it short (unix socket paths). Each run "
                                                      "makes and removes its own folder inside it")
    p.add_argument("--wait", type=float, default=1800.0, help="seconds to wait for a simulator start")
    ns = p.parse_args(argv)
    if not os.path.isfile(layout.SIM_PYTHON):
        print(f"the simulator is not installed in {layout.BX}: run {os.path.join(layout.HARNESS, 'install.sh')}",
              file=sys.stderr)
        return 2
    if not os.path.isdir(layout.RCP_TREE):
        print(f"no measured code tree at {layout.RCP_TREE}", file=sys.stderr)
        return 2
    os.environ.update(layout.simulator_env(os.environ))   # macOS: Apple's OpenGL (macos/envhost_macos.sh)
    for k, v in ENV.items():
        os.environ.setdefault(k, v)
    records = layout.RECORDS
    os.chdir(layout.RCP_TREE)
    sys.path.insert(0, layout.RCP_TREE)
    os.makedirs(ns.work, exist_ok=True)
    scratch = tempfile.mkdtemp(prefix="r", dir=ns.work)
    rows, wrote, problems = [], 0, 0
    try:
        for probe, source in PROBES:
            ident, key = identity_here(probe, scratch, ns.wait)
            row = {"start_state": probe, "key_here": key, "body_sha1_here": ident["body_sha1"],
                   "measured_record": source, "record_found_here": os.path.isfile(os.path.join(records,
                                                                                               key + ".json"))}
            rows.append(row)
            if ns.probe or row["record_found_here"]:
                continue
            with open(os.path.join(records, source + ".json"), encoding="utf-8") as f:
                doc = json.load(f)
            same = {k: v for k, v in doc["ident"].items() if k != "body_sha1"}
            here = {k: v for k, v in json.loads(json.dumps(ident)).items() if k != "body_sha1"}
            if same != here:
                row["problem"] = ("this machine's robot differs from the measured one in more than the body "
                                  "hash: " + ", ".join(sorted(k for k in here if here[k] != same.get(k))))
                problems += 1
                continue
            doc["ident"], doc["key"] = json.loads(json.dumps(ident)), key
            with open(os.path.join(records, key + ".json"), "w", encoding="utf-8") as f:
                json.dump(doc, f)
            row["wrote"] = key + ".json"
            wrote += 1
    finally:
        shutil.rmtree(scratch, ignore_errors=True)   # only the folder this run made: what else is in --work stays
        try:
            os.rmdir(ns.work)   # --work itself goes only when nothing else is in it
        except OSError:
            pass
    print(json.dumps({"rows": rows, "written": wrote, "problems": problems}, indent=2))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
