# MoveIt Hero Demo: unmodified MoveIt 2 on Cerulion

Runs the **stock ROS 2 Jazzy + MoveIt 2 Panda demo, with zero MoveIt source
edits**, on `rmw_cerulion` over iceoryx2 shared memory by setting one env var:
`RMW_IMPLEMENTATION=rmw_cerulion`. `move_group` comes up, plans a motion via its
MoveGroup action (OMPL), and the pilz industrial planner produces
**bit-identical** plans across repeated runs. MoveIt's variable-length messages
still take the rmw flatten and copy path; see the scope section below.

Everything needed to reproduce it lives in this directory, so the result is
gated in CI rather than depending on a hand-configured external container.

**What this directory is, and what it needs.** It is a Docker-driven gate for
`rmw_cerulion`, not a Cerulion workspace: there are no `nodes/`, no `graphs/`
and nothing here runs through `cerulion graph run`. The demo needs **Docker**
and builds a multi-gigabyte image (ROS 2 Jazzy, MoveIt 2, a Rust toolchain);
without Docker, `run_demo.sh` stops at its first `docker` call. The only check
that runs without Docker is the launcher self-test,
`python3 examples/moveit_hero/test_run_demo.py -v` (see [CI](#ci)).

## What the example runs

| Claim | Backed by |
|---|---|
| Unmodified MoveIt 2 runs on Cerulion's zero-copy transport | `move_group` launched with `RMW_IMPLEMENTATION=rmw_cerulion`, zero source edits |
| A motion plan succeeds | OMPL plan via the MoveGroup action returns `SUCCESS` + a non-empty trajectory |
| The middleware really is rmw_cerulion | `scripted_plan.py` reports the identifier rclpy actually bound, not the env var it was asked for |
| Deterministic planning | 3 identical pilz PTP requests: SHA-256 over every trajectory float is identical across all 3 |

## Scope of the claims

OMPL produces a plan, and repeated pilz requests produce matching trajectories
for a fixed scene. That checks planning on this transport, and nothing beyond
it. In particular:

- **It is not replay-grade host determinism.** The pilz bit-identity is
  plan-**output** determinism for a deterministic planner over a
  static-after-init scene, not whole-process replay.
- **It is not a zero-copy sensor path.** MoveIt's variable-length messages go
  through the rmw flatten and copy path here, not a zero-copy loan. A C++
  zero-copy consumer bridge would be needed for that, and there is none.
- **It does not execute a trajectory.** See the execution note at the end.

OMPL is a randomized planner and is **not** claimed deterministic; pilz is the
determinism vehicle, by design. Pilz PTP is analytic: for a fixed request and a
fixed scene it computes the same trajectory every time, so a digest mismatch
means something underneath changed. OMPL's RRTConnect samples randomly, so
repeated runs legitimately differ and prove nothing about the transport.

## Reproduce

Requires Docker. From the repo root:

```bash
bash examples/moveit_hero/run_demo.sh
```

That builds the image (ROS 2 Jazzy + MoveIt 2 + Panda config + Rust), then runs
the container with the flags iceoryx2 + move_group need
(`--shm-size=1g --ulimit nofile=524288`), building `rmw_cerulion` from the
mounted source and launching the headless Panda stack.

Expected tail of the output (point counts and timings vary by planner, seed
and machine, so treat the numbers as shape, not as a target). The
`OMPL_PLAN_OK` / `PILZ_DETERMINISM_PASS`
sentinels are emitted via `print(..., flush=True)` on guaranteed stdout, NOT the
ROS 2 logger, so the CI gate never depends on logger level or routing:

```
OMPL_PLAN_OK RESULT ompl points=<N> wall_ms=<T>
pilz run 0: points=<M> sha256=<hash>...
pilz run 1: points=<M> sha256=<hash>...
pilz run 2: points=<M> sha256=<hash>...
PILZ_DETERMINISM_PASS RESULT pilz runs=3 points=<M> identical=1 sha256=<full-hash>
=== DEMO PASS (log: /work/examples/moveit_hero/out/run.log) ===
```

The full run log lands in `examples/moveit_hero/out/run.log` (git-ignored).
Alongside it, `scripted_plan.py` writes machine-readable `ompl.json` / `pilz.json`
(e.g. `{"status":"pass","rmw":"rmw_cerulion","points":29,"wall_ms":11.5,...}`).
These are the primary CI gate; the stdout sentinels are the secondary check.

The `rmw` field is the identifier **rclpy actually loaded**, not the
`RMW_IMPLEMENTATION` the entrypoint exported. Asserting on the env var would be
circular (this demo is what sets it), so the gate reads what the middleware
really bound, and fails closed if that cannot be determined.

## Save diagnostic logs

```bash
bash examples/moveit_hero/run_demo.sh --capture
```

`--capture` keeps `move_group`'s own log alongside the run log, so a failed or
slow run can be read afterwards. The demo is headless: its output is the
sentinel lines, the timings and the JSON results. `entrypoint.sh` (`cleanup()`)
marks the point at which the plan run finishes, which is where an external
screen recorder would wrap it.

## CI

`.github/workflows/moveit-hero.yml` runs this nightly + on manual dispatch (not
per-PR: it is heavy). It builds the image, runs the demo headless, and gates on
the machine-readable `ompl.json` / `pilz.json` result files (`jq` asserts
`status=="pass"`, `rmw=="rmw_cerulion"`, non-empty plan, and pilz `identical==true`)
as the primary signal, with the stdout `OMPL_PLAN_OK` / `PILZ_DETERMINISM_PASS`
sentinels as a secondary grep. It then uploads the run log + result JSON as an
artifact.

The host launcher can also be checked without Docker: build context and bind-mount
paths, capture mode, prebuilt images, and failed build/run exit codes. Run:

```bash
python3 examples/moveit_hero/test_run_demo.py -v
```

This checks launcher wiring; the container integration run checks actual planning.

## Files

| File | Role |
|---|---|
| `Dockerfile` | ROS 2 Jazzy + MoveIt 2 + Panda config + Rust toolchain |
| `launch_move_group.py` | headless Panda `move_group` + control stack (no RViz) |
| `scripted_plan.py` | OMPL plan + pilz determinism check via the MoveGroup action |
| `entrypoint.sh` | in-container: build+register rmw_cerulion, launch, plan, assert |
| `run_demo.sh` | host one-command driver (build image + run with the right flags) |
| `test_run_demo.py` | launcher file mappings and failure propagation, without Docker |
| `iceoryx2.toml` | raises the per-service publisher/subscriber cap so a multi-node ROS graph fits (see below) |

### Why the iceoryx2 config

A full MoveIt graph runs several ROS nodes and every node opens a publisher on
the shared `/rosout` and `/parameter_events` topics. iceoryx2's built-in default
caps publishers per service at **2**, so the third node dies at startup. The
demo image bakes `iceoryx2.toml` into the user config path
(`$HOME/.config/iceoryx2/iceoryx2.toml`), which `Config::global_config()`
auto-loads, raising the cap to 32. Baking that configuration into the image
is what makes the demo reproducible from a clean clone: the image installs the
configuration automatically. It lives in the demo image only,
never in `cerulion_core`, whose own defaults are unchanged by it.

## Known limits

- `move_group` can `SIGSEGV` in `rclcpp::Executor::~Executor()` during
  post-SIGINT teardown, strictly after all work has completed. This is a known
  shutdown fault, still under investigation, and it is not attributed to a
  specific upstream issue. Planning results are written before shutdown, so the
  crash does not affect them, and it validates neither trajectory execution nor
  process lifecycle.
- TRANSIENT_LOCAL history replay is at-least-once on late-join (idempotent for
  ROS latched topics like `robot_description` / `/tf_static`).
- This demo is **plan-only**: it asserts that a plan succeeds and does not
  execute one. The trajectory-execution action
  (`.../follow_joint_trajectory/_action/status`) hits the same create-order race
  as the planning action and is not exercised here. Executing a trajectory
  would need the readiness handling extended to that action.
