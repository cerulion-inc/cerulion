# aspire_arm

A simulated arm that a coding agent drives through the Robot Context Protocol,
and a recording of what the agent did that replays byte for byte with no agent
and no model.

```
ASPIRE skill program
   │ EnvironmentAdapter "cerulion"
   ▼
cerulion-rcpd  (MCP over stdio; tools from rcp.yaml; receipt per call)
   │ /rcp/calls   (std_msgs/String JSON, one frame per tool call)
   ▼
arm_sim node ──► /rcp/results  (the answer to each call)
             └─► /arm/state    (the plant state after each call)
```

- **`arm_sim`** (`nodes/arm_sim/`) is a six-joint arm with a gripper and one
  cube on a table. It fires once per frame on `/rcp/calls`; `arm.step` advances
  the plant exactly one 50 ms control period. The plant is a pure function of
  the calls it has received (`nodes/arm_sim/src/plant.rs`), which is what lets
  a replay recompute it.
- **`rcp.yaml`** is the agent's whole view of the robot. The tools are
  `arm.reset` and `arm.step`, plus the built-in `estop`; anything else does not
  exist. Arguments are checked against each tool's schema before anything is
  published, and each call writes a hash-chained receipt to disk before it
  reaches the graph.
- **[ASPIRE](https://github.com/NVlabs/ASPIRE)**'s `cerulion` embodiment is the
  agent side. Its `EnvironmentAdapter` spawns `cerulion-rcpd --stdio` and turns
  each control period into one `arm.step` call.

## Run it

You need `cerulion` and `cerulion-rcpd` on PATH, and an ASPIRE checkout with the
`cerulion` embodiment installed in a Python 3.12 environment:

```bash
ASPIRE_DIR=~/src/ASPIRE ASPIRE_PYTHON=~/src/ASPIRE/.venv/bin/python ./run_demo.sh
```

`run_demo.sh` builds `arm_sim`, starts `cerulion graph run aspire_arm --record`,
runs ASPIRE's `examples/cerulion/pick_cube.py`, stops the graph, checks the
receipt chain with `cerulion-rcpd --verify-receipts`, and replays the bag:

```bash
cerulion bag play runs/<ts>/recordings/aspire_arm_<ts>.mcap --resim all --verify
```

### By hand

```bash
cerulion node build arm_sim --release
cerulion graph validate aspire_arm
cerulion graph run aspire_arm --release --record      # terminal 1; Ctrl+C stops it
# terminal 2, from the ASPIRE checkout:
python -m cap_harness.cli run --benchmark cerulion --suite cerulion --task-id 0 \
  --program examples/cerulion/pick_cube.py \
  --rcpd-manifest <this directory>/rcp.yaml --rcpd-receipts /tmp/rcp/receipts.jsonl \
  --output-root /tmp/aspire-out
cerulion-rcpd --verify-receipts /tmp/rcp/receipts.jsonl
# after Ctrl+C in terminal 1:
cerulion bag play recordings/aspire_arm_<ts>.mcap --resim all --verify
```

Start the ASPIRE program only after the recording has opened (the run prints
the bag path); a call published before that is not in the bag.

The replay re-injects the recorded `/rcp/calls` frames into a fresh `arm_sim`
and compares every `/rcp/results` and `/arm/state` frame it produces with the
recorded one, byte for byte. Exit code 0 and `replay PASS` mean the episode
was reproduced exactly. No agent, MCP server or model runs during the replay.

Change a constant the episode exercises, such as `MAX_JOINT_STEP_RAD` in
`nodes/arm_sim/src/plant.rs`, rebuild with
`cerulion node build arm_sim --release`, and run the same `bag play` command:
it exits 1 and names the first frame that differs. (A constant the episode
never reaches, such as the joint limit, changes nothing the bag can show.)

## Scope

- The plant is kinematic: joints move toward their target at a capped rate,
  and the cube attaches to the tool tip when the closed gripper is within 3 cm
  of it. There is no contact dynamics.
- There is no perception. The program reads the cube position from the task
  metadata the adapter takes from `arm.reset`'s answer.
- The replay check is for the machine and build that recorded the bag. The
  plant uses `f64` trigonometry from the platform's math library, so another
  platform can differ in the last bit.
- `estop` closes a latch in `cerulion-rcpd` and writes it to
  `<receipts>.estop`; every later motion call is refused until an operator
  deletes that file and restarts the daemon. `arm_sim` latches too, until its
  next `arm.reset`.
- The receipt log records refused calls as well as published ones; the
  recording holds only the calls that reached the graph.
