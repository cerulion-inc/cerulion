# Exact replay of a ROS 2 process

`rmw_cerulion` can record what one ROS 2 process consumed, in the order it consumed it, and later replay that process alone against the recording. The replayed process runs its own unmodified code. It gets every message, service request and response and `/clock` tick it got live, in the same order, and nothing else. The rmw checks each callback's outputs against the recording and writes a verdict: `identical`, or the first node, topic and window whose outputs differ although its inputs matched.

This is the debugging loop it is for: a node misbehaved once in a run you cannot reproduce. Record the run, then replay the node by itself, as often as you like, under a debugger or with extra logging. The replay never waits for wall time: on the test probe, 20 s of ROS time replays in about a second, Python start-up included.

## Recording

Record every ROS 2 process a verb starts:

```bash
cerulion ros2 run --record-deliveries ./logs my_pkg my_node --ros-args -p use_sim_time:=true
cerulion ros2 launch --record-deliveries ./logs my_robot bringup.launch.py
```

Each process writes `./logs/<exe>-<pid>.rdl.mcap` (for a Python node, `<exe>` is the script's name). An existing file is never overwritten: process ids repeat across containers, so a second process with the same name and pid writes `<exe>-<pid>.1.rdl.mcap`, and so on. The flag must come right after `run` or `launch`; everything after it goes to `ros2` verbatim. The same recording is on for any process started with the variable set, whatever launched it:

```bash
CERULION_RMW_DELIVERY_LOG=./logs RMW_IMPLEMENTATION=rmw_cerulion ros2 run my_pkg my_node
```

Topic recording (`cerulion bag record`) can run at the same time. The two record different things: a bag records the frames on topics, a delivery log records what one process took from them, in its order.

## Replaying

Replay one recorded process, alone, with the same command it was recorded with:

```bash
cerulion ros2 run --replay-deliveries ./logs/my_node-4242.rdl.mcap my_pkg my_node --ros-args -p use_sim_time:=true
```

or `CERULION_RMW_REPLAY=<log>` on any launch path. The verdict is written to `<log>.verdict.json` once the log is used up; stop the process after that (Ctrl-C). Nothing else needs to run: no talker, no clock, no robot. `ros2 launch` has no replay flag, because a replay is one process.

## The verdict

| `verdict` | meaning |
|---|---|
| `identical` | every compared output equals the recording and every recorded window was served |
| `diverged` | the `culprit` names the first segment whose inputs matched and whose outputs differ: `node`, `topic`, `window` (the recorded `rmw_wait` it follows), `delivery` (deliveries the thread had taken by then) and `output_index` |
| `input_mismatch` | the replayed process asked for an input in an order the recording does not have (`input_mismatch.what` says which); later differences are inherited, never named as the culprit |
| `log_exhausted_early` | the replay used up a log that did not close cleanly (the recorded process was killed, or the recorder hit a write error) and matched up to there |
| `replay_stopped_early` | the process exited before it was served every recorded window |
| `watchdog` | a callback held the replay for 5 s without progress, waiting on another thread the replay cannot run (see the conditions below); `watchdog.entity` and `node` name it |
| `refused` | the replay did not start; `refused` says why |

A divergence is also printed on stderr when it is found, for example:

```
rmw_cerulion replay DIVERGED: process `python3 mixer.py --ros-args -p use_sim_time:=true`, node 'mixer', topic /out: window 79 (after delivery index 30) inputs matched, outputs differ (output 0 of 1 recorded, 1 replayed). Verdict: ./logs/mixer-4242.rdl.mcap.verdict.json
```

Outputs are compared as a 64-bit FNV-1a hash of each payload, one segment at a time: the outputs a thread makes between two rmw events (a wait return or a take). Timer callbacks are invisible to the rmw, so two timers that fire back to back share one segment and the culprit is named down to the segment, not the timer. `/rosout` is recorded but never compared: rcl stamps every log line with the wall clock. Nor are the outputs a thread makes before its first rmw event, while the node is being constructed: they cannot depend on a delivery, and rclpy's construction-time `/parameter_events` message carries a wall-clock stamp because `use_sim_time` takes effect only after it. The verdict counts both (`outputs_excluded`, `outputs_before_first_event`). The message header's wall-clock timestamp is never part of the comparison either.

## When a replay is exact

Replay reproduces the recorded run when all of these hold for the process:

1. **C1** Every thread that runs callbacks is a single-threaded executor thread that calls `rmw_wait`. Several single-threaded executors on several threads are fine: the replay serves their windows in the recorded order and lets one thread run at a time.
2. **C2** No node logic reads the wall or steady clock. Use ROS time (`use_sim_time`, with `/clock` recorded); a wall timer (`create_wall_timer`) or a `time.time_ns()` read diverges, and the verdict names it.
3. **C3** Random numbers are seeded or absent.
4. **C4** No callback blocks on work another thread owns. If one does, the watchdog names it.
5. **C5** No input reaches the node outside the rmw: no sockets, devices, files it reads during the run, or shared memory of its own.
6. **C6** The same binaries, parameters and distro are used.

The rmw checks what it can and refuses the rest by name, at `rmw_init`, before the node runs:

- a log recorded by a different process: another command line, another `ROS_DISTRO`, or another build of `librmw_cerulion.so` (its file digest is in the log). The refusal names the field.
- a multi-threaded executor. An rclcpp `MultiThreadedExecutor` (two threads waiting on one wait set) and an rclpy `MultiThreadedExecutor` (takes from a pool thread that never waits) are detected while recording, the log is marked not replayable, and its replay fails with `ros2 process <command> (nodes <names>): MultiThreadedExecutor, not replayable`.
- a node the recording did not create at that point.
- `CERULION_RMW_DELIVERY_LOG` and `CERULION_RMW_REPLAY` both set.
- a serialized take (`rmw_take_serialized_message`, used by rosbag2's recorder): the log records decoded takes only, so it is marked not replayable.

Graph queries are answered from the recording too: `count_publishers`, `count_subscribers`, `service_server_is_available` and the two matched-endpoint counts. Service servers and clients are replayed like subscriptions: requests and responses are served from the log, and every response or request the process sends is compared.

## What is never claimed

- That a node is deterministic. A verdict of `identical` proves that this recording replays; a race that did not fire while recording stays hidden.
- That any ROS 2 process replays. Only processes that meet C1 to C6 do; the others are named where the rmw can see them, and otherwise diverge.
- That a launch replays as a whole. Each process replays alone, against its own log; processes that feed each other are not replayed together.
- That replay covers `EventsExecutor`, rclcpp intra-process communication or loaned takes. A replay turns loans off (rclcpp then takes by copy, the same message).
- That every distro is runtime-tested. The tests run on ROS 2 Humble. Foxy, Jazzy and Lyrical builds compile the same code.

## Files

The log is an MCAP file written by Cerulion's bag writer, with four channels: `rdl/process` (the process header first, then node and entity declarations, a `not_replayable` mark, the trailer), `rdl/waits`, `rdl/deliveries` (takes with their payload bodies, graph answers) and `rdl/outputs` (output hashes). A cleanly closed log also carries an `rdl/process.json` attachment. Records reach the file a 1 MiB chunk at a time (and at every `rmw_shutdown`), so a process killed with SIGKILL loses at most its last unflushed chunk; a file torn inside a chunk is read up to its last whole record. The replay of such a log matches up to there and says `log_exhausted_early`. The internals (serving units, the token, the oracle) are in `docs/internals/rmw.md`, section "Delivery log and replay gate".
