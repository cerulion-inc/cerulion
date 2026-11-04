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

## What recording costs

With neither variable set, each rmw call pays one check. While recording, each take copies its payload into the log and each output is hashed byte by byte (64-bit FNV-1a) on the calling thread, so the cost grows with message size. Round trip on one Linux host (ROS 2 Humble, rclcpp ping-pong over `rmw_cerulion` shared memory, 100 Hz, 2,000 samples per size). The not-recording column is the build before recording support, the median of 4 to 7 runs; with recording support built in and off, p10 matches it within 0.4 µs at every size. The recording columns are the median of 2 runs:

| payload | p50 not recording | p50 recording | p99 recording |
|---|---|---|---|
| 64 B | 14 µs | 16 µs | 57 µs |
| 16 KiB | 16 µs | 48 µs | 0.55 ms |
| 64 KiB | 21 µs | 148 µs | 1.2 ms |
| 1 MiB | 166 µs | 3.0 ms | 4.4 ms |

The log also grows by every payload taken. Record a process whose large topics run at a few Hz; a 1 MiB topic at camera rate spends most of a 30 Hz frame on recording. Hashing a sample of each large payload, or a faster hash, and moving the body copy off the calling thread are the planned ways to cut this.

Under `use_sim_time`, rclcpp takes `/clock` on its own thread. While recording, a `/clock` update and another thread's timer check are kept apart, each waiting at most 5 ms for the other (see C1). On the rclcpp test probe, with `/clock` at 2 kHz for 5 s, the recorder waited 2 to 10 times per process, 34 to 394 µs in all. The count and the total are logged when the log closes. Without this wait, 1 of 10 replays of that probe diverged.

## Replaying

Replay one recorded process, alone, with the same command it was recorded with:

```bash
cerulion ros2 run --replay-deliveries ./logs/my_node-4242.rdl.mcap my_pkg my_node --ros-args -p use_sim_time:=true
```

or `CERULION_RMW_REPLAY=<log>` on any launch path. When the replay starts, `<log>.verdict.json` says `running`; the real verdict replaces it once the log is used up, so stop the process after the file no longer says `running` (Ctrl-C). A replay that crashes or is killed leaves `running`, never an earlier replay's verdict; a replay that cannot write the file (a read-only directory, a full disk) is refused at `rmw_init`. Nothing else needs to run: no talker, no clock, no robot.

A replay really publishes. The replayed process sends its outputs on the transport as it did live, and through `cerulion-netd` they can reach other hosts. Do not replay a controller or a driver where a live robot, or anything that acts on its topics, can receive them: replay on a machine of its own, or on one where no live node runs and `cerulion-netd` is stopped or started with `CERULION_NETD_NETWORK=off`. `ros2 launch` has no replay flag, because a replay is one process.

## The verdict

| `verdict` | meaning |
|---|---|
| `identical` | every compared output equals the recording, every recorded window was served and every recorded delivery was taken |
| `diverged` | the `culprit` names the first segment whose inputs matched and whose behaviour differs: `node`, `topic`, `window` (the recorded `rmw_wait` it follows), `delivery` (deliveries the thread had taken by then), `what` (`outputs differ`, or a recorded delivery the replay never took) and `output_index` |
| `input_mismatch` | the replayed process asked for an input in an order the recording does not have (`input_mismatch.what` says which); later differences are inherited, never named as the culprit |
| `log_exhausted_early` | the replay used up a log that did not close cleanly (the recorded process was killed, or the recorder hit a write error) and matched up to there (see "Files") |
| `replay_stopped_early` | the process exited before it was served every recorded window; `takes_not_replayed` counts the recorded deliveries it never took |
| `watchdog` | a callback held the replay for 5 s without progress, waiting on another thread the replay cannot run (see the conditions below), or the last callback never returned to `rmw_wait`; `watchdog.entity` and `node` name it. Another thread of the process that waits in `rmw_wait` reports it, so a process with one executor thread whose callback never returns leaves `running` |
| `refused` | the process is not replayable; `refused` says why and `refused_at` says when: `start` (at `rmw_init` or node creation, before the node ran), `run` (the replay stopped serving when it saw the cause) or `end` (the process ran to the end, and some of its outputs could not be compared) |
| `running` | the replay has started and has not written its verdict yet; a replay that crashed or was killed leaves this |

A divergence is also printed on stderr when the verdict is written, for example:

```
rmw_cerulion replay DIVERGED: process `python3 mixer.py --ros-args -p use_sim_time:=true`, node 'mixer', topic /out: window 79 (after delivery index 30) inputs matched, outputs differ (output 0 of 1 recorded, 1 replayed). Verdict: ./logs/mixer-4242.rdl.mcap.verdict.json
```

Outputs are compared as a 64-bit FNV-1a hash of each payload, one segment at a time: the outputs a thread makes between two rmw events (a wait return or a take). Timer callbacks are invisible to the rmw, so two timers that fire back to back share one segment and the culprit is named down to the segment, not the timer. `/rosout` is recorded but never compared: rcl stamps every log line with the wall clock. Nor are the outputs a thread makes before its first rmw event, while the node is being constructed: they cannot depend on a delivery, and rclpy's construction-time `/parameter_events` message carries a wall-clock stamp because `use_sim_time` takes effect only after it. The verdict counts both (`outputs_excluded`, `outputs_before_first_event`). The message header's wall-clock timestamp is never part of the comparison either.

## When a replay is exact

Replay reproduces the recorded run when all of these hold for the process:

1. **C1** Every thread that runs callbacks is a single-threaded executor thread that calls `rmw_wait`. Several single-threaded executors on several threads are fine: the replay serves their windows and takes in the recorded order and lets one thread run at a time. That includes rclcpp's clock thread, which takes `/clock` on its own executor when `use_sim_time` is set: while recording, a `/clock` update and another thread's timer check are kept apart (each waits at most 5 ms for the other), so the recorded order is one the replay can serve.
2. **C2** No node logic reads the wall or steady clock. Use ROS time (`use_sim_time`, with `/clock` recorded); a wall timer (`create_wall_timer`) or a `time.time_ns()` read can diverge, and when it changes an output the verdict names it; a wall-clock read that changes no output in the recorded span replays as `identical`.
3. **C3** Random numbers are seeded or absent.
4. **C4** No callback blocks on work another thread owns. If one does, the watchdog names it. The watchdog also fires, after 5 s, when the thread of the last recorded callback leaves its executor without calling `rmw_wait` again (for example after `spin_until_future_complete`) and the process stays up while another executor thread still waits; that verdict is `watchdog` although the outputs matched. A callback that blocks forever in a process with one executor thread is not named: the verdict stays `running`.
5. **C5** No input reaches the node outside the rmw: no sockets, devices, files it reads during the run, or shared memory of its own.
6. **C6** The same binaries, parameters and distro are used.

The rmw checks what it can and refuses the rest by name. Each refusal reads `ros2 process <command> (nodes <names>): <cause>, not replayable (<detail>)` where a cause applies. At `rmw_init`, before the node runs, a replay refuses:

- a log recorded by a different process: another command line, another `ROS_DISTRO`, or another build of `librmw_cerulion.so` (its file digest is in the log). The refusal names the field.
- a log marked not replayable while it was recorded, under the cause the recording found (the two below).
- `CERULION_RMW_DELIVERY_LOG` and `CERULION_RMW_REPLAY` both set.

While recording, the rmw marks the log not replayable on:

- a multi-threaded executor (cause `MultiThreadedExecutor`). An rclcpp `MultiThreadedExecutor` shows as two threads waiting on one wait set, an rclpy one as takes from a pool thread that never waits. The rclcpp mark needs a second executor thread to reach `rmw_wait`: with callbacks that finish in microseconds, the thread that just ran one takes the executor's wait mutex again first, the second thread never waits, and the recording carries no mark (that run executed as a single thread). Such a log replays.
- a serialized take (cause `serialized take`; `rmw_take_serialized_message`, used by rosbag2's recorder and rclpy's `raw=True` subscriptions): the log records decoded takes only.

During the replay, the rmw refuses:

- a multi-threaded executor the recording did not show: `rmw_wait` fails with the `MultiThreadedExecutor` refusal.
- a node the recording did not create at that point (at `rmw_create_node`).
- a thread outside every executor that publishes (cause `publishing thread outside every executor`): a plain thread that never calls `rmw_wait` is not ordered by the replay, so its outputs cannot be compared. Outputs it makes before any thread waits are construction time; after that, the verdict is `refused`, naming the topic of its first output, and counts them (`outputs_unordered`), including outputs made after the log is used up (the verdict file is rewritten when the process exits). A thread that publishes while constructing a node, after another thread already waits, and never waits itself (a node constructed on the main thread and spun on another thread, under `use_sim_time`) is refused the same way, although its outputs were construction time.

Five graph queries are answered from the recording: `count_publishers`, `count_subscribers`, `service_server_is_available` and the two matched-endpoint counts. Every other graph query (`rmw_get_node_names`, `rmw_get_topic_names_and_types`, the endpoint info queries and the rest) is answered live during a replay, from a graph in which only the replayed process runs. A node whose behaviour depends on those answers does not meet C5. The recorded answers to one query on one name are served in the order the process asked them, across all its threads; the executor threads ask in the recorded order, because the replay runs one callback at a time, but a thread outside every executor that asks while they run can receive an answer another thread was recorded getting.

Service servers and clients are replayed like subscriptions: requests and responses are served from the log, and every response or request the process sends is compared, except one sent before the process's first rmw event, which is construction time like any other output (the stock `add_two_ints_client` sends its one request that way). A client matches a response to its request by the sequence number `rmw_cerulion` mints, per client, counting from 1 at the client's creation, so a replay that sends the recorded requests gets the recorded numbers. Tested on Humble with an rclpy client and server and with the stock `demo_nodes_cpp` `add_two_ints_server` and `add_two_ints_client`, each replayed alone.

## What is never claimed

- That a node is deterministic. A verdict of `identical` proves that this recording replays; a race that did not fire while recording stays hidden.
- That any ROS 2 process replays. Only processes that meet C1 to C6 do; the others are named where the rmw can see them, and otherwise diverge.
- That a process with more than one ROS context replays. The verdict is written at the first `rmw_shutdown`, so a context shut down while another keeps running ends the comparison there.
- That a launch replays as a whole. Each process replays alone, against its own log; processes that feed each other are not replayed together.
- That replay covers `EventsExecutor`, rclcpp intra-process communication or loaned takes. A replay turns loans off (rclcpp then takes by copy, the same message).
- That every distro is runtime-tested. The tests run on ROS 2 Humble, with rclpy nodes, the stock `demo_nodes_cpp` executables and rclcpp nodes on ROS time. Foxy, Jazzy and Lyrical builds compile the same code.
- That two replays write byte-identical verdict files in every field. `guards_not_triggered_live` counts recorded-ready guard conditions with no live trigger when served; the last windows carry the shutdown guards the recording's Ctrl-C triggered, so the count can differ by one between replays of an rclcpp log.

## Files

The log is an MCAP file written by Cerulion's bag writer, with four channels: `rdl/process` (the process header first, then node and entity declarations, a `not_replayable` mark, the trailer), `rdl/waits`, `rdl/deliveries` (takes with their payload bodies, graph answers) and `rdl/outputs` (output hashes). A cleanly closed log also carries an `rdl/process.json` attachment. Records reach the file a 1 MiB chunk at a time (and at every `rmw_shutdown`), so a process killed with SIGKILL loses at most its last unflushed chunk; a file torn inside a chunk is read up to its last whole record. The cut can fall between any two records, for example between a take and the outputs of its callback. So in the replay of such a log, each thread's last surviving segment is compared only as far as the log goes (its recorded outputs must be the first ones replayed, and the rest are counted as `outputs_past_log_end`), a take past a thread's last recorded take is served as not taken and a graph query past the recorded answers is answered live (both counted as `inputs_past_log_end`). If everything before the cut matched, the verdict is `log_exhausted_early`. The internals (serving units, the token, the oracle) are in `docs/internals/rmw.md`, section "Delivery log and replay gate".
