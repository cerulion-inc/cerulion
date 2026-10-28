# replay_probe fixtures

rclpy nodes for the exact-replay tests (the rclcpp ones are the ament_cmake package `../replay_probe_cpp`: `talker_cpp`, `mixer_cpp` (node `mixer`; `MIXER_WALLCLOCK=1`, `PROBE_MT=1`, `PROBE_CB_SLEEP_US`) and `listener_cpp`, built by colcon inside `rmw_replay_rclcpp_mixer_test`) (`tests/rmw_replay_*_test.rs`). Each
node logs every callback, in order, to `<PROBE_OUT>/<node>.jsonl`; the tests
compare those logs between a recorded run and its replays. None of them is
built or run by a plain `cargo test`: the replay tests are `#[ignore]` and need
ROS 2 sourced with `librmw_cerulion.so` staged (see the test files).

| file | node(s) | role |
|---|---|---|
| `clockpub.py` | `clockpub` | publishes `/clock` with deterministic values (`k` x 5 ms of ROS time), starting once `CLOCK_SUBS` subscribers match, stopping after `TICKS` ticks; writes `<PROBE_OUT>/clockpub.done` |
| `talker.py` | `talker_a` | a 10 Hz ROS-time timer publishes `a:N` on `/a` |
| `mixer.py` | `mixer` | subscribes `/a`; a 7 Hz ROS-time timer publishes `sha256(state)` on `/out`, where `state` folds every input and every fire's ROS time in callback order. `MIXER_WALLCLOCK=1` also folds `time.time_ns()` (the wall-clock control); `--mt` spins a `MultiThreadedExecutor` |
| `two_exec.py` | `left`, `right` | two single-threaded executors on two Python threads; `left` folds `/a` and publishes `/left`, `right` folds `/left` and publishes `/right`. `--block` makes `right`'s callback wait for `left`'s next callback (the watchdog case) |
