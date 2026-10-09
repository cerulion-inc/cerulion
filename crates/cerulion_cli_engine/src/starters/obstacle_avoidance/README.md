# Obstacle avoidance starter

These complete sources ship with the installed Cerulion CLI. `starter.toml`
records the CLI version and compiler fingerprint. The workspace manifest uses
exact release dependency versions, or the matching checkout for a source-built
CLI. No branch or latest-release download is used.

The scanner generates a synthetic 180-beam scan. The controller publishes
`x = 0.0` for a close obstacle and `x = 0.3` for a clear path. This is a
teaching example; a physical robot also needs sensor validity checks,
stale-input handling and an actuator shutdown path.

## Build and run

```bash
cerulion node build laser_scanner --release
cerulion node build safety_controller --release
cerulion graph validate obstacle_avoidance
cerulion graph run obstacle_avoidance --release --network off
```

Press Enter if offered a partition save prompt. In another terminal in this
workspace, observe the velocity:

```bash
CERULION_NETWORK=off cerulion topic echo /obstacle_avoidance/safety_controller/linear_velocity
```

Observe both velocity values, then stop each command with Ctrl+C.

## Connect the nodes yourself

Read each node's `src/lib.rs` and `src/tests.rs`. Each crate contains one node
type; all wiring belongs in graph YAML. To create your own graph explicitly:

```bash
cerulion graph create lesson -n lesson
cerulion node stage laser_scanner --graph lesson
cerulion node stage safety_controller --graph lesson -I scan laser_scanner/scan
cerulion graph validate lesson
cerulion graph run lesson --release --network off
```

To write both implementations from scratch, create an empty workspace with
`cerulion workspace create my_robot` and follow the getting-started guide.

## Source regression

```bash
cargo test -p laser_scanner -p safety_controller --lib -- --test-threads=1
```
