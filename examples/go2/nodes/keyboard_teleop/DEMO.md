<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Keyboard teleop

The `keyboard_teleop` node reads the terminal (WASD) and publishes
`geometry_msgs/Twist` on `/go2/cmd_vel/keyboard`. It runs in two shipped
shapes: on the companion computer in `graphs/teleop.yaml` (over an `ssh -t`
session, beside the gamepad and the mux), or on your desk in
`graphs/teleop_desk.yaml`, imported by the companion's
`graphs/teleop_remote.yaml` through an `ingress:` block and arbitrated
against the gamepad there. The node logic is verified without a terminal
(`tests/keyboard_e2e_test.rs`); this is the demo wiring.

## Controls (as shipped)

| Key(s) | Effect |
|---|---|
| `w` / `s` | forward / back (`linear.x`, ±0.6 m/s at full scale) |
| `a` / `d` | strafe left / right (`linear.y`, ±0.4 m/s; left = +y) |
| `q` / `e` | yaw left / right (`angular.z`, ±1.0 rad/s; left = +z) |
| `space` / `Esc` | immediate zero |
| `1`..`5` | speed-scale presets (0.2 .. 1.0 of max) |
| `Ctrl-C` | quit, see below (raw mode disables ISIG, so the node itself translates it) |

The bindings are WASD-style, inspired by, but deliberately not identical to,
ROS's `teleop_twist_keyboard` (whose actual layout is `i`/`j`/`k`/`l`/…).

Latched velocity: a direction key persists and republishes at 10 Hz. **Any
500 ms without a keypress auto-zeroes** (runaway guard). A minimal one-line
status (`vx/vy/vyaw`, moving/idle) prints to stderr.

**How Ctrl-C actually works:** in raw mode the terminal driver never turns
Ctrl-C into SIGINT (ISIG is off), so the node handles the Ctrl-C *keypress*
itself: it publishes one zero Twist and restores the terminal, then, one
pump cycle later, so the zero publish gets processed first, raises SIGINT
so `cerulion graph run`'s existing Ctrl-C shutdown path runs as if the
terminal had delivered the signal. The zero publish on Ctrl-C is
**best-effort** (the signal still races the graph's shutdown), and it does
not have to arrive: once the keyboard goes silent for more than 750 ms the
mux drops its arbitration slot, and with both sources stale the mux commands
sustained zero velocity. That is what the software guarantees, a command to
stop, not the stop itself: keep the hardware stop within reach, because a
wedged process or a fault below the driver can leave the robot moving.

**Terminal restore is three-layered** (a wedged terminal is the classic
teleop failure): the node's `shutdown()` restores on clean graph shutdown,
the pump's RAII guard restores when the helper closure drops (and on Ctrl-C),
and a process-wide panic hook restores before any panic output. All three
are idempotent.

## The shipped graphs

Two graphs wire this node, both under `graphs/`:

- `teleop.yaml` runs it on the companion computer beside the gamepad, the
  mux and the sport driver. Drive it from an `ssh -t` session: raw mode
  needs a TTY.
- `teleop_desk.yaml` runs it on your desk, publishing the canonical absolute
  `/go2/cmd_vel/keyboard` (a `topic:` override), and `teleop_remote.yaml`
  on the companion imports that name through an `ingress:` block and
  arbitrates it against the gamepad. The two headers say how the machines
  find each other: on one LAN nothing is needed; off it, point the desk's
  `cerulion-netd` at the companion with `CERULION_NETD_CONNECT`.

The desk graph carries no `network:` block on purpose: with none it runs
PERMISSIVE, its egress registers with the desk's `cerulion-netd`, which
announces the topic and forwards it once the companion's ingress demands
it. A block does not "turn networking on"; it TIGHTENS the run to Strict,
which binds the block's locators VERBATIM in a per-run gateway and ignores
`CERULION_NETD_LISTEN`. The companion side needs one because `ingress:` is
only declarable inside a block. See `../../../../docs/networking.md`.

## Run

```bash
# From the examples/go2 workspace:
cerulion node build keyboard_teleop --release

# On the companion, in a real terminal (ssh -t), with the gamepad + mux + driver:
cerulion graph run teleop --release --single-process
# Or on your desk, paired with teleop_remote on the companion:
cerulion graph run teleop_desk --release --single-process
# WASD to move; space to stop; Ctrl-C to quit (terminal restored).
```

### Watching the teleop stream

Both the mux's arbitrated `/go2/cmd_vel` and this raw `/go2/cmd_vel/keyboard`
render on YOUR machine, nothing is staged into this or any sibling graph:

```bash
# `--robot` requires at least one topic, name the ones you want:
cerulion viz --robot go2 /go2/cmd_vel /go2/cmd_vel/keyboard
```

`cerulion-netd` mirrors each demanded topic into desk-local shared memory and
`cerulion-vizd` does the decode + archetype construction there. This node needs
no change either way, it just publishes the Twist.

> **Visualization runs on your computer.** No visualization node is staged on
> the robot or in any graph here. The robot publishes sensor and command data
> as raw frames; decoding and rasterization are always desk-side.

## Notes

- `crossterm` is pinned to `0.28` to match `cerulion_cli_tui` (one workspace
  version of the TUI family).
- The node is portable: it runs on the companion computer (`teleop.yaml`,
  no network hop) as well as on a desk (`teleop_desk.yaml`).
- If there is no TTY (piped stdin), the node logs a warn, publishes its single
  startup zero, and stays quiet (no fabricated keys).
