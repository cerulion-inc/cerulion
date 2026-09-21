<!-- SPDX-License-Identifier: AGPL-3.0-only -->
# Joystick teleop bring-up

The `joystick_teleop` node reads a Bluetooth gamepad on the companion
computer (via `gilrs`, evdev backend on Linux) and publishes `geometry_msgs/Twist` on
`/go2/cmd_vel/joystick`. The pad is just an evdev device to Linux, the Go2
SDK never sees it. This is the hardware bring-up checklist; the node's logic
is CI-verified without hardware (`tests/joystick_e2e_test.rs`).

## 1. Bluetooth pairing

```bash
bluetoothctl
# in the prompt:
power on
agent on
default-agent
scan on            # put the pad in pairing mode; note its MAC (XX:XX:...)
pair  XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
scan off
quit
```

Verify the pad enumerated as an evdev device:

```bash
ls -l /dev/input/by-id/ | grep -i "joystick\|gamepad\|controller"
# or:
cat /proc/bus/input/devices        # find the Name= / Handlers=event* line
sudo evtest /dev/input/eventN       # (optional) confirm axis/button events
```

## 2. Permissions (when the user is not in the `input` group)

`gilrs`/evdev needs read access to `/dev/input/event*`. A stock image's user
is usually not a member of `input`, so pick ONE of:

**A. Add the user to `input` (persistent, preferred):**

```bash
sudo usermod -aG input "$USER"
# log out / back in (or `newgrp input`) for the group to take effect
groups | tr ' ' '\n' | grep -x input     # verify
```

**B. A udev rule granting the gamepad to the running user (no group change):**

```bash
# /etc/udev/rules.d/99-go2-gamepad.rules
# match your pad's vendor/product (from `udevadm info -a -n /dev/input/eventN`)
SUBSYSTEM=="input", ATTRS{idVendor}=="054c", ATTRS{idProduct}=="0ce6", MODE="0660", TAG+="uaccess"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

`TAG+="uaccess"` grants the locally-logged-in user access via systemd-logind;
no group membership needed. Use MODE+GROUP instead if the process runs
headless (no active login session).

If `gilrs` cannot open the device the node still starts and publishes its
single startup zero, then logs a loud `gilrs init failed` warn and stays quiet
(no fake motion), check the permissions above.

## 3. Mapping + safety (as shipped)

| Physical input | Output | Notes |
|---|---|---|
| left stick up/down | `linear.x` (±0.6 m/s) | up = forward |
| left stick left/right | `linear.y` (±0.4 m/s) | left = +y (strafe left) |
| right stick left/right | `angular.z` (±1.0 rad/s) | left = +z (yaw-left) |
| **RB shoulder (deadman)** | arms motion | must be HELD; released ⇒ one zero then quiet |

Deadzone 0.1; conservative saturation limits (well under the Go2's sport-mode
maxima). A Bluetooth drop publishes one zero + a loud warn and keeps scanning
for reconnect (the node never exits; re-press RB to re-arm after reconnect).

**Liveness watchdog (2 s), and one behavior to know about:** while the
deadman is held, the node expects the pad link to keep delivering events (any
kind, modern pads stream axis/sensor traffic continuously). If NO event
arrives for 2 s with RB held, the link is treated as suspect (a BT-stalled
radio would otherwise keep republishing the last command at 20 Hz until the
kernel's BT supervision timeout, seconds later): the node publishes one zero,
clears the deadman latch, and logs a loud warn. A healthy but event-silent
pad, perfectly still with RB held for more than 2 s, also trips the safe zero:
release and press RB again to re-arm.

## 4. Measure button-to-command latency

Measure button/stick → Twist-publish end-to-end (RB press to first non-zero
frame on `/go2/cmd_vel/joystick`). A true button-to-publish figure needs a
timestamp at the input event as well as at publication, so take both from one
monotonic clock:

```bash
RUST_LOG=joystick_teleop=info cerulion graph run teleop --release --single-process --record
# then inspect the recorded frames, or add an in-tick real_ns() stamp for a
# one-off number
```

Record these fields for your setup:

- Pad model (for example 8BitDo, DualSense, Xbox).
- Measured button-to-publish p50 and p99.
- Bluetooth connection interval and any dropouts seen during the run.
