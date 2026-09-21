// SPDX-License-Identifier: AGPL-3.0-only
//! Go2 sport driver: the actuation end of the teleop stack.
//!
//! A data-triggered node that turns each arbitrated `geometry_msgs/Twist`
//! (the safety mux's `/go2/cmd_vel`, 50 Hz) into a `unitree_api/Request` on
//! the robot's `/api/sport/request` DDS topic: a nonzero command becomes a
//! `Move` request every tick, an all-zero command becomes `StopMove` on the
//! transition and then at a 5 Hz keepalive. The WHOLE policy is the pure
//! [`request`] module (oracle-vector tests, no DDS); this file is the
//! transport wrapper plus the live DDS writer in [`dds`].
//!
//! # Where the safety lives
//!
//! This node is the LAST soft layer of the stop ladder, and deliberately a
//! dumb one: it forwards what the mux decided. The mux's staleness gate
//! (250 ms joystick, 750 ms keyboard) is what turns a dead teleop source
//! into sustained zeros, and sustained zeros are what this node turns into
//! `StopMove`. The hardware e-stop stays above all of it. This node never
//! issues a posture command (StandUp, StandDown, Damp): the operator stands
//! the robot up with the vendor remote, on a stand, before any teleop, and
//! nothing here can undo that on a stray frame.
//!
//! # Configuration (read once, from the frozen env snapshot)
//!
//! | variable | default | meaning |
//! |---|---|---|
//! | `GO2_DOMAIN_ID` | `0` | the DDS domain (the Go2 factory `ROS_DOMAIN_ID`) |
//! | `GO2_IFACE` | unset | the companion's robot-LAN interface IP(s); REQUIRED on a multi-homed host (the CycloneDDS DATAFRAG constraint the bridge documents) |
//!
//! Both are read through `ctx.env` / `ctx.env_str` in `init`, so a recording
//! replays with the configuration it ran under. A DDS failure at `init`
//! FAILS THE GRAPH BUILD naming this node: a driver that cannot reach the
//! robot must never run inert.
//!
//! # ONE participant per process
//!
//! The writer rides the example's `Go2Participant` guard: a second DDS
//! participant in the same process (another instance of this node, or the
//! ingress bridge loaded into the SAME process) fails `init` loudly. Run the
//! bridge graph and the teleop graph as separate `cerulion graph run`s.
//!
//! # Determinism (Principle 7)
//!
//! The tick is a pure function of (node clock, the triggering command, three
//! plain state fields). The DDS write is the node's side effect on the
//! world, exactly like a camera's frame is a driver's; replay re-executes
//! the decision sequence bit for bit and the captured-sink e2e pins it.

// NOTE: no crate-level `#![forbid(unsafe_code)]`: the `#[cerulion_node]`
// macro expands cdylib FFI entry points containing `unsafe`. The PURE
// `request` module keeps the forbid at module scope.

// The project logging rule: library code never prints, it logs through
// `tracing`. Scoped `not(test)` so unit tests keep printing diagnostics.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod dds;
pub mod request;

pub use dds::SPORT_REQUEST_TOPIC;
pub use request::{MAX_VX, MAX_VY, MAX_VYAW};

use std::sync::{Arc, Mutex};

use cerulion_core::prelude::*;
use cerulion_go2_dds::messages::Request;
use cerulion_go2_dds::participant::{parse_iface_list, ParticipantConfig, GO2_IFACE_ENV};
use native_ros2_messages::geometry_msgs::Twist;

use crate::request::{build_request, decide, DriverState, Velocity};

/// The env var naming the DDS domain id (default 0, the Go2 factory value).
pub const DOMAIN_ENV: &str = "GO2_DOMAIN_ID";

/// Where requests go: the live DDS writer, or a test sink capturing them.
#[derive(Debug, Default)]
enum RequestSink {
    /// `init` has not run (or failed): every tick is a loud, counted no-op.
    #[default]
    Unopened,
    /// The live writer on `/api/sport/request`.
    Dds(Box<dds::DdsRequestWriter>),
    /// The test seam: every request is pushed here instead of DDS.
    Captured(Arc<Mutex<Vec<Request>>>),
}

/// The sport driver node. See the module docs for the full contract.
// Field notes (plain comments; port fields carry only the macro attrs):
// - cmd: the arbitrated velocity command (fires the node per frame).
// - sink: the DDS writer, a HANDLE rebuilt by `init` (never captured).
// - prev_was_move / last_stop_sent_ns / next_request_id: the pure policy's
//   carried state (plain fields, captured and replayed like any state).
// - sent / publish_failures / rejected_non_finite: observability counters,
//   never reset, summarised at shutdown.
// - publish_failure_regime_open: the flood latch for publish failures (loud
//   first, debug repeats, one recovery line).
#[cerulion_node]
#[derive(Default)]
pub struct SportDriver {
    #[input(trigger)]
    cmd: Twist,
    #[cerulion(reconstruct)]
    sink: RequestSink,
    prev_was_move: bool,
    last_stop_sent_ns: Option<u64>,
    next_request_id: i64,
    sent: u64,
    publish_failures: u64,
    rejected_non_finite: u64,
    publish_failure_regime_open: bool,
}

impl SportDriver {
    /// Construct the node over a caller-provided capture sink: the test seam
    /// `tests/driver_e2e_test.rs` uses to drive the whole trigger, decide,
    /// build, publish path over real iceoryx2 with NO DDS. `init` skips the
    /// DDS writer when a sink is already installed.
    pub fn with_captured_sink(sink: Arc<Mutex<Vec<Request>>>) -> Self {
        Self {
            sink: RequestSink::Captured(sink),
            ..Default::default()
        }
    }

    /// Lifetime requests handed to the sink (DDS or captured).
    pub fn sent(&self) -> u64 {
        self.sent
    }

    /// Lifetime publish failures (counted whether or not they were logged).
    pub fn publish_failures(&self) -> u64 {
        self.publish_failures
    }

    /// Lifetime commands carrying a non-finite component (each treated as
    /// all zero).
    pub fn rejected_non_finite(&self) -> u64 {
        self.rejected_non_finite
    }

    fn state(&self) -> DriverState {
        DriverState {
            prev_was_move: self.prev_was_move,
            last_stop_sent_ns: self.last_stop_sent_ns,
            // A zero-initialized (fresh or restored-from-default) node starts
            // at identity 1, the policy's own default.
            next_request_id: if self.next_request_id == 0 {
                DriverState::default().next_request_id
            } else {
                self.next_request_id
            },
        }
    }

    fn set_state(&mut self, state: DriverState) {
        self.prev_was_move = state.prev_was_move;
        self.last_stop_sent_ns = state.last_stop_sent_ns;
        self.next_request_id = state.next_request_id;
    }

    /// The publish-failure flood latch: the first failure of a regime is a
    /// loud `warn!`, repeats log at `debug!` with the running total, and the
    /// first success afterwards logs one `info!`. The counter is
    /// unconditional (Principle 3: the number survives the log level).
    fn record_publish(&mut self, result: Result<(), String>) {
        match result {
            Ok(()) => {
                self.sent += 1;
                if self.publish_failure_regime_open {
                    self.publish_failure_regime_open = false;
                    tracing::info!(
                        total_failures = self.publish_failures,
                        "sport_driver: request publish recovered"
                    );
                }
            }
            Err(e) => {
                self.publish_failures += 1;
                if self.publish_failure_regime_open {
                    tracing::debug!(
                        error = %e,
                        total_failures = self.publish_failures,
                        "sport_driver: request publish failed (suppressed repeat)"
                    );
                } else {
                    self.publish_failure_regime_open = true;
                    tracing::warn!(
                        error = %e,
                        total_failures = self.publish_failures,
                        "sport_driver: request publish FAILED; the robot is not \
                         receiving commands (repeats log at debug until it recovers)"
                    );
                }
            }
        }
    }
}

#[cerulion_node_impl]
impl SportDriver {
    /// Open the DDS writer from the frozen configuration. A failure here
    /// fails the graph build naming this node.
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        if matches!(self.sink, RequestSink::Captured(_)) {
            tracing::info!("sport_driver: driving a CAPTURED sink (test seam); no DDS writer");
            return Ok(());
        }
        let domain_id: u16 = ctx.env(DOMAIN_ENV, 0);
        let only_networks = parse_iface_list(&ctx.env_str(GO2_IFACE_ENV, ""));
        let config = ParticipantConfig::new(domain_id, only_networks);
        let writer = dds::DdsRequestWriter::open(&config).map_err(|e| {
            NodeError::Fatal(format!(
                "sport_driver could not open its DDS request writer ({e}); a driver that \
                 cannot reach the robot must not run. Check {GO2_IFACE_ENV} (the \
                 companion's robot-LAN interface IP) and {DOMAIN_ENV}."
            ))
        })?;
        self.sink = RequestSink::Dds(Box::new(writer));
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns();
        let cmd = Velocity {
            vx: self.cmd.linear.x,
            vy: self.cmd.linear.y,
            vyaw: self.cmd.angular.z,
        };
        let decision = decide(now, cmd, self.state());
        if decision.rejected_non_finite {
            self.rejected_non_finite += 1;
        }
        let id = self.state().next_request_id;
        self.set_state(decision.state);

        let Some(command) = decision.command else {
            return Ok(());
        };
        let request = build_request(id, command);
        let result = match &self.sink {
            RequestSink::Dds(writer) => writer.publish(request),
            RequestSink::Captured(sink) => {
                sink.lock()
                    .unwrap_or_else(std::sync::PoisonError::into_inner)
                    .push(request);
                Ok(())
            }
            RequestSink::Unopened => Err(
                "no request writer: init() did not run or failed (a restored node \
                 rebuilds it at init)"
                    .to_string(),
            ),
        };
        self.record_publish(result);
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        tracing::info!(
            sent = self.sent,
            publish_failures = self.publish_failures,
            rejected_non_finite = self.rejected_non_finite,
            "sport_driver teardown summary"
        );
        // Drop the writer explicitly so the participant's disposes go out
        // now, before the process exits.
        self.sink = RequestSink::Unopened;
        Ok(())
    }
}
