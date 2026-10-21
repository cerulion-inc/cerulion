// SPDX-License-Identifier: AGPL-3.0-only
//! Test cdylib that returns one chosen `NodeError` arm, so a test can check
//! that the error's KIND crosses the cdylib boundary as a typed value.
//!
//! The node reads two keys from its env snapshot in `init` (the snapshot is
//! frozen at graph build, so a value cannot change mid-run). The snapshot is
//! taken from the process env of the run that builds the graph, so a resim
//! must have the same two keys in ITS env; the `CERULION_` prefix keeps them
//! in a recording's `env.json`, which the resim compares against and warns on
//! a divergence:
//!
//! - `CERULION_TEST_ERROR_KIND`: `logic`, `invalid_input`, `fatal`, `custom`,
//!   `transport` or `io`. Empty means the node never fails.
//! - `CERULION_TEST_ERROR_AT`: `init`, `shutdown`, or a tick number `N` (1-based): the
//!   node publishes on ticks before `N` and returns the error from tick `N`
//!   onward. Empty means tick 1.
//!
//! Each published `out` frame carries the tick number in `x`, so a test can
//! read exactly which ticks published.

#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

/// The error for one `CERULION_TEST_ERROR_KIND` word, or `None` for no failure.
fn error_for(kind: &str) -> Option<NodeError> {
    Some(match kind {
        "logic" => NodeError::Logic("probe logic".to_string()),
        "invalid_input" => NodeError::InvalidInput {
            input: "probe".to_string(),
            reason: "probe invalid".to_string(),
        },
        "fatal" => NodeError::Fatal("probe fatal".to_string()),
        "custom" => NodeError::Custom(Box::new(std::io::Error::other("probe custom"))),
        "transport" => NodeError::Transport(TransportError::NotInitialized),
        "io" => NodeError::Io(std::io::Error::other("probe io")),
        _ => return None,
    })
}

#[cerulion_node(period_ms = 10)]
struct ErrorKindProbe {
    #[output]
    out: Vector3,
    kind: String,
    at: String,
    ticks: u64,
}

#[cerulion_node_impl]
impl ErrorKindProbe {
    fn init(&mut self, ctx: &mut NodeContext) -> Result<(), NodeError> {
        self.kind = ctx.env_str("CERULION_TEST_ERROR_KIND", "");
        self.at = ctx.env_str("CERULION_TEST_ERROR_AT", "");
        if self.at == "init" {
            if let Some(e) = error_for(&self.kind) {
                return Err(e);
            }
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        let fail_from: u64 = match self.at.as_str() {
            "init" | "shutdown" => u64::MAX,
            "" => 1,
            n => n.parse().unwrap_or(1),
        };
        if self.ticks >= fail_from {
            if let Some(e) = error_for(&self.kind) {
                return Err(e);
            }
        }
        self.out.x = self.ticks as f64;
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if self.at == "shutdown" {
            if let Some(e) = error_for(&self.kind) {
                return Err(e);
            }
        }
        Ok(())
    }
}
