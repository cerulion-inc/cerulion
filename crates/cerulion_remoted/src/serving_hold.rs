// SPDX-License-Identifier: AGPL-3.0-only
//! Work which retains the owned remote endpoint through idle teardown.
//!
//! Shell hosts acquire a Shell guard before admitting work and retain it through
//! child drain and final witness. The daemon registers its latch and owed e-stop
//! sources, so disconnect cannot strand safety state. The native exclusive fence
//! used by netd prevents fresh work from racing a committed idle stop.

pub use cerulion_netd::serving_hold::{HoldReason, ServingHold, ServingHolds};
