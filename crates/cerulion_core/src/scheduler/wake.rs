// SPDX-License-Identifier: AGPL-3.0-only
//! Inert in production builds: classify a node's `TriggerPolicy` by how the
//! iceoryx2 WaitSet wakes it. Defines the wake-origin taxonomy a
//! WaitSet reactor attaches against (Period -> shared interval timer,
//! Data/Sync -> input-Listener notification, External -> out-of-band eventfd).
//!
//! This module is `#[cfg(test)]`-gated because it has no production
//! consumer: a `pub(crate)` item with zero in-crate callers trips this
//! crate's `dead_code = "deny"`. Wiring the reactor to it REMOVES the
//! `#[cfg(test)]` gate. Classification is pure (no
//! transport, no clock) and fully unit-tested here so the taxonomy is reviewed
//! independently of the reactor wiring.

use super::trigger::TriggerPolicy;
use std::time::Duration;

/// How the WaitSet wakes a node, derived from its `TriggerPolicy`.
///
/// This is a wake-*class* view, NOT a standalone attach descriptor: the
/// name -> Listener mapping (for both `Data` and `Sync` inputs) is resolved by
/// the reactor from `GraphRuntime::data_trigger_bindings`, not from
/// `WakeSource`. `SyncNotification` carries input *names* only (mirroring
/// `TriggerPolicy::Sync`); `Data`'s single trigger-input name likewise lives in
/// the runtime bindings, which is why `DataNotification` is a unit variant.
///
/// NOTE on `Period`: only `interval` is carried, NOT `max_catchup`.
/// `max_catchup` bounds how many *fires* a single wake produces (a
/// scheduler firing concern handled in `evaluate_node`); it does not change
/// *how the node is woken*, so it is intentionally dropped from the wake-source
/// taxonomy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum WakeSource {
    /// `Period` node: woken by a shared interval timer; never by a data event.
    Timer { interval: Duration },
    /// `Data` node: woken by a notification on its trigger input's Listener.
    DataNotification,
    /// `Sync` node: woken by a notification on any of its sync inputs' Listeners.
    SyncNotification { inputs: Vec<String> },
    /// `External` node: woken by an explicit out-of-band trigger (eventfd).
    External,
}

impl From<&TriggerPolicy> for WakeSource {
    fn from(policy: &TriggerPolicy) -> Self {
        match policy {
            TriggerPolicy::Period { interval, .. } => WakeSource::Timer {
                interval: *interval,
            },
            TriggerPolicy::Data => WakeSource::DataNotification,
            TriggerPolicy::Sync { inputs, .. } => WakeSource::SyncNotification {
                inputs: inputs.clone(),
            },
            TriggerPolicy::External => WakeSource::External,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `Period` classifies to `Timer` carrying the declared interval verbatim.
    #[test]
    fn period_classifies_as_timer_carrying_interval() {
        let policy = TriggerPolicy::Period {
            interval: Duration::from_millis(16),
            max_catchup: None,
        };
        assert_eq!(
            WakeSource::from(&policy),
            WakeSource::Timer {
                interval: Duration::from_millis(16),
            }
        );
    }

    /// `max_catchup` is a firing concern, not a wake-origin concern: two
    /// `Period`s with the SAME interval but different `max_catchup` must
    /// classify to the SAME `WakeSource`. Pins the documented drop contract.
    #[test]
    fn period_max_catchup_does_not_affect_wake_source() {
        let unbounded = TriggerPolicy::Period {
            interval: Duration::from_millis(10),
            max_catchup: None,
        };
        let bounded = TriggerPolicy::Period {
            interval: Duration::from_millis(10),
            max_catchup: Some(3),
        };
        assert_eq!(WakeSource::from(&unbounded), WakeSource::from(&bounded));
    }

    /// `Data` is a unit variant -> `DataNotification`.
    #[test]
    fn data_classifies_as_data_notification() {
        let policy = TriggerPolicy::Data;
        assert_eq!(WakeSource::from(&policy), WakeSource::DataNotification);
    }

    /// `Sync` carries its `inputs` verbatim (order + contents exact) and the
    /// `window` does NOT leak into the wake source. Inputs are declared in
    /// REVERSE-alphabetical order (`zeta`, `alpha`) so a stray `.sort()` slipped
    /// into the `From` impl would reorder them to (`alpha`, `zeta`) and fail the
    /// assertion — an alphabetical fixture (`a`, `b`) would survive that
    /// regression silently. (Same sort-defeating fixture rationale as
    /// `on_event_multi_handler_test`; the reactor attaches input Listeners in this
    /// order, so order is load-bearing.)
    #[test]
    fn sync_classifies_as_sync_notification_preserving_inputs() {
        let policy = TriggerPolicy::Sync {
            inputs: vec!["zeta".to_string(), "alpha".to_string()],
            window: Some(Duration::from_millis(50)),
        };
        assert_eq!(
            WakeSource::from(&policy),
            WakeSource::SyncNotification {
                inputs: vec!["zeta".to_string(), "alpha".to_string()],
            }
        );
    }

    /// `External` is a unit variant -> `External`.
    #[test]
    fn external_classifies_as_external() {
        let policy = TriggerPolicy::External;
        assert_eq!(WakeSource::from(&policy), WakeSource::External);
    }
}
