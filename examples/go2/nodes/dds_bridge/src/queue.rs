// SPDX-License-Identifier: AGPL-3.0-only
//! The per-port latest-wins sample queue shared between the DDS helper thread
//! (pushes) and the node tick (drains). PURE — oracle-tested below.
//!
//! One slot per fixed output port ([`RosType::slot`] order). A push into an
//! occupied slot REPLACES the held sample (latest wins — stale robot state /
//! lidar frames are worthless, the camera_jpeg drop-oldest philosophy) and
//! counts the eviction. `drain_all` empties every slot in fixed port order —
//! the deterministic drain order the tick's write sequence (and hence the
//! published wire order within one fire) inherits.

use crate::registry::{BridgeSample, RosType, ALL_ROS_TYPES};

/// The number of fixed ports (= queue slots).
pub const PORT_COUNT: usize = ALL_ROS_TYPES.len();

/// The shared queue: one latest-wins slot per port + eviction accounting.
///
/// CAPTURED. These are frames the pump has decoded and the tick has
/// not yet published — losing them at an anchor would lose data the run really
/// held, so this is state by the sharpest reading of the rule.
#[derive(Debug, Default, cerulion_core::state::CerulionState)]
pub struct SampleQueue {
    /// `#[cerulion(serde)]` because [`BridgeSample`]'s payloads live in
    /// `cerulion_go2_dds`, which has no `cerulion_core` dependency; they do
    /// derive serde. The cost is stated where the escape is documented — a
    /// serde field folds only its NAME into `STATE_SHAPE` and forfeits
    /// `INLINE_SAFE` — and neither bites here: this queue is reached through
    /// an `Arc<Mutex<..>>`, whose inventory row already forces `INLINE_SAFE`
    /// false, so the bridge takes the fork carrier either way (amendment 9
    /// predicted exactly that for this node).
    #[cerulion(serde)]
    slots: [Option<BridgeSample>; PORT_COUNT],
    /// Lifetime evictions per port (push into an occupied slot). Never reset
    /// (Principle #3 queryability).
    dropped: [u64; PORT_COUNT],
    /// Lifetime samples handed out by `drain_all` — the queue→tick hop
    /// counter (only the tick drains; pair with the pump's
    /// `samples_pushed_total` to localize a dead hop). Never reset.
    drained: u64,
}

/// One drain's outcome: the samples (fixed port order) and how many samples
/// were EVICTED by pushes since the previous drain (the per-drain delta is
/// the caller's to compute from [`SampleQueue::lifetime_dropped`]).
#[derive(Debug, Default)]
pub struct Drained {
    /// At most one sample per port, in fixed port order.
    pub samples: Vec<BridgeSample>,
}

impl SampleQueue {
    /// Push one decoded sample into its port slot (latest wins; eviction
    /// counted).
    pub fn push(&mut self, sample: BridgeSample) {
        let slot = sample.ros_type().slot();
        if self.slots[slot].replace(sample).is_some() {
            self.dropped[slot] += 1;
        }
    }

    /// Empty every slot, returning held samples in fixed port order.
    pub fn drain_all(&mut self) -> Drained {
        let mut samples = Vec::with_capacity(PORT_COUNT);
        for slot in &mut self.slots {
            if let Some(s) = slot.take() {
                samples.push(s);
            }
        }
        self.drained += samples.len() as u64;
        Drained { samples }
    }

    /// Lifetime evictions for one port (never reset).
    pub fn lifetime_dropped(&self, t: RosType) -> u64 {
        self.dropped[t.slot()]
    }

    /// Lifetime evictions across all ports.
    pub fn lifetime_dropped_total(&self) -> u64 {
        self.dropped.iter().sum()
    }

    /// Lifetime samples drained by the tick (never reset — the queue→tick hop
    /// counter, Principle #3).
    pub fn lifetime_drained_total(&self) -> u64 {
        self.drained
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_go2_dds::messages::{Request, Twist, Vector3};

    fn twist(x: f64) -> BridgeSample {
        BridgeSample::Twist(Twist {
            linear: Vector3 { x, y: 0.0, z: 0.0 },
            angular: Vector3::default(),
        })
    }

    fn request(id: i64) -> BridgeSample {
        let mut r = Request::default();
        r.header.identity.id = id;
        BridgeSample::Request(r)
    }

    #[test]
    fn drain_returns_fixed_port_order_regardless_of_push_order() {
        let mut q = SampleQueue::default();
        // Push in REVERSE port order; drain must come back in port order
        // (twist slot 2 before request slot 3 — wait, request IS after twist;
        // use request-then-twist to invert).
        q.push(request(1));
        q.push(twist(1.0));
        let d = q.drain_all();
        assert_eq!(d.samples.len(), 2);
        assert_eq!(d.samples[0].ros_type(), RosType::Twist, "slot 2 first");
        assert_eq!(d.samples[1].ros_type(), RosType::Request, "slot 3 second");
        // Drained queue is empty; the drained hop counter is exact and
        // lifetime (2 after the real drain, unchanged by the empty one).
        assert_eq!(q.lifetime_drained_total(), 2);
        assert!(q.drain_all().samples.is_empty());
        assert_eq!(q.lifetime_drained_total(), 2);
    }

    #[test]
    fn latest_wins_with_exact_eviction_accounting() {
        let mut q = SampleQueue::default();
        q.push(twist(1.0));
        q.push(twist(2.0));
        q.push(twist(3.0));
        assert_eq!(q.lifetime_dropped(RosType::Twist), 2, "two evicted");
        let d = q.drain_all();
        assert_eq!(d.samples, vec![twist(3.0)], "newest wins");
        // Counter is lifetime (never reset by drain).
        q.push(twist(4.0));
        q.push(twist(5.0));
        assert_eq!(q.lifetime_dropped(RosType::Twist), 3);
        assert_eq!(q.lifetime_dropped_total(), 3);
        // Other ports unaffected.
        assert_eq!(q.lifetime_dropped(RosType::PointCloud2), 0);
    }

    #[test]
    fn empty_drain_is_empty_not_a_panic() {
        let mut q = SampleQueue::default();
        assert!(q.drain_all().samples.is_empty());
        assert_eq!(q.lifetime_dropped_total(), 0);
        assert_eq!(q.lifetime_drained_total(), 0);
    }
}
