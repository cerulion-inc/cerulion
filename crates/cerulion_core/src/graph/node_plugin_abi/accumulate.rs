// SPDX-License-Identifier: AGPL-3.0-only
//! Node-local accumulation identities; all buffered frames and drops stay on host.

use std::sync::Arc;

use crate::error::{TransportError, TransportResult};
use crate::message::ShmMessage;
use crate::node_abi::{AccumulateInfoV1, PortHandle};
use crate::state::{StateCursor, StateSink};

use super::{Accumulated, HostClient, NodeContext};

struct Accumulator {
    name: Arc<str>,
    topic: Arc<str>,
    port: PortHandle,
    client: HostClient,
    cap: usize,
    serving: AccumulateInfoV1,
}

/// Only configuration and current scalar observations are stored locally.
#[derive(Default)]
pub struct AccumulatorSet {
    inputs: Vec<Accumulator>,
}

impl AccumulatorSet {
    pub fn new() -> Self {
        Self::default()
    }

    // hot-path-alloc-ok-fn: create registers declaration-order accumulate port identities once.
    pub fn register<T: ShmMessage>(
        &mut self,
        context: &mut NodeContext,
        name: &str,
        cap: usize,
    ) -> TransportResult<()> {
        let _: () = T::_SHM_INVARIANTS;
        if cap == 0 || cap > crate::transport::accumulate::MAX_ACCUMULATE_CAP {
            return Err(super::argument_error(
                "accumulate capacity is outside the supported bounds",
            ));
        }
        if self.inputs.iter().any(|input| &*input.name == name) {
            return Err(super::argument_error(
                "accumulate input was registered twice",
            ));
        }
        let subscriber = context
            .subscriber(name)
            .ok_or_else(|| TransportError::NodeError {
                node_id: name.into(),
                reason: "accumulate input has no bound subscriber".into(),
            })?;
        subscriber.client.extend_deferred_budget(
            cap.checked_mul(2)
                .ok_or_else(|| super::argument_error("deferred accumulate budget overflows"))?,
        )?;
        self.inputs.push(Accumulator {
            name: Arc::clone(&subscriber.name),
            topic: Arc::clone(&subscriber.topic),
            port: subscriber.handle,
            client: subscriber.client.clone(),
            cap,
            serving: AccumulateInfoV1::default(),
        });
        Ok(())
    }

    pub fn mark_served(&mut self, name: &str) {
        if let Some(input) = self.inputs.iter_mut().find(|input| &*input.name == name) {
            match input.client.accumulate_count(input.port) {
                Ok(info) if info.count as usize <= input.cap => input.serving = info,
                Ok(_) => {
                    input.serving = AccumulateInfoV1::default();
                    input.client.record_failure(super::argument_error(
                        "host accumulate frame count exceeds its bound",
                    ));
                }
                Err(error) => {
                    input.serving = AccumulateInfoV1::default();
                    input.client.record_failure(error);
                }
            }
        }
    }

    pub fn view<'a, T: ShmMessage + 'a>(&'a self, name: &str) -> Accumulated<'a, T> {
        match self.inputs.iter().find(|input| &*input.name == name) {
            Some(input) => Accumulated::new(
                &input.topic,
                Some(HostClient::share(&input.client)),
                input.port,
                input.serving.count as usize,
                input.serving.dropped,
                input.serving.dropped_total,
            ),
            None => Accumulated::new("", None, PortHandle::default(), 0, 0, 0),
        }
    }

    /// Append exactly the native accumulator encoding after the user fields.
    pub fn capture(&self, sink: &mut dyn StateSink) -> TransportResult<()> {
        if self.inputs.is_empty() {
            return Ok(());
        }
        let count = u32::try_from(self.inputs.len())
            .map_err(|_| super::argument_error("accumulate input count exceeds u32"))?;
        sink.write(&count.to_le_bytes())
            .map_err(|_| super::state_error(crate::state::StateError::SinkFull))?;
        let mut bridge = super::state::SinkBridge::new(sink);
        for input in &self.inputs {
            let result = input
                .client
                .capture_accumulate(input.port, &bridge.descriptor());
            // Preserve the native sink failure before a generic host status.
            bridge.check()?;
            result?;
        }
        Ok(())
    }

    /// Restore borrowed native per-port records without copying any frame.
    // hot-path-alloc-ok-fn: canonical restore is a cold lifecycle operation; only refusals format names.
    pub fn restore(&mut self, cursor: &mut StateCursor<'_>) -> TransportResult<()> {
        if self.inputs.is_empty() {
            return Ok(());
        }
        let count = cursor.read_element_count().map_err(super::state_error)?;
        if count != self.inputs.len() {
            return Err(super::state_error(
                crate::state::StateError::AccumulateRestore {
                    input: self.inputs[0].name.to_string(),
                    reason: format!(
                        "the recording holds {count} accumulate input(s), the node declares {}",
                        self.inputs.len()
                    ),
                },
            ));
        }
        for input in &mut self.inputs {
            // Clone only the bounded cursor (no bytes/allocation), locate one
            // canonical record and borrow its exact bytes from the real cursor.
            let mut scan = StateCursor::clone(cursor);
            let start = scan.position();
            let name_len = scan.read_payload_len().map_err(super::state_error)?;
            let name = scan.take(name_len).map_err(super::state_error)?;
            if name != input.name.as_bytes() {
                return Err(super::state_error(
                    crate::state::StateError::AccumulateRestore {
                        input: input.name.to_string(),
                        reason: format!(
                            "the recording names input `{}` in its place",
                            String::from_utf8_lossy(name)
                        ),
                    },
                ));
            }
            let _ = scan.take(16).map_err(super::state_error)?;
            let frames = scan.read_element_count().map_err(super::state_error)?;
            if frames > input.cap {
                return Err(super::state_error(
                    crate::state::StateError::AccumulateRestore {
                        input: input.name.to_string(),
                        reason: format!(
                            "the recording holds {frames} frame(s), above the cap of {}",
                            input.cap
                        ),
                    },
                ));
            }
            for _ in 0..frames {
                let size = scan.read_payload_len().map_err(super::state_error)?;
                let _ = scan.take(size).map_err(super::state_error)?;
            }
            cursor.charge_elements(frames);
            let record = cursor
                .take(scan.position() - start)
                .map_err(super::state_error)?;
            input.client.restore_accumulate(input.port, record)?;
            input.serving = AccumulateInfoV1::default();
        }
        Ok(())
    }
}
