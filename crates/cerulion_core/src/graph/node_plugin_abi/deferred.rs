// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded node-local terminal operations whose admission was refused.
use crate::error::TransportResult;
use crate::node_abi::LeaseHandle;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Terminal {
    Finish(LeaseHandle, u32),
    Discard(LeaseHandle, u32),
    Release(LeaseHandle),
}
impl Terminal {
    fn lease(self) -> LeaseHandle {
        match self {
            Self::Finish(lease, _) | Self::Discard(lease, _) | Self::Release(lease) => lease,
        }
    }
}
#[derive(Default)]
pub(super) struct Deferred {
    slots: Vec<Option<Terminal>>,
    head: usize,
    len: usize,
}
impl Deferred {
    // hot-path-alloc-ok-fn: provision the complete host lease budget only during cold node creation.
    pub(super) fn extend_capacity(&mut self, additional: usize) -> TransportResult<()> {
        if self.len != 0 {
            return Err(super::argument_error(
                "deferred release capacity must be configured before use",
            ));
        }
        let size = self
            .slots
            .len()
            .checked_add(additional)
            .ok_or_else(|| super::argument_error("deferred release budget overflows"))?;
        self.slots.resize(size, None);
        self.head = 0;
        Ok(())
    }
    pub(super) fn len(&self) -> usize {
        self.len
    }
    pub(super) fn push(&mut self, operation: Terminal) -> TransportResult<()> {
        if self
            .slots
            .iter()
            .flatten()
            .any(|pending| pending.lease() == operation.lease())
        {
            return Ok(());
        }
        if self.len == self.slots.len() {
            return Err(super::argument_error("deferred release budget exhausted"));
        }
        let index = (self.head + self.len) % self.slots.len();
        self.slots[index] = Some(operation);
        self.len += 1;
        Ok(())
    }
    pub(super) fn pop(&mut self) -> Option<Terminal> {
        if self.len == 0 {
            return None;
        }
        let operation = self.slots[self.head].take();
        self.head = (self.head + 1) % self.slots.len();
        self.len -= 1;
        operation
    }
    pub(super) fn front(&self) -> Option<Terminal> {
        if self.len == 0 {
            None
        } else {
            self.slots[self.head]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lease(value: u64) -> LeaseHandle {
        LeaseHandle { value }
    }

    #[test]
    fn bounded_fifo_deduplicates_leases_and_survives_ring_wrap() {
        let mut queue = Deferred::default();
        queue.extend_capacity(2).unwrap();
        queue.push(Terminal::Release(lease(11))).unwrap();
        queue.push(Terminal::Release(lease(11))).unwrap();
        queue.push(Terminal::Discard(lease(22), 1)).unwrap();
        assert_eq!(queue.len(), 2);
        assert!(queue.push(Terminal::Finish(lease(33), 64)).is_err());
        assert_eq!(queue.front(), Some(Terminal::Release(lease(11))));
        assert_eq!(queue.front(), Some(Terminal::Release(lease(11))));
        assert_eq!(queue.pop(), Some(Terminal::Release(lease(11))));
        queue.push(Terminal::Finish(lease(33), 64)).unwrap();
        assert_eq!(queue.pop(), Some(Terminal::Discard(lease(22), 1)));
        assert_eq!(queue.pop(), Some(Terminal::Finish(lease(33), 64)));
        assert_eq!(queue.pop(), None);
        assert_eq!(queue.front(), None);
    }

    #[test]
    fn unprovisioned_in_use_and_overflow_budgets_are_refused() {
        let mut queue = Deferred::default();
        assert!(queue.push(Terminal::Release(lease(1))).is_err());
        queue.extend_capacity(1).unwrap();
        assert!(queue.extend_capacity(usize::MAX).is_err());
        queue.push(Terminal::Release(lease(1))).unwrap();
        assert!(queue.extend_capacity(1).is_err());
        assert_eq!(queue.pop(), Some(Terminal::Release(lease(1))));
        queue.extend_capacity(1).unwrap();
        assert_eq!(queue.len(), 0);
    }
}
