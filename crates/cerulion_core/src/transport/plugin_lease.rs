// SPDX-License-Identifier: AGPL-3.0-only
//! Bounded, host-owned storage for native plugin leases.
//!
//! Handles are integers, never addresses. A row is reserved before touching
//! transport, and its token is retired on its single terminal transition.
//! Principle #1: neither reservation nor return grows storage on a fire.

use std::sync::atomic::{AtomicU32, Ordering};

use crate::wire::WireHeader;

/// Schema facts checked at binding and cached beside a host-owned port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PluginWireLayout {
    /// Canonical schema hash; calculated by the existing schema implementation.
    pub schema_hash: u64,
    /// Bytes of fixed payload before the variable offset table.
    pub fixed_size: u32,
    /// Number of eight-byte offset entries.
    pub variable_fields: u32,
    /// Required frame-base alignment, a power of two no larger than eight.
    pub alignment: u32,
}

impl PluginWireLayout {
    /// Complete initialized prefix, including header and offset table.
    pub fn prefix_len(self) -> Option<u32> {
        if !self.alignment.is_power_of_two() || self.alignment > 8 {
            return None;
        }
        self.variable_fields
            .checked_mul(8)?
            .checked_add(self.fixed_size)?
            .checked_add(WireHeader::SIZE as u32)
    }

    /// Check the entire frame structure before consuming a commit sequence.
    /// The payload is borrowed in place; no bytes are copied or retained.
    pub fn accepts(self, frame: &[u8], total_size: u32) -> bool {
        let Some(prefix) = self.prefix_len() else {
            return false;
        };
        let size = total_size as usize;
        if total_size < prefix || size > frame.len() {
            return false;
        }
        let Some(header) = WireHeader::read_from_buf(frame) else {
            return false;
        };
        if header.schema_hash != self.schema_hash
            || header.offset_table_offset != WireHeader::SIZE as u32 + self.fixed_size
            || header.offset_table_count != self.variable_fields
        {
            return false;
        }
        self.accepts_payload(&frame[WireHeader::SIZE..], total_size)
    }

    /// Validate initialized spill bytes with cached host schema facts.
    /// The original header and loan timestamp remain host-owned.
    pub fn accepts_payload(self, payload: &[u8], total_size: u32) -> bool {
        let Some(prefix) = self.prefix_len() else {
            return false;
        };
        if total_size < prefix || total_size as usize - WireHeader::SIZE > payload.len() {
            return false;
        }
        let table_start = self.fixed_size as usize;
        for entry in payload[table_start..prefix as usize - WireHeader::SIZE].chunks_exact(8) {
            let offset = u32::from_le_bytes(entry[..4].try_into().expect("four-byte offset"));
            let len = u32::from_le_bytes(entry[4..].try_into().expect("four-byte length"));
            // OffsetEntry measures from payload[0]. Empty fields may carry
            // offset zero, matching the existing writer's explicit empty value.
            if len != 0 && offset < prefix - WireHeader::SIZE as u32 {
                return false;
            }
            let Some(end) = offset.checked_add(len) else {
                return false;
            };
            if end > total_size - WireHeader::SIZE as u32 {
                return false;
            }
        }
        true
    }
}

// Shared across contexts: a lease from another context cannot accidentally
// match the first generation of the same row in this arena.
static NEXT_GENERATION: AtomicU32 = AtomicU32::new(1);

/// A refusal before changing a lease row or touching its native resource.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeaseError {
    /// Zero, stale, foreign, or already returned token.
    InvalidHandle,
    /// The token belongs to another port or operation direction.
    WrongOwner,
    /// Every provisioned row is reserved or holds a live sample.
    BudgetExhausted,
    /// No unique generation remains; tokens must never wrap and alias.
    GenerationExhausted,
    /// The requested allocation cannot be represented on this target.
    InvalidBudget,
    /// A reserved row was populated twice or returned before population.
    InvalidTransition,
}

/// Host-local lease operation direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LeaseDirection {
    /// Exclusive, unsent publisher sample.
    Write,
    /// Immutable input sample or held-frame borrow.
    Read,
}

struct Row<T> {
    generation: u32,
    port: u64,
    direction: LeaseDirection,
    reserved: bool,
    value: Option<T>,
    next_free: Option<usize>,
}

/// A fixed-capacity arena. Native owners are dropped only by host code.
pub struct LeaseArena<T> {
    rows: Box<[Row<T>]>,
    first_free: Option<usize>,
    active: usize,
}

impl<T> LeaseArena<T> {
    /// Provision all rows at context construction, before any node fires.
    // hot-path-alloc-ok-fn: fixed lease storage allocated at graph build only
    pub fn new(capacity: usize) -> Result<Self, LeaseError> {
        if capacity > u32::MAX as usize
            || capacity > (isize::MAX as usize) / std::mem::size_of::<Row<T>>()
        {
            return Err(LeaseError::InvalidBudget);
        }
        let rows = (0..capacity)
            .map(|index| Row {
                generation: 0,
                port: 0,
                direction: LeaseDirection::Read,
                reserved: false,
                value: None,
                next_free: (index + 1 < capacity).then_some(index + 1),
            })
            .collect::<Vec<_>>()
            .into_boxed_slice();
        Ok(Self {
            rows,
            first_free: (capacity > 0).then_some(0),
            active: 0,
        })
    }

    /// Number of rows currently reserved or holding native resources.
    pub fn active(&self) -> usize {
        self.active
    }

    /// Resolve a lease-only terminal operation to its checked local port.
    pub fn owner(&self, token: u64) -> Result<(u64, LeaseDirection), LeaseError> {
        let row = self.row(token)?;
        if row.value.is_none() {
            return Err(LeaseError::InvalidTransition);
        }
        Ok((row.port, row.direction))
    }

    /// Reserve before a loan/receive so budget rejection has no queue effects.
    pub fn claim(&mut self, port: u64, direction: LeaseDirection) -> Result<u64, LeaseError> {
        let index = self.first_free.ok_or(LeaseError::BudgetExhausted)?;
        let generation = mint_generation(&NEXT_GENERATION)?;
        let row = &mut self.rows[index];
        self.first_free = row.next_free.take();
        row.generation = generation;
        row.port = port;
        row.direction = direction;
        row.reserved = true;
        self.active += 1;
        Ok((u64::from(generation) << 32) | (index as u64 + 1))
    }

    /// Attach a native owner to a reservation without allocating.
    pub fn insert(&mut self, token: u64, value: T) -> Result<(), LeaseError> {
        let row = self.row_mut(token)?;
        if row.value.is_some() {
            return Err(LeaseError::InvalidTransition);
        }
        row.value = Some(value);
        Ok(())
    }

    /// Inspect ownership before changing any port state or terminal counter.
    pub fn get(&self, token: u64, port: u64, direction: LeaseDirection) -> Result<&T, LeaseError> {
        let row = self.row(token)?;
        check_owner(row, port, direction)?;
        row.value.as_ref().ok_or(LeaseError::InvalidTransition)
    }

    /// Borrow a native owner after validating context-local port ownership.
    pub fn get_mut(
        &mut self,
        token: u64,
        port: u64,
        direction: LeaseDirection,
    ) -> Result<&mut T, LeaseError> {
        let row = self.row_mut(token)?;
        check_owner(row, port, direction)?;
        row.value.as_mut().ok_or(LeaseError::InvalidTransition)
    }

    /// Consume the owner exactly once. Wrong direction leaves the row intact.
    pub fn take(
        &mut self,
        token: u64,
        port: u64,
        direction: LeaseDirection,
    ) -> Result<T, LeaseError> {
        let row = self.row_mut(token)?;
        check_owner(row, port, direction)?;
        let value = row.value.take().ok_or(LeaseError::InvalidTransition)?;
        self.recycle(token);
        Ok(value)
    }

    /// Cancel an empty reservation after an empty read or failed transport call.
    pub fn cancel(&mut self, token: u64) -> Result<(), LeaseError> {
        if self.row(token)?.value.is_some() {
            return Err(LeaseError::InvalidTransition);
        }
        self.recycle(token);
        Ok(())
    }

    /// Return all owners during teardown, while their ports still exist.
    pub fn revoke_all(&mut self) {
        let capacity = self.rows.len();
        for (index, row) in self.rows.iter_mut().enumerate() {
            // Clear before drop, including a destructor that unwinds. The
            // boundary execution gate catches while retaining the context lock.
            row.reserved = false;
            row.generation = 0;
            drop(row.value.take());
            row.next_free = (index + 1 < capacity).then_some(index + 1);
        }
        self.first_free = (!self.rows.is_empty()).then_some(0);
        self.active = 0;
    }

    fn row(&self, token: u64) -> Result<&Row<T>, LeaseError> {
        let index = token_index(token)?;
        let row = self.rows.get(index).ok_or(LeaseError::InvalidHandle)?;
        validate_token(row, token)?;
        Ok(row)
    }

    fn row_mut(&mut self, token: u64) -> Result<&mut Row<T>, LeaseError> {
        let index = token_index(token)?;
        let row = self.rows.get_mut(index).ok_or(LeaseError::InvalidHandle)?;
        validate_token(row, token)?;
        Ok(row)
    }

    fn recycle(&mut self, token: u64) {
        // Called only after row()/row_mut() validated the index and token.
        let index = (token as u32 - 1) as usize;
        let row = &mut self.rows[index];
        row.reserved = false;
        row.generation = 0;
        row.next_free = self.first_free;
        self.first_free = Some(index);
        self.active -= 1;
    }
}

fn token_index(token: u64) -> Result<usize, LeaseError> {
    (token as u32)
        .checked_sub(1)
        .map(|index| index as usize)
        .ok_or(LeaseError::InvalidHandle)
}

fn validate_token<T>(row: &Row<T>, token: u64) -> Result<(), LeaseError> {
    if !row.reserved || row.generation == 0 || row.generation != (token >> 32) as u32 {
        return Err(LeaseError::InvalidHandle);
    }
    Ok(())
}

fn check_owner<T>(row: &Row<T>, port: u64, direction: LeaseDirection) -> Result<(), LeaseError> {
    if row.port != port || row.direction != direction {
        return Err(LeaseError::WrongOwner);
    }
    Ok(())
}

fn mint_generation(counter: &AtomicU32) -> Result<u32, LeaseError> {
    counter
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |current| {
            current.checked_add(1)
        })
        .map_err(|_| LeaseError::GenerationExhausted)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn returning_a_native_owner_retires_its_token() {
        let mut arena = LeaseArena::new(1).unwrap();
        let token = arena.claim(7, LeaseDirection::Write).unwrap();
        arena.insert(token, [1, 2, 3]).unwrap();
        assert_eq!(arena.get(token, 7, LeaseDirection::Write), Ok(&[1, 2, 3]));
        assert_eq!(arena.take(token, 7, LeaseDirection::Write), Ok([1, 2, 3]));
        assert_eq!(arena.active(), 0);
        assert_eq!(
            arena.take(token, 7, LeaseDirection::Write),
            Err(LeaseError::InvalidHandle)
        );
        let next = arena.claim(7, LeaseDirection::Write).unwrap();
        assert_ne!(token, next);
        assert_eq!(
            arena.get(token, 7, LeaseDirection::Write),
            Err(LeaseError::InvalidHandle)
        );
    }

    #[test]
    fn invalid_foreign_and_wrong_direction_tokens_leave_the_owner_intact() {
        let mut arena = LeaseArena::new(1).unwrap();
        let mut foreign = LeaseArena::<u32>::new(1).unwrap();
        let token = arena.claim(9, LeaseDirection::Read).unwrap();
        arena.insert(token, 42).unwrap();
        let other = foreign.claim(9, LeaseDirection::Read).unwrap();
        for invalid in [0, 1, u64::MAX, other] {
            assert_eq!(
                arena.take(invalid, 9, LeaseDirection::Read),
                Err(LeaseError::InvalidHandle)
            );
        }
        assert_eq!(
            arena.take(token, 10, LeaseDirection::Read),
            Err(LeaseError::WrongOwner)
        );
        assert_eq!(
            arena.take(token, 9, LeaseDirection::Write),
            Err(LeaseError::WrongOwner)
        );
        assert_eq!(arena.active(), 1);
        assert_eq!(arena.take(token, 9, LeaseDirection::Read), Ok(42));
    }

    #[test]
    fn budget_and_transition_errors_do_not_change_storage() {
        let mut empty = LeaseArena::<u32>::new(0).unwrap();
        assert_eq!(
            empty.claim(1, LeaseDirection::Read),
            Err(LeaseError::BudgetExhausted)
        );
        let mut arena = LeaseArena::new(1).unwrap();
        let token = arena.claim(1, LeaseDirection::Read).unwrap();
        assert_eq!(
            arena.claim(1, LeaseDirection::Read),
            Err(LeaseError::BudgetExhausted)
        );
        assert_eq!(
            arena.take(token, 1, LeaseDirection::Read),
            Err(LeaseError::InvalidTransition)
        );
        arena.insert(token, 5).unwrap();
        assert_eq!(arena.insert(token, 6), Err(LeaseError::InvalidTransition));
        assert_eq!(arena.cancel(token), Err(LeaseError::InvalidTransition));
        assert_eq!(arena.get(token, 1, LeaseDirection::Read), Ok(&5));
        arena.revoke_all();
        assert_eq!(arena.active(), 0);
        let reserved = arena.claim(1, LeaseDirection::Read).unwrap();
        arena.cancel(reserved).unwrap();
        assert_eq!(arena.cancel(reserved), Err(LeaseError::InvalidHandle));
    }

    #[test]
    fn generation_exhaustion_never_wraps_and_invalid_budget_never_allocates() {
        let counter = AtomicU32::new(u32::MAX - 1);
        assert_eq!(mint_generation(&counter), Ok(u32::MAX - 1));
        assert_eq!(
            mint_generation(&counter),
            Err(LeaseError::GenerationExhausted)
        );
        assert_eq!(counter.load(Ordering::Relaxed), u32::MAX);
        assert!(matches!(
            LeaseArena::<u8>::new(usize::MAX),
            Err(LeaseError::InvalidBudget)
        ));
    }

    #[test]
    fn independent_runs_match_the_content_and_terminal_count_oracle() {
        for _ in 0..2 {
            let mut arena = LeaseArena::new(2).unwrap();
            for value in [12, 34, 56] {
                let token = arena.claim(3, LeaseDirection::Read).unwrap();
                arena.insert(token, value).unwrap();
                assert_eq!(arena.take(token, 3, LeaseDirection::Read), Ok(value));
            }
            assert_eq!(arena.active(), 0);
        }
    }
}
