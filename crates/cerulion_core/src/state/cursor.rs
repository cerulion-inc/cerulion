//! The restore-side reader over a captured state blob.

use super::StateError;

/// A bounds-checked forward cursor over one node's captured state bytes.
///
/// Every read is checked and every failure is a named [`StateError`]; the
/// cursor never yields fabricated bytes and never silently short-reads, so a
/// truncated or corrupt blob is a terminal restore failure rather than a
/// value that decodes "successfully" into nonsense.
#[derive(Debug, Clone)]
pub struct StateCursor<'a> {
    buf: &'a [u8],
    pos: usize,
    element_budget: usize,
}

impl<'a> StateCursor<'a> {
    /// A cursor positioned at the start of `buf`.
    ///
    /// Seeds the decode-wide element budget at `buf.len()` — see
    /// [`Self::element_budget`].
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            element_budget: buf.len(),
        }
    }

    /// Container elements this decode may still iterate over.
    ///
    /// # The invariant, and why a per-count check cannot provide it
    ///
    /// Every container count is drawn from this one budget, so **the total
    /// number of elements a decode iterates over is bounded by the blob's
    /// length, decided before any of them run**. That is the property that
    /// makes decode work linear in its input.
    ///
    /// Checking each count against [`Self::remaining`] instead does NOT give
    /// it, because the same tail bytes justify many counts. `Vec<Vec<()>>`
    /// places one four-byte inner count per element, and each inner count is
    /// justified against the counts that follow it — MEASURED on an 8 004-byte
    /// blob: the counts claim **7 998 000 elements, 999x the blob**, and the
    /// shape is quadratic, so a 1 MiB blob claims ~1.4e11. A shared budget
    /// makes the same blob linear because the outer count's draw leaves the
    /// inner ones nothing to spend.
    ///
    /// A CLONE copies the budget rather than sharing it, so cloning can double
    /// a decode's work but never unbound it.
    pub fn element_budget(&self) -> usize {
        self.element_budget
    }

    /// Draw `n` elements from the budget, saturating at zero.
    ///
    /// Callers must check [`Self::element_budget`] first; this only accounts.
    pub fn charge_elements(&mut self, n: usize) {
        self.element_budget = self.element_budget.saturating_sub(n);
    }

    /// Read a `u32` LE **payload length** — bytes the caller consumes at once.
    ///
    /// Charges **nothing** against the element budget, and that is the whole
    /// distinction from [`Self::read_element_count`]: a payload length is
    /// bounded by the [`Self::take`] that immediately follows it, which cannot
    /// yield more bytes than the blob has. Charging it too was a real
    /// false-refusal bug — a `(String, Vec<()>)` capture of 108 bytes had its
    /// 100 payload bytes billed as iterations, leaving budget 8 for a count of
    /// 9, so the module could not restore its own output.
    ///
    /// A caller that reads a length here and does NOT immediately consume it
    /// gets no bound from anything; use [`Self::read_element_count`] for
    /// anything that drives a loop.
    pub fn read_payload_len(&mut self) -> Result<usize, StateError> {
        Ok(u32::from_le_bytes(self.take_array::<4>()?) as usize)
    }

    /// Read a `u32` LE **iteration count** and charge it to the element budget.
    ///
    /// The budgeted primitive every container decoder uses, and the one a
    /// hand-written [`CerulionState::cer_read`](super::CerulionState::cer_read)
    /// MUST use for anything blob-driven that it loops over — see that
    /// method's contract.
    pub fn read_element_count(&mut self) -> Result<usize, StateError> {
        let declared = self.read_payload_len()?;
        let budget = self.element_budget;
        if declared > budget {
            return Err(StateError::CountUnjustified { declared, budget });
        }
        self.charge_elements(declared);
        Ok(declared)
    }

    /// Bytes not yet consumed.
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Bytes consumed so far.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Whether the blob is fully consumed.
    pub fn is_empty(&self) -> bool {
        self.remaining() == 0
    }

    /// Consume and return exactly `n` bytes.
    pub fn take(&mut self, n: usize) -> Result<&'a [u8], StateError> {
        let remaining = self.remaining();
        if n > remaining {
            return Err(StateError::Truncated {
                needed: n,
                remaining,
            });
        }
        let out = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(out)
    }

    /// Consume and return exactly `N` bytes as an array.
    pub fn take_array<const N: usize>(&mut self) -> Result<[u8; N], StateError> {
        let bytes = self.take(N)?;
        let mut out = [0u8; N];
        out.copy_from_slice(bytes);
        Ok(out)
    }

    /// Assert the blob is fully consumed.
    ///
    /// Trailing bytes mean the decoder's field list disagrees with the
    /// encoder's — a real drift signal, so it is an error rather than
    /// something to ignore.
    pub fn finish(self) -> Result<(), StateError> {
        let remaining = self.remaining();
        if remaining == 0 {
            Ok(())
        } else {
            Err(StateError::TrailingBytes { remaining })
        }
    }
}
