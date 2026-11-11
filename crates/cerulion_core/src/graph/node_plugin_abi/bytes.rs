// SPDX-License-Identifier: AGPL-3.0-only
//! Checked byte spans shared by the node-local boundary adapters.

/// Invalid caller-owned or host-leased span.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum SpanError {
    Null,
    Length,
    Alignment,
}

/// Check the integer bounds before constructing a Rust slice.
fn validate(ptr: *const u8, len: u64, alignment: usize) -> Result<usize, SpanError> {
    let len = usize::try_from(len).map_err(|_| SpanError::Length)?;
    if len > isize::MAX as usize || ptr.addr().checked_add(len).is_none() {
        return Err(SpanError::Length);
    }
    if len != 0 && ptr.is_null() {
        return Err(SpanError::Null);
    }
    if !alignment.is_power_of_two() || (!ptr.is_null() && !ptr.addr().is_multiple_of(alignment)) {
        return Err(SpanError::Alignment);
    }
    Ok(len)
}

/// Borrow a checked immutable span for the owner's call or lease lifetime.
///
/// # Safety
/// `ptr` covers `len` initialized bytes in one allocation that the caller owns
/// or holds through a live read lease. The bytes remain immutable and allocated
/// for `'a`; validation cannot authenticate arbitrary native pointers.
pub(super) unsafe fn read<'a>(
    ptr: *const u8,
    len: u64,
    alignment: usize,
) -> Result<&'a [u8], SpanError> {
    let len = validate(ptr, len, alignment)?;
    if len == 0 {
        return Ok(&[]);
    }
    // SAFETY (U08/U12): the owner's contract provides the allocation and
    // immutable lifetime; validate proves non-null, alignment and slice bounds.
    Ok(unsafe { std::slice::from_raw_parts(ptr, len) })
}

/// Borrow a checked exclusive span for the owner's call or write-lease lifetime.
///
/// # Safety
/// `ptr` covers `len` initialized bytes in one live allocation, with exclusive
/// access for `'a`. No reference or schema writer may survive the lease's finish
/// or discard. The host must initialize bytes before exposing them as `u8`.
pub(super) unsafe fn write<'a>(
    ptr: *mut u8,
    len: u64,
    alignment: usize,
) -> Result<&'a mut [u8], SpanError> {
    let len = validate(ptr, len, alignment)?;
    if len == 0 {
        return Ok(&mut []);
    }
    // SAFETY (U09/U12): the caller provides exclusive live storage; validate
    // proves non-null, alignment and representable non-wrapping slice bounds.
    Ok(unsafe { std::slice::from_raw_parts_mut(ptr, len) })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_null_span_is_empty_and_never_dereferenced() {
        // SAFETY: empty spans require no allocation or dereference.
        assert_eq!(unsafe { read(std::ptr::null(), 0, 1) }, Ok(&[][..]));
        // SAFETY: no byte is borrowed and no address is dereferenced.
        assert!(unsafe { write(std::ptr::null_mut(), 0, 1) }
            .unwrap()
            .is_empty());
    }

    #[test]
    fn length_null_and_alignment_errors_precede_access() {
        assert_eq!(validate(std::ptr::null(), 1, 1), Err(SpanError::Null));
        assert_eq!(
            validate(std::ptr::null(), u64::MAX, 1),
            Err(SpanError::Length)
        );
        let bytes = [4_u8, 5, 6];
        assert_eq!(validate(bytes.as_ptr(), 3, 3), Err(SpanError::Alignment));
        let addr = bytes.as_ptr().addr();
        let unaligned = if addr.is_multiple_of(8) { 1 } else { 0 };
        assert_eq!(
            validate(bytes[unaligned..].as_ptr(), 1, 8),
            Err(SpanError::Alignment)
        );
        assert_eq!(bytes, [4, 5, 6]);
    }

    #[test]
    fn borrowed_bytes_use_the_owner_storage_and_match_the_literal_oracle() {
        let mut bytes = [1_u8, 2, 3];
        let address = bytes.as_ptr();
        // SAFETY: initialized array is exclusively borrowed for this scope.
        let output = unsafe { write(bytes.as_mut_ptr(), 3, 1) }.unwrap();
        assert_eq!(output.as_ptr(), address);
        output.copy_from_slice(&[4, 5, 6]);
        // SAFETY: the exclusive borrow ended; bytes remain live and immutable.
        let input = unsafe { read(bytes.as_ptr(), 3, 1) }.unwrap();
        assert_eq!(input.as_ptr(), address);
        assert_eq!(input, &[4, 5, 6]);
    }
}
