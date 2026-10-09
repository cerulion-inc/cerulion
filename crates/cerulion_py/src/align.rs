// SPDX-License-Identifier: AGPL-3.0-only
//! Aligned access to a received frame, shared by the extension module and
//! the interop fixture (which includes this file by path: the fixture is a
//! plain binary, so `cargo test -p cerulion_py_fixtures` runs the tests
//! below, which a `cdylib` built as a Python extension module cannot link).

/// Widest primitive alignment a typed frame field can require (`f64`/`u64`).
pub const MAX_FIELD_ALIGN: usize = 8;

/// `bytes` itself when it is `MAX_FIELD_ALIGN`-aligned, else a copy of it in
/// an aligned region of `scratch`. SHM slots are not guaranteed 8-byte
/// aligned, and `FrameView` refuses a primitive array at a misaligned
/// address. Resolution only reads offsets relative to the payload start, so
/// they index the original frame unchanged and the NumPy views stay
/// zero-copy over the slot (NumPy reads unaligned arrays).
pub fn aligned_for_validation<'a>(bytes: &'a [u8], scratch: &'a mut Vec<u8>) -> &'a [u8] {
    if (bytes.as_ptr() as usize).is_multiple_of(MAX_FIELD_ALIGN) {
        return bytes;
    }
    scratch.clear();
    scratch.resize(bytes.len() + MAX_FIELD_ALIGN, 0);
    let start = scratch.as_ptr().align_offset(MAX_FIELD_ALIGN);
    scratch[start..start + bytes.len()].copy_from_slice(bytes);
    &scratch[start..start + bytes.len()]
}

#[cfg(test)]
mod tests {
    use super::{aligned_for_validation, MAX_FIELD_ALIGN};

    #[repr(align(8))]
    struct Backing([u8; 32]);

    #[test]
    fn a_misaligned_frame_is_validated_from_an_aligned_copy() {
        let backing = Backing(core::array::from_fn(|i| i as u8));
        let misaligned = &backing.0[1..17];
        let mut scratch = Vec::new();
        let aligned = aligned_for_validation(misaligned, &mut scratch);
        assert_eq!(
            aligned,
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        );
        assert!((aligned.as_ptr() as usize).is_multiple_of(MAX_FIELD_ALIGN));
    }

    #[test]
    fn an_aligned_frame_is_used_in_place() {
        let backing = Backing([7; 32]);
        let mut scratch = Vec::new();
        let used = aligned_for_validation(&backing.0[8..24], &mut scratch);
        assert_eq!(used.as_ptr(), backing.0[8..24].as_ptr());
        assert!(scratch.is_empty());
    }
}
