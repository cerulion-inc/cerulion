//! Follow-up: prove that `MaxSliceLen::const_new(31)`
//! (one below the `WireHeader::SIZE` floor) fails at const-eval time.
//!
//! Mirror of `max_slice_len_const_new_zero.rs` but pinning a value
//! INSIDE the rejection band rather than at zero. The newtype
//! documents `n >= WireHeader::SIZE` (32); this test pins the band
//! `[1, 31]` as rejected. Note: this test does NOT catch a tightening
//! from `>= 32` to `> 32` (because 31 fails under BOTH formulations);
//! the inclusive `== 32` boundary is pinned positively by
//! `try_new_accepts_exactly_wire_header_size` and
//! `const_new_accepts_at_floor` in `max_slice_len_newtype_test.rs`.

use cerulion_core::wire::MaxSliceLen;

const _: MaxSliceLen = MaxSliceLen::const_new(31);

fn main() {}
