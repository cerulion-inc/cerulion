//! Follow-up: prove that `MaxSliceLen::const_new(39)`
//! (one below the `WireHeader::SIZE` floor) fails at const-eval time.
//!
//! Mirror of `max_slice_len_const_new_zero.rs` but pinning a value
//! INSIDE the rejection band rather than at zero. The newtype
//! documents `n >= WireHeader::SIZE` (40); this test pins the band
//! `[1, 39]` as rejected. Note: this test does NOT catch a tightening
//! from `>= 40` to `> 40` (because 39 fails under BOTH formulations);
//! the inclusive `== 40` boundary is pinned positively by
//! `try_new_accepts_exactly_wire_header_size` and
//! `const_new_accepts_at_floor` in `max_slice_len_newtype_test.rs`.

use cerulion_core::wire::MaxSliceLen;

const _: MaxSliceLen = MaxSliceLen::const_new(39);

fn main() {}
