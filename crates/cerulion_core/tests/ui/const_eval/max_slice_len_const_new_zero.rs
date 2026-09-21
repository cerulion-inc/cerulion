//! Follow-up: prove that `MaxSliceLen::const_new(0)`
//! fails at const-eval time (compile-time), not at runtime.
//!
//! This is the load-bearing CI enforcement for the "compile-time
//! prevention" claim in the PR description — without this trybuild
//! test, a future refactor of `const_new` to silently fall back to
//! a runtime panic or `unwrap_or_default` would not be caught.

use cerulion_core::wire::MaxSliceLen;

const _: MaxSliceLen = MaxSliceLen::const_new(0);

fn main() {}
