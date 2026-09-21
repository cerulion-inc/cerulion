// A node that FAILS VALIDATION and
// carries a `#[cerulion(...)]` escape must report the real problem and NOT a
// second error about the escape.
//
// Both of `#[cerulion_node]`'s error arms re-emit the user's struct, so a
// validation failure does not also produce "cannot find type" everywhere the
// node is named. That struct still carries `#[cerulion(...)]` — and nothing
// registers an inert helper attribute for an ATTRIBUTE macro (only a
// `#[proc_macro_derive(.., attributes(cerulion))]` does that, and a node goes
// through no derive), so left in place it reaches rustc as an extra
// `cannot find attribute 'cerulion' in this scope` BELOW the real diagnostic.
//
// Dropping `strip_cerulion_field_attrs` from the two error arms makes this
// fixture render one MORE error block, naming `cerulion` on the
// `#[cerulion(reconstruct)]` line.
//
// The `input`/`output` cascade below is a separate, known wart, deliberately left
// alone — `input_unknown_attribute.stderr` already records it, and
// re-blessing the whole `tests/ui/` set to tidy it
// is the toolchain-drift risk the `#[ignore]`d group exists to avoid. This
// fixture pins only the `cerulion` escape cascade.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

#[derive(Default)]
struct Handle;

// `#[input(trigger)]` alongside `period_ms` is a validation failure: a
// data-driven trigger cannot coexist with a time-driven policy.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct FailingNode {
    #[input(trigger)]
    scan: Vector3,
    #[cerulion(reconstruct)]
    handle: Handle,
}

fn main() {}
