// The SECOND half, and the sibling of
// `node_escape_on_a_failing_node_does_not_cascade.rs`: a node that FAILS
// VALIDATION and carries a redundant `#[derive(CerulionState)]` must report the
// real problem and NOT a second, WRONG story about its ports.
//
// `#[cerulion_node]`'s two error arms re-emit the user's struct so a validation
// failure does not also produce "cannot find type" everywhere the node is
// named. If that struct goes back out with the derive INTACT,
// the derive expands over the `#[input]`/`#[output]` port fields.
//
// With the derive left intact, exactly this fixture renders:
//
//     error[E0277]: `Vector3` cannot be part of a Cerulion node's state
//        = note: if this is real state, make it capturable:
//                add `#[derive(CerulionState)]` to `Vector3`
//
// — nineteen lines of E0277 whose HELP TEXT is actively wrong (`Vector3` is a
// zero-sized SHM marker; deriving state on it is not a thing anyone should do),
// stacked on top of the one diagnostic the user actually needed. The success
// path strips the derive; the error arms must strip it too, and this fixture
// pins that they do.
//
// The strip is SILENT here rather than a second `compile_error!`. This path
// exists to surface the REAL error; the redundant derive is not lost, it is
// reported in full, naming the fix, on the next compile, once the
// validation failure is gone and the success path runs
// (`state_derive_on_node_struct.rs` pins that message).
//
// The `input` cascade below is a separate, known wart, deliberately left alone, for
// the reason recorded in the sibling fixture.
//
// OUR `compile_error!` ⇒ the BLOCKING group. Validation fails before codegen,
// so no `#[cfg(feature = "cdylib")]` block is emitted and there is no
// `unexpected_cfgs` warning to allow.

use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::geometry_msgs::Vector3;

// `#[input(trigger)]` alongside `period_ms` is a validation failure: a
// data-driven trigger cannot coexist with a time-driven policy.
#[cerulion_node(period_ms = 10)]
#[derive(Default, CerulionState)]
struct FailingNode {
    #[input(trigger)]
    scan: Vector3,
    frames: u64,
}

fn main() {}
