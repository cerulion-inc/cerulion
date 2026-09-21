//! A custom non-port user field whose type does NOT
//! implement `Default` must produce a clear, locatable error. The
//! macro-synthesised `Default` impl says
//! `field: <FieldType as ::std::default::Default>::default()`, so the
//! diagnostic should point at that field's type — not at the macro itself.

use cerulion_core::prelude::*;
use cerulion_core::state::CerulionState;
use native_ros2_messages::geometry_msgs::Vector3;

/// A type intentionally lacking a `Default` impl.
///
/// It DOES derive `CerulionState`, and that is load-bearing rather
/// than incidental: every node field carries a
/// `CerulionState` obligation too, so without the derive this fixture reports
/// the state-capture error INSTEAD OF (and, because the runtime seams are wired,
/// several times around) the `Default` one it was written to pin. Deriving it
/// isolates the fixture back to its single subject — a field whose type lacks
/// `Default` — which is what a compile-fail fixture is for.
#[derive(CerulionState)]
pub struct NoDefault {
    _x: u32,
}

#[cerulion_node(period_ms = 100)]
struct FieldWithoutDefault {
    #[output]
    out: Vector3,
    /// This field's type doesn't implement Default; the macro-synthesised
    /// Default impl must fail to compile, with a diagnostic pointing
    /// somewhere actionable (ideally at the field, not the macro).
    bad: NoDefault,
}

#[cerulion_node_impl]
impl FieldWithoutDefault {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.bad._x as f64;
        Ok(())
    }
}

fn main() {}
