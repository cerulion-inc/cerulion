// The FOLD-IN's diagnostic. A node that declares
// nothing still gets the field-precise refusal — this is the error the whole
// migration story rests on ("94% zero-edit; the rest FAIL TO COMPILE, at the
// field, naming the fix"), so it is pinned rather than described.
//
// The message is `CerulionState`'s own `#[diagnostic::on_unimplemented]`
// (`cerulion_core/src/state.rs`); the fold-in only points rustc at the field's
// span. This fixture pins the RENDERING, which is toolchain-fragile — hence
// the `#[ignore]`d group, the same class as its `#[derive(CerulionState)]`
// sibling next door.
//
// TWO things are pinned that the derive's sibling cannot show:
//
// 1. The offending field is `cuda`, and the diagnostic points AT IT — not at
//    `#[cerulion_node]`, and not at the ports. A node carries port fields
//    whose declared types have no `CerulionState` impl and never will, so an
//    emission that walked the whole struct would name `scan`/`cmd` here (or
//    the hidden `__cer_rt`) and bury the real cause. A prototype of that emission
//    measured THREE phantom lossy fields on a node whose real
//    state is fully capturable.
// 2. The count is ONE. Every port and the injected runtime field are excluded
//    from the walk, so exactly one block renders.

use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;

struct CudaContext;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SlamNode {
    #[input]
    scan: Vector3,
    #[output]
    cmd: Vector3,
    pose: f64,
    cuda: CudaContext,
}

#[cerulion_node_impl]
impl SlamNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.pose;
        Ok(())
    }
}

impl Default for CudaContext {
    fn default() -> Self {
        Self
    }
}

fn main() {}
