#![allow(unexpected_cfgs)]
//! Pass case: a GENERIC `#[cerulion_node]` struct compiles with
//! the bounds it needed before D6, and its entry is usable as a `NodeEntry`.
//!
//! D6 gave the generated `NodeEntry` impl three state forwards, each calling
//! `CerulionState` on the wrapped struct. The folded state impl is
//! `impl<T> CerulionState for Node<T> where T: CerulionState` — satisfiable at
//! every instantiation, but proving nothing INSIDE `impl<T>`, where `T` is
//! unknown. So the forwards raised an obligation at their call sites that
//! nothing discharged, and a generic node that compiled before D6 stopped
//! compiling with an `E0277` the user had to answer by hand-writing a bound the
//! macro never asked for.
//!
//! The fix states the obligation in the generated impl's `where` clause instead
//! — the same serde-style per-type-parameter rule `#[derive(CerulionState)]`
//! already applies, so the two emissions cannot drift.
//!
//! `T: Send` is written out because it always was: `NodeEntry: Send`, and the
//! macro has never propagated bounds. That is the point of this fixture — D6
//! must not have ADDED a second such obligation.
//!
//! Deleting the `where` propagation in `codegen.rs::gen_zero_copy_node_entry_impl`
//! fails this fixture with `E0277: T cannot be part of a Cerulion node's state`.

use cerulion_core::prelude::*;
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct GenericNode<T: Default + Send> {
    #[output]
    image: Image,
    payload: T,
}

#[cerulion_node_impl]
impl<T: Default + Send> GenericNode<T> {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.height = 1;
        Ok(())
    }
}

/// A node whose parameter is constrained through a `where` clause instead of
/// inline bounds — the user's own clause must survive the propagation rather
/// than be replaced by it.
#[cerulion_node(period_ms = 16)]
#[derive(Default)]
struct WhereClauseNode<T>
where
    T: Default + Send,
{
    #[output]
    image: Image,
    payload: T,
}

#[cerulion_node_impl]
impl<T> WhereClauseNode<T>
where
    T: Default + Send,
{
    fn tick(&mut self) -> Result<(), NodeError> {
        self.image.height = 2;
        Ok(())
    }
}

fn main() {
    // Constructing is not enough: `NodeEntry` is what D6 touched, so the
    // fixture must require the entry to actually IMPLEMENT it at a concrete
    // instantiation.
    fn takes_entry<E: NodeEntry>(_e: E) {}
    takes_entry(GenericNodeEntry::<u32>::new());
    takes_entry(WhereClauseNodeEntry::<u64>::new());
}
