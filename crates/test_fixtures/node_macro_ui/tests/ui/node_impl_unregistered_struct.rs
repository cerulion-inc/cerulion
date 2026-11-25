use cerulion_core::prelude::*;

/// Macro diagnostic (R-family / impl-block contract): `#[cerulion_node_impl]`
/// must be applied to an impl block whose `Self` type carries a sibling
/// `#[cerulion_node]` attribute (registered before this impl in source order).
/// An impl over an un-annotated struct must be rejected with the "could not
/// find a registered #[cerulion_node]" diagnostic — not a confusing
/// post-expansion cascade.
struct PlainStruct {
    seen: f64,
}

#[cerulion_node_impl]
impl PlainStruct {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen += 1.0;
        Ok(())
    }
}

fn main() {}
