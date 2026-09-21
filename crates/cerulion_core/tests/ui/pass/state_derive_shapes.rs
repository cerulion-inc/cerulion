// The happy shapes must COMPILE.
//
// A pass fixture is the anti-tautology half of the compile-fail group: without
// it, a derive that refused everything would satisfy every `compile_fail` case
// in the suite.

use std::collections::HashMap;
use std::sync::atomic::AtomicU64;
use std::sync::{Arc, Mutex};

use cerulion_core::state::CerulionState;

// An ORDINARY struct — zero attributes.
#[derive(CerulionState)]
struct Pose {
    x: f64,
    y: f64,
}

// `Arc<Mutex<T>>` is captured, not refused.
#[derive(CerulionState)]
struct Costmap {
    grid: Arc<Mutex<Vec<u8>>>,
    revision: u64,
}

// Counters, by value: no escape needed.
#[derive(CerulionState)]
struct Stats {
    ticks: AtomicU64,
    drops: Arc<AtomicU64>,
}

// A handle the RESOURCE INVENTORY recognises, and a trait object recognised
// structurally: zero tags on either.
struct Device;
#[derive(CerulionState)]
struct Driver {
    frames: u32,
    socket: Option<std::net::TcpStream>,
    hook: Option<Box<dyn Fn() + Send>>,
    #[cerulion(reconstruct)]
    device: Device,
}

// A key with no total order, through the escape.
#[derive(PartialEq, Eq, Hash, CerulionState)]
struct FloatCell {
    bits: u64,
}
#[derive(CerulionState)]
struct Grid {
    #[cerulion(unordered)]
    cells: HashMap<FloatCell, u8>,
}

// Enums: unit, named and tuple variants in one type.
#[derive(CerulionState)]
enum PadEvent {
    Disconnected,
    Button { id: u8, down: bool },
    Axis(u8, f32),
}

// A generic node type: the bound is added for the user.
#[derive(CerulionState)]
struct Holder<T> {
    value: T,
    count: u32,
}

// A unit struct captures nothing and is still a well-formed state type.
#[derive(CerulionState)]
struct Marker;

fn main() {
    fn assert_state<T: CerulionState>() {}
    assert_state::<Pose>();
    assert_state::<Costmap>();
    assert_state::<Stats>();
    assert_state::<Driver>();
    assert_state::<Grid>();
    assert_state::<PadEvent>();
    assert_state::<Holder<u64>>();
    assert_state::<Marker>();
}
