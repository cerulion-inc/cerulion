//! The vendored build's C++ arm reads `ROS_DISTRO` ONCE per process (no
//! allocation on the per-message resolve path), so the wired verdict for a
//! given environment is pinned in a process of its own: this binary sets the
//! environment BEFORE the first call, and its single test is the whole
//! process.
#![cfg(cerulion_rmw_vendored_bindings)]

use rmw_cerulion::era::{cpp_bridge_gate_for, CppBridgeGate, CppBypassMode};

#[test]
fn a_vendored_build_admits_the_cpp_arm_when_the_runtime_names_lyrical() {
    // SAFETY: single-threaded at this point (the one test in this binary),
    // before any read of the variable.
    unsafe { std::env::set_var("ROS_DISTRO", "lyrical") };
    assert_eq!(
        cpp_bridge_gate_for(CppBypassMode::Off),
        CppBridgeGate::Supported,
        "a runtime the snapshot admits: the vendored build resolves the C++ arm"
    );
}
