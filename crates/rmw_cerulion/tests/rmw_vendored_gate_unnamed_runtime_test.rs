//! The vendored build's C++ arm reads `ROS_DISTRO` ONCE per process (no
//! allocation on the per-message resolve path), so the wired verdict for a
//! given environment is pinned in a process of its own: this binary sets the
//! environment BEFORE the first call, and its single test is the whole
//! process.
#![cfg(cerulion_rmw_vendored_bindings)]

use rmw_cerulion::era::{cpp_bridge_gate_for, CppBridgeGate, CppBypassMode};

#[test]
fn a_vendored_build_refuses_the_cpp_arm_under_an_unnamed_runtime() {
    // SAFETY: single-threaded at this point (the one test in this binary),
    // before any read of the variable.
    unsafe { std::env::remove_var("ROS_DISTRO") };
    assert_eq!(
        cpp_bridge_gate_for(CppBypassMode::Off),
        CppBridgeGate::RefuseVendoredUnnamedRuntime,
        "no runtime name: the vendored build must refuse the C++ arm"
    );
    // The bypass the crate's own tests ride keeps admitting.
    assert_eq!(
        cpp_bridge_gate_for(CppBypassMode::TestSilent),
        CppBridgeGate::Supported
    );
    // The admission is a process-lifetime snapshot: naming the distro now
    // changes nothing for this process.
    unsafe { std::env::set_var("ROS_DISTRO", "lyrical") };
    assert_eq!(
        cpp_bridge_gate_for(CppBypassMode::Off),
        CppBridgeGate::RefuseVendoredUnnamedRuntime
    );
}
