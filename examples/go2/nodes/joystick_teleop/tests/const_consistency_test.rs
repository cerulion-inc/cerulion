// SPDX-License-Identifier: AGPL-3.0-only
//! Cross-crate saturation-constant consistency pin.
//!
//! The joystick node's saturation constants (`MAX_VX`/`MAX_VY`/`MAX_VYAW`) are
//! CANONICAL; the keyboard node duplicates them. We deliberately do
//! NOT factor them into a shared crate under `examples/go2/nodes/*` — that path's
//! `members = ["nodes/*"]` glob would treat a constants crate as a node crate.
//! Instead the joystick crate dev-depends on the keyboard crate by path and
//! this test fails the moment the two ever drift apart. (Direction chosen so
//! the portable Mac keyboard crate keeps a gilrs-free production build; the
//! joystick crate already pulls gilrs, so it absorbs the dev-dep.)

#[test]
fn joystick_and_keyboard_saturation_constants_match() {
    assert_eq!(
        joystick_teleop::MAX_VX,
        keyboard_teleop::MAX_VX,
        "MAX_VX drifted between the joystick and keyboard teleop nodes"
    );
    assert_eq!(
        joystick_teleop::MAX_VY,
        keyboard_teleop::MAX_VY,
        "MAX_VY drifted between the joystick and keyboard teleop nodes"
    );
    assert_eq!(
        joystick_teleop::MAX_VYAW,
        keyboard_teleop::MAX_VYAW,
        "MAX_VYAW drifted between the joystick and keyboard teleop nodes"
    );
}
