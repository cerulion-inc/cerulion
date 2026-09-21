// SPDX-License-Identifier: AGPL-3.0-only
//! The driver's clamp limits DUPLICATE the joystick crate's canonical teleop
//! saturation constants (so this crate carries no gilrs edge). This pins the
//! two in lockstep: a change to one side without the other fails here.

#[test]
fn driver_clamp_limits_match_the_canonical_teleop_limits() {
    assert_eq!(sport_driver::MAX_VX, joystick_teleop::MAX_VX);
    assert_eq!(sport_driver::MAX_VY, joystick_teleop::MAX_VY);
    assert_eq!(sport_driver::MAX_VYAW, joystick_teleop::MAX_VYAW);
}
