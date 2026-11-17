// SPDX-License-Identifier: AGPL-3.0-only
//! Which builds carry the period-learned park in `rmw_wait`. Shared with `build.rs`, which
//! emits the `cerulion_period_park` cfg from [`period_park_compiled`], and with the crate's
//! tests, which pin the rule with hand-written vectors (`#[cfg(test)]` strips them from the
//! build script's copy).

/// PURE: does a build for `target_os` and `target_arch` carry the period-learned park? The
/// names are Cargo's `CARGO_CFG_TARGET_OS` and `CARGO_CFG_TARGET_ARCH` values, which
/// `std::env::consts::OS` and `std::env::consts::ARCH` spell the same way. Every Linux aarch64
/// build carries it, where the park waits in `WFE`. Any other build carries it only with the
/// `period-park` Cargo feature (`feature`). The park still runs only where a ring wakes it and
/// the CPU has a monitor-wait primitive; `rmw_wait` checks both at run time.
pub fn period_park_compiled(feature: bool, target_os: &str, target_arch: &str) -> bool {
    feature || (target_os == "linux" && target_arch == "aarch64")
}

#[cfg(test)]
mod tests {
    use super::period_park_compiled;

    #[test]
    fn linux_aarch64_carries_the_park_with_or_without_the_feature() {
        assert!(period_park_compiled(false, "linux", "aarch64"));
        assert!(period_park_compiled(true, "linux", "aarch64"));
    }

    #[test]
    fn every_other_target_carries_it_only_with_the_feature() {
        for (os, arch) in [
            ("linux", "x86_64"),
            ("linux", "arm"),
            ("linux", "riscv64"),
            ("android", "aarch64"),
            ("macos", "aarch64"),
            ("macos", "x86_64"),
            ("windows", "aarch64"),
        ] {
            assert!(
                !period_park_compiled(false, os, arch),
                "{os} {arch} without the feature"
            );
            assert!(
                period_park_compiled(true, os, arch),
                "{os} {arch} with the feature"
            );
        }
    }

    /// The cfg `build.rs` emitted for THIS build is the rule applied to this build's target
    /// and features: a build script that stopped emitting it, or emitted it everywhere,
    /// fails here on the target it gets wrong.
    #[test]
    fn the_emitted_cfg_follows_the_rule_for_this_build() {
        assert_eq!(
            cfg!(cerulion_period_park),
            period_park_compiled(
                cfg!(feature = "period-park"),
                std::env::consts::OS,
                std::env::consts::ARCH
            ),
            "target {} {}",
            std::env::consts::OS,
            std::env::consts::ARCH
        );
    }
}
