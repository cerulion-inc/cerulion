// SPDX-License-Identifier: AGPL-3.0-only
//! `ros.distro` and `ros.rmw`: the ROS 2 environment this shell sourced, and
//! whether `cerulion ros2 run` finds the Cerulion rmw library.
//!
//! `ros.distro` reads `ROS_DISTRO`, `ROS_VERSION` and `AMENT_PREFIX_PATH`; a
//! shell with `ROS_VERSION=1` sourced ROS 1 and counts as no ROS 2 shell
//! ([`ros2_distro`]). `ros.rmw`
//! resolves the library directory the way `cerulion ros2 run` does
//! ([`crate::ros2_cmd::resolve_lib_dir`]: `CERULION_LIB_DIR`, else the
//! directory of the running `cerulion`) and looks for
//! [`crate::ros2_cmd::RMW_LIB_FILENAME`] there; it reads `RMW_IMPLEMENTATION`,
//! which `ros2 run` overrides for the child it starts.

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const DISTRO: Check = Check {
    id: "ros.distro",
    why: "`cerulion ros2 run`, `ros2 launch` and `ros2 attach` run against the ROS 2 \
          distribution this shell sourced.",
    network: false,
    run: run_distro,
};

pub(super) const RMW: Check = Check {
    id: "ros.rmw",
    why: "`cerulion ros2 run` and `ros2 launch` load librmw_cerulion.so and exit 69 when it is \
          not where they look.",
    network: false,
    run: run_rmw,
};

/// The ROS 2 distribution a shell sourced: `ROS_DISTRO` when it is set and
/// `ROS_VERSION` is not `1`. A ROS 1 setup script also sets `ROS_DISTRO`, with
/// `ROS_VERSION=1`.
pub(super) fn ros2_distro<'a>(distro: Option<&'a str>, version: Option<&str>) -> Option<&'a str> {
    let distro = distro.map(str::trim).filter(|d| !d.is_empty())?;
    (version.map(str::trim) != Some("1")).then_some(distro)
}

pub(super) fn decide_distro(distro: Option<&str>, prefixes: usize) -> Outcome {
    match distro.map(str::trim).filter(|d| !d.is_empty()) {
        None => Outcome::new(
            Status::Skip,
            "not run: no ROS 2 environment is sourced in this shell (ROS_DISTRO is not set, or \
             ROS_VERSION is 1)",
        ),
        Some(d) => Outcome::new(
            Status::Info,
            format!("ROS 2 {d} is sourced ({prefixes} ament prefixes)"),
        )
        .reading("ros_distro", Reading::Text(d.to_string()))
        .reading(
            "ament_prefixes",
            Reading::Int(u64::try_from(prefixes).unwrap_or(u64::MAX)),
        ),
    }
}

/// `lib` is the resolved rmw library path (or the resolution error),
/// `present` whether it exists, `ros_sourced` whether [`ros2_distro`] finds one,
/// `rmw_env` the raw `RMW_IMPLEMENTATION`. A problem fails only when ROS 2 is
/// sourced, the shell `cerulion ros2 run` would be started from; otherwise it
/// warns or informs. On macOS the check does not apply: ROS 2 Jazzy publishes
/// no macOS binaries.
pub(super) fn decide_rmw(
    host_os: &str,
    lib: Result<&std::path::Path, &str>,
    present: bool,
    ros_sourced: bool,
    rmw_env: Option<&str>,
) -> Outcome {
    if host_os == "macos" {
        return Outcome::not_applicable(
            "ROS 2 Jazzy publishes no macOS binaries, so `cerulion ros2 run` needs a Linux \
             machine",
        );
    }
    let lib = match lib {
        Ok(lib) => lib,
        Err(err) => {
            return Outcome::new(
                if ros_sourced {
                    Status::Fail
                } else {
                    Status::Warn
                },
                format!("cannot resolve the library directory: {err}"),
            )
            .with_fix(format!(
                "set {} to the directory that holds {}",
                crate::ros2_cmd::LIB_DIR_ENV,
                crate::ros2_cmd::RMW_LIB_FILENAME
            ));
        }
    };
    let other_rmw = rmw_env
        .map(str::trim)
        .filter(|v| !v.is_empty() && *v != "rmw_cerulion");
    let note = other_rmw.map_or(String::new(), |v| {
        format!("; RMW_IMPLEMENTATION={v} here, which `cerulion ros2 run` overrides for its child")
    });
    let base = |status, summary: String| {
        let mut o = Outcome::new(status, summary)
            .reading("rmw_library", Reading::Text(lib.display().to_string()))
            .reading("present", Reading::Bool(present));
        if let Some(v) = rmw_env {
            o = o.reading("rmw_implementation", Reading::Text(v.to_string()));
        }
        o
    };
    match (present, ros_sourced) {
        (true, _) => base(Status::Pass, format!("{} is present{note}", lib.display())),
        (false, true) => base(Status::Fail, format!("{} is missing{note}", lib.display()))
            .with_fix(crate::ros2_cmd::missing_rmw_lib_message(host_os, lib)),
        (false, false) => base(
            Status::Info,
            format!(
                "{} is not installed; only `cerulion ros2 run` and `ros2 launch` need it{note}",
                lib.display()
            ),
        ),
    }
}

fn run_distro(_ctx: &Context) -> Outcome {
    let distro = std::env::var("ROS_DISTRO").ok();
    let version = std::env::var("ROS_VERSION").ok();
    let distro = ros2_distro(distro.as_deref(), version.as_deref());
    let prefixes = std::env::var_os("AMENT_PREFIX_PATH").map_or(0, |p| {
        std::env::split_paths(&p)
            .filter(|p| !p.as_os_str().is_empty())
            .count()
    });
    decide_distro(distro, prefixes)
}

fn run_rmw(_ctx: &Context) -> Outcome {
    let resolved = crate::ros2_cmd::resolve_lib_dir()
        .map(|dir| dir.join(crate::ros2_cmd::RMW_LIB_FILENAME))
        .map_err(|e| e.to_string());
    // `exists`, as `cerulion ros2 run` tests the path.
    let present = resolved.as_ref().is_ok_and(|p| p.exists());
    let ros_sourced = ros2_distro(
        std::env::var("ROS_DISTRO").ok().as_deref(),
        std::env::var("ROS_VERSION").ok().as_deref(),
    )
    .is_some();
    let rmw_env = std::env::var("RMW_IMPLEMENTATION").ok();
    decide_rmw(
        std::env::consts::OS,
        resolved.as_deref().map_err(String::as_str),
        present,
        ros_sourced,
        rmw_env.as_deref(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ros_1_shell_is_not_a_ros_2_one() {
        assert_eq!(ros2_distro(Some("noetic"), Some("1")), None);
        assert_eq!(ros2_distro(Some("jazzy"), Some("2")), Some("jazzy"));
        assert_eq!(ros2_distro(Some(" humble\n"), None), Some("humble"));
        assert_eq!(ros2_distro(Some(" "), Some("2")), None);
        assert_eq!(ros2_distro(None, Some("2")), None);
    }
    use std::path::Path;

    const LIB: &str = "/opt/cerulion/lib/librmw_cerulion.so";

    #[test]
    fn no_ros_environment_is_a_skip() {
        for distro in [None, Some(""), Some("  ")] {
            let o = decide_distro(distro, 0);
            assert_eq!(o.status, Status::Skip);
            assert_eq!(
                o.summary,
                "not run: no ROS 2 environment is sourced in this shell (ROS_DISTRO is not set, or \
                 ROS_VERSION is 1)"
            );
        }
    }

    #[test]
    fn a_sourced_distro_is_info_with_its_prefixes() {
        let o = decide_distro(Some("jazzy"), 3);
        assert_eq!(o.status, Status::Info);
        assert_eq!(o.summary, "ROS 2 jazzy is sourced (3 ament prefixes)");
        assert_eq!(o.readings.get("ament_prefixes"), Some(&Reading::Int(3)));
    }

    #[test]
    fn a_present_library_passes_and_notes_another_rmw() {
        let o = decide_rmw(
            "linux",
            Ok(Path::new(LIB)),
            true,
            true,
            Some("rmw_fastrtps_cpp"),
        );
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.summary,
            "/opt/cerulion/lib/librmw_cerulion.so is present; RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
             here, which `cerulion ros2 run` overrides for its child"
        );
        let plain = decide_rmw(
            "linux",
            Ok(Path::new(LIB)),
            true,
            true,
            Some("rmw_cerulion"),
        );
        assert_eq!(
            plain.summary,
            "/opt/cerulion/lib/librmw_cerulion.so is present"
        );
    }

    #[test]
    fn a_missing_library_fails_only_when_ros_is_sourced() {
        let fail = decide_rmw("linux", Ok(Path::new(LIB)), false, true, None);
        assert_eq!(fail.status, Status::Fail);
        assert_eq!(
            fail.fix.as_deref(),
            Some(crate::ros2_cmd::missing_rmw_lib_message("linux", Path::new(LIB)).as_str())
        );
        let info = decide_rmw("linux", Ok(Path::new(LIB)), false, false, None);
        assert_eq!(info.status, Status::Info);
        assert_eq!(info.fix, None);
    }

    #[test]
    fn an_unresolvable_library_directory_fails_naming_the_variable() {
        let o = decide_rmw(
            "linux",
            Err("CERULION_LIB_DIR is set but empty"),
            false,
            true,
            None,
        );
        assert_eq!(o.status, Status::Fail);
        assert!(o.fix.unwrap().starts_with("set CERULION_LIB_DIR"));
        let unsourced = decide_rmw(
            "linux",
            Err("CERULION_LIB_DIR is set but empty"),
            false,
            false,
            None,
        );
        assert_eq!(unsourced.status, Status::Warn);
    }

    #[test]
    fn macos_is_not_applicable_whatever_the_reading() {
        let o = decide_rmw("macos", Ok(Path::new(LIB)), false, true, None);
        assert_eq!(o.status, Status::Skip);
        assert!(o
            .summary
            .starts_with("not applicable: ROS 2 Jazzy publishes no macOS binaries"));
    }
}
