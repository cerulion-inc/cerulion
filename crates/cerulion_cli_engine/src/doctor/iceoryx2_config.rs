// SPDX-License-Identifier: AGPL-3.0-only
//! `iceoryx2.cwd-config`: a `config/iceoryx2.toml` in the current directory.
//!
//! iceoryx2 loads the first config file of three that exists: the project
//! file `config/iceoryx2.toml` relative to the process's current directory,
//! the user file `~/.config/iceoryx2/iceoryx2.toml`, then the system file
//! `/etc/iceoryx2/iceoryx2.toml`, and uses its built-in defaults when none
//! does (`Config::global_config` in iceoryx2). A file that exists but cannot be
//! read is skipped. The `[global]` keys `root-path` and `prefix` name the
//! directory and file prefix every shared-memory topic lives under, so two
//! processes that load different values do not see each other's topics.
//!
//! This check reads the project file of the current directory and the file
//! (or the defaults) a process started anywhere else would load, and compares
//! the two keys.

use std::path::{Path, PathBuf};

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const CWD_CONFIG: Check = Check {
    id: "iceoryx2.cwd-config",
    why: "iceoryx2 reads config/iceoryx2.toml from the directory a process starts in before \
          the user and system config files, so a process started here and one started \
          elsewhere can use different shared-memory roots and not see each other's topics.",
    network: false,
    run,
};

/// The two `[global]` keys that decide where topics live.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct Namespace {
    pub(super) root_path: String,
    pub(super) prefix: String,
}

/// One candidate config file as read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ConfigFile {
    /// No file at the path.
    Absent,
    /// The file deserializes into iceoryx2's `Config`, every key it does not
    /// set taking iceoryx2's default.
    Read(Namespace),
    /// The file exists but cannot be read or deserialized, so iceoryx2 skips
    /// it.
    Unreadable(String),
}

impl ConfigFile {
    /// The namespace a process loading this file uses, or `None` when
    /// iceoryx2 would move on to the next file.
    fn namespace(&self) -> Option<Namespace> {
        match self {
            ConfigFile::Read(ns) => Some(ns.clone()),
            ConfigFile::Absent | ConfigFile::Unreadable(_) => None,
        }
    }
}

/// Deserialize a config file's text into iceoryx2's own `Config`, as
/// `Config::from_file` does, so a file iceoryx2 refuses (a wrong type in any
/// section, an invalid prefix) is refused here too.
pub(super) fn parse(text: &str) -> ConfigFile {
    match toml::from_str::<iceoryx2::config::Config>(text) {
        Ok(config) => ConfigFile::Read(namespace_of(&config)),
        Err(err) => ConfigFile::Unreadable(err.message().to_string()),
    }
}

fn namespace_of(config: &iceoryx2::config::Config) -> Namespace {
    Namespace {
        root_path: config.global.root_path().to_string(),
        prefix: config.global.prefix.to_string(),
    }
}

/// The decision. `project` is the current directory's file; `others` are the
/// user and system files in iceoryx2's order, each with its path.
pub(super) fn decide(
    project_path: &Path,
    project: &ConfigFile,
    others: &[(PathBuf, ConfigFile)],
    defaults: &Namespace,
) -> Outcome {
    let elsewhere = others
        .iter()
        .find_map(|(path, file)| file.namespace().map(|ns| (Some(path), ns)))
        .unwrap_or((None, defaults.clone()));
    let (elsewhere_path, elsewhere_ns) = elsewhere;
    let elsewhere_name = elsewhere_path.map_or_else(
        || "iceoryx2's built-in defaults".to_string(),
        |p| p.display().to_string(),
    );
    let shown = project_path.display().to_string();
    match project {
        ConfigFile::Absent => Outcome::new(
            Status::Pass,
            format!(
                "no {shown}; processes started here use {elsewhere_name} like every other \
                 directory"
            ),
        )
        .reading("in_effect", Reading::Text(elsewhere_name)),
        ConfigFile::Unreadable(err) => Outcome::new(
            Status::Warn,
            format!("{shown} exists but iceoryx2 cannot read it ({err}), so it skips the file"),
        )
        .with_fix(format!(
            "fix or remove {shown}; until then processes started here use {elsewhere_name}"
        ))
        .reading("in_effect", Reading::Text(elsewhere_name)),
        ConfigFile::Read(_) => {
            let here = project.namespace().unwrap_or_else(|| defaults.clone());
            let base = |status, summary: String| {
                Outcome::new(status, summary)
                    .reading("in_effect", Reading::Text(shown.clone()))
                    .reading("root_path", Reading::Text(here.root_path.clone()))
                    .reading("prefix", Reading::Text(here.prefix.clone()))
                    .reading(
                        "elsewhere_root_path",
                        Reading::Text(elsewhere_ns.root_path.clone()),
                    )
                    .reading(
                        "elsewhere_prefix",
                        Reading::Text(elsewhere_ns.prefix.clone()),
                    )
            };
            if here == elsewhere_ns {
                base(
                    Status::Info,
                    format!(
                        "{shown} is in effect here and sets the same root-path and prefix as \
                         {elsewhere_name}"
                    ),
                )
            } else {
                base(
                    Status::Warn,
                    format!(
                        "{shown} is in effect here: root-path {} prefix {}, while processes \
                         started elsewhere use root-path {} prefix {} ({elsewhere_name})",
                        here.root_path, here.prefix, elsewhere_ns.root_path, elsewhere_ns.prefix
                    ),
                )
                .with_fix(format!(
                    "move {shown} to ~/.config/iceoryx2/iceoryx2.toml so every process loads \
                     it, or start every Cerulion process from this directory"
                ))
            }
        }
    }
}

fn read(path: &Path) -> ConfigFile {
    match std::fs::read_to_string(path) {
        Ok(text) => parse(&text),
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => ConfigFile::Absent,
        Err(err) => ConfigFile::Unreadable(err.to_string()),
    }
}

/// iceoryx2's built-in `root-path` and `prefix`, the namespace a process uses
/// when no file is loaded. Only compared against, never used to open a
/// namespace.
fn builtin_defaults() -> Namespace {
    namespace_of(&iceoryx2::config::Config::default())
}

fn run(ctx: &Context) -> Outcome {
    let project_path = ctx.cwd.join("config").join("iceoryx2.toml");
    let mut others = Vec::new();
    if let Some(home) = std::env::var_os("HOME").filter(|h| !h.is_empty()) {
        let user = PathBuf::from(home)
            .join(".config")
            .join("iceoryx2")
            .join("iceoryx2.toml");
        let file = read(&user);
        others.push((user, file));
    }
    let system = PathBuf::from("/etc/iceoryx2/iceoryx2.toml");
    let file = read(&system);
    others.push((system, file));
    decide(
        &project_path,
        &read(&project_path),
        &others,
        &builtin_defaults(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn defaults() -> Namespace {
        builtin_defaults()
    }

    fn here() -> PathBuf {
        PathBuf::from("/w/config/iceoryx2.toml")
    }

    #[test]
    fn passes_when_the_directory_has_no_project_file() {
        let o = decide(&here(), &ConfigFile::Absent, &[], &defaults());
        assert_eq!(o.status, Status::Pass);
        assert_eq!(
            o.readings.get("in_effect"),
            Some(&Reading::Text("iceoryx2's built-in defaults".to_string()))
        );
    }

    #[test]
    fn warns_when_the_project_file_moves_the_root_path() {
        let project = parse("[global]\nroot-path = \"/tmp/mine/\"\n");
        let o = decide(&here(), &project, &[], &defaults());
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.summary,
            "/w/config/iceoryx2.toml is in effect here: root-path /tmp/mine/ prefix iox2_, \
             while processes started elsewhere use root-path /tmp/iceoryx2/ prefix iox2_ \
             (iceoryx2's built-in defaults)"
        );
        assert!(o.fix.unwrap().contains("~/.config/iceoryx2/iceoryx2.toml"));
    }

    #[test]
    fn warns_when_only_the_prefix_differs_from_the_user_file() {
        let project = parse("[global]\nprefix = \"a_\"\n");
        let user = (
            PathBuf::from("/h/.config/iceoryx2/iceoryx2.toml"),
            parse("[global]\nprefix = \"b_\"\n"),
        );
        let o = decide(&here(), &project, &[user], &defaults());
        assert_eq!(o.status, Status::Warn);
        assert_eq!(
            o.readings.get("elsewhere_prefix"),
            Some(&Reading::Text("b_".to_string()))
        );
    }

    #[test]
    fn a_project_file_that_agrees_with_elsewhere_is_info() {
        let project = parse("[global]\nroot-path = \"/tmp/iceoryx2/\"\n[defaults]\n");
        let o = decide(&here(), &project, &[], &defaults());
        assert_eq!(o.status, Status::Info);
        assert_eq!(o.fix, None);
    }

    #[test]
    fn an_unreadable_user_file_is_skipped_like_iceoryx2_skips_it() {
        let project = parse("[global]\nprefix = \"a_\"\n");
        let others = [
            (
                PathBuf::from("/h/.config/iceoryx2/iceoryx2.toml"),
                ConfigFile::Unreadable("bad".to_string()),
            ),
            (
                PathBuf::from("/etc/iceoryx2/iceoryx2.toml"),
                parse("[global]\nprefix = \"a_\"\n"),
            ),
        ];
        let o = decide(&here(), &project, &others, &defaults());
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary.ends_with("/etc/iceoryx2/iceoryx2.toml"),
            "{}",
            o.summary
        );
    }

    #[test]
    fn a_wrong_typed_key_anywhere_makes_iceoryx2_skip_the_file() {
        let project = parse("[global]\nprefix = \"a_\"\ncreation-timeout = \"soon\"\n");
        assert!(matches!(project, ConfigFile::Unreadable(_)), "{project:?}");
    }

    #[test]
    fn an_unparseable_project_file_warns() {
        let project = parse("[global\n");
        assert!(matches!(project, ConfigFile::Unreadable(_)));
        let o = decide(&here(), &project, &[], &defaults());
        assert_eq!(o.status, Status::Warn);
        assert!(o.fix.is_some());
    }
}
