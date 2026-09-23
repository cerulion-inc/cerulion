// SPDX-License-Identifier: AGPL-3.0-only
//! Shared state-root selection for automatic robots and beacon-facts readers.

use std::ffi::OsStr;
use std::path::{Path, PathBuf};

/// Never-renamed sibling lock shared by login writers and consistent readers.
pub const AUTH_STORE_LOCK_FILE: &str = ".studio-auth.lock";

/// Resolve the login configuration directory using the shared home policy.
pub fn config_dir() -> Option<PathBuf> {
    config_dir_from(
        std::env::var_os("CERULION_HOME").as_deref(),
        dirs::home_dir().as_deref(),
    )
}

fn config_dir_from(cerulion_home: Option<&OsStr>, home: Option<&Path>) -> Option<PathBuf> {
    match cerulion_home.filter(|value| !value.is_empty()) {
        Some(config) => Some(PathBuf::from(config)),
        None => Some(home?.join(".cerulion")),
    }
}

/// Resolve the automatic robot root using the login configuration's home rule.
pub fn resolve() -> Option<PathBuf> {
    resolve_from(
        std::env::var_os("CERULION_STATE_ROOT").as_deref(),
        std::env::var_os("CERULION_HOME").as_deref(),
        dirs::home_dir().as_deref(),
    )
}

/// Resolve the automatic robot root without accessing the environment or disk.
/// Explicit deployment roots are preserved verbatim. Otherwise the root lives
/// under the login configuration directory. Filesystem writers independently
/// require an absolute path with private, trusted ancestry.
pub fn resolve_from(
    state_root: Option<&OsStr>,
    cerulion_home: Option<&OsStr>,
    home: Option<&Path>,
) -> Option<PathBuf> {
    if let Some(root) = state_root {
        return Some(PathBuf::from(root));
    }
    Some(config_dir_from(cerulion_home, home)?.join("robot-state"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deployment_override_wins_without_rewriting_its_path() {
        assert_eq!(
            resolve_from(
                Some(OsStr::new("/srv/robot")),
                Some(OsStr::new("/config")),
                Some(Path::new("/home/operator"))
            ),
            Some(PathBuf::from("/srv/robot"))
        );
        assert_eq!(
            resolve_from(Some(OsStr::new("relative")), None, None),
            Some(PathBuf::from("relative"))
        );
    }

    #[test]
    fn login_config_override_precedes_the_home_default() {
        assert_eq!(
            resolve_from(
                None,
                Some(OsStr::new("/config")),
                Some(Path::new("/home/operator"))
            ),
            Some(PathBuf::from("/config/robot-state"))
        );
        assert_eq!(
            resolve_from(
                None,
                Some(OsStr::new("")),
                Some(Path::new("/home/operator"))
            ),
            Some(PathBuf::from("/home/operator/.cerulion/robot-state"))
        );
        assert_eq!(resolve_from(None, None, None), None);
    }
}
