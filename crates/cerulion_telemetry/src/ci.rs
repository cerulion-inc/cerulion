// SPDX-License-Identifier: AGPL-3.0-only
//! Continuous integration detection. A run under CI still sends, tagged
//! `ci: true` (see `payload`), so CI traffic can be kept out of user counts.

/// The variables CI services set. Each counts when it is set and not empty;
/// `CI` also does not count when it is `false`, `0`, `no` or `off` (either
/// case).
pub const CI_ENV_VARS: &[&str] = &[
    "CI",
    "GITHUB_ACTIONS",
    "GITLAB_CI",
    "BUILDKITE",
    "CIRCLECI",
    "TRAVIS",
    "JENKINS_URL",
    "TEAMCITY_VERSION",
    "TF_BUILD",
    "BITBUCKET_BUILD_NUMBER",
    "CODEBUILD_BUILD_ID",
    "DRONE",
    "APPVEYOR",
];

/// Pure detection over a variable lookup, so tests need no real environment.
pub fn detect(lookup: impl Fn(&str) -> Option<String>) -> bool {
    CI_ENV_VARS.iter().any(|name| {
        let Some(value) = lookup(name) else {
            return false;
        };
        let value = value.trim();
        if *name == "CI" {
            !value.is_empty()
                && !["false", "0", "no", "off"]
                    .iter()
                    .any(|w| value.eq_ignore_ascii_case(w))
        } else {
            !value.is_empty()
        }
    })
}

/// Whether this process runs under CI, read from the real environment.
pub fn detected() -> bool {
    detect(|name| std::env::var(name).ok())
}
