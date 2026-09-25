// SPDX-License-Identifier: AGPL-3.0-only
//! Bakes the PostHog project key a release build is given into the binary.
//!
//! `CERULION_POSTHOG_KEY` in the build environment becomes the compile-time
//! `CERULION_BAKED_POSTHOG_KEY` that `src/telemetry.rs` reads. The release
//! workflow sets it from a repository secret; a source build does not, so the
//! binary carries no key and sends nothing. It is a write-only project key.

fn main() {
    println!("cargo:rerun-if-env-changed=CERULION_POSTHOG_KEY");
    if let Some(key) = std::env::var("CERULION_POSTHOG_KEY")
        .ok()
        .map(|key| key.trim().to_owned())
        .filter(|key| !key.is_empty())
    {
        println!("cargo:rustc-env=CERULION_BAKED_POSTHOG_KEY={key}");
    }
}
