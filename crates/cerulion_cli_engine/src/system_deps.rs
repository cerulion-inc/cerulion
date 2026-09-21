// SPDX-License-Identifier: AGPL-3.0-only
//! Optional SYSTEM dependencies for node crates — a node crate
//! declares which of its cargo features are backed by a system library, and
//! `cerulion node build` probes for that library and enables the feature when
//! it is there.
//!
//! # Why this lives in the CLI and not in a build script
//!
//! A Cargo build script **cannot enable a feature of its own crate**. Cargo
//! resolves the feature graph before any `build.rs` runs, and `cargo:rustc-cfg`
//! only sets a `cfg`, not a feature — so it cannot turn on an optional
//! *dependency*. There is therefore no pure-Cargo way to say "link GStreamer
//! if this machine has it". The smartness has to live one level up, in
//! whatever invokes cargo. That is `cerulion node build`.
//!
//! # The contract
//!
//! A node's `Cargo.toml` declares, per feature:
//!
//! ```toml
//! [package.metadata.cerulion.optional-system-deps.gstreamer]
//! pkg-config = ["gstreamer-1.0", "gstreamer-app-1.0"]
//! summary = "live camera capture (H.264 decode -> JPEG)"
//! without-it = "the node has no capture source, so `cerulion graph run` refuses it at launch"
//! install.macos = "brew install gstreamer"
//! install.debian = "sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
//! ```
//!
//! Then `cerulion node build <type>`:
//!
//! 1. probes every listed pkg-config module,
//! 2. passes `--features <feature>` when **all** of them resolve,
//! 3. and when any is missing, builds WITHOUT the feature but says so LOUDLY —
//!    naming what is unavailable, what will happen at run time, and the exact
//!    install command for this platform.
//!
//! The feature must NOT be in the crate's `default` features, or the plain
//! `cargo build` that every contributor runs would still require the system
//! library. Making it non-default is what lets `cargo build` succeed on a
//! machine with no GStreamer at all; this module is what puts the capability
//! back when the library IS present. `cerulion node build` WARNS loudly when a
//! declared feature IS in `default` (see [`gated_features_in_default`]) — the
//! rule is a contract, and a contract nothing checks is a suggestion.
//!
//! # Loud, never silent
//!
//! Deliberate refusals and warnings, all because the alternative is a silent
//! capability loss:
//!
//! - A `[package.metadata.cerulion]` block that fails to parse is a hard
//!   [`SystemDepError`], never an "ignore it and build plain". A typo in
//!   `pkg-config` would otherwise silently disable the probe forever and the
//!   node would ship inert with a green build.
//! - A misspelling of the TABLE NAME itself (`ceruleon`, `Cerulion`,
//!   `cerulion-deps`) cannot be a parse error — `package.metadata` is a shared
//!   namespace and every other tool's table lives there too — so it is a loud
//!   WARN instead, naming both spellings ([`near_miss_cerulion_keys`]).
//!   Without it the whole block is skipped in total silence.
//! - `summary`, `without-it` and at least one `install` command are REQUIRED,
//!   and required means NON-BLANK. A dep whose absence is not explained cannot
//!   produce an actionable message, and an unactionable warning is only
//!   marginally better than silence — a blank one renders that warning
//!   verbatim, so presence alone is not the contract.
//! - The table name must be a cargo feature the crate ACTUALLY has (an
//!   explicit `[features]` key or an optional dependency). Drift there fails
//!   only on a machine that HAS the library, i.e. green everywhere except the
//!   robot ([`SystemDepError::UnknownFeature`]).
//!
//! An ABSENT metadata block is not an error — it means "this node has no
//! optional system deps", which is true of every node but one.
//!
//! # What a probe can and cannot prove
//!
//! `pkg-config` answers exactly one question: do these `.pc` files resolve.
//! Two things it does NOT answer, both stated in the rendered notice rather
//! than glossed over:
//!
//! - A resolved module is a BUILD-time fact. Whatever run-time components the
//!   feature needs (GStreamer *elements*, CUDA devices, udev rules) are the
//!   feature's own business and are checked when it runs.
//! - A missing module is not the same as a missing library. If the
//!   `pkg-config` EXECUTABLE is itself absent, nothing was probed at all —
//!   [`PkgConfigTool`] carries that distinction so the notice names the tool
//!   (and the tool's install command) instead of a library the user may
//!   already have.

use std::collections::BTreeMap;
use std::path::Path;

use serde::Deserialize;

// ---------------------------------------------------------------------------
// Declaration (parsed from Cargo.toml)
// ---------------------------------------------------------------------------

/// One optional system dependency: a cargo feature gated on the presence of
/// one or more pkg-config modules.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OptionalSystemDep {
    /// The cargo feature to enable when every module below resolves.
    pub feature: String,
    /// pkg-config module names, ALL of which must resolve.
    pub modules: Vec<String>,
    /// What the feature provides, in user words ("live camera capture …").
    pub summary: String,
    /// What happens at RUN time without it — the node's own plain statement.
    pub without_it: String,
    /// Platform key → install command. Never empty.
    pub install: BTreeMap<String, String>,
}

/// A malformed or unusable `[package.metadata.cerulion]` declaration. Always a
/// hard failure: see the module docs.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SystemDepError {
    #[error(
        "{manifest}: could not parse [package.metadata.cerulion]: {detail}\n  \
         A node's cerulion metadata block must parse or the build cannot know \
         which optional system dependencies to probe for. Fix the block (or \
         delete it if the node has no optional system deps) — it is NOT \
         ignored, because a typo here would silently disable the probe and \
         ship the node without its capability."
    )]
    Malformed { manifest: String, detail: String },

    #[error(
        "{manifest}: optional system dep `{feature}` declares an empty \
         `pkg-config` list — there is nothing to probe for, so the feature \
         could never be enabled. List the pkg-config module(s) the feature \
         needs."
    )]
    NoModules { manifest: String, feature: String },

    #[error(
        "{manifest}: optional system dep `{feature}` declares no `install` \
         command. At least one platform entry is required (e.g. \
         `install.macos` / `install.debian`) so a user without the library is \
         told how to get it instead of just being told it is missing."
    )]
    NoInstallCommand { manifest: String, feature: String },

    #[error(
        "{manifest}: optional system dep `{feature}` does not name a cargo \
         feature of this crate. The table name IS the feature to enable, and \
         this crate declares [{known}]. Add `{feature} = [...]` under \
         `[features]` (or rename the table to match). Left unchecked this \
         fails ONLY on a machine that HAS the library — the probe resolves, \
         `--features {feature}` is passed, and cargo refuses the build there \
         while every machine without it stays green."
    )]
    UnknownFeature {
        manifest: String,
        feature: String,
        known: String,
    },

    #[error(
        "{manifest}: optional system dep `{feature}` declares a BLANK `{field}`. \
         The field is required because an unexplained warning is not \
         actionable — and a blank one renders exactly the empty, unactionable \
         notice the requirement exists to prevent. Give it real text."
    )]
    BlankField {
        manifest: String,
        feature: String,
        field: String,
    },
}

// The serde shapes for the `cerulion` sub-table ONLY — the rest of a manifest
// is none of our business, and `deny_unknown_fields` applied any higher would
// reject every real one. `deny_unknown_fields` HERE is load-bearing: it makes
// `pkgconfig` (or any other typo) a loud parse failure rather than a silently
// ignored key that leaves the probe dead.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawCerulion {
    #[serde(rename = "optional-system-deps", default)]
    optional_system_deps: BTreeMap<String, RawDep>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RawDep {
    #[serde(rename = "pkg-config")]
    pkg_config: Vec<String>,
    summary: String,
    #[serde(rename = "without-it")]
    without_it: String,
    install: BTreeMap<String, String>,
}

/// Parse `[package.metadata.cerulion.optional-system-deps]` out of a
/// `Cargo.toml`'s text. Returns an EMPTY vec when the block is absent (the
/// normal case). Deterministic order (the `BTreeMap` sorts by feature name).
pub fn parse_optional_system_deps(
    manifest_label: &str,
    cargo_toml: &str,
) -> Result<Vec<OptionalSystemDep>, SystemDepError> {
    // Walk to `package.metadata.cerulion` as an untyped `toml::Value` FIRST,
    // then deserialize only that sub-table into the strict `RawCerulion`. Going
    // straight to a typed shape from the document root is not an option: our
    // `deny_unknown_fields` would then have to describe every key a real
    // manifest can carry (dependencies, lints, target tables, other tools'
    // `package.metadata`), and it would reject all of them.
    let doc: toml::Value = toml::from_str(cargo_toml).map_err(|e| SystemDepError::Malformed {
        manifest: manifest_label.to_string(),
        detail: format!("manifest is not valid TOML: {e}"),
    })?;

    let Some(pkg) = doc.get("package") else {
        return Ok(Vec::new());
    };
    // A `[package]` table with no `metadata` is by far the common case; only
    // dig further when `metadata.cerulion` actually exists, so an unrelated
    // `metadata.docs.rs` block cannot trip `deny_unknown_fields`.
    let Some(cerulion) = pkg.get("metadata").and_then(|m| m.get("cerulion")) else {
        // Still validate that `package.metadata` is a table if present, so a
        // scalar there is reported rather than silently skipped.
        if let Some(meta) = pkg.get("metadata") {
            if !meta.is_table() {
                return Err(SystemDepError::Malformed {
                    manifest: manifest_label.to_string(),
                    detail: "[package.metadata] is not a table".to_string(),
                });
            }
            // The one remaining silent-skip: the TABLE NAME itself is
            // misspelled. `deny_unknown_fields` cannot reach it (it only
            // applies once `cerulion` has already resolved) and it cannot be a
            // hard error either — `package.metadata` is shared with docs.rs,
            // cargo-machete and anything else, so refusing unknown siblings
            // would reject most real manifests. Warn LOUDLY instead, naming
            // both spellings, because the alternative is a node that ships
            // without its capability forever under a green build.
            for near in near_miss_cerulion_keys(meta) {
                tracing::warn!(
                    manifest = %manifest_label,
                    found = %near,
                    expected = "cerulion",
                    "[package.metadata.{near}] looks like a misspelling of \
                     [package.metadata.cerulion] — it is being IGNORED, so any \
                     optional-system-deps block under it never runs and the node \
                     builds without its feature. Rename the table to `cerulion` \
                     (or, if the key really belongs to another tool, ignore this)."
                );
            }
        }
        return Ok(Vec::new());
    };

    let parsed: RawCerulion =
        cerulion
            .clone()
            .try_into()
            .map_err(|e| SystemDepError::Malformed {
                manifest: manifest_label.to_string(),
                detail: e.to_string(),
            })?;

    // The feature names this crate actually has, resolved ONCE — the table
    // name is the feature to enable, and drift between the two fails only on a
    // machine that HAS the library (see `UnknownFeature`).
    let declarable = declarable_feature_names(&doc);

    let mut out = Vec::with_capacity(parsed.optional_system_deps.len());
    for (feature, raw) in parsed.optional_system_deps {
        if raw.pkg_config.is_empty() {
            return Err(SystemDepError::NoModules {
                manifest: manifest_label.to_string(),
                feature,
            });
        }
        if raw.install.is_empty() {
            return Err(SystemDepError::NoInstallCommand {
                manifest: manifest_label.to_string(),
                feature,
            });
        }
        // Presence is not content: serde proves the keys exist, and a blank
        // value then renders a notice naming no capability, no consequence and
        // no command — the exact state the required-fields rule exists to
        // prevent. Modules too: a blank module name can never resolve.
        for (field, value) in [
            ("summary", raw.summary.as_str()),
            ("without-it", raw.without_it.as_str()),
        ] {
            if value.trim().is_empty() {
                return Err(SystemDepError::BlankField {
                    manifest: manifest_label.to_string(),
                    feature,
                    field: field.to_string(),
                });
            }
        }
        if raw.pkg_config.iter().any(|m| m.trim().is_empty()) {
            return Err(SystemDepError::BlankField {
                manifest: manifest_label.to_string(),
                feature,
                field: "pkg-config".to_string(),
            });
        }
        if let Some((key, _)) = raw.install.iter().find(|(_, cmd)| cmd.trim().is_empty()) {
            return Err(SystemDepError::BlankField {
                manifest: manifest_label.to_string(),
                feature,
                field: format!("install.{key}"),
            });
        }
        if !declarable.contains(&feature) {
            let mut known: Vec<&str> = declarable.iter().map(String::as_str).collect();
            known.sort_unstable();
            return Err(SystemDepError::UnknownFeature {
                manifest: manifest_label.to_string(),
                known: known.join(", "),
                feature,
            });
        }
        out.push(OptionalSystemDep {
            feature,
            modules: raw.pkg_config,
            summary: raw.summary,
            without_it: raw.without_it,
            install: raw.install,
        });
    }
    Ok(out)
}

/// The table name this module owns, under `[package.metadata]`.
const CERULION_KEY: &str = "cerulion";

/// Every name that can legally appear as an optional-system-dep table name:
/// the crate's explicit `[features]` keys PLUS the name of every OPTIONAL
/// dependency (cargo mints an implicit feature per optional dep).
///
/// Deliberately PERMISSIVE at one corner: naming a dep with `dep:foo` anywhere
/// in `[features]` suppresses its implicit feature, and this does not model
/// that. Accepting one name cargo would reject costs a build error the user
/// still sees; REFUSING a name cargo accepts would break a valid manifest, and
/// this check is not worth that.
fn declarable_feature_names(doc: &toml::Value) -> std::collections::BTreeSet<String> {
    let mut names = std::collections::BTreeSet::new();
    if let Some(table) = doc.get("features").and_then(|f| f.as_table()) {
        names.extend(table.keys().cloned());
    }
    // Optional deps: normal + build, at the top level and under any
    // `[target.<cfg>.…]` table. Dev-dependencies cannot be optional.
    let mut collect_optional = |section: Option<&toml::Value>| {
        let Some(table) = section.and_then(|s| s.as_table()) else {
            return;
        };
        for (name, spec) in table {
            if spec
                .get("optional")
                .and_then(toml::Value::as_bool)
                .unwrap_or(false)
            {
                names.insert(name.clone());
            }
        }
    };
    collect_optional(doc.get("dependencies"));
    collect_optional(doc.get("build-dependencies"));
    if let Some(targets) = doc.get("target").and_then(|t| t.as_table()) {
        for cfg in targets.values() {
            collect_optional(cfg.get("dependencies"));
            collect_optional(cfg.get("build-dependencies"));
        }
    }
    names
}

/// Sibling `[package.metadata.<key>]` tables whose name looks like a
/// misspelling of `cerulion` — the one remaining way to disable the probe in
/// total silence (see the module docs).
///
/// `meta` is the `package.metadata` value; a non-table yields nothing. A key is
/// a near miss when it is not exactly `cerulion` AND either
///
/// - it CONTAINS `cerulion` (case-insensitively) — `cerulion-deps`,
///   `cerulion_optional`, `Cerulion`; or
/// - it is within Levenshtein distance 2 of `cerulion` — `ceruleon`,
///   `celurion`, `cerulon`.
///
/// Both halves are needed: a suffixed key is many edits away, and a
/// transposition contains nothing. Unrelated keys stay silent — `docs`,
/// `cargo-machete`, `playground` are neither, and any key shorter than 6 or
/// longer than 10 characters is >2 edits from `cerulion` by length alone.
pub fn near_miss_cerulion_keys(meta: &toml::Value) -> Vec<String> {
    let Some(table) = meta.as_table() else {
        return Vec::new();
    };
    table
        .keys()
        .filter(|k| k.as_str() != CERULION_KEY)
        .filter(|k| {
            k.to_ascii_lowercase().contains(CERULION_KEY)
                || crate::near_miss::levenshtein_at_most(
                    k,
                    CERULION_KEY,
                    crate::near_miss::MAX_EDITS,
                )
        })
        .cloned()
        .collect()
}

/// Declared features that are ALSO in the crate's `default` list — the
/// contract violation the whole mechanism exists to prevent.
///
/// A gated feature left in `default` makes the system library a hard
/// prerequisite for the plain `cargo build` every contributor (and every CI
/// job) runs, which is exactly the breakage this module was built to remove;
/// the probe then buys nothing, since the feature is already on. Returned in
/// declaration order so the caller can name every offender.
///
/// This is a WARNING, not a refusal: the build still produces a correct
/// artifact, and refusing would break a user's node over a manifest smell.
pub fn gated_features_in_default<'a>(
    deps: &'a [OptionalSystemDep],
    default_features: &[String],
) -> Vec<&'a str> {
    deps.iter()
        .filter(|d| default_features.iter().any(|f| f == &d.feature))
        .map(|d| d.feature.as_str())
        .collect()
}

/// The crate's `[features] default = [...]` list, or an empty vec when there is
/// none.
///
/// Exists so the "a gated feature must NOT be in `default`" guard can be
/// asserted over real manifests without re-implementing TOML handling at the
/// test site. That rule is the whole point of this module: a gated feature left
/// in `default` makes the system library a hard prerequisite for a plain
/// `cargo build` again, and the probe buys nothing.
pub fn declared_default_features(
    manifest_label: &str,
    cargo_toml: &str,
) -> Result<Vec<String>, SystemDepError> {
    let doc: toml::Value = toml::from_str(cargo_toml).map_err(|e| SystemDepError::Malformed {
        manifest: manifest_label.to_string(),
        detail: format!("manifest is not valid TOML: {e}"),
    })?;
    Ok(doc
        .get("features")
        .and_then(|f| f.get("default"))
        .and_then(|d| d.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str().map(str::to_string))
                .collect()
        })
        .unwrap_or_default())
}

/// Read + parse a node crate's manifest. A MISSING manifest yields no deps
/// (`cargo build` will produce the real, better error); an unreadable one is
/// reported.
pub fn optional_system_deps_of_node(
    node_dir: &Path,
) -> Result<Vec<OptionalSystemDep>, SystemDepError> {
    let manifest = node_dir.join("Cargo.toml");
    let label = manifest.display().to_string();
    match std::fs::read_to_string(&manifest) {
        Ok(text) => parse_optional_system_deps(&label, &text),
        // Let cargo produce the canonical "no such package" diagnostic.
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Vec::new()),
        Err(e) => Err(SystemDepError::Malformed {
            manifest: label,
            detail: format!("could not read the manifest: {e}"),
        }),
    }
}

// ---------------------------------------------------------------------------
// Probing
// ---------------------------------------------------------------------------

/// Result of probing ONE pkg-config module.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ModuleProbe {
    /// Present, with the version pkg-config reported.
    Found { module: String, version: String },
    /// Not resolved. Whether that means "the library is absent" or "nothing
    /// could be probed at all" is [`PkgConfigTool`]'s business, not this
    /// enum's — see [`render_report`].
    Missing { module: String },
}

/// Whether the `pkg-config` EXECUTABLE itself is usable on this machine.
///
/// Load-bearing for the RENDERED message, never for the decision: a feature
/// whose modules did not resolve stays off either way (without `pkg-config`,
/// `system-deps`-style build scripts cannot link the library anyway). What
/// changes is what the user is told. Collapsing the two states — the
/// earlier-review behaviour — told a user with GStreamer installed but no
/// `pkg-config` that GStreamer was missing, and handed them the GStreamer
/// install command; they run it, re-run the build, and get a byte-identical
/// message. A dead end, from a diagnosis that was simply wrong.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum PkgConfigTool {
    /// The executable ran (whatever it then said about any module).
    #[default]
    Present,
    /// Not found on `PATH` (`io::ErrorKind::NotFound` from the spawn). Nothing
    /// was probed; no conclusion about any library is available.
    NotInstalled,
    /// Found but not runnable — permissions, exec-format, a broken shim. The
    /// OS error rides along verbatim so the notice never claims more than it
    /// knows.
    Unusable(String),
}

impl PkgConfigTool {
    /// `true` when module verdicts mean what they say. When `false`, every
    /// `Missing` is "unprobed", not "absent".
    pub fn can_probe(&self) -> bool {
        matches!(self, PkgConfigTool::Present)
    }
}

/// Probe for the `pkg-config` EXECUTABLE — once per build, before the module
/// loop, so a missing tool is diagnosed as a missing tool.
///
/// `pkg-config --version` rather than `--modversion <mod>`: it succeeds on
/// every install and asks nothing about any library, so the two questions
/// ("is the tool here" / "is the library here") stay separate at the source.
/// A non-zero exit still counts as PRESENT — an executable that ran is
/// installed, whatever it thought of its arguments.
pub fn pkg_config_tool() -> PkgConfigTool {
    match std::process::Command::new("pkg-config")
        .arg("--version")
        .output()
    {
        Ok(_) => PkgConfigTool::Present,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => PkgConfigTool::NotInstalled,
        Err(e) => PkgConfigTool::Unusable(e.to_string()),
    }
}

/// Install commands for `pkg-config` ITSELF, by the same platform keys a
/// manifest's `install` table uses.
///
/// A built-in table rather than a manifest field: every node that declares a
/// pkg-config-gated dep needs the identical answer, and asking each node
/// author to repeat it is how one of them gets it wrong. (`pkgconf` is the
/// maintained implementation Homebrew ships; Debian's `pkg-config` package is
/// a transitional wrapper around it.)
fn pkg_config_install_table() -> BTreeMap<String, String> {
    BTreeMap::from([
        ("macos".to_string(), "brew install pkgconf".to_string()),
        (
            "debian".to_string(),
            "sudo apt install pkg-config".to_string(),
        ),
    ])
}

/// Whether a declared dep's feature will be enabled.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DepStatus {
    /// Every module resolved — the feature is enabled.
    Satisfied,
    /// At least one module is missing — the feature stays off.
    Missing,
}

/// One dep's verdict plus the per-module evidence behind it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DepOutcome {
    pub dep: OptionalSystemDep,
    pub status: DepStatus,
    pub probes: Vec<ModuleProbe>,
}

impl DepOutcome {
    /// Modules that were not found (empty when [`DepStatus::Satisfied`]).
    pub fn missing_modules(&self) -> Vec<&str> {
        self.probes
            .iter()
            .filter_map(|p| match p {
                ModuleProbe::Missing { module } => Some(module.as_str()),
                ModuleProbe::Found { .. } => None,
            })
            .collect()
    }
}

/// The whole build's system-dep decision.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SystemDepReport {
    pub outcomes: Vec<DepOutcome>,
}

impl SystemDepReport {
    /// Features to pass to `cargo build --features`, in the report's own
    /// outcome order.
    ///
    /// That order is feature-NAME order, not manifest order: the declarations
    /// are parsed out of a `BTreeMap`, which sorts by key, and
    /// [`evaluate`] preserves it. Deterministic either way — which is the
    /// property that matters for a reproducible argv — but "declaration order"
    /// was simply the wrong description of it.
    pub fn features_to_enable(&self) -> Vec<String> {
        self.outcomes
            .iter()
            .filter(|o| o.status == DepStatus::Satisfied)
            .map(|o| o.dep.feature.clone())
            .collect()
    }

    /// True when at least one declared dep could not be satisfied.
    pub fn has_missing(&self) -> bool {
        self.outcomes.iter().any(|o| o.status == DepStatus::Missing)
    }

    /// True when the node declared no optional system deps at all.
    pub fn is_empty(&self) -> bool {
        self.outcomes.is_empty()
    }
}

/// Probe every declared dep. `probe` is injected so tests drive both arms
/// without needing (or lacking) a real system library.
pub fn evaluate(
    deps: Vec<OptionalSystemDep>,
    probe: &dyn Fn(&str) -> Option<String>,
) -> SystemDepReport {
    let outcomes = deps
        .into_iter()
        .map(|dep| {
            let probes: Vec<ModuleProbe> = dep
                .modules
                .iter()
                .map(|m| match probe(m) {
                    Some(version) => ModuleProbe::Found {
                        module: m.clone(),
                        version,
                    },
                    None => ModuleProbe::Missing { module: m.clone() },
                })
                .collect();
            let status = if probes
                .iter()
                .all(|p| matches!(p, ModuleProbe::Found { .. }))
            {
                DepStatus::Satisfied
            } else {
                DepStatus::Missing
            };
            DepOutcome {
                dep,
                status,
                probes,
            }
        })
        .collect();
    SystemDepReport { outcomes }
}

/// The production probe: `pkg-config --modversion <module>`.
///
/// Returns `None` on ANY failure — module absent, non-zero exit, empty output,
/// or the tool itself missing. All of those mean the same thing for the
/// DECISION ("the library's presence cannot be proven"), which is why one `Option`
/// is enough here. They do NOT mean the same thing for the MESSAGE: the
/// tool-missing case is separated out by [`pkg_config_tool`], probed once
/// before this runs, and the caller passes that verdict to [`render_report`].
pub fn pkg_config_probe(module: &str) -> Option<String> {
    let out = std::process::Command::new("pkg-config")
        .arg("--modversion")
        .arg(module)
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let v = String::from_utf8_lossy(&out.stdout).trim().to_string();
    if v.is_empty() {
        None
    } else {
        Some(v)
    }
}

// ---------------------------------------------------------------------------
// Platform + rendering (the user-facing contract)
// ---------------------------------------------------------------------------

/// One `install.<key>` preference: the key, plus whether it is PROVEN to
/// describe the running machine or is only the best guess for its family.
///
/// The distinction exists because `target_os` cannot tell Debian from Fedora.
/// A bare, unlabelled command reads as "this is the command for your machine";
/// handing a Fedora user `sudo apt install …` with no marker is a claim we
/// cannot make, and it fails with `apt: command not found` and no hint that
/// the line was written for a different distribution. A guessed key is
/// therefore SHOWN (an approximate answer beats none) but LABELLED.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InstallKeyPreference {
    /// The `install.<key>` key to look for.
    pub key: &'static str,
    /// `true` when the key is proven to describe THIS machine.
    pub exact: bool,
}

impl InstallKeyPreference {
    /// A key proven to describe the running machine (`macos` on macOS).
    pub const fn exact(key: &'static str) -> Self {
        Self { key, exact: true }
    }
    /// A key that is only the best guess for this machine (`debian` on an
    /// arbitrary Linux; `brew` anywhere — a package MANAGER never checked
    /// for, not a platform).
    pub const fn guess(key: &'static str) -> Self {
        Self { key, exact: false }
    }
}

/// Which `install.<key>` entries to prefer on the running machine, best first.
///
/// Free-form keys are allowed in the manifest; this only decides which ones to
/// PREFER, and — via [`InstallKeyPreference::exact`] — which of them we may
/// present as unqualified fact. When none of the preferred keys is present,
/// the notice lists every declared command labelled by key: better an
/// over-full answer than none.
pub fn preferred_install_keys() -> &'static [InstallKeyPreference] {
    // `macos` is proven (we ARE on macOS). `brew` names a package MANAGER that
    // may not be installed, so it is shown labelled.
    #[cfg(target_os = "macos")]
    const KEYS: &[InstallKeyPreference] = &[
        InstallKeyPreference::exact("macos"),
        InstallKeyPreference::guess("brew"),
    ];
    // `debian`/`ubuntu` are FAMILY GUESSES — `target_os = "linux"` is true on
    // Fedora, Arch, NixOS and everything else. `linux` is the only key this
    // platform can claim exactly.
    #[cfg(target_os = "linux")]
    const KEYS: &[InstallKeyPreference] = &[
        InstallKeyPreference::guess("debian"),
        InstallKeyPreference::guess("ubuntu"),
        InstallKeyPreference::exact("linux"),
    ];
    #[cfg(not(any(target_os = "macos", target_os = "linux")))]
    const KEYS: &[InstallKeyPreference] = &[];
    KEYS
}

/// Pick the install line(s) to show: the first preferred key that exists —
/// labelled unless that key is an EXACT match for this machine — else every
/// declared entry labelled by key.
///
/// `None` in the returned label means "this is your machine's command" and is
/// rendered bare/copy-pasteable; `Some(key)` is rendered `[key] cmd`.
pub fn install_lines(
    install: &BTreeMap<String, String>,
    preferred: &[InstallKeyPreference],
) -> Vec<(Option<String>, String)> {
    for pref in preferred {
        if let Some(cmd) = install.get(pref.key) {
            let label = if pref.exact {
                None
            } else {
                Some(pref.key.to_string())
            };
            return vec![(label, cmd.clone())];
        }
    }
    install
        .iter()
        .map(|(k, v)| (Some(k.clone()), v.clone()))
        .collect()
}

/// Render the user-facing notice for a build. Returns `None` when the node
/// declared no optional system deps (nothing to say — the overwhelmingly
/// common case, and silence there is correct).
///
/// `tool` is [`pkg_config_tool`]'s verdict. It does not change any decision;
/// it decides whether an unresolved module is reported as "the library is
/// missing" or as "nothing could be probed".
///
/// This text IS the contract; it is pinned by the tests below.
pub fn render_report(
    node_type: &str,
    report: &SystemDepReport,
    tool: &PkgConfigTool,
    preferred: &[InstallKeyPreference],
) -> Option<String> {
    if report.is_empty() {
        return None;
    }
    let mut s = String::new();
    for o in &report.outcomes {
        match o.status {
            DepStatus::Satisfied => {
                let found: Vec<String> = o
                    .probes
                    .iter()
                    .map(|p| match p {
                        ModuleProbe::Found { module, version } => format!("{module} {version}"),
                        ModuleProbe::Missing { module } => module.clone(),
                    })
                    .collect();
                s.push_str(&format!(
                    "{node_type}: system dependency `{}` FOUND ({}).\n  \
                     Building WITH `--features {}` — {} is enabled.\n  \
                     (pkg-config resolved those modules: a BUILD-time fact. Any \
                     run-time components the feature needs are checked when it \
                     runs.)\n",
                    o.dep.feature,
                    found.join(", "),
                    o.dep.feature,
                    o.dep.summary,
                ));
            }
            DepStatus::Missing if !tool.can_probe() => {
                // NOTHING was probed. Saying "the library is missing" here is
                // a claim this check cannot support, and the library's install command
                // is a dead end for a user who already has it.
                let cause = match tool {
                    PkgConfigTool::NotInstalled => {
                        "the `pkg-config` TOOL is not installed".to_string()
                    }
                    PkgConfigTool::Unusable(detail) => {
                        format!("the `pkg-config` TOOL could not be run ({detail})")
                    }
                    PkgConfigTool::Present => unreachable!("guarded by !can_probe()"),
                };
                s.push_str(&format!(
                    "{node_type}: system dependency `{}` NOT PROVEN — {cause}, so \
                     its module(s) ({}) could not be probed AT ALL. This is not \
                     evidence that the library is missing.\n  \
                     Building WITHOUT `--features {}`. The build will SUCCEED, \
                     but {} will be unavailable:\n  \
                     {}\n  \
                     Install pkg-config first, then re-run this command:\n",
                    o.dep.feature,
                    o.dep.modules.join(", "),
                    o.dep.feature,
                    o.dep.summary,
                    o.dep.without_it,
                ));
                push_install_lines(&mut s, &pkg_config_install_table(), preferred);
                s.push_str("  If the library is also absent, install it too:\n");
                push_install_lines(&mut s, &o.dep.install, preferred);
            }
            DepStatus::Missing => {
                s.push_str(&format!(
                    "{node_type}: system dependency `{}` NOT FOUND \
                     (missing pkg-config module(s): {}).\n  \
                     Building WITHOUT `--features {}`. The build will SUCCEED, \
                     but {} will be unavailable:\n  \
                     {}\n  \
                     To enable it, install the library and re-run this command:\n",
                    o.dep.feature,
                    o.missing_modules().join(", "),
                    o.dep.feature,
                    o.dep.summary,
                    o.dep.without_it,
                ));
                push_install_lines(&mut s, &o.dep.install, preferred);
            }
        }
    }
    Some(s)
}

/// Append the chosen install line(s). A `None` label is this machine's own
/// command and prints bare (copy-pasteable); a `Some(key)` label prints
/// `[key] cmd` — either a family guess or one row of the
/// nothing-matched listing, and in both cases the bracket is what stops the
/// line from claiming to be this machine's answer.
fn push_install_lines(
    s: &mut String,
    install: &BTreeMap<String, String>,
    preferred: &[InstallKeyPreference],
) {
    for (key, cmd) in install_lines(install, preferred) {
        match key {
            Some(k) => s.push_str(&format!("      [{k}] {cmd}\n")),
            None => s.push_str(&format!("      {cmd}\n")),
        }
    }
}

// ---------------------------------------------------------------------------
// Oracle tests (pure — no cargo, no pkg-config)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use tracing_test::traced_test;

    const CAMERA_MANIFEST: &str = r#"
[package]
name = "camera_jpeg"
version = "0.1.0"

[features]
cdylib = []
gstreamer = []
default = ["cdylib"]

[package.metadata.cerulion.optional-system-deps.gstreamer]
pkg-config = ["gstreamer-1.0", "gstreamer-app-1.0"]
summary = "live camera capture (H.264 decode -> JPEG)"
without-it = "the node has no capture source, so `cerulion graph run` refuses it at launch"
install.macos = "brew install gstreamer"
install.debian = "sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"

[dependencies]
tracing = "0.1"
"#;

    /// A SYNTHETIC fixture modelled on camera_jpeg — deliberately NOT a mirror
    /// of the shipped manifest. It is the stimulus for the verbatim
    /// notice-text pins, which must not churn every time a demo's prose is
    /// reworded. Anything that needs the REAL manifest reads the file: see
    /// [`shipped_camera_dep`].
    fn camera_dep() -> OptionalSystemDep {
        OptionalSystemDep {
            feature: "gstreamer".to_string(),
            modules: vec!["gstreamer-1.0".to_string(), "gstreamer-app-1.0".to_string()],
            summary: "live camera capture (H.264 decode -> JPEG)".to_string(),
            without_it:
                "the node has no capture source, so `cerulion graph run` refuses it at launch"
                    .to_string(),
            install: BTreeMap::from([
                ("macos".to_string(), "brew install gstreamer".to_string()),
                (
                    "debian".to_string(),
                    "sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
                        .to_string(),
                ),
            ]),
        }
    }

    /// A SECOND dep, so multi-dep behaviour is exercised at all. Sorts AFTER
    /// `gstreamer` by feature name, which is what makes the ordering assertions
    /// non-vacuous.
    fn realsense_dep() -> OptionalSystemDep {
        OptionalSystemDep {
            feature: "realsense".to_string(),
            modules: vec!["realsense2".to_string()],
            summary: "depth camera capture".to_string(),
            without_it: "the depth stream is unavailable".to_string(),
            install: BTreeMap::from([(
                "macos".to_string(),
                "brew install librealsense".to_string(),
            )]),
        }
    }

    /// A hand-built preference list standing in for "we are on macOS".
    const MACOS_KEYS: &[InstallKeyPreference] = &[InstallKeyPreference::exact("macos")];

    /// The dep as the SHIPPED `examples/go2/nodes/camera_jpeg/Cargo.toml` really
    /// declares it, parsed from the file — never a hand copy.
    ///
    /// A hand copy cannot pin the shipped manifest: it drifts (this one had,
    /// within a single PR — the `without-it` text differed), and worse, the
    /// mutation the platform-key test exists to catch (renaming `install.macos`
    /// to `install.darwin` in the REAL file) leaves a hand copy untouched and
    /// every assertion green.
    fn shipped_camera_dep() -> OptionalSystemDep {
        let manifest = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../examples/go2/nodes/camera_jpeg/Cargo.toml");
        let text = std::fs::read_to_string(&manifest)
            .unwrap_or_else(|e| panic!("reading {}: {e}", manifest.display()));
        let deps = parse_optional_system_deps(&manifest.display().to_string(), &text)
            .expect("the shipped manifest must parse");
        deps.into_iter()
            .find(|d| d.feature == "gstreamer")
            .expect("camera_jpeg must still declare the `gstreamer` optional system dep")
    }

    #[test]
    fn parses_the_declaration_against_a_hand_built_oracle() {
        let got = parse_optional_system_deps("Cargo.toml", CAMERA_MANIFEST).expect("parses");
        assert_eq!(got, vec![camera_dep()]);
    }

    #[test]
    fn a_manifest_with_no_cerulion_metadata_declares_nothing() {
        let plain = "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n[dependencies]\n";
        assert_eq!(
            parse_optional_system_deps("Cargo.toml", plain).expect("parses"),
            vec![]
        );
    }

    /// An UNRELATED `[package.metadata.*]` block must not be mistaken for ours
    /// — `docs.rs` metadata is extremely common and would otherwise trip
    /// `deny_unknown_fields`.
    #[test]
    fn unrelated_package_metadata_is_left_alone() {
        let m = "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.docs.rs]\nall-features = true\n";
        assert_eq!(
            parse_optional_system_deps("Cargo.toml", m).expect("parses"),
            vec![]
        );
    }

    /// THE silent-failure guard: a typo'd key is a LOUD parse error, not a
    /// silently-skipped probe. Without `deny_unknown_fields` this manifest
    /// would parse as "no modules" or worse, and the node would ship inert
    /// with a green build.
    #[test]
    fn a_typod_key_is_a_loud_error_never_a_silent_skip() {
        let typo = r#"
[package]
name = "camera_jpeg"
version = "0.1.0"

[package.metadata.cerulion.optional-system-deps.gstreamer]
pkgconfig = ["gstreamer-1.0"]
summary = "s"
without-it = "w"
install.macos = "brew install gstreamer"
"#;
        let err = parse_optional_system_deps("Cargo.toml", typo).expect_err("must refuse");
        assert!(
            matches!(err, SystemDepError::Malformed { .. }),
            "expected Malformed, got {err:?}"
        );
        // The message must name the manifest so the user knows where to look.
        assert!(err.to_string().contains("Cargo.toml"), "{err}");
    }

    /// A dep that omits the run-time consequence (or the summary) is refused:
    /// an unexplained warning is not actionable.
    #[test]
    fn a_dep_missing_its_consequence_or_summary_is_refused() {
        // (which field is omitted, the OTHER field's line)
        for (omitted, kept) in [
            ("summary", "without-it = \"w\"\n"),
            ("without-it", "summary = \"s\"\n"),
        ] {
            let m = format!(
                "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.cerulion.optional-system-deps.f]\n\
                 pkg-config = [\"m\"]\n{kept}install.macos = \"c\"\n"
            );
            match parse_optional_system_deps("Cargo.toml", &m) {
                Err(SystemDepError::Malformed { detail, .. }) => assert!(
                    detail.contains(omitted),
                    "omitting `{omitted}` must be reported by name; detail was: {detail}"
                ),
                other => panic!("omitting `{omitted}` must be refused as Malformed, got {other:?}"),
            }
        }
    }

    /// The anti-tautology control for the two refusal tests above: the SAME
    /// shape with every required field present parses cleanly. Without this,
    /// a change that made every manifest fail would leave those tests green.
    #[test]
    fn a_fully_declared_minimal_dep_parses() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [features]\nf = []\n\n\
                 [package.metadata.cerulion.optional-system-deps.f]\n\
                 pkg-config = [\"m\"]\nsummary = \"s\"\nwithout-it = \"w\"\n\
                 install.macos = \"c\"\n";
        let got = parse_optional_system_deps("Cargo.toml", m).expect("parses");
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].feature, "f");
        assert_eq!(got[0].modules, vec!["m".to_string()]);
    }

    #[test]
    fn an_empty_module_list_is_refused() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.cerulion.optional-system-deps.f]\n\
                 pkg-config = []\nsummary = \"s\"\nwithout-it = \"w\"\n\
                 install.macos = \"c\"\n";
        assert_eq!(
            parse_optional_system_deps("m.toml", m),
            Err(SystemDepError::NoModules {
                manifest: "m.toml".to_string(),
                feature: "f".to_string()
            })
        );
    }

    #[test]
    fn a_dep_with_no_install_command_is_refused() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.cerulion.optional-system-deps.f]\n\
                 pkg-config = [\"m\"]\nsummary = \"s\"\nwithout-it = \"w\"\n\
                 install = {}\n";
        assert_eq!(
            parse_optional_system_deps("m.toml", m),
            Err(SystemDepError::NoInstallCommand {
                manifest: "m.toml".to_string(),
                feature: "f".to_string()
            })
        );
    }

    #[test]
    fn default_features_are_read_out_of_the_manifest() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [features]\ncdylib = []\ngstreamer = []\n\
                 default = [\"cdylib\"]\n";
        assert_eq!(
            declared_default_features("m.toml", m).expect("parses"),
            vec!["cdylib".to_string()]
        );
        // A manifest with no [features] table has no defaults.
        assert_eq!(
            declared_default_features("m.toml", "[package]\nname = \"n\"\nversion = \"0.1.0\"\n")
                .expect("parses"),
            Vec::<String>::new()
        );
        // Invalid TOML is reported, not silently treated as "no defaults" —
        // otherwise the guard that uses this would pass vacuously.
        assert!(matches!(
            declared_default_features("m.toml", "[package\nbroken"),
            Err(SystemDepError::Malformed { .. })
        ));
    }

    #[test]
    fn all_modules_present_enables_the_feature() {
        let r = evaluate(vec![camera_dep()], &|_m| Some("1.16.3".to_string()));
        assert_eq!(r.features_to_enable(), vec!["gstreamer".to_string()]);
        assert!(!r.has_missing());
    }

    /// PARTIAL presence must NOT enable the feature — linking half a library
    /// family is exactly the breakage this probe exists to avoid.
    #[test]
    fn one_missing_module_of_several_leaves_the_feature_off() {
        let r = evaluate(vec![camera_dep()], &|m| {
            if m == "gstreamer-1.0" {
                Some("1.16.3".to_string())
            } else {
                None
            }
        });
        assert_eq!(r.features_to_enable(), Vec::<String>::new());
        assert!(r.has_missing());
        assert_eq!(r.outcomes[0].missing_modules(), vec!["gstreamer-app-1.0"]);
    }

    #[test]
    fn nothing_present_leaves_the_feature_off_and_lists_every_missing_module() {
        let r = evaluate(vec![camera_dep()], &|_m| None);
        assert_eq!(r.features_to_enable(), Vec::<String>::new());
        assert_eq!(
            r.outcomes[0].missing_modules(),
            vec!["gstreamer-1.0", "gstreamer-app-1.0"]
        );
    }

    #[test]
    fn a_node_with_no_declared_deps_renders_nothing() {
        let r = SystemDepReport::default();
        assert!(r.is_empty());
        assert_eq!(
            render_report("plain_node", &r, &PkgConfigTool::Present, MACOS_KEYS),
            None
        );
    }

    /// The FOUND text, pinned verbatim — including the SCOPE clause. A
    /// resolved `.pc` file proves the headers are here, not that the feature's
    /// run-time components are: GStreamer's dev headers and its elements are
    /// separate packages, so a user who follows the notice's own install line
    /// on Debian gets `gstreamer` enabled and then a `MissingElements` failure
    /// at pipeline start. The notice must not promise more than pkg-config can
    /// answer.
    #[test]
    fn the_found_notice_names_the_feature_versions_and_scopes_what_it_proved() {
        let r = evaluate(vec![camera_dep()], &|_m| Some("1.16.3".to_string()));
        let text =
            render_report("camera_jpeg", &r, &PkgConfigTool::Present, MACOS_KEYS).expect("some");
        assert_eq!(
            text,
            "camera_jpeg: system dependency `gstreamer` FOUND \
             (gstreamer-1.0 1.16.3, gstreamer-app-1.0 1.16.3).\n  \
             Building WITH `--features gstreamer` — live camera capture \
             (H.264 decode -> JPEG) is enabled.\n  \
             (pkg-config resolved those modules: a BUILD-time fact. Any \
             run-time components the feature needs are checked when it runs.)\n"
        );
    }

    /// The MISSING text, pinned verbatim: the platform install line must be
    /// bare and copy-pasteable (no `[key]` prefix) when the key EXACTLY
    /// describes this machine.
    #[test]
    fn the_missing_notice_is_actionable_and_platform_specific() {
        let r = evaluate(vec![camera_dep()], &|_m| None);
        let text =
            render_report("camera_jpeg", &r, &PkgConfigTool::Present, MACOS_KEYS).expect("some");
        assert_eq!(
            text,
            "camera_jpeg: system dependency `gstreamer` NOT FOUND \
             (missing pkg-config module(s): gstreamer-1.0, gstreamer-app-1.0).\n  \
             Building WITHOUT `--features gstreamer`. The build will SUCCEED, \
             but live camera capture (H.264 decode -> JPEG) will be \
             unavailable:\n  \
             the node has no capture source, so `cerulion graph run` refuses \
             it at launch\n  \
             To enable it, install the library and re-run this command:\n      \
             brew install gstreamer\n"
        );
        // A Linux preference set gets the apt line, not the brew one.
        let linux = render_report(
            "camera_jpeg",
            &r,
            &PkgConfigTool::Present,
            &[InstallKeyPreference::exact("debian")],
        )
        .expect("some");
        assert!(
            linux.contains("sudo apt install libgstreamer1.0-dev"),
            "{linux}"
        );
        assert!(!linux.contains("brew"), "{linux}");
    }

    /// A GUESSED key is shown but LABELLED. `target_os = "linux"` is true on
    /// Fedora, and handing that user a bare `sudo apt install …` presents a
    /// Debian command as their machine's answer; they run it, get
    /// `apt: command not found`, and have no signal that the line was written
    /// for a different distribution. The `[debian]` marker is that signal.
    #[test]
    fn a_guessed_platform_key_is_shown_but_labelled_never_bare() {
        let r = evaluate(vec![camera_dep()], &|_m| None);
        let guessed = render_report(
            "camera_jpeg",
            &r,
            &PkgConfigTool::Present,
            &[
                InstallKeyPreference::guess("debian"),
                InstallKeyPreference::exact("linux"),
            ],
        )
        .expect("some");
        assert!(
            guessed.contains("      [debian] sudo apt install libgstreamer1.0-dev"),
            "a family-guess key must carry its label: {guessed}"
        );
        // Anti-tautology: the SAME key marked exact renders bare, so the label
        // tracks `exact` and not merely "a label is always emitted".
        let exact = render_report(
            "camera_jpeg",
            &r,
            &PkgConfigTool::Present,
            &[InstallKeyPreference::exact("debian")],
        )
        .expect("some");
        assert!(
            exact.contains("      sudo apt install libgstreamer1.0-dev"),
            "{exact}"
        );
        assert!(!exact.contains("[debian]"), "{exact}");
    }

    /// On a platform the manifest does not name, show EVERY command labelled —
    /// never silently show nothing.
    #[test]
    fn an_unknown_platform_lists_every_install_command_labelled() {
        let r = evaluate(vec![camera_dep()], &|_m| None);
        let text = render_report("camera_jpeg", &r, &PkgConfigTool::Present, &[]).expect("some");
        assert!(text.contains("[macos] brew install gstreamer"), "{text}");
        assert!(
            text.contains("[debian] sudo apt install libgstreamer1.0-dev"),
            "{text}"
        );
    }

    /// THE tool-vs-library pin. With `pkg-config` itself absent NOTHING was
    /// probed, so the notice must not assert the library is missing and must
    /// not lead with the library's install command — a user who already has
    /// GStreamer would run it, re-run the build, and get a byte-identical
    /// message with no path to the real fix.
    #[test]
    fn an_absent_pkg_config_tool_is_reported_as_the_tool_not_the_library() {
        let r = evaluate(vec![camera_dep()], &|_m| None);
        let text = render_report("camera_jpeg", &r, &PkgConfigTool::NotInstalled, MACOS_KEYS)
            .expect("some");
        assert!(
            text.contains("the `pkg-config` TOOL is not installed"),
            "{text}"
        );
        assert!(
            text.contains("This is not evidence that the library is missing."),
            "{text}"
        );
        // The TOOL's remedy comes first, and it is pkgconf — not gstreamer.
        let tool_at = text
            .find("brew install pkgconf")
            .unwrap_or_else(|| panic!("the tool's install line must appear: {text}"));
        let lib_at = text
            .find("brew install gstreamer")
            .unwrap_or_else(|| panic!("the library's line must still appear: {text}"));
        assert!(
            tool_at < lib_at,
            "the tool's remedy must precede the library's: {text}"
        );
        // It must NOT claim the modules were found missing — that verdict was
        // never reached.
        assert!(!text.contains("NOT FOUND"), "{text}");
        assert!(text.contains("NOT PROVEN"), "{text}");
        // Anti-tautology: the SAME report with a working tool says the
        // opposite, so the arm is selected by `tool` and not by the report.
        let probed =
            render_report("camera_jpeg", &r, &PkgConfigTool::Present, MACOS_KEYS).expect("some");
        assert!(probed.contains("NOT FOUND"), "{probed}");
        assert!(!probed.contains("pkgconf"), "{probed}");
    }

    /// A tool that exists but cannot RUN is a third state, and the notice
    /// quotes the OS error rather than inventing a cause.
    #[test]
    fn an_unusable_pkg_config_tool_names_the_os_error() {
        let r = evaluate(vec![camera_dep()], &|_m| None);
        let text = render_report(
            "camera_jpeg",
            &r,
            &PkgConfigTool::Unusable("permission denied (os error 13)".to_string()),
            MACOS_KEYS,
        )
        .expect("some");
        assert!(
            text.contains("could not be run (permission denied (os error 13))"),
            "{text}"
        );
        assert!(!text.contains("is not installed"), "{text}");
    }

    #[test]
    fn install_lines_prefers_the_first_matching_key_in_order() {
        let linux_prefs = &[
            InstallKeyPreference::guess("debian"),
            InstallKeyPreference::guess("ubuntu"),
            InstallKeyPreference::exact("linux"),
        ];
        let install = BTreeMap::from([
            ("linux".to_string(), "generic".to_string()),
            ("debian".to_string(), "apt".to_string()),
        ]);
        // `debian` is preferred over `linux`, even though `linux` sorts later
        // — and it is LABELLED, because it is a guess.
        assert_eq!(
            install_lines(&install, linux_prefs),
            vec![(Some("debian".to_string()), "apt".to_string())]
        );
        // With only `linux` declared, the later — and EXACT — preference hits,
        // so the line is bare.
        let only_linux = BTreeMap::from([("linux".to_string(), "generic".to_string())]);
        assert_eq!(
            install_lines(&only_linux, linux_prefs),
            vec![(None, "generic".to_string())]
        );
    }

    /// The SHIPPED preference list, exercised against the SHIPPED manifest's
    /// install table — the combination production actually runs, and the one
    /// every other rendering test dodges by hand-writing its keys. A wrong
    /// platform key on EITHER side (`darwin` for `macos` in the code, or in the
    /// manifest) leaves every hand-keyed test green while every real user falls
    /// through to the labelled-everything degrade.
    ///
    /// Reads the real `Cargo.toml`; a hand copy would make the manifest half of
    /// that mutation invisible.
    #[test]
    fn the_shipped_preference_list_resolves_the_shipped_manifests_install_table() {
        let lines = install_lines(&shipped_camera_dep().install, preferred_install_keys());
        #[cfg(target_os = "macos")]
        assert_eq!(
            lines,
            vec![(None, "brew install gstreamer".to_string())],
            "macOS must get the bare brew line (the `macos` key, exact)"
        );
        #[cfg(target_os = "linux")]
        assert_eq!(
            lines,
            vec![(
                Some("debian".to_string()),
                "sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev".to_string()
            )],
            "Linux must get the apt line LABELLED (target_os cannot prove Debian)"
        );
        #[cfg(not(any(target_os = "macos", target_os = "linux")))]
        assert_eq!(
            lines.len(),
            2,
            "an unknown platform lists everything: {lines:?}"
        );
    }

    /// The shipped preference list must be non-empty on the two platforms we
    /// build on, or the notice degrades to the labelled-everything form.
    #[test]
    fn the_running_platform_has_a_preference_on_macos_and_linux() {
        #[cfg(any(target_os = "macos", target_os = "linux"))]
        assert!(!preferred_install_keys().is_empty());
        #[cfg(not(any(target_os = "macos", target_os = "linux")))]
        assert!(preferred_install_keys().is_empty());
    }

    // -----------------------------------------------------------------------
    // Multi-dep: every evaluate/render oracle above uses exactly ONE dep, so
    // `take(1)` (or a reversed order) in `evaluate` was a free mutation.
    // -----------------------------------------------------------------------

    #[test]
    fn two_deps_are_both_probed_and_only_the_satisfied_one_is_enabled() {
        let r = evaluate(vec![camera_dep(), realsense_dep()], &|m| {
            (m == "realsense2").then(|| "2.55.1".to_string())
        });
        assert_eq!(r.outcomes.len(), 2, "BOTH deps must be probed: {r:?}");
        assert_eq!(r.features_to_enable(), vec!["realsense".to_string()]);
        assert!(r.has_missing());
        // Feature-NAME order, which is what `features_to_enable`'s doc now
        // says (a `BTreeMap` sorts the parse; `evaluate` preserves it).
        let both = evaluate(vec![camera_dep(), realsense_dep()], &|_m| {
            Some("1".to_string())
        });
        assert_eq!(
            both.features_to_enable(),
            vec!["gstreamer".to_string(), "realsense".to_string()],
            "the enabled set is deterministic and feature-name ordered"
        );
        // The rendered notice carries a section per dep, in the same order.
        let text = render_report("multi", &r, &PkgConfigTool::Present, MACOS_KEYS).expect("some");
        let gst_at = text
            .find("`gstreamer` NOT FOUND")
            .expect("gstreamer section");
        let rs_at = text.find("`realsense` FOUND").expect("realsense section");
        assert!(gst_at < rs_at, "sections follow outcome order: {text}");
    }

    /// The multi-dep parse round trip: two blocks in ONE manifest both arrive,
    /// sorted by feature name.
    #[test]
    fn a_manifest_declaring_two_deps_parses_both() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [features]\nzlib = []\nalsa = []\n\n\
                 [package.metadata.cerulion.optional-system-deps.zlib]\n\
                 pkg-config = [\"zlib\"]\nsummary = \"s\"\nwithout-it = \"w\"\n\
                 install.macos = \"c\"\n\n\
                 [package.metadata.cerulion.optional-system-deps.alsa]\n\
                 pkg-config = [\"alsa\"]\nsummary = \"s2\"\nwithout-it = \"w2\"\n\
                 install.macos = \"c2\"\n";
        let got = parse_optional_system_deps("m.toml", m).expect("parses");
        assert_eq!(
            got.iter().map(|d| d.feature.as_str()).collect::<Vec<_>>(),
            vec!["alsa", "zlib"],
            "two blocks both parse, feature-name ordered"
        );
    }

    // -----------------------------------------------------------------------
    // The table name must be a REAL cargo feature.
    // -----------------------------------------------------------------------

    /// THE drift pin. The table name IS the feature to enable, and nothing
    /// checked it: renaming `[features] gstreamer` to `capture` while leaving
    /// the metadata table alone stays green on every machine WITHOUT the
    /// library (probe Missing ⇒ no `--features` appended) and fails only on the
    /// machine that HAS it — where the notice first announces the feature is
    /// enabled and cargo then refuses with "none of the selected packages
    /// contains these features". I.e. green in CI, broken on the robot.
    #[test]
    fn a_table_naming_a_feature_the_crate_does_not_have_is_refused() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [features]\ncdylib = []\ncapture = []\n\n\
                 [package.metadata.cerulion.optional-system-deps.gstreamer]\n\
                 pkg-config = [\"gstreamer-1.0\"]\nsummary = \"s\"\n\
                 without-it = \"w\"\ninstall.macos = \"c\"\n";
        match parse_optional_system_deps("Cargo.toml", m) {
            Err(SystemDepError::UnknownFeature {
                manifest,
                feature,
                known,
            }) => {
                assert_eq!(manifest, "Cargo.toml", "name where to look");
                assert_eq!(feature, "gstreamer");
                assert_eq!(known, "capture, cdylib", "list what the crate does have");
            }
            other => panic!("expected UnknownFeature, got {other:?}"),
        }
    }

    /// An OPTIONAL DEPENDENCY is a feature too — cargo mints an implicit one
    /// per optional dep, so a manifest that gates on `dep:`-less optional deps
    /// must NOT be refused. Covers the top-level and `[target.<cfg>]` forms.
    #[test]
    fn an_optional_dependency_counts_as_a_declarable_feature() {
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [dependencies]\nlibusb1-sys = { version = \"0.7\", optional = true }\n\
                 serde = \"1\"\n\n\
                 [target.'cfg(unix)'.dependencies]\nnix = { version = \"0.29\", optional = true }\n\n\
                 [package.metadata.cerulion.optional-system-deps.libusb1-sys]\n\
                 pkg-config = [\"libusb-1.0\"]\nsummary = \"s\"\n\
                 without-it = \"w\"\ninstall.macos = \"c\"\n\n\
                 [package.metadata.cerulion.optional-system-deps.nix]\n\
                 pkg-config = [\"nix\"]\nsummary = \"s\"\nwithout-it = \"w\"\n\
                 install.macos = \"c\"\n";
        let got = parse_optional_system_deps("Cargo.toml", m).expect("optional deps are features");
        assert_eq!(got.len(), 2);
        // A NON-optional dependency is NOT a feature — the anti-tautology half
        // (otherwise the check would accept every dependency name).
        let bad = m.replace("optional-system-deps.nix", "optional-system-deps.serde");
        assert!(
            matches!(
                parse_optional_system_deps("Cargo.toml", &bad),
                Err(SystemDepError::UnknownFeature { .. })
            ),
            "a plain (non-optional) dependency mints no feature"
        );
    }

    /// Presence is not content. Serde proves the keys exist; a blank value
    /// renders a notice naming no capability, no consequence and no command —
    /// exactly the unactionable state the required-fields rule exists to
    /// prevent, and unreachable by the repo walk for a user's own node.
    #[test]
    fn a_blank_required_field_is_refused_by_name() {
        // (the field blanked, the line that blanks it)
        for (field, line) in [
            (
                "summary",
                "summary = \"\"\nwithout-it = \"w\"\ninstall.macos = \"c\"\n",
            ),
            (
                "without-it",
                "summary = \"s\"\nwithout-it = \"   \"\ninstall.macos = \"c\"\n",
            ),
            (
                "install.macos",
                "summary = \"s\"\nwithout-it = \"w\"\ninstall.macos = \"\"\n",
            ),
        ] {
            let m = format!(
                "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [features]\nf = []\n\n\
                 [package.metadata.cerulion.optional-system-deps.f]\n\
                 pkg-config = [\"m\"]\n{line}"
            );
            match parse_optional_system_deps("Cargo.toml", &m) {
                Err(SystemDepError::BlankField { field: got, .. }) => {
                    assert_eq!(got, field, "the blank field must be named")
                }
                other => panic!("blank `{field}` must be refused, got {other:?}"),
            }
        }
        // A blank MODULE name can never resolve, so it is refused too.
        let m = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [features]\nf = []\n\n\
                 [package.metadata.cerulion.optional-system-deps.f]\n\
                 pkg-config = [\"m\", \"\"]\nsummary = \"s\"\nwithout-it = \"w\"\n\
                 install.macos = \"c\"\n";
        assert!(matches!(
            parse_optional_system_deps("Cargo.toml", m),
            Err(SystemDepError::BlankField { .. })
        ));
    }

    // -----------------------------------------------------------------------
    // The `cerulion` TABLE NAME near-miss guard.
    // -----------------------------------------------------------------------

    fn metadata_of(manifest: &str) -> toml::Value {
        let doc: toml::Value = toml::from_str(manifest).expect("valid TOML");
        doc.get("package")
            .and_then(|p| p.get("metadata"))
            .cloned()
            .unwrap_or_else(|| toml::Value::Table(Default::default()))
    }

    /// Oracle vectors: the spellings a user actually produces are caught, and
    /// unrelated tool tables stay silent. Over-eager matching here would warn
    /// on every manifest carrying a `docs.rs` block, which trains people to
    /// ignore the warning.
    #[test]
    fn near_miss_table_names_are_caught_and_foreign_tables_stay_silent() {
        for near in [
            "cerulions",       // pluralised
            "ceruleon",        // one substitution
            "celurion",        // transposition
            "Cerulion",        // wrong case
            "cerulion-deps",   // suffixed
            "cerulion_config", // suffixed, other separator
        ] {
            let m = format!(
                "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.{near}.optional-system-deps.f]\n\
                 pkg-config = [\"m\"]\n"
            );
            assert_eq!(
                near_miss_cerulion_keys(&metadata_of(&m)),
                vec![near.to_string()],
                "`{near}` must be reported as a near miss"
            );
        }
        for foreign in ["docs", "cargo-machete", "playground", "deb", "wasm-pack"] {
            let m = format!(
                "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.{foreign}]\nkey = true\n"
            );
            assert!(
                near_miss_cerulion_keys(&metadata_of(&m)).is_empty(),
                "`{foreign}` is an unrelated tool table and must stay silent"
            );
        }
        // The correct spelling is never its own near miss.
        let ok = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                  [package.metadata.cerulion]\n";
        assert!(near_miss_cerulion_keys(&metadata_of(ok)).is_empty());
        // A non-table `metadata` yields nothing rather than panicking.
        assert!(near_miss_cerulion_keys(&toml::Value::Integer(1)).is_empty());
    }

    /// The guard is only worth having if the PARSE path fires it — the key is
    /// skipped silently otherwise, and the node ships without its feature
    /// under a green build forever.
    #[traced_test]
    #[test]
    fn a_misspelled_table_name_warns_loudly_naming_both_spellings() {
        let typo = "[package]\nname = \"n\"\nversion = \"0.1.0\"\n\n\
                    [package.metadata.ceruleon.optional-system-deps.gstreamer]\n\
                    pkg-config = [\"gstreamer-1.0\"]\nsummary = \"s\"\n\
                    without-it = \"w\"\ninstall.macos = \"c\"\n";
        // Still not an error: `package.metadata` is a shared namespace.
        assert_eq!(
            parse_optional_system_deps("Cargo.toml", typo).expect("parses"),
            vec![]
        );
        assert!(logs_contain("ceruleon"), "the found spelling must be named");
        assert!(
            logs_contain("cerulion"),
            "the expected spelling must be named"
        );
    }

    /// Anti-tautology for the warn above: an unrelated `package.metadata`
    /// table must produce NO near-miss warning.
    #[traced_test]
    #[test]
    fn an_unrelated_metadata_table_produces_no_near_miss_warning() {
        let m = "[package]\nname = \"x\"\nversion = \"0.1.0\"\n\n\
                 [package.metadata.docs.rs]\nall-features = true\n";
        assert_eq!(
            parse_optional_system_deps("Cargo.toml", m).expect("parses"),
            vec![]
        );
        assert!(!logs_contain("looks like a misspelling"));
    }

    // -----------------------------------------------------------------------
    // The "must not be in `default`" contract.
    // -----------------------------------------------------------------------

    #[test]
    fn a_gated_feature_left_in_default_is_reported() {
        let deps = vec![camera_dep(), realsense_dep()];
        assert_eq!(
            gated_features_in_default(&deps, &["cdylib".to_string(), "gstreamer".to_string()]),
            vec!["gstreamer"],
            "only the offender is named"
        );
        // The compliant shape reports nothing (anti-tautology).
        assert!(gated_features_in_default(&deps, &["cdylib".to_string()]).is_empty());
        // Both offenders are named, in declaration order.
        assert_eq!(
            gated_features_in_default(&deps, &["realsense".to_string(), "gstreamer".to_string()]),
            vec!["gstreamer", "realsense"]
        );
    }
}
