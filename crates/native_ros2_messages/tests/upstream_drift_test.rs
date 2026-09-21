// SPDX-License-Identifier: AGPL-3.0-only
//! The vendored `.msg` corpus must not silently drift from upstream
//! ROS 2.
//!
//! Upstream is pinned PER PACKAGE, recorded in the manifest's `!source`
//! lines. Of the 22 vendored packages: 20 are pinned to a ROS 2 distro
//! (19 Jazzy — 15 via the repo's `jazzy` branch, 4 via a ros2-gbp
//! `release/jazzy/...` tag — and `control_msgs` to **Lyrical**, because 13
//! of its 39 messages do not exist in Jazzy at all); the remaining 2
//! (`autoware_perception_msgs`, `autoware_planning_msgs`) have NO distro
//! pin at all and are recorded `UNVERIFIED` against `main`. Do not read
//! "Jazzy" as covering the whole corpus — it does not.
//!
//! Only 5 of those 22 pins are IMMUTABLE (the ros2-gbp `release/...` tags).
//! The other 17 name a moving branch, so two refreshes months apart can
//! emit byte-identical `!source` lines over different upstream states; the
//! manifest's git history, not the `!source` line, is what dates a
//! branch-pinned signature.
//!
//! # Why this gate exists
//!
//! Cerulion resolves message layout by **schema hash**, computed over the
//! field list. A stock ROS 2 robot's hash is computed over the REAL upstream
//! field list; ours over our hand-transcribed copy. When they disagree,
//! `walk_by_hash` refuses at the hash gate *before framing is consulted* —
//! the topic renders nothing and the user is told nothing. That is exactly
//! how `visualization_msgs/Marker` shipped with 15 fields against upstream
//! Jazzy's 19 from the original import until this gate was added.
//!
//! The pre-existing tests could not catch it by construction:
//! `layout_equivalence_test` and `schema_hash_pin_test` both diff the
//! vendored tree (or its generated constants) **against itself**, so a
//! self-consistent transcription error is invisible to them. This file is
//! the only test that compares the corpus against an EXTERNAL truth.
//!
//! # Design: hermetic, snapshot-based, fail-closed
//!
//! CI has no network and no ROS install, so the gate cannot fetch upstream.
//! Instead `upstream_msg_manifest.txt` is a CHECKED-IN snapshot of the
//! normalized upstream signature of every vendored message, together with
//! the exact provenance (repo + ref) each signature came from. This test
//! recomputes our side and compares.
//!
//! **What it catches:** any edit to the vendored corpus that moves it away
//! from the recorded upstream signature — a dropped field, a retyped field,
//! a reordered field, a changed or missing constant, a newly vendored file
//! with no recorded upstream, a deleted file.
//!
//! **What it CANNOT catch:** upstream itself changing
//! after the snapshot was taken. Detecting that requires network and is the
//! job of the deliberate refresh (`scripts/refresh_upstream_msg_manifest.sh`),
//! whose output is a reviewable diff of this manifest. The tradeoff is
//! intentional: it makes the corpus immutable-without-review in CI, and
//! makes "we adopted an upstream change" an explicit, reviewed act rather
//! than an invisible one.
//!
//! **Fail-closed:** a missing, unparseable, or empty manifest FAILS. A
//! vendored message with no manifest entry FAILS. A manifest entry whose
//! vendored file vanished FAILS. A waiver that no longer corresponds to a
//! real divergence FAILS, INCLUDING one whose message is neither vendored
//! nor in the manifest (so waivers cannot accumulate as dead weight, and
//! cannot lie in wait to pre-authorise a future real divergence). There is
//! no path on which "cannot verify" is reported as success.
//!
//! # Waiver granularity: FIELDS are always compared
//!
//! A waiver is scoped, not a blanket mute, because the blanket form is what
//! hid `control_msgs/VDA5050State`'s `string`-vs-`uint32` fork:
//!
//! * `!accept <pkg/Name> <reason>` waives ONLY the CONSTANT lines. The
//!   field list — the thing that reaches `MessageSchema::schema_hash` and
//!   therefore decides whether a stock robot's frames are accepted — is
//!   still compared, and a field divergence under a bare `!accept` FAILS.
//! * `!accept-fields <pkg/Name> <reason>` waives the field list too. It is
//!   the only directive that can silence the wire-affecting class, so it
//!   must be written deliberately and its reason must say why. A message
//!   with NO recorded upstream signature at all (upstream does not have it)
//!   compares nothing, fields included, so it requires `!accept-fields`.
//!
//! A "constants only" justification can therefore never silence field drift
//! by accident: the directive that permits it is spelled differently.
//!
//! # Normalization (ONE implementation, applied to BOTH sides)
//!
//! Both our text and the recorded upstream text are reduced by the same
//! function, so the comparison is about layout-relevant content only:
//!
//! * Field types come from `parse_rosmsg`, i.e. the REAL production parser,
//!   so its alias collapsing is inherited rather than re-implemented:
//!   `byte`/`char` -> `uint8`, `wstring` -> `string`, `time`/`duration` ->
//!   `builtin_interfaces/{Time,Duration}`, and a bounded `T[<=N]` -> `T[]`
//!   (the bound is an upper limit, not a layout fact).
//! * Same-package nested references are QUALIFIED on both sides — a
//!   normalization this gate is structurally blind to, so it is paired with
//!   a separate zero-tolerance gate rather than left to trust. See
//!   `normalize_type` for the full statement and
//!   `the_vendored_corpus_declares_no_bare_nested_refs` for what pins it.
//!   Briefly: upstream `.msg` text writes intra-package references bare
//!   (`Point`) by ROS convention, so NOT collapsing would report every such
//!   upstream file as drifted. Bare and qualified hash DIFFERENTLY and
//!   nothing in the pipeline normalizes them, so the collapse could hide a
//!   real wire skew — except that our side is now provably never bare
//!   (the qualification pass qualified all 80, and the zero-tolerance gate keeps it
//!   that way). A bare `Header` qualifies to `std_msgs/Header`, matching the
//!   ROS1-ism our resolver honours.
//! * Comments and whitespace are dropped — they carry no layout meaning.
//! * Constants ARE compared, even though they do not reach `MessageSchema`
//!   and so cannot affect the hash: they are user-visible in `cerulion
//!   schema info`, and a wrong constant type (Marker's were `uint8` where
//!   upstream says `int32`) is a real fidelity defect worth gating.
//!
//! # Refreshing
//!
//! ```text
//! ./scripts/refresh_upstream_msg_manifest.sh          # re-fetch + rewrite
//! git diff native_ros2_messages/upstream_msg_manifest.txt   # review it
//! ```

use cerulion_core::codegen::{parse_rosmsg, FieldType};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

const MANIFEST: &str = "upstream_msg_manifest.txt";

// ─────────────────────────── normalization ───────────────────────────

/// Fully-qualified canonical rendering of a field type.
///
/// # THE ONE NORMALIZATION THIS GATE CANNOT SEE THROUGH
///
/// Every other reduction this file performs is either the production
/// parser's own aliasing or provably wire-neutral. This one is NOT: it
/// collapses a bare (same-package) nested reference onto the qualified
/// form, and the two **hash differently**.
///
/// The collapse is applied to BOTH sides, so it cannot report a false
/// divergence — but it also means this gate is structurally incapable of
/// seeing bare-vs-qualified skew in either direction. Nothing in the
/// pipeline normalizes them (`resolve_fixed_nested`'s rewrite phase mutates
/// only the `fixed` flag; `canonical_str` renders the bare name straight
/// into `schema_hash`), while `rmw_cerulion` always emits the qualified
/// form (it reads the rosidl introspection namespace). A bare ref in our
/// corpus would therefore hash differently from what an rmw publisher
/// produces -> `UnknownSchemaHash` -> the topic renders nothing, silently:
/// the same class this gate exists to close.
///
/// WHY IT IS COLLAPSED HERE ANYWAY: upstream `.msg` text writes
/// intra-package refs bare by ROS convention, so NOT collapsing would make
/// this gate report every such upstream file as drifted, drowning the
/// field-list class it exists to catch in convention noise.
///
/// WHY THAT IS SAFE NOW: the collapse can only hide a divergence in which
/// one side is bare, and OUR side never is. The qualification pass qualified all 80
/// bare declarations across 48 of the 254 vendored files — a wire-affecting
/// change (48 native schema-hash bumps) made only after a live rmw capture
/// confirmed the qualified hash is exactly what a stock robot emits — and
/// `the_vendored_corpus_declares_no_bare_nested_refs` holds the line at
/// ZERO: a newly vendored or newly edited bare ref fails loudly.
fn normalize_type(ft: &FieldType, pkg: &str) -> String {
    match ft {
        FieldType::FixedArray {
            element_type,
            length,
        } => format!("{}[{}]", normalize_type(element_type, pkg), length),
        FieldType::DynamicArray { element_type } => {
            format!("{}[]", normalize_type(element_type, pkg))
        }
        FieldType::Nested {
            schema_name,
            package,
            ..
        } => match package {
            Some(p) => format!("{p}/{schema_name}"),
            // Bare reference: same package, except `Header`, which ROS1
            // allowed unqualified and our resolver still binds to std_msgs.
            None if schema_name == "Header" => "std_msgs/Header".to_string(),
            None => format!("{pkg}/{schema_name}"),
        },
        other => other.canonical_str(),
    }
}

/// One message's layout-relevant signature: field lines then constant lines.
///
/// Field lines are `f <type> <name>`; constant lines are `c <type> <NAME>=<value>`.
/// Field ORDER is preserved (it is layout); constants are sorted by name
/// (their declaration position carries no meaning).
fn signature(text: &str, msg: &str, pkg: &str) -> Result<Vec<String>, String> {
    let schema = parse_rosmsg(text, msg, Some(pkg)).map_err(|e| format!("{e:?}"))?;
    let mut out: Vec<String> = schema
        .fields
        .iter()
        .map(|f| format!("f {} {}", normalize_type(&f.field_type, pkg), f.name))
        .collect();
    let mut consts = parse_constants(text);
    consts.sort();
    out.extend(consts);
    Ok(out)
}

/// Scan constant declarations (`TYPE NAME=VALUE`) out of raw `.msg` text.
///
/// `parse_rosmsg` deliberately SKIPS constants (they are not fields and do
/// not reach the wire), so they have to be read here. A string constant's
/// value runs to end of line and may contain `#`, so comment stripping is
/// conditional on the declared type.
fn parse_constants(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    for raw in text.lines() {
        let is_str = {
            let t = raw.trim_start();
            t.starts_with("string ") || t.starts_with("wstring ")
        };
        let line = if is_str {
            raw.trim_end()
        } else {
            raw.split('#').next().unwrap_or("").trim_end()
        };
        let line = line.trim();
        let Some(eq) = line.find('=') else { continue };
        let (decl, value) = line.split_at(eq);
        let value = &value[1..];
        let mut parts = decl.split_whitespace();
        let (Some(ty), Some(name)) = (parts.next(), parts.next()) else {
            continue;
        };
        if parts.next().is_some() {
            continue; // not a `TYPE NAME=VALUE` shape
        }
        // A constant name is SCREAMING_CASE by ROS convention; anything else
        // is a field with a default value, which is not a constant.
        if name.is_empty()
            || !name
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            continue;
        }
        // Collapse the same primitive aliases the parser collapses, so a
        // `byte`/`uint8` spelling difference is not reported as drift.
        let ty = match ty {
            "byte" | "char" => "uint8",
            "wstring" => "string",
            other => other,
        };
        out.push(format!("c {ty} {name}={}", value.trim()));
    }
    out
}

// ─────────────────────────── manifest model ───────────────────────────

/// How much of a message's signature a `!accept` line is allowed to silence.
///
/// The split exists because a blanket waiver is what hid the original
/// `VDA5050State` fork: its stated reason was about the message being
/// absent from the pinned distro, and that reason silently also covered a
/// `string`-vs-`uint32` FIELD divergence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WaiverScope {
    /// `!accept` — constants only. Field drift under this still FAILS.
    ConstantsOnly,
    /// `!accept-fields` — the field list too. The only directive that can
    /// silence a wire-affecting divergence.
    Fields,
}

#[derive(Debug, Clone)]
struct Waiver {
    scope: WaiverScope,
    /// Read by `manifest_parser_fails_closed_on_malformed_input`, which pins
    /// that a waiver's justification survives parsing intact — the reason is
    /// the whole record of WHY a divergence was accepted.
    reason: String,
}

/// The `!source` trust token, as a CLOSED set.
///
/// Parsed rather than stored opaquely so `every_vendored_package_has_recorded_provenance`
/// can actually make the distinction its failure message claims — an
/// unverifiable package must not be indistinguishable from a verified one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Trust {
    /// Pinned to a distro branch (e.g. `jazzy`) — MUTABLE: the branch moves.
    DistroBranch,
    /// Pinned to a ros2-gbp `release/<distro>/...` tag — IMMUTABLE.
    ReleaseTag,
    /// No distro pin exists for this package at all.
    Unverified,
}

#[derive(Debug, Clone)]
struct Provenance {
    trust: Trust,
    /// The `<owner/repo>@<ref>` spec. NOT decorative: it is the AUTHORITATIVE
    /// half of the CI cross-check against the refresh script's `clone` table
    /// (`refresh_script_clone_refs_match_the_manifest_source_pins`), which is
    /// what stops the two copies of each distro pin from drifting apart.
    source: String,
}

#[derive(Default)]
struct Manifest {
    /// `pkg` -> parsed provenance
    provenance: BTreeMap<String, Provenance>,
    /// `pkg/Name` -> recorded upstream signature lines
    messages: BTreeMap<String, Vec<String>>,
    /// `pkg/Name` -> the scoped waiver for a divergence from upstream
    accepted: BTreeMap<String, Waiver>,
}

fn parse_manifest(text: &str) -> Result<Manifest, String> {
    let mut m = Manifest::default();
    let mut current: Option<String> = None;
    for (i, raw) in text.lines().enumerate() {
        let line = raw.trim_end();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let lineno = i + 1;
        if let Some(rest) = line.strip_prefix("!source ") {
            let (pkg, desc) = rest
                .split_once(' ')
                .ok_or_else(|| format!("line {lineno}: !source needs '<pkg> <trust> <desc>'"))?;
            let (token, source) = desc.split_once(' ').ok_or_else(|| {
                format!("line {lineno}: !source needs '<pkg> <trust> <desc>' (no trust token)")
            })?;
            let trust = match token {
                "release" => Trust::ReleaseTag,
                "UNVERIFIED" => Trust::Unverified,
                // Any other token names the distro BRANCH it is pinned to.
                t if !t.is_empty() && t.chars().all(|c| c.is_ascii_lowercase() || c == '_') => {
                    Trust::DistroBranch
                }
                other => {
                    return Err(format!(
                        "line {lineno}: unrecognized !source trust token '{other}' \
                         (expected 'release', 'UNVERIFIED', or a lowercase distro name)"
                    ))
                }
            };
            m.provenance.insert(
                pkg.to_string(),
                Provenance {
                    trust,
                    source: source.to_string(),
                },
            );
        // NOTE: `!accept-fields` must be tested BEFORE `!accept ` cannot match
        // it (the prefixes are disjoint thanks to the trailing space), but
        // ordering is kept explicit so a future edit cannot silently widen one.
        } else if let Some(rest) = line.strip_prefix("!accept-fields ") {
            let (k, why) = rest.split_once(' ').ok_or_else(|| {
                format!("line {lineno}: !accept-fields needs '<pkg/Name> <reason>'")
            })?;
            m.accepted.insert(
                k.to_string(),
                Waiver {
                    scope: WaiverScope::Fields,
                    reason: why.to_string(),
                },
            );
        } else if let Some(rest) = line.strip_prefix("!accept ") {
            let (k, why) = rest
                .split_once(' ')
                .ok_or_else(|| format!("line {lineno}: !accept needs '<pkg/Name> <reason>'"))?;
            m.accepted.insert(
                k.to_string(),
                Waiver {
                    scope: WaiverScope::ConstantsOnly,
                    reason: why.to_string(),
                },
            );
        } else if line.starts_with("!not-vendored") {
            // REMOVED. The directive was self-destructing:
            // it only ever suppressed an orphan signature block, but the
            // refresh regenerates the body strictly from the vendored tree,
            // so the block it suppressed can never be re-emitted — one
            // refresh made every `!not-vendored` line permanently inert.
            // Rejected loudly rather than ignored, so a legacy line cannot
            // sit in a manifest doing nothing while looking meaningful.
            return Err(format!(
                "line {lineno}: '!not-vendored' was REMOVED — it had no durable effect. It \
                 existed to suppress the 'recorded upstream signature has no vendored .msg' \
                 error, but the refresh rebuilds the body from the vendored tree, so that block \
                 disappears on its own. Delete this line and re-run \
                 ./scripts/refresh_upstream_msg_manifest.sh. (Upstream messages we deliberately \
                 do not vendor are recorded as prose in the manifest header — they need no \
                 directive, because nothing in the manifest refers to them.)"
            ));
        } else if let Some(key) = line.strip_prefix('=') {
            if key.is_empty() || !key.contains('/') {
                return Err(format!("line {lineno}: bad message key '{key}'"));
            }
            if m.messages.insert(key.to_string(), Vec::new()).is_some() {
                return Err(format!("line {lineno}: duplicate message '{key}'"));
            }
            current = Some(key.to_string());
        } else if line.starts_with("f ") || line.starts_with("c ") {
            let key = current
                .as_ref()
                .ok_or_else(|| format!("line {lineno}: signature line before any '=<pkg/Name>'"))?;
            m.messages
                .get_mut(key)
                .expect("current key present")
                .push(line.to_string());
        } else {
            return Err(format!("line {lineno}: unrecognized directive '{line}'"));
        }
    }
    if m.messages.is_empty() {
        return Err("manifest declares no messages — refusing to pass vacuously".to_string());
    }
    Ok(m)
}

// ─────────────────────────── corpus walk ───────────────────────────

fn msg_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("msg")
}

/// `(pkg, Name)` -> raw vendored `.msg` text, walked off disk.
fn vendored_corpus() -> BTreeMap<(String, String), String> {
    let root = msg_root();
    let mut out = BTreeMap::new();
    let mut pkgs: Vec<PathBuf> = std::fs::read_dir(&root)
        .unwrap_or_else(|e| panic!("read {}: {e}", root.display()))
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    pkgs.sort();
    for pkg_dir in pkgs {
        // Mirror build.rs's package-name sanitization.
        let pkg = pkg_dir
            .file_name()
            .expect("package dir name")
            .to_string_lossy()
            .replace('-', "_");
        let mut files: Vec<PathBuf> = std::fs::read_dir(&pkg_dir)
            .unwrap_or_else(|e| panic!("read {}: {e}", pkg_dir.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "msg"))
            .collect();
        files.sort();
        for f in files {
            let name = f
                .file_stem()
                .expect("msg file stem")
                .to_string_lossy()
                .to_string();
            let text = std::fs::read_to_string(&f).unwrap_or_else(|e| panic!("read {f:?}: {e}"));
            out.insert((pkg.clone(), name), text);
        }
    }
    out
}

fn manifest_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(MANIFEST)
}

// ─────────────────────────── the gate ───────────────────────────

/// Split a signature into its FIELD lines and its CONSTANT lines.
///
/// The two halves are adjudicated separately: fields are the wire, and a
/// waiver has to be spelled `!accept-fields` to silence them.
fn split_signature(sig: &[String]) -> (Vec<&String>, Vec<&String>) {
    sig.iter().partition(|l| l.starts_with("f "))
}

/// The PURE adjudication: given the vendored corpus and a parsed manifest,
/// which combinations of (vendored, signature block, waiver) are problems?
///
/// Extracted from the `#[test]` so its FAILURE branches can be driven with
/// synthetic input: run only against the real corpus, every one of these
/// branches is unreachable by construction (the tree is in the passing
/// state), so a broken branch would go uncaught without synthetic input.
fn adjudicate(corpus: &BTreeMap<(String, String), String>, m: &Manifest) -> Vec<String> {
    let mut problems: Vec<String> = Vec::new();
    let mut seen: BTreeSet<String> = BTreeSet::new();
    // Waivers that have not yet been shown to describe a REAL divergence.
    let mut unused_waivers: BTreeSet<String> = m.accepted.keys().cloned().collect();

    for ((pkg, name), our_text) in corpus {
        let key = format!("{pkg}/{name}");
        seen.insert(key.clone());
        let waiver = m.accepted.get(&key);

        let Some(upstream_sig) = m.messages.get(&key) else {
            // No upstream signature AT ALL: nothing is compared, fields
            // included. That is the strictly-widest silence a waiver can
            // buy, so it takes the field-scoped directive — a
            // constants-only justification must never reach here (that is
            // exactly how VDA5050State's fork hid).
            match waiver.map(|w| w.scope) {
                Some(WaiverScope::Fields) => {
                    unused_waivers.remove(&key);
                }
                Some(WaiverScope::ConstantsOnly) => {
                    unused_waivers.remove(&key);
                    problems.push(format!(
                        "{key}: has NO recorded upstream signature, but its waiver is a \
                         constants-only '!accept'. Absence silences the FIELD comparison too, \
                         so it cannot ride a constants-only justification. Either re-pin the \
                         package to the distro that actually ships this message (check its \
                         '!source' line — that is what the original drift turned out to be), or, if it is \
                         genuinely ours alone, say so explicitly with \
                         '!accept-fields {key} <reason>'."
                    ));
                }
                None => problems.push(format!(
                    "{key}: vendored but has NO recorded upstream signature. Most likely the \
                     package is pinned to a distro that does not ship this message — check its \
                     '!source' line and re-pin if so (that was the original drift's actual root cause), or \
                     re-run ./scripts/refresh_upstream_msg_manifest.sh if the pin is right and \
                     the snapshot is merely stale. Only if the message is genuinely ours alone, \
                     record '!accept-fields {key} <reason>' — that waiver compares NOTHING for \
                     this message, fields included, so it is the last resort, not the first."
                )),
            }
            continue;
        };

        let our_sig = match signature(our_text, name, pkg) {
            Ok(s) => s,
            Err(e) => {
                problems.push(format!("{key}: vendored .msg does not parse: {e}"));
                continue;
            }
        };

        let (our_fields, our_consts) = split_signature(&our_sig);
        let (up_fields, up_consts) = split_signature(upstream_sig);

        let fields_differ = our_fields != up_fields;
        let consts_differ = our_consts != up_consts;

        // A waiver is STALE only when the message has stopped diverging
        // ENTIRELY. If some half still differs, the waiver is answering a
        // live question — even where it does not cover that half, in which
        // case the uncovered drift is reported on its own terms below.
        // Reporting "your waiver is stale" alongside "this drift is not
        // covered by your waiver" would be actively misleading: the fix is
        // to address the drift, not to delete the waiver.
        if fields_differ || consts_differ {
            unused_waivers.remove(&key);
        }

        // FIELD drift needs the field-scoped directive; a bare `!accept`
        // does NOT cover it.
        if fields_differ && waiver.map(|w| w.scope) != Some(WaiverScope::Fields) {
            let note = if waiver.is_some() {
                " (its '!accept' waiver is CONSTANTS-ONLY and does not cover fields — if \
                 this divergence is genuinely intended, it must be re-declared as \
                 '!accept-fields', which states in the manifest that a wire-affecting \
                 difference is being accepted)"
            } else {
                ""
            };
            problems.push(format!(
                "{key}: FIELD list DRIFTED from upstream{note}{}",
                render_delta(
                    &our_fields.into_iter().cloned().collect::<Vec<_>>(),
                    &up_fields.into_iter().cloned().collect::<Vec<_>>(),
                )
            ));
        }

        // CONSTANT drift is covered by EITHER directive.
        if consts_differ && waiver.is_none() {
            problems.push(format!(
                "{key}: CONSTANTS drifted from upstream{}",
                render_delta(
                    &our_consts.into_iter().cloned().collect::<Vec<_>>(),
                    &up_consts.into_iter().cloned().collect::<Vec<_>>(),
                )
            ));
        }
    }

    for key in m.messages.keys() {
        if seen.contains(key) {
            continue;
        }
        problems.push(format!(
            "{key}: recorded upstream signature has no vendored .msg — a message was deleted \
             or renamed. If that was INTENDED, re-run \
             ./scripts/refresh_upstream_msg_manifest.sh: it rebuilds the body from the vendored \
             tree, so the stale block disappears and the manifest matches reality again. If it \
             was NOT intended, restore the .msg — this gate is telling you the corpus lost a \
             message."
        ));
    }

    for key in &unused_waivers {
        if seen.contains(key) {
            problems.push(format!(
                "{key}: waiver is STALE — the vendored text now matches upstream. Remove it so \
                 waivers cannot accumulate as dead weight."
            ));
        } else {
            // NOT VENDORED: the waiver's message is absent from the corpus,
            // so the per-message loop never reaches it and the waiver
            // excuses nothing. Left alone it is worse than dead weight — if
            // the message is ever re-vendored with a real fork, the waiver
            // is already sitting there to accept it in silence.
            //
            // Whether upstream still HAS the message decides the wording:
            // this branch only knows the message is not vendored, so it
            // must consult the manifest rather than assert either way.
            let where_it_stands = if m.messages.contains_key(key) {
                "its upstream signature is still recorded, but we no longer vendor the message, so \
                 nothing compares it"
            } else {
                "it is neither vendored nor present in the manifest, so nothing can ever check it"
            };
            problems.push(format!(
                "{key}: waiver is ORPHANED — {where_it_stands}. Remove it: left in place it \
                 silently pre-authorises a divergence if this message is re-vendored."
            ));
        }
    }

    problems
}

#[test]
fn vendored_corpus_matches_recorded_upstream_signatures() {
    let path = manifest_path();
    // FAIL CLOSED: an absent or unreadable manifest is a failure, never a skip.
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "upstream drift gate cannot read its manifest {}: {e}\n\
             This gate must fail closed — regenerate it with \
             ./scripts/refresh_upstream_msg_manifest.sh",
            path.display()
        )
    });
    let manifest = parse_manifest(&text)
        .unwrap_or_else(|e| panic!("drift manifest {} is unparseable: {e}", path.display()));

    let problems = adjudicate(&vendored_corpus(), &manifest);

    assert!(
        problems.is_empty(),
        "upstream drift: the vendored .msg corpus has drifted from the recorded upstream ROS 2 \
         signatures.\n\nA drifted field list means a stock robot's schema hash will not match \
         ours, and `walk_by_hash` then refuses the frame at the hash gate — the topic silently \
         renders nothing. In order of preference: fix the .msg file to match upstream; re-pin \
         the package's '!source' distro if upstream simply is not where we are looking; re-run \
         ./scripts/refresh_upstream_msg_manifest.sh and review the diff if upstream genuinely \
         changed. A waiver is the LAST resort — it buys silence, which is what hid the fork \
         this gate was built for.\n\n{}\n\n{} problem(s).",
        problems.join("\n\n"),
        problems.len()
    );
}

fn render_delta(ours: &[String], upstream: &[String]) -> String {
    let o: BTreeSet<&String> = ours.iter().collect();
    let u: BTreeSet<&String> = upstream.iter().collect();
    let mut s = String::new();
    for missing in u.difference(&o) {
        s.push_str(&format!("\n    MISSING (upstream has, we lack): {missing}"));
    }
    for extra in o.difference(&u) {
        s.push_str(&format!("\n    EXTRA   (we have, upstream lacks): {extra}"));
    }
    // Same set but different order => a field reordering, which IS layout.
    if s.is_empty() {
        s.push_str(&format!(
            "\n    ORDER differs (field order is layout):\n      ours:     {ours:?}\n      upstream: {upstream:?}"
        ));
    }
    s
}

/// Packages whose upstream truth has NO distro pin at all.
///
/// Declared here rather than merely tolerated: the point of the `!source`
/// trust token is that an unverifiable package must not be
/// indistinguishable from a verified one, and a token nothing ever reads
/// makes exactly that distinction unenforceable. Flipping any other package
/// to `UNVERIFIED` (or adding a new one) now fails loudly instead of
/// silently passing as an unremarked manifest diff.
const DECLARED_UNVERIFIED_PACKAGES: &[&str] =
    &["autoware_perception_msgs", "autoware_planning_msgs"];

/// Every vendored package must declare where its upstream truth came from,
/// and the set that is UNVERIFIED must be exactly the declared one.
#[test]
fn every_vendored_package_has_recorded_provenance() {
    let text = std::fs::read_to_string(manifest_path())
        .expect("upstream drift manifest must exist (fail closed)");
    let manifest = parse_manifest(&text).expect("upstream drift manifest must parse");
    let mut missing: Vec<String> = Vec::new();
    let mut unverified: BTreeSet<String> = BTreeSet::new();
    for (pkg, _) in vendored_corpus().keys().cloned().collect::<BTreeSet<_>>() {
        match manifest.provenance.get(&pkg) {
            None => missing.push(pkg),
            Some(p) => {
                if p.trust == Trust::Unverified {
                    unverified.insert(pkg);
                }
            }
        }
    }
    missing.dedup();
    assert!(
        missing.is_empty(),
        "upstream drift: these vendored packages have no '!source' provenance line in {MANIFEST}: {missing:?}\n\
         Record the upstream repo + ref each was verified against (or mark it UNVERIFIED \
         explicitly) — an unverifiable package must not be indistinguishable from a verified one."
    );

    let declared: BTreeSet<String> = DECLARED_UNVERIFIED_PACKAGES
        .iter()
        .map(|s| s.to_string())
        .collect();
    assert_eq!(
        unverified, declared,
        "upstream drift: the set of UNVERIFIED packages changed.\n\
         An UNVERIFIED '!source' means the package has no distro pin at all, so its recorded \
         signature is not evidence of anything a robot ships. That is a deliberate, declared \
         exception — not something a refresh may quietly widen. If a package genuinely has no \
         rosdistro entry, add it to DECLARED_UNVERIFIED_PACKAGES with that justification; if it \
         does have one, pin it."
    );
}

// ────────────── the vendored corpus carries NO bare refs ──────────────

/// Collect bare nested refs from one parsed schema, recursing through arrays.
fn collect_bare_refs(ft: &FieldType, field: &str, key: &str, out: &mut Vec<String>) {
    match ft {
        FieldType::FixedArray { element_type, .. } | FieldType::DynamicArray { element_type } => {
            collect_bare_refs(element_type, field, key, out)
        }
        FieldType::Nested {
            schema_name,
            package: None,
            ..
        } => out.push(format!("{key} {field} {schema_name}")),
        _ => {}
    }
}

/// ZERO bare (unqualified) nested references may exist in the vendored
/// corpus. There is no inventory and no waiver — the only allowed count is
/// zero.
///
/// The qualification pass qualified the 80 that used to be here (48 of the 254
/// files). That was wire-affecting — 48 native `schema_hash` bumps — and it
/// was made only after a live rmw capture measured, on 5 sampled types
/// across 4 of the 5 affected packages, that the QUALIFIED hash is exactly
/// what a stock `rclpy` publisher under `RMW_IMPLEMENTATION=rmw_cerulion`
/// puts on the wire (all 64 bits), while the bare hash matched nothing. Two
/// no-bare-ref controls hashed identically before and after, so the change
/// is attributable to the bare refs rather than to a harness artifact.
///
/// This test is what keeps that at zero. It is deliberately SEPARATE from
/// the upstream-drift gate, which cannot see this class at all:
/// `normalize_type` collapses bare and qualified on BOTH sides so that
/// upstream's bare intra-package convention does not drown the field-list
/// class it exists to catch.
#[test]
fn the_vendored_corpus_declares_no_bare_nested_refs() {
    let mut found: Vec<String> = Vec::new();
    for ((pkg, name), text) in &vendored_corpus() {
        let key = format!("{pkg}/{name}");
        let schema = match parse_rosmsg(text, name, Some(pkg)) {
            Ok(s) => s,
            // A non-parsing file is the drift gate's problem, not this one.
            Err(_) => continue,
        };
        for f in &schema.fields {
            collect_bare_refs(&f.field_type, &f.name, &key, &mut found);
        }
    }
    found.sort();

    assert!(
        found.is_empty(),
        "upstream drift: the vendored corpus grew {} BARE (unqualified) nested reference(s).\n\n\
         A bare `Foo` and a qualified `pkg/Foo` produce DIFFERENT schema hashes, and nothing in \
         the pipeline normalizes them — `resolve_fixed_nested` rewrites only the `fixed` flag, so \
         the bare name reaches `MessageSchema::schema_hash` verbatim. `rmw_cerulion` always emits \
         the QUALIFIED form (it reads the rosidl introspection namespace), so a message with a \
         bare ref hashes differently from what an rmw publisher produces: `walk_by_hash` returns \
         `UnknownSchemaHash` and the topic renders nothing, silently. The upstream-drift gate \
         CANNOT see this class — `normalize_type` collapses both spellings on both sides, by \
         design, so upstream's bare intra-package convention does not drown the field-list class \
         it exists to catch.\n\n\
         THE FIX IS ALWAYS THE SAME: qualify each of these as `pkg/Type` in its `.msg`, using the \
         package the resolver would bind (the parent's own package; `Header` binds CROSS-package \
         to `std_msgs/Header`). Do NOT add a waiver — this gate has none, deliberately.\n\n\
         BARE REFS FOUND (`<pkg>/<Msg> <field> <type>`):\n  {found:#?}",
        found.len()
    );
}

/// The normalizer itself, against hand-written oracles — so the gate's
/// verdicts rest on tested behaviour, not on the normalizer agreeing with
/// itself on both sides.
#[test]
fn normalizer_collapses_only_what_it_should() {
    // Bare and qualified same-package refs normalize together...
    let bare = signature("Point position\n", "Pose", "geometry_msgs").unwrap();
    let qual = signature("geometry_msgs/Point position\n", "Pose", "geometry_msgs").unwrap();
    assert_eq!(bare, vec!["f geometry_msgs/Point position"]);
    assert_eq!(bare, qual);

    // ...but a DIFFERENT package does not collapse into the parent's.
    let other = signature("shape_msgs/Point position\n", "Pose", "geometry_msgs").unwrap();
    assert_ne!(bare, other);

    // Bare `Header` binds to std_msgs, not the parent package.
    assert_eq!(
        signature("Header header\n", "X", "moveit_msgs").unwrap(),
        vec!["f std_msgs/Header header"]
    );

    // Primitive aliases collapse; a genuine signedness change does NOT.
    assert_eq!(
        signature("byte a\n", "X", "p").unwrap(),
        signature("uint8 a\n", "X", "p").unwrap()
    );
    assert_ne!(
        signature("int8 a\n", "X", "p").unwrap(),
        signature("uint8 a\n", "X", "p").unwrap()
    );

    // A bounded array is an upper limit, not a layout fact.
    assert_eq!(
        signature("float64[<=3] d\n", "X", "p").unwrap(),
        signature("float64[] d\n", "X", "p").unwrap()
    );
    // A FIXED-length array is a layout fact and must not collapse.
    assert_ne!(
        signature("float64[3] d\n", "X", "p").unwrap(),
        signature("float64[] d\n", "X", "p").unwrap()
    );

    // Field ORDER is preserved (it is layout).
    assert_ne!(
        signature("int32 a\nint32 b\n", "X", "p").unwrap(),
        signature("int32 b\nint32 a\n", "X", "p").unwrap()
    );

    // Comments carry no meaning; a MISSING FIELD does.
    assert_eq!(
        signature("# lead\nint32 a  # trail\n", "X", "p").unwrap(),
        signature("int32 a\n", "X", "p").unwrap()
    );
    assert_ne!(
        signature("int32 a\nint32 b\n", "X", "p").unwrap(),
        signature("int32 a\n", "X", "p").unwrap()
    );

    // Constants are captured, type-sensitive, and order-insensitive.
    assert_eq!(
        signature("int32 FOO=1\nint32 v\n", "X", "p").unwrap(),
        vec!["f int32 v", "c int32 FOO=1"]
    );
    assert_ne!(
        signature("uint8 FOO=1\nint32 v\n", "X", "p").unwrap(),
        signature("int32 FOO=1\nint32 v\n", "X", "p").unwrap()
    );
    assert_eq!(
        signature("int32 A=1\nint32 B=2\nint32 v\n", "X", "p").unwrap(),
        signature("int32 B=2\nint32 A=1\nint32 v\n", "X", "p").unwrap()
    );
    // A missing constant is drift.
    assert_ne!(
        signature("int32 A=1\nint32 v\n", "X", "p").unwrap(),
        signature("int32 v\n", "X", "p").unwrap()
    );
    // A string constant's value may legitimately contain '#'.
    assert_eq!(
        signature("string TAG=a#b\nint32 v\n", "X", "p").unwrap(),
        vec!["f int32 v", "c string TAG=a#b"]
    );
}

// ────────────── adjudication: the FAILURE branches, driven ──────────────
//
// Run only against the real corpus + real manifest, every branch below is
// unreachable — the tree is by construction in the passing state. That is
// why six failure paths would go uncaught without synthetic input (deleted-file
// detection, stale-waiver reporting, the no-signature `is_accepted` arm,
// `render_delta` itself, the directive parsing, and the malformed-
// directive arms). These tests drive `adjudicate` with SYNTHETIC input so
// each one is genuinely covered.

/// Build a one-message synthetic corpus.
fn corpus_of(entries: &[(&str, &str, &str)]) -> BTreeMap<(String, String), String> {
    entries
        .iter()
        .map(|(pkg, name, text)| ((pkg.to_string(), name.to_string()), text.to_string()))
        .collect()
}

fn assert_one_problem_containing(problems: &[String], needle: &str) {
    assert_eq!(
        problems.len(),
        1,
        "expected exactly one problem, got: {problems:#?}"
    );
    assert!(
        problems[0].contains(needle),
        "problem did not mention {needle:?}: {}",
        problems[0]
    );
}

/// A matching corpus produces NO problems — the anti-tautology control
/// without which every assertion below could pass on a broken adjudicator.
#[test]
fn adjudication_accepts_a_corpus_that_matches_its_manifest() {
    let m = parse_manifest("!source p jazzy r@ref\n=p/A\nf int32 a\nc int32 K=1\n").unwrap();
    let c = corpus_of(&[("p", "A", "int32 K=1\nint32 a\n")]);
    assert!(
        adjudicate(&c, &m).is_empty(),
        "matching corpus must be clean: {:#?}",
        adjudicate(&c, &m)
    );
}

/// A vendored file that vanished must be reported (the loop over
/// `manifest.messages` that never fires against the real tree).
#[test]
fn adjudication_reports_a_manifest_entry_whose_vendored_file_vanished() {
    let m = parse_manifest("=p/A\nf int32 a\n=p/Gone\nf int32 b\n").unwrap();
    let c = corpus_of(&[("p", "A", "int32 a\n")]);
    assert_one_problem_containing(&adjudicate(&c, &m), "p/Gone");

    // The remedy it names must be RE-RUNNING THE REFRESH, never a directive:
    // the refresh rebuilds the body from the vendored tree, so a stale block
    // disappears on its own. (`!not-vendored` used to be offered here and was
    // SELF-DESTRUCTING — one refresh made every such line permanently inert,
    // so the documented remedy looped into a hard failure. It is removed; see
    // `parse_manifest`.)
    let ps = adjudicate(&c, &m);
    assert!(
        ps[0].contains("refresh_upstream_msg_manifest.sh") && !ps[0].contains("!not-vendored"),
        "the remedy must name the refresh and offer no directive: {}",
        ps[0]
    );
}

/// `!not-vendored` is REMOVED, and a legacy line must fail LOUDLY at parse
/// rather than be ignored — a silently-dropped directive would look
/// meaningful in the manifest while doing nothing.
#[test]
fn a_legacy_not_vendored_directive_is_rejected_with_migration_guidance() {
    let err = match parse_manifest(
        "!not-vendored p/Gone upstream has it, we do not\n=p/A\nf int32 a\n",
    ) {
        Err(e) => e,
        Ok(_) => panic!("a removed directive must not parse"),
    };
    assert!(
        err.contains("REMOVED") && err.contains("refresh_upstream_msg_manifest.sh"),
        "the rejection must say it was removed and what to do instead: {err}"
    );
    // Rejected on the bare form too (no reason), not only the well-formed one.
    assert!(parse_manifest("!not-vendored\n=p/A\nf int32 a\n").is_err());
}

/// A waiver whose divergence has since been fixed must be
/// reported STALE (the sweep that never fires while both live waivers are
/// genuinely live).
#[test]
fn adjudication_reports_a_stale_waiver_whose_divergence_was_fixed() {
    let m = parse_manifest("!accept p/A no longer true\n=p/A\nf int32 a\n").unwrap();
    let c = corpus_of(&[("p", "A", "int32 a\n")]);
    assert_one_problem_containing(&adjudicate(&c, &m), "STALE");
}

/// C3 / N2: a waiver naming a message that is NEITHER vendored NOR in the
/// manifest is reached by no other loop. Left alone it silently
/// pre-authorises a divergence if the message is ever re-vendored.
#[test]
fn adjudication_reports_an_orphaned_waiver_no_other_loop_can_reach() {
    let m = parse_manifest("!accept p/Ghost left behind\n=p/A\nf int32 a\n").unwrap();
    let c = corpus_of(&[("p", "A", "int32 a\n")]);
    assert_one_problem_containing(&adjudicate(&c, &m), "ORPHANED");
}

/// THE CLASS ITSELF: a message with NO recorded upstream
/// signature. Unwaived it fails; under a CONSTANTS-ONLY waiver it STILL
/// fails (absence silences fields too — this is the exact shape that hid
/// `VDA5050State`); only `!accept-fields` accepts it.
#[test]
fn adjudication_requires_a_field_scoped_waiver_for_an_absent_upstream_signature() {
    let c = corpus_of(&[("p", "A", "int32 a\n"), ("p", "Ours", "int32 b\n")]);

    let unwaived = parse_manifest("=p/A\nf int32 a\n").unwrap();
    let ps = adjudicate(&c, &unwaived);
    assert_one_problem_containing(&ps, "p/Ours");
    assert!(
        ps[0].contains("'!source'"),
        "the remedy must name re-pinning the distro FIRST — a waiver is the \
         mechanism that hid the original drift: {}",
        ps[0]
    );

    let const_waived =
        parse_manifest("!accept p/Ours constants do not reach the wire\n=p/A\nf int32 a\n")
            .unwrap();
    assert_one_problem_containing(&adjudicate(&c, &const_waived), "constants-only");

    let field_waived =
        parse_manifest("!accept-fields p/Ours genuinely ours alone\n=p/A\nf int32 a\n").unwrap();
    assert!(
        adjudicate(&c, &field_waived).is_empty(),
        "an explicit field-scoped waiver must accept an absent signature: {:#?}",
        adjudicate(&c, &field_waived)
    );
}

/// C1: the headline granularity contract. A constants-only `!accept` must
/// NOT silence FIELD drift — the mechanism the PR narrative blames for
/// hiding the fork, closed in code rather than in prose.
#[test]
fn a_constants_only_waiver_does_not_silence_field_drift() {
    // Upstream says `uint32 v`; we say `string v` — the VDA5050State shape.
    let m = parse_manifest(
        "!accept p/A constants only, they do not reach the wire\n=p/A\nf uint32 v\nc int32 K=1\n",
    )
    .unwrap();
    let c = corpus_of(&[("p", "A", "int32 K=1\nstring v\n")]);
    let ps = adjudicate(&c, &m);
    assert_one_problem_containing(&ps, "FIELD list DRIFTED");
    assert!(
        ps[0].contains("CONSTANTS-ONLY") && ps[0].contains("!accept-fields"),
        "the failure must explain WHY the existing waiver did not cover it: {}",
        ps[0]
    );

    // The same divergence under the field-scoped directive IS accepted.
    let m2 = parse_manifest(
        "!accept-fields p/A deliberate fork, reason stated\n=p/A\nf uint32 v\nc int32 K=1\n",
    )
    .unwrap();
    assert!(
        adjudicate(&c, &m2).is_empty(),
        "!accept-fields must cover field drift: {:#?}",
        adjudicate(&c, &m2)
    );

    // ...and a CONSTANT-only divergence is what a bare `!accept` is for.
    let m3 = parse_manifest("!accept p/A we inline an extra constant\n=p/A\nf uint32 v\n").unwrap();
    let c3 = corpus_of(&[("p", "A", "int32 EXTRA=7\nuint32 v\n")]);
    assert!(
        adjudicate(&c3, &m3).is_empty(),
        "a bare !accept must still cover constants: {:#?}",
        adjudicate(&c3, &m3)
    );
}

/// Field drift and constant drift are reported INDEPENDENTLY — a waiver
/// covering one must not swallow the other.
#[test]
fn adjudication_separates_field_drift_from_constant_drift() {
    // No waiver, both halves differ => two distinct problems.
    let m = parse_manifest("=p/A\nf uint32 v\nc int32 K=1\n").unwrap();
    let c = corpus_of(&[("p", "A", "int32 K=2\nstring v\n")]);
    let ps = adjudicate(&c, &m);
    assert_eq!(ps.len(), 2, "expected both halves reported: {ps:#?}");
    assert!(ps.iter().any(|p| p.contains("FIELD list DRIFTED")));
    assert!(ps.iter().any(|p| p.contains("CONSTANTS drifted")));

    // A constants-only waiver silences the constant half ONLY.
    let m2 =
        parse_manifest("!accept p/A constant delta is deliberate\n=p/A\nf uint32 v\nc int32 K=1\n")
            .unwrap();
    let ps2 = adjudicate(&c, &m2);
    assert_one_problem_containing(&ps2, "FIELD list DRIFTED");

    // An unparseable vendored file is reported, not silently skipped.
    // (`a/b/c` is a genuinely invalid message type — note that prose like
    // "this is not a field" DOES parse, as a same-package nested ref, which
    // is precisely why the fixture has to be chosen against the real parser
    // rather than assumed.)
    let bad = corpus_of(&[("p", "A", "a/b/c v\n")]);
    let ps3 = adjudicate(&bad, &m);
    assert!(
        ps3.iter().any(|p| p.contains("does not parse")),
        "a corrupt .msg must be reported: {ps3:#?}"
    );
}

/// `render_delta` is reachable only from a FAILING gate, so a
/// change returning `""` would pass every passing test. Drive it directly.
#[test]
fn render_delta_names_both_sides_and_distinguishes_a_reordering() {
    let d = render_delta(&["f string v".to_string()], &["f uint32 v".to_string()]);
    assert!(
        d.contains("MISSING (upstream has, we lack): f uint32 v"),
        "{d}"
    );
    assert!(
        d.contains("EXTRA   (we have, upstream lacks): f string v"),
        "{d}"
    );

    // Same SET, different ORDER — field order is layout, so this must not
    // render as "no difference".
    let reordered = render_delta(
        &["f int32 a".to_string(), "f int32 b".to_string()],
        &["f int32 b".to_string(), "f int32 a".to_string()],
    );
    assert!(reordered.contains("ORDER differs"), "{reordered}");
    assert!(!reordered.is_empty());
}

/// The two live waivers must be CONSTANTS-ONLY and their FIELD halves must
/// genuinely match upstream — the claim their reasons make ("constants do
/// not reach the wire"), asserted rather than assumed.
#[test]
fn every_live_waiver_is_constants_only_and_its_fields_match_upstream() {
    let text = std::fs::read_to_string(manifest_path()).expect("manifest must exist");
    let manifest = parse_manifest(&text).expect("manifest must parse");
    let corpus = vendored_corpus();

    assert!(
        !manifest.accepted.is_empty(),
        "anti-tautology: this test is vacuous with zero waivers"
    );

    for (key, waiver) in &manifest.accepted {
        assert_eq!(
            waiver.scope,
            WaiverScope::ConstantsOnly,
            "{key}: a field-scoped '!accept-fields' waiver is shipping. That is permitted, but \
             it silences a WIRE-AFFECTING divergence, so it must be a deliberate, reviewed \
             decision — update this test with the justification if so."
        );
        let (pkg, name) = key.split_once('/').expect("waiver key is pkg/Name");
        let our_text = corpus
            .get(&(pkg.to_string(), name.to_string()))
            .unwrap_or_else(|| panic!("{key}: waiver names a message that is not vendored"));
        let upstream = manifest
            .messages
            .get(key)
            .unwrap_or_else(|| panic!("{key}: constants-only waiver but NO upstream signature"));
        let ours = signature(our_text, name, pkg).expect("vendored .msg parses");
        let (our_fields, _) = split_signature(&ours);
        let (up_fields, _) = split_signature(upstream);
        assert_eq!(
            our_fields, up_fields,
            "{key}: waiver claims a constants-only divergence, but the FIELD lists differ"
        );
    }
}

/// C9 / N0: each distro pin is written in TWO places — the refresh script's
/// `clone` line and the manifest's `!source` line — and the refresh
/// preserves `!source` verbatim, so a pin bump on one side turns the other
/// into a silent lie while every other test stays green.
///
/// The manifest is the authority (it is what the gate
/// reads, and it is what the refresh preserves); this cross-check is what
/// makes the script's copy unable to drift from it. A cross-check rather
/// than deriving the script's table FROM the manifest, because the check
/// runs in CI on every PR — a script that reads the manifest only helps on
/// the days someone actually runs the script.
#[test]
fn refresh_script_clone_refs_match_the_manifest_source_pins() {
    let script_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tools/scripts/refresh_upstream_msg_manifest.sh");
    let script = std::fs::read_to_string(&script_path)
        .unwrap_or_else(|e| panic!("read {}: {e}", script_path.display()));

    // `clone <owner/repo> <ref> <dest>` — only real invocations (leading
    // `clone ` at column 0), never the function definition or prose.
    let mut from_script: BTreeSet<(String, String)> = BTreeSet::new();
    for line in script.lines() {
        let Some(rest) = line.strip_prefix("clone ") else {
            continue;
        };
        let mut parts = rest.split_whitespace();
        let (Some(repo), Some(git_ref)) = (parts.next(), parts.next()) else {
            panic!("malformed clone line in refresh script: {line}");
        };
        from_script.insert((repo.to_string(), git_ref.to_string()));
    }
    assert!(
        !from_script.is_empty(),
        "anti-tautology: found no `clone` lines in {} — the parser drifted from the script's \
         shape, so this cross-check would pass vacuously",
        script_path.display()
    );

    let text = std::fs::read_to_string(manifest_path()).expect("manifest must exist");
    let manifest = parse_manifest(&text).expect("manifest must parse");
    // A `!source` description is `<owner/repo>@<ref>` plus optional prose.
    let mut from_manifest: BTreeSet<(String, String)> = BTreeSet::new();
    for p in manifest.provenance.values() {
        let spec = p.source.split_whitespace().next().unwrap_or_default();
        let (repo, git_ref) = spec
            .split_once('@')
            .unwrap_or_else(|| panic!("!source description is not '<owner/repo>@<ref>': {spec}"));
        from_manifest.insert((repo.to_string(), git_ref.to_string()));
    }

    assert_eq!(
        from_script, from_manifest,
        "upstream drift: the refresh script's `clone` refs and the manifest's `!source` pins \
         disagree.\n\nThey are the same fact written twice, and the refresh preserves `!source` \
         lines VERBATIM — so a pin bumped on one side leaves the other claiming an upstream the \
         signatures did not come from, with nothing else failing. Update both, or delete \
         whichever is wrong.\n  script:   {from_script:#?}\n  manifest: {from_manifest:#?}"
    );
}

/// The manifest parser must reject malformed input rather than silently
/// producing an empty (vacuously-passing) manifest.
#[test]
fn manifest_parser_fails_closed_on_malformed_input() {
    assert!(parse_manifest("").is_err(), "empty manifest must fail");
    assert!(
        parse_manifest("# only comments\n").is_err(),
        "manifest with no messages must fail"
    );
    assert!(
        parse_manifest("f int32 a\n").is_err(),
        "signature line before any message key must fail"
    );
    assert!(
        parse_manifest("=nopackage\n").is_err(),
        "message key without a package must fail"
    );
    assert!(
        parse_manifest("=p/A\nf int32 a\n=p/A\nf int32 b\n").is_err(),
        "duplicate message key must fail"
    );
    assert!(
        parse_manifest("=p/A\nf int32 a\ngarbage line\n").is_err(),
        "unrecognized directive must fail"
    );

    // Malformed DIRECTIVE bodies must fail, not silently degrade to an
    // empty key or an empty reason (the `split_once` error arms).
    assert!(
        parse_manifest("!source ponly\n=p/A\nf int32 a\n").is_err(),
        "!source without a trust token must fail"
    );
    assert!(
        parse_manifest("!source p jazzy\n=p/A\nf int32 a\n").is_err(),
        "!source without a source description must fail"
    );
    assert!(
        parse_manifest("!source p WEIRD repo@ref\n=p/A\nf int32 a\n").is_err(),
        "an unrecognized !source trust token must fail (closed set)"
    );
    assert!(
        parse_manifest("!accept p/A\n=p/A\nf int32 a\n").is_err(),
        "!accept without a reason must fail"
    );
    assert!(
        parse_manifest("!accept-fields p/A\n=p/A\nf int32 a\n").is_err(),
        "!accept-fields without a reason must fail"
    );

    // A well-formed minimal manifest parses, and each trust token maps to
    // its own variant (an opaque string could not make this distinction).
    let m = parse_manifest(
        "!source p jazzy repo@ref\n\
         !source q release gbp@release/jazzy/q/1.0.0-1\n\
         !source r UNVERIFIED someone@main (not a rosdistro package)\n\
         =p/A\nf int32 a\nc int32 K=1\n",
    )
    .expect("well-formed manifest parses");
    assert_eq!(m.messages["p/A"], vec!["f int32 a", "c int32 K=1"]);
    assert_eq!(m.provenance["p"].trust, Trust::DistroBranch);
    assert_eq!(m.provenance["p"].source, "repo@ref");
    assert_eq!(m.provenance["q"].trust, Trust::ReleaseTag);
    assert_eq!(m.provenance["r"].trust, Trust::Unverified);

    // `!accept` and `!accept-fields` are DISTINCT scopes — a parser that
    // collapsed them would re-open the hole.
    let w = parse_manifest(
        "!accept p/A constants only\n\
         !accept-fields p/B fields too\n\
         =p/A\nf int32 a\n",
    )
    .expect("waiver manifest parses");
    assert_eq!(w.accepted["p/A"].scope, WaiverScope::ConstantsOnly);
    assert_eq!(w.accepted["p/A"].reason, "constants only");
    assert_eq!(w.accepted["p/B"].scope, WaiverScope::Fields);
    assert_eq!(w.accepted["p/B"].reason, "fields too");
}

// ─────────────────────────── refresh mode ───────────────────────────

/// Regenerate the manifest from a local tree of upstream `.msg` files.
///
/// NOT part of the CI gate (`#[ignore]`d): it needs upstream text that CI
/// does not have. Driven by `scripts/refresh_upstream_msg_manifest.sh`,
/// which fetches the pinned upstream refs and lays them out as
/// `<root>/<package>/<Name>.msg`.
///
/// Deliberately regenerates ONLY the signature blocks, and preserves the
/// existing `!source` / `!accept` / `!accept-fields` lines verbatim — those
/// are human judgements and must never be silently rewritten by a refresh. A MISSING manifest is therefore refused outright
/// rather than regenerated from nothing: silently emitting a bare header
/// would destroy every waiver, every provenance pin and the written
/// rationale behind them, and would exit 0 with a success banner while
/// doing it.
#[test]
#[ignore = "refresh tool: needs UPSTREAM_REFRESH_FROM pointing at upstream .msg text"]
fn refresh_manifest_from_upstream_tree() {
    let root = std::env::var("UPSTREAM_REFRESH_FROM").expect(
        "set UPSTREAM_REFRESH_FROM=<dir> laid out as <dir>/<package>/<Name>.msg \
         (use scripts/refresh_upstream_msg_manifest.sh)",
    );
    let root = Path::new(&root);
    let existing = std::fs::read_to_string(manifest_path()).unwrap_or_else(|e| {
        panic!(
            "upstream manifest refresh REFUSES to run without an existing manifest at {}: {e}\n\n\
             The refresh regenerates only the SIGNATURE BLOCKS; the '!source', '!accept', \
             and '!accept-fields' lines are human judgements it preserves \
             verbatim. With no file to preserve them from, a refresh would silently delete \
             every one of them (and the written rationale in the header) while reporting \
             success. Restore the manifest from git first — `git checkout -- {}`.",
            manifest_path().display(),
            manifest_path().display(),
        )
    });

    // Preserve every human-authored header line, in order — including the
    // BLANK lines that separate its sections. Capture stops at the first
    // signature block, so nothing from the generated body can leak in; a
    // comment-only filter would silently eat the section breaks and make
    // every refresh degrade the very doc block it is told to preserve.
    let mut header: Vec<String> = Vec::new();
    for line in existing.lines() {
        let t = line.trim_end();
        if t.starts_with('=') {
            break;
        }
        if t.is_empty()
            || t.starts_with('#')
            || t.starts_with("!source ")
            || t.starts_with("!accept ")
            || t.starts_with("!accept-fields ")
        {
            header.push(t.to_string());
        }
    }
    assert!(
        header.iter().any(|l| l.starts_with("!source ")),
        "upstream manifest refresh REFUSES to run: the existing manifest at {} carries no '!source' \
         provenance lines, so this is not a manifest whose human judgements can be preserved. \
         Restore it from git rather than letting a refresh overwrite it.",
        manifest_path().display()
    );
    // The body is emitted after exactly one blank line; drop any trailing
    // blanks so a refresh cannot accumulate them run over run.
    while header.last().is_some_and(|l| l.is_empty()) {
        header.pop();
    }

    let mut body = String::new();
    let mut written = 0usize;
    let mut skipped: Vec<String> = Vec::new();
    for (pkg, name) in vendored_corpus().keys() {
        let up = root.join(pkg).join(format!("{name}.msg"));
        let Ok(text) = std::fs::read_to_string(&up) else {
            skipped.push(format!("{pkg}/{name}"));
            continue;
        };
        match signature(&text, name, pkg) {
            Ok(sig) => {
                body.push_str(&format!("={pkg}/{name}\n"));
                for l in sig {
                    body.push_str(&l);
                    body.push('\n');
                }
                written += 1;
            }
            Err(e) => skipped.push(format!("{pkg}/{name} (parse error: {e})")),
        }
    }

    // The header is guaranteed non-empty (asserted above), so there is no
    // silent-regeneration fallback to get wrong.
    let mut out = String::new();
    for h in &header {
        out.push_str(h);
        out.push('\n');
    }
    out.push('\n');
    out.push_str(&body);
    std::fs::write(manifest_path(), out).expect("write manifest");
    eprintln!("upstream manifest refresh: wrote {written} signature block(s)");
    if !skipped.is_empty() {
        eprintln!(
            "upstream manifest refresh: NO upstream text for {} message(s).\n\
             \n\
             READ THE PACKAGE'S '!source' PIN FIRST. A whole package's worth of messages \
             missing almost always means it is pinned to a distro that does not ship them — \
             that was the original drift's actual root cause, where 13 control_msgs messages were waived \
             as 'absent from Jazzy' when the package simply needed a Lyrical pin, and one of \
             those waivers was hiding a real string-vs-uint32 fork.\n\
             \n\
             In order of preference: (1) re-pin the package's distro and re-run; (2) if the \
             message really is upstream but not at that path, fix the layout; (3) if upstream \
             genuinely does not have it, record '!accept-fields <pkg/Name> <reason>' — that \
             waiver compares NOTHING for the message, fields included, so it is a last resort \
             and the gate will make you spell it that way.\n  {}",
            skipped.len(),
            skipped.join("\n  ")
        );
    }
}
