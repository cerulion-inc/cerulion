// SPDX-License-Identifier: AGPL-3.0-only
//! The per-distro absent-export table as a GATE instead of a hand-typed list.
//!
//! # The class
//!
//! An entry point whose parameter type a distro's headers do not declare is
//! compiled out WHOLE under `#[cfg(cerulion_has_<cap>)]`, never stubbed, and
//! `tools/ci/rmw-distros/gate.sh` names those symbols per distro row in
//! `absent_symbols` so each lane can read the built library with `nm` and fail
//! where one is defined. That audit is ONE-DIRECTIONAL: it proves every symbol
//! on a row's list is undefined, never that the list is COMPLETE. A new guarded
//! export, or a capability that moves era, would land missing from every row
//! and every lane would stay green while the library exported a symbol its
//! headers cannot back.
//!
//! # The rule
//!
//! For every `build` row in the gate table, the exports that row's distro lacks
//! are DERIVED here from the tree: each `extern "C"` `rmw_*` export guarded by
//! `#[cfg(cerulion_has_<cap>)]` whose capability arrived in an era LATER than
//! the row's own era (`era_check::CAPABILITY_MIN_ERA` and
//! `era_check::DISTRO_ERAS`). The derived set must equal the row's hand-typed
//! list EXACTLY, and the failure prints the symmetric difference, so a new
//! guarded export fails until every row names it and a stale row fails until it
//! drops the name.
//!
//! Only two guard shapes carry a rule this gate can apply:
//! `#[cfg(cerulion_has_<cap>)]` and `#[cfg(not(cerulion_has_<cap>))]`. The
//! negated one INVERTS the rule (the export exists where the capability is
//! absent), and any other `#[cfg(...)]` or `#[cfg_attr(...)]` directly over an
//! export (a compound `all(...)`, an `any(...)`, a platform predicate) carries a
//! rule this file does not model. Reading either as "unguarded" would be a
//! SILENT FALSE GREEN in the completeness check: the export lands in no derived
//! set, every row agrees, and the audit never asks any row to name it. So an
//! unmodelled guard shape is a REFUSAL that names the export and the attribute
//! text, never an unguarded export. A bare `#[cfg(...)]` whose reach a code line
//! ends still guards nothing, which is not a refusal.
//!
//! # The oracles
//!
//! Both parsers are pinned by hand-written fixtures with hand-written expected
//! readings, never by the tree they read: a source snippet carrying a guarded
//! export, an unguarded export, a negated guard, a COMPOUND guard, a cfg whose
//! reach a code line ends and a platform cfg whose reach a code line ends, and a
//! gate snippet carrying a list-valued row, an empty row and the fallback arm.
//! Both directions of the comparison are pinned on a fixture era table: the
//! baseline agrees, and a symbol added to or removed from either side fails,
//! naming it on the side it appeared. The compound arm pins the false green
//! itself: the comparison AGREES over that fixture, and the refusal is what
//! fails. The tree comparison refuses a vacuous read as well: the walk must find
//! the whole export surface, and at least one row's derived set must be
//! non-empty, so a parser that silently matched nothing cannot pass by reading
//! zero against zero.
//!
//! Pure text and two constant tables: no transport, no iceoryx2 root, nothing
//! that behaves differently under `--release`.

use rmw_cerulion::era_check::{CAPABILITY_MIN_ERA, DISTRO_ERAS, ERA_NAMES};
use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

/// One `extern "C"` export and the capability cfg guarding it, if any.
#[derive(Debug, PartialEq, Eq)]
struct Export {
    name: String,
    capability: Option<String>,
}

/// One `build` or `refuse` row of the gate's expected-state table.
#[derive(Debug, PartialEq, Eq)]
struct GateRow {
    distro: String,
    expect: String,
    absent: BTreeSet<String>,
}

/// What one source file yields: the exports, every export a NEGATED capability
/// cfg guards (the inverted rule), and every export under a guard shape this
/// file does not interpret (export name, attribute text). The last two are
/// refusals, not readings.
#[derive(Debug, Default, PartialEq, Eq)]
struct Parsed {
    exports: Vec<Export>,
    negated_guards: Vec<(String, String)>,
    uninterpreted_guards: Vec<(String, String)>,
}

/// The guard attribute standing over the next export, as far as this file reads
/// attributes.
#[derive(Debug)]
enum Guard {
    /// `#[cfg(cerulion_has_<cap>)]`: the export is absent before that era.
    Capability(String),
    /// `#[cfg(not(cerulion_has_<cap>))]`: the inverse rule.
    NegatedCapability(String),
    /// Any other `#[cfg(...)]` or `#[cfg_attr(...)]`, kept verbatim.
    Uninterpreted(String),
}

const CFG_OPEN: &str = "#[cfg(cerulion_has_";
const CFG_NOT_OPEN: &str = "#[cfg(not(cerulion_has_";
const CFG_ANY_OPEN: &str = "#[cfg(";
const CFG_ATTR_OPEN: &str = "#[cfg_attr(";
const ABSENT_VAR_SUFFIX: &str = "_absent_symbols=\"";

/// The export name on this line, if it declares one.
fn export_name(line: &str) -> Option<String> {
    let rest = line
        .strip_prefix("pub unsafe extern \"C\" fn ")
        .or_else(|| line.strip_prefix("pub extern \"C\" fn "))?;
    let name: String = rest
        .chars()
        .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
        .collect();
    name.starts_with("rmw_").then_some(name)
}

/// The guard this line declares, if it is a cfg attribute at all.
fn guard_on_line(trimmed: &str) -> Option<Guard> {
    if let Some(cap) = trimmed
        .strip_prefix(CFG_NOT_OPEN)
        .and_then(|rest| rest.strip_suffix("))]"))
    {
        return Some(Guard::NegatedCapability(cap.to_string()));
    }
    if let Some(cap) = trimmed
        .strip_prefix(CFG_OPEN)
        .and_then(|rest| rest.strip_suffix(")]"))
    {
        return Some(Guard::Capability(cap.to_string()));
    }
    if trimmed.starts_with(CFG_ANY_OPEN) || trimmed.starts_with(CFG_ATTR_OPEN) {
        return Some(Guard::Uninterpreted(trimmed.to_string()));
    }
    None
}

/// Read every `rmw_*` export out of one source file with the guard standing over
/// it. Only attributes, doc comments and blank lines may sit between a guard and
/// the export it covers; any other line ends its reach, so a cfg over a constant
/// cannot be read as a guard on the next export. A guard shape this file does not
/// interpret is recorded as a refusal rather than dropped, because dropping it
/// would read the export as unguarded and hide it from every row.
fn parse_exports(src: &str) -> Parsed {
    let mut parsed = Parsed::default();
    let mut pending: Option<Guard> = None;
    for line in src.lines() {
        let trimmed = line.trim();
        if let Some(guard) = guard_on_line(trimmed) {
            pending = Some(guard);
            continue;
        }
        if let Some(name) = export_name(trimmed) {
            let capability = match pending.take() {
                Some(Guard::Capability(cap)) => Some(cap),
                Some(Guard::NegatedCapability(cap)) => {
                    parsed.negated_guards.push((name.clone(), cap));
                    None
                }
                Some(Guard::Uninterpreted(attribute)) => {
                    parsed.uninterpreted_guards.push((name.clone(), attribute));
                    None
                }
                None => None,
            };
            parsed.exports.push(Export { name, capability });
            continue;
        }
        if !(trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with("//")) {
            pending = None;
        }
    }
    parsed
}

/// `None` when every guard over an export carries a rule this file models;
/// otherwise the refusal, naming each export and the attribute over it.
fn unmodelled_guard_refusal(parsed: &Parsed) -> Option<String> {
    if parsed.negated_guards.is_empty() && parsed.uninterpreted_guards.is_empty() {
        return None;
    }
    let mut lines = Vec::new();
    for (name, capability) in &parsed.negated_guards {
        lines.push(format!(
            "{name} is guarded by `#[cfg(not(cerulion_has_{capability}))]`, the INVERSE rule: it \
             exists where the capability is absent, so this gate's derivation does not hold for it"
        ));
    }
    for (name, attribute) in &parsed.uninterpreted_guards {
        lines.push(format!(
            "{name} is guarded by `{attribute}`, a shape this gate does not interpret; reading it \
             as unguarded would hide the export from every row's list"
        ));
    }
    Some(lines.join("\n"))
}

/// `<prefix>_absent_symbols="` at the head of a line: the FULL variable name,
/// the way a row spells it, with whatever follows the opening quote.
fn absent_var_head(line: &str) -> Option<(String, &str)> {
    let (prefix, rest) = line.split_once(ABSENT_VAR_SUFFIX)?;
    let ident_only = !prefix.is_empty()
        && prefix
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_');
    ident_only.then(|| (format!("{prefix}_absent_symbols"), rest))
}

/// The value of `key="..."` on one line, when the value closes on that line.
fn quoted_value<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let rest = line.split_once(&format!("{key}=\""))?.1;
    Some(rest.split_once('"')?.0)
}

/// One expected-state row, when this line is one.
fn case_row(line: &str, vars: &BTreeMap<String, BTreeSet<String>>) -> Option<GateRow> {
    let trimmed = line.trim();
    let (head, tail) = trimmed.split_once(')')?;
    let distro_only =
        !head.is_empty() && head.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    if !distro_only {
        return None;
    }
    let expect = tail.split_once("expect=")?.1.split(';').next()?.trim();
    let absent = match quoted_value(tail, "absent_symbols") {
        Some(value) => match value.strip_prefix('$') {
            Some(var) => vars.get(var).cloned().unwrap_or_else(|| {
                panic!("gate row `{head}` names `${var}`, which the table never assigns")
            }),
            None => value.split_whitespace().map(str::to_string).collect(),
        },
        None => BTreeSet::new(),
    };
    Some(GateRow {
        distro: head.to_string(),
        expect: expect.to_string(),
        absent,
    })
}

/// Read the gate's expected-state table: the `<distro>_absent_symbols` lists,
/// which may span lines, and every row that names one.
fn parse_gate_rows(gate: &str) -> Vec<GateRow> {
    let lines: Vec<&str> = gate.lines().collect();
    let mut vars: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut rows: Vec<GateRow> = Vec::new();
    let mut index = 0;
    while index < lines.len() {
        if let Some((name, first)) = absent_var_head(lines[index]) {
            let mut symbols: BTreeSet<String> = BTreeSet::new();
            let mut cursor = index;
            let mut body = first;
            loop {
                match body.split_once('"') {
                    Some((head, _)) => {
                        symbols.extend(head.split_whitespace().map(str::to_string));
                        break;
                    }
                    None => {
                        symbols.extend(body.split_whitespace().map(str::to_string));
                        cursor += 1;
                        match lines.get(cursor) {
                            Some(next) => body = next,
                            None => break,
                        }
                    }
                }
            }
            vars.insert(name, symbols);
            index = cursor + 1;
            continue;
        }
        if let Some(row) = case_row(lines[index], &vars) {
            rows.push(row);
        }
        index += 1;
    }
    rows
}

/// The exports a distro at `era` lacks: those whose guarding capability arrived
/// later. The era table is a parameter so a fixture can supply its own.
fn derived_absent(exports: &[Export], caps: &[(&str, usize)], era: usize) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    for export in exports {
        let Some(capability) = export.capability.as_deref() else {
            continue;
        };
        let min_era = caps
            .iter()
            .find(|(name, _)| *name == capability)
            .unwrap_or_else(|| {
                panic!(
                    "`{}` is guarded by `cerulion_has_{capability}`, which the capability era \
                     table does not name",
                    export.name
                )
            })
            .1;
        if min_era > era {
            out.insert(export.name.clone());
        }
    }
    out
}

/// `None` when the two sets agree; otherwise the symmetric difference, named on
/// the side each symbol appeared.
fn disagreement(derived: &BTreeSet<String>, pinned: &BTreeSet<String>) -> Option<String> {
    if derived == pinned {
        return None;
    }
    let only_derived: Vec<&str> = derived.difference(pinned).map(String::as_str).collect();
    let only_pinned: Vec<&str> = pinned.difference(derived).map(String::as_str).collect();
    Some(format!(
        "derived from the tree but not named by the row: {only_derived:?}; \
         named by the row but not derived from the tree: {only_pinned:?}"
    ))
}

fn era_of_distro(distro: &str) -> usize {
    DISTRO_ERAS
        .iter()
        .find(|(name, _)| *name == distro)
        .unwrap_or_else(|| panic!("the gate names distro `{distro}`, which DISTRO_ERAS does not"))
        .1
}

fn repo_root() -> PathBuf {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    crate_root
        .parent()
        .and_then(Path::parent)
        .expect("the crate sits two levels under the repository root")
        .to_path_buf()
}

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let entries = std::fs::read_dir(dir).unwrap_or_else(|e| panic!("read_dir {dir:?}: {e}"));
    for entry in entries {
        let path = entry.expect("a readable directory entry").path();
        if path.is_dir() {
            rust_sources(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

/// Every export in the crate's own source tree, with its guard.
fn tree_exports() -> Parsed {
    let mut files = Vec::new();
    rust_sources(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("src"),
        &mut files,
    );
    files.sort();
    let mut all = Parsed::default();
    for file in &files {
        let text = std::fs::read_to_string(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
        let mut parsed = parse_exports(&text);
        all.exports.append(&mut parsed.exports);
        all.negated_guards.append(&mut parsed.negated_guards);
    }
    all
}

// ---------------------------------------------------------------------------
// Hand-written fixtures: the oracle for both parsers and for the comparison.
// ---------------------------------------------------------------------------

const SOURCE_FIXTURE: &str = r#"
/// # Safety
/// rmw ABI contract.
#[cfg(cerulion_has_fixture_late)]
#[no_mangle]
pub unsafe extern "C" fn rmw_fixture_late_call(
    _handle: *mut u8,
) -> i32 {
    0
}

/// # Safety
/// rmw ABI contract.
#[no_mangle]
pub unsafe extern "C" fn rmw_fixture_always() -> i32 {
    0
}

#[cfg(cerulion_has_fixture_early)]
#[no_mangle]
pub extern "C" fn rmw_fixture_early_probe() -> bool {
    false
}

#[cfg(not(cerulion_has_fixture_late))]
#[no_mangle]
pub unsafe extern "C" fn rmw_fixture_legacy_only() -> i32 { 0 }

#[cfg(cerulion_has_fixture_late)]
const FIXTURE_REACH_ENDS_HERE: bool = true;

#[no_mangle]
pub unsafe extern "C" fn rmw_fixture_after_a_code_line() -> i32 { 0 }

#[cfg(all(cerulion_has_fixture_late, target_os = "linux"))]
#[no_mangle]
pub unsafe extern "C" fn rmw_fixture_compound_guard() -> i32 { 0 }

#[cfg(target_os = "linux")]
const FIXTURE_PLATFORM_REACH_ENDS_HERE: bool = true;

#[no_mangle]
pub unsafe extern "C" fn rmw_fixture_after_a_platform_cfg() -> i32 { 0 }
"#;

/// The attribute text the compound arm must report VERBATIM.
const FIXTURE_COMPOUND_ATTRIBUTE: &str =
    "#[cfg(all(cerulion_has_fixture_late, target_os = \"linux\"))]";

const GATE_FIXTURE: &str = r#"
# a comment naming absent_symbols must not read as an assignment
fixturelate_absent_symbols="rmw_fixture_late_call
rmw_fixture_after_a_code_line"
case "$distro" in
    fixturenew) expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="" ;;
    fixtureold) expect=build; min_targets=25; min_tests=400; known_failures=""; absent_symbols="$fixturelate_absent_symbols" ;;
    fixturerefuse) expect=refuse; min_targets=25; min_tests=400 ;;
    *) echo "FATAL: no expected state for distro '$distro'"; exit 1 ;;
esac
"#;

/// The fixture's era table, hand written: `fixture_early` at the fixture's own
/// era, `fixture_late` one era later.
const FIXTURE_CAPS: &[(&str, usize)] = &[("fixture_early", 1), ("fixture_late", 2)];
const FIXTURE_ERA: usize = 1;

fn fixture_pinned() -> BTreeSet<String> {
    ["rmw_fixture_late_call"]
        .into_iter()
        .map(str::to_string)
        .collect()
}

#[test]
fn the_source_parser_reads_the_hand_built_snippet() {
    let parsed = parse_exports(SOURCE_FIXTURE);
    let expected = vec![
        Export {
            name: "rmw_fixture_late_call".to_string(),
            capability: Some("fixture_late".to_string()),
        },
        Export {
            name: "rmw_fixture_always".to_string(),
            capability: None,
        },
        Export {
            name: "rmw_fixture_early_probe".to_string(),
            capability: Some("fixture_early".to_string()),
        },
        // A negated guard never becomes a forward one: the rule it carries is
        // the inverse, so the export reads as unguarded here and is reported.
        Export {
            name: "rmw_fixture_legacy_only".to_string(),
            capability: None,
        },
        // The cfg above the constant does not reach past it.
        Export {
            name: "rmw_fixture_after_a_code_line".to_string(),
            capability: None,
        },
        // A compound guard carries a rule this file does not model, so it is
        // reported rather than read as a capability or as unguarded.
        Export {
            name: "rmw_fixture_compound_guard".to_string(),
            capability: None,
        },
        // Nor does a platform cfg over a constant reach past it.
        Export {
            name: "rmw_fixture_after_a_platform_cfg".to_string(),
            capability: None,
        },
    ];
    assert_eq!(parsed.exports, expected);
    assert_eq!(
        parsed.negated_guards,
        vec![(
            "rmw_fixture_legacy_only".to_string(),
            "fixture_late".to_string()
        )]
    );
    assert_eq!(
        parsed.uninterpreted_guards,
        vec![(
            "rmw_fixture_compound_guard".to_string(),
            FIXTURE_COMPOUND_ATTRIBUTE.to_string()
        )],
        "only the compound guard is uninterpreted: the two recognised shapes parse as guards, and \
         a cfg whose reach a code line ends guards nothing"
    );
}

#[test]
fn an_unmodelled_guard_shape_is_a_refusal_never_an_unguarded_export() {
    let parsed = parse_exports(SOURCE_FIXTURE);
    // The false green itself: the comparison is BLIND to the compound-guarded
    // export, because an export read as unguarded lands in no derived set.
    let derived = derived_absent(&parsed.exports, FIXTURE_CAPS, FIXTURE_ERA);
    assert!(
        !derived.contains("rmw_fixture_compound_guard"),
        "a compound-guarded export cannot reach the derivation, which is why it needs a refusal"
    );
    assert!(
        disagreement(&derived, &fixture_pinned()).is_none(),
        "the comparison agrees over this fixture, so only the refusal can catch the omission"
    );
    // The refusal is what fails, naming the export and the attribute verbatim.
    let report = unmodelled_guard_refusal(&parsed).expect("an unmodelled guard shape must refuse");
    assert!(
        report.contains("rmw_fixture_compound_guard")
            && report.contains(FIXTURE_COMPOUND_ATTRIBUTE),
        "the refusal must name the export and the attribute over it: {report}"
    );
    assert!(
        report.contains("rmw_fixture_legacy_only"),
        "the negated guard is refused by the same reader: {report}"
    );
    // Both recognised shapes still parse as guards rather than as refusals.
    let recognised = parse_exports(
        "#[cfg(cerulion_has_fixture_late)]\n#[no_mangle]\npub unsafe extern \"C\" fn \
         rmw_fixture_recognised() -> i32 { 0 }\n",
    );
    assert_eq!(
        recognised.exports,
        vec![Export {
            name: "rmw_fixture_recognised".to_string(),
            capability: Some("fixture_late".to_string()),
        }]
    );
    assert!(
        unmodelled_guard_refusal(&recognised).is_none(),
        "a recognised capability guard is a reading, not a refusal"
    );
}

#[test]
fn the_gate_parser_reads_the_hand_built_snippet() {
    let rows = parse_gate_rows(GATE_FIXTURE);
    let expected = vec![
        GateRow {
            distro: "fixturenew".to_string(),
            expect: "build".to_string(),
            absent: BTreeSet::new(),
        },
        GateRow {
            distro: "fixtureold".to_string(),
            expect: "build".to_string(),
            absent: ["rmw_fixture_late_call", "rmw_fixture_after_a_code_line"]
                .into_iter()
                .map(str::to_string)
                .collect(),
        },
        GateRow {
            distro: "fixturerefuse".to_string(),
            expect: "refuse".to_string(),
            absent: BTreeSet::new(),
        },
    ];
    assert_eq!(rows, expected, "the fallback arm is not a row");
}

#[test]
fn the_comparison_agrees_on_the_fixture_and_fails_in_both_directions() {
    let exports = parse_exports(SOURCE_FIXTURE).exports;
    let derived = derived_absent(&exports, FIXTURE_CAPS, FIXTURE_ERA);
    assert_eq!(
        derived,
        fixture_pinned(),
        "the fixture baseline must agree, or the four failing arms below prove nothing"
    );
    assert!(disagreement(&derived, &fixture_pinned()).is_none());

    // 1. An extra guarded export in the source: derived grows.
    let extra = format!(
        "{SOURCE_FIXTURE}\n#[cfg(cerulion_has_fixture_late)]\n#[no_mangle]\npub unsafe extern \
         \"C\" fn rmw_fixture_synthetic() -> i32 {{ 0 }}\n"
    );
    let grown = derived_absent(&parse_exports(&extra).exports, FIXTURE_CAPS, FIXTURE_ERA);
    let report = disagreement(&grown, &fixture_pinned())
        .expect("an extra guarded export must fail the comparison");
    assert!(
        report.contains("but not named by the row: [\"rmw_fixture_synthetic\"]"),
        "the failure must name the new export on the derived side: {report}"
    );

    // 2. That export removed from the source while the row still names it:
    //    the same disagreement from the other side.
    let mut pinned_extra = fixture_pinned();
    pinned_extra.insert("rmw_fixture_synthetic".to_string());
    let report = disagreement(&derived, &pinned_extra)
        .expect("a row naming an export the tree does not guard must fail the comparison");
    assert!(
        report.contains("but not derived from the tree: [\"rmw_fixture_synthetic\"]"),
        "the failure must name the stale symbol on the row's side: {report}"
    );

    // 3. A symbol dropped from the row's list.
    let report = disagreement(&derived, &BTreeSet::new())
        .expect("an empty row against a non-empty derivation must fail");
    assert!(
        report.contains("but not named by the row: [\"rmw_fixture_late_call\"]"),
        "the failure must name the dropped symbol: {report}"
    );

    // 4. The capability moving era moves the derivation: at the later era the
    //    export is present, so the row that still names it fails.
    let later = derived_absent(&exports, FIXTURE_CAPS, FIXTURE_ERA + 1);
    assert!(later.is_empty(), "nothing is absent at the later era");
    assert!(
        disagreement(&later, &fixture_pinned()).is_some(),
        "a capability that moves era must fail every row that still names its export"
    );
}

#[test]
fn every_gate_row_names_exactly_the_exports_its_distro_lacks() {
    let parsed = tree_exports();
    if let Some(report) = unmodelled_guard_refusal(&parsed) {
        panic!(
            "every guard over an export must carry a rule this gate models; extend the reader and \
             the fixtures before such an export lands:\n{report}"
        );
    }
    // Anti-vacuity: a parser that matched nothing would read zero against the
    // empty rows and pass.
    assert!(
        parsed.exports.len() >= 100,
        "the walk found only {} exports; the crate's surface is the whole rmw C API",
        parsed.exports.len()
    );
    let guarded = parsed
        .exports
        .iter()
        .filter(|e| e.capability.is_some())
        .count();
    assert!(guarded > 0, "no export read as guarded by a capability cfg");
    println!(
        "export walk: {} exports, {guarded} of them guarded by a capability cfg",
        parsed.exports.len()
    );

    let gate = repo_root().join("tools/ci/rmw-distros/gate.sh");
    let text = std::fs::read_to_string(&gate).unwrap_or_else(|e| panic!("read {gate:?}: {e}"));
    let rows = parse_gate_rows(&text);
    assert!(!rows.is_empty(), "the gate table read as having no rows");

    let mut failures: Vec<String> = Vec::new();
    let mut nonempty_rows = 0;
    for row in &rows {
        if row.expect != "build" {
            continue;
        }
        let era = era_of_distro(&row.distro);
        let derived = derived_absent(&parsed.exports, CAPABILITY_MIN_ERA, era);
        if !derived.is_empty() {
            nonempty_rows += 1;
        }
        println!(
            "{:9} era {era} ({}): {} absent export(s) {:?}",
            row.distro,
            ERA_NAMES[era],
            derived.len(),
            derived.iter().collect::<Vec<_>>()
        );
        if let Some(report) = disagreement(&derived, &row.absent) {
            failures.push(format!("{}: {report}", row.distro));
        }
    }
    assert!(
        nonempty_rows > 0,
        "no build row derived a non-empty absent set, so every comparison was zero against zero"
    );
    assert!(
        failures.is_empty(),
        "the gate's absent_symbols lists disagree with the tree:\n{}",
        failures.join("\n")
    );
}
