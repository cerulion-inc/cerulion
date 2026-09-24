// SPDX-License-Identifier: AGPL-3.0-only
//! Where the generated code points.
//!
//! Everything these macros emit reaches the runtime by an absolute path:
//! `::cerulion_core::graph::node::NodeEntry`, and roughly two hundred more.
//! Rust resolves the first segment of such a path only against a crate the
//! consuming package names in its own manifest. A re-export cannot put one
//! there, and Cargo does not hand a package its transitive dependencies, so
//! the spelling of that first segment is not a constant: it depends on which
//! crate the package in front of the macro actually named.
//!
//! Two spellings are supported, and this module picks between them once per
//! compilation by reading the consuming package's `Cargo.toml`:
//!
//! | The package names | Generated paths start with |
//! |---|---|
//! | `cerulion_core` | `::cerulion_core` |
//! | `cerulion` (the umbrella crate) | `::cerulion::core` |
//!
//! The umbrella re-exports the runtime whole (`pub use cerulion_core as
//! core;`), so the two roots address the same items under the same names.
//! Nothing here curates a list of what the macros may reach, and a macro that
//! starts using a new runtime item needs no change on the umbrella side.
//!
//! A package that names both gets `::cerulion_core`. That ordering is not a
//! preference, it is the compatibility rule: every package written before
//! this resolver existed named `cerulion_core`, and for those the generated
//! tokens are what they always were.
//!
//! A rename is carried through, because the resolver reports the name the
//! package chose rather than the name on the package. `cer = { package =
//! "cerulion_core" }` emits `::cer`.
//!
//! # What the resolver cannot see
//!
//! It reads a manifest, not a compilation. `[dependencies]`,
//! `[dev-dependencies]` and every `[target.'cfg(..)'.dependencies]` table are
//! one flat set to it, whether or not the cfg holds for the target being
//! compiled. So a package that names the umbrella in `[dependencies]` and the
//! runtime in `[dev-dependencies]` gets `::cerulion_core` in its library
//! target too, where that name is not linked, and the missing-dependency
//! diagnostic does NOT fire, because the runtime was found. The remedy is the
//! one the compiler's own resolution error already points at: name in
//! `[dependencies]` whichever crate the code that expands macros is built
//! against.
//!
//! It also cannot see a manifest that is not there. A driver that is not
//! cargo sets no `CARGO_MANIFEST_DIR`, and a manifest can be unreadable or
//! unparseable; none of those is a missing dependency, so none of them is
//! reported as one. See `Root::Undecidable`.
//!
//! One consequence inside this repository: `cerulion_macros` names neither
//! crate, because the dependency runs the other way, so its own unit tests
//! resolve to `Root::Unresolved` and read the fallback. That is why they can
//! keep asserting `:: cerulion_core` in generated token text. Adding the
//! umbrella as a dev-dependency here would flip those assertions.

use proc_macro2::{Span, TokenStream};
use proc_macro_crate::{crate_name, FoundCrate};
use quote::quote;
use std::sync::Mutex;
use syn::Ident;

/// The runtime crate, by its published name.
const RUNTIME: &str = "cerulion_core";

/// The umbrella crate: one dependency that re-exports the runtime, the node
/// macros and the message types.
const UMBRELLA: &str = "cerulion";

/// The module the umbrella re-exports the runtime under, so that
/// `::cerulion::core::graph` and `::cerulion_core::graph` are the same path.
const UMBRELLA_RUNTIME_MODULE: &str = "core";

/// The version the missing-dependency message tells a user to add.
///
/// This crate's own, read at compile time rather than written out. The
/// workspace releases in lockstep with every internal dependency pinned
/// exactly, so the macro crate's version IS the runtime's and the umbrella's,
/// and advice derived from it cannot go stale the first time the workspace is
/// bumped.
const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Which crate the consuming package reaches the runtime through.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Root {
    /// The package names the runtime crate. The identifier is how the package
    /// spells it, which is the crate's own name unless the dependency was
    /// renamed.
    Runtime(String),
    /// The package names the umbrella crate and nothing else, so the runtime
    /// is reached through the umbrella's re-export.
    Umbrella(String),
    /// The package names neither. The macros cannot be invoked at all without
    /// one of them in the dependency graph, so this is reachable only by
    /// depending on the macro crate directly.
    Unresolved,
    /// The manifest could not be read, so nothing is known either way.
    ///
    /// Kept apart from `Unresolved` because the two need opposite handling. A
    /// package that names neither crate has a manifest to fix and is told so;
    /// a compilation with no readable manifest may be perfectly well formed,
    /// and before this resolver existed it compiled, since the macros emitted
    /// `::cerulion_core` unconditionally and any driver passing
    /// `--extern cerulion_core` was served. Reporting it as a missing
    /// dependency would both break that build and name the wrong cause.
    Undecidable,
}

impl Root {
    /// The path the generated code puts in front of every runtime item.
    ///
    /// `Root::Unresolved` falls back to `::cerulion_core`, which is what
    /// this crate emitted before the root was resolvable at all. The fallback
    /// is never silent: the macro entry points emit
    /// `unresolved_diagnostic` alongside the expansion, so the named
    /// remedy is the first error the compiler prints.
    fn path(&self) -> TokenStream {
        match self {
            Self::Runtime(name) => {
                let krate = Ident::new(name, Span::call_site());
                quote! { ::#krate }
            }
            Self::Umbrella(name) => {
                let krate = Ident::new(name, Span::call_site());
                let module = Ident::new(UMBRELLA_RUNTIME_MODULE, Span::call_site());
                quote! { ::#krate::#module }
            }
            Self::Unresolved | Self::Undecidable => {
                let krate = Ident::new(RUNTIME, Span::call_site());
                quote! { ::#krate }
            }
        }
    }
}

/// What asking the resolver about one crate settled.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Lookup {
    /// The package names it, under this spelling.
    Found(FoundCrate),
    /// The manifest was read and does not name it.
    Absent,
    /// Nothing was read, so nothing is known.
    Undecidable,
}

/// Turn a lookup identity into the name to emit.
///
/// `FoundCrate::Itself` means the consuming package IS the crate asked about,
/// and a caller would normally answer it with `crate`. That is wrong here and
/// deliberately not done. Cargo reports `Itself` for the runtime crate's own
/// library target AND for its doctests, and a doctest is a separate crate
/// that reaches the runtime as `::cerulion_core` through `--extern`. Nothing
/// distinguishes the two at expansion time, so both get the crate name, which
/// is what they got before this resolver existed: the runtime's doctests keep
/// compiling, and a macro invoked in the runtime's own library target fails
/// exactly as it did before.
fn spelling(found: FoundCrate, fallback: &str) -> String {
    let name = match found {
        FoundCrate::Itself => return fallback.to_owned(),
        FoundCrate::Name(name) => name,
    };
    // The resolver hands back the manifest KEY with `-` turned into `_` and
    // nothing else, so a rename key that is a Rust keyword, or not an
    // identifier at all, reaches here. `Ident::new` would panic on it, and a
    // proc-macro panic carries no span. Fall back to the crate's own name:
    // the compiler then reports THAT as unresolved, which is true, and
    // reports it against the user's own code.
    if syn::parse_str::<Ident>(&name).is_ok() {
        name
    } else {
        fallback.to_owned()
    }
}

/// Decide the root from what the resolver settled for each candidate.
///
/// Split out from the lookup so it can be tested against every combination
/// without a manifest on disk. `runtime` and `umbrella` are the outcomes of
/// asking for `RUNTIME` and `UMBRELLA` in that order.
fn classify(runtime: Lookup, umbrella: Lookup) -> Root {
    match runtime {
        Lookup::Found(found) => Root::Runtime(spelling(found, RUNTIME)),
        Lookup::Undecidable => Root::Undecidable,
        Lookup::Absent => match umbrella {
            Lookup::Found(found) => Root::Umbrella(spelling(found, UMBRELLA)),
            Lookup::Undecidable => Root::Undecidable,
            Lookup::Absent => Root::Unresolved,
        },
    }
}

/// Ask the resolver whether the consuming package names one crate.
///
/// Only `CrateNotFound` is an absence. Every other error means the manifest
/// was never read, and answering those with "you forgot a dependency" would
/// report a cause the resolver cannot observe with the confidence of one it
/// can, while breaking a build that used to work.
fn look_up(name: &str) -> Lookup {
    match crate_name(name) {
        Ok(found) => Lookup::Found(found),
        Err(proc_macro_crate::Error::CrateNotFound { .. }) => Lookup::Absent,
        Err(_) => Lookup::Undecidable,
    }
}

/// Which crates the consuming package names, decided once per compilation.
///
/// Memoised on `CARGO_MANIFEST_DIR` rather than resolved at each of the
/// nineteen call sites. The point is not the two `fs::metadata` calls
/// `proc-macro-crate`'s own cache still makes: it is that one expansion must
/// not be able to mix two answers. Without this, a manifest edited while
/// rustc is running could put `::cerulion::core` in one generated impl and
/// `::cerulion_core` in the next.
///
/// Keyed rather than a bare `OnceLock` because a proc-macro server can serve
/// more than one package in one process, where the key really does change.
fn resolve() -> Root {
    static CACHE: Mutex<Option<(String, Root)>> = Mutex::new(None);

    let key = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    // A poisoned diagnostic cache must never wedge the expansion it serves.
    let mut cache = CACHE
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    if let Some((cached_key, root)) = cache.as_ref() {
        if *cached_key == key {
            return root.clone();
        }
    }
    let root = resolve_uncached();
    *cache = Some((key, root.clone()));
    root
}

/// Ask the resolver which crates the consuming package names.
fn resolve_uncached() -> Root {
    let runtime = look_up(RUNTIME);
    // Only asked when the runtime was not found, so a package that names the
    // runtime pays one lookup rather than two.
    let umbrella = if matches!(runtime, Lookup::Absent) {
        look_up(UMBRELLA)
    } else {
        Lookup::Absent
    };
    classify(runtime, umbrella)
}

/// The path every generated runtime item hangs off.
///
/// Interpolate it: `quote! { #root::graph::node::NodeEntry }`.
pub(crate) fn root() -> TokenStream {
    resolve().path()
}

/// The text a package that names neither crate is shown.
///
/// Built here rather than inline so the test reads the message the user
/// reads, instead of an escaped string literal pulled back out of a
/// `TokenStream`.
fn unresolved_message() -> String {
    format!(
        "a package that uses the Cerulion node macros must name the runtime in \
         its own Cargo.toml, because the generated code reaches the runtime by \
         an absolute path. Either add the umbrella crate, which brings the \
         runtime, the macros and the message types together:\n\
         \n    [dependencies]\n    {UMBRELLA} = \"{VERSION}\"\n\
         \nor name the runtime crate directly:\n\
         \n    [dependencies]\n    {RUNTIME} = \"{VERSION}\"\n\
         \nA re-export in a crate you already depend on cannot stand in for \
         either of those: Cargo does not hand a package its transitive \
         dependencies, so the name has to be in this package's own manifest."
    )
}

/// The diagnostic for a package that names neither crate, or `None` when
/// the root resolved.
///
/// Emitted beside the expansion rather than instead of it. Without it the
/// compiler reports `cannot find cerulion_core in the crate root` against a
/// path the user never wrote, in generated code they cannot see; with it, the
/// first error names the two manifests that fix it.
pub(crate) fn unresolved_diagnostic(span: Span) -> Option<TokenStream> {
    if resolve() != Root::Unresolved {
        return None;
    }
    Some(syn::Error::new(span, unresolved_message()).to_compile_error())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rendered(root: &Root) -> String {
        root.path().to_string()
    }

    fn found(name: &str) -> Lookup {
        Lookup::Found(FoundCrate::Name(name.to_owned()))
    }

    #[test]
    fn naming_the_runtime_emits_the_runtime_path() {
        let root = classify(found("cerulion_core"), Lookup::Absent);
        assert_eq!(root, Root::Runtime("cerulion_core".into()));
        assert_eq!(rendered(&root), ":: cerulion_core");
    }

    #[test]
    fn naming_only_the_umbrella_emits_the_umbrella_path() {
        let root = classify(Lookup::Absent, found("cerulion"));
        assert_eq!(root, Root::Umbrella("cerulion".into()));
        assert_eq!(rendered(&root), ":: cerulion :: core");
    }

    #[test]
    fn naming_both_emits_the_runtime_path() {
        // The compatibility rule. A package written before this resolver
        // existed names the runtime, and some also name the umbrella; the
        // tokens they get must not move.
        let root = classify(found("cerulion_core"), found("cerulion"));
        assert_eq!(rendered(&root), ":: cerulion_core");
    }

    #[test]
    fn a_renamed_dependency_is_carried_through() {
        assert_eq!(rendered(&classify(found("cer"), Lookup::Absent)), ":: cer");
        assert_eq!(
            rendered(&classify(Lookup::Absent, found("cer_umbrella"))),
            ":: cer_umbrella :: core"
        );
    }

    #[test]
    fn a_rename_that_is_not_an_identifier_falls_back_to_the_crate_name() {
        // `Ident::new` would panic on either of these, and a proc-macro panic
        // carries no span at all. The fallback path is wrong for that package
        // but it is a PATH, so the compiler reports the missing name against
        // the user's own code.
        for key in ["crate", "self", "123", ""] {
            assert_eq!(
                rendered(&classify(found(key), Lookup::Absent)),
                ":: cerulion_core",
                "a `{key}` rename must not reach Ident::new"
            );
        }
        assert_eq!(
            rendered(&classify(Lookup::Absent, found("crate"))),
            ":: cerulion :: core"
        );
    }

    #[test]
    fn the_runtime_compiling_itself_is_spelled_by_name_not_by_crate() {
        // `crate` would break the runtime's own doctests, which Cargo reports
        // as `Itself` and which reach the runtime through `--extern`.
        let root = classify(Lookup::Found(FoundCrate::Itself), Lookup::Absent);
        assert_eq!(rendered(&root), ":: cerulion_core");
    }

    #[test]
    fn the_umbrella_compiling_itself_is_spelled_by_name() {
        let root = classify(Lookup::Absent, Lookup::Found(FoundCrate::Itself));
        assert_eq!(rendered(&root), ":: cerulion :: core");
    }

    #[test]
    fn naming_neither_falls_back_to_the_runtime_path_and_is_reported() {
        let root = classify(Lookup::Absent, Lookup::Absent);
        assert_eq!(root, Root::Unresolved);
        // The fallback is the pre-resolver spelling, so a package in this
        // state compiles exactly as badly as it did before, with one named
        // error added in front.
        assert_eq!(rendered(&root), ":: cerulion_core");
        assert!(unresolved_diagnostic_applies(&root));
    }

    /// An unreadable manifest is not a missing dependency, and must not be
    /// reported as one.
    ///
    /// Before this resolver existed the macros emitted `::cerulion_core`
    /// unconditionally, so a driver that is not cargo, or one whose manifest
    /// cannot be read or parsed, compiled fine as long as it passed
    /// `--extern cerulion_core`. Collapsing those onto the missing-dependency
    /// arm would break that build AND name the wrong cause.
    #[test]
    fn an_unreadable_manifest_is_not_reported_as_a_missing_dependency() {
        for root in [
            classify(Lookup::Undecidable, Lookup::Absent),
            classify(Lookup::Absent, Lookup::Undecidable),
            classify(Lookup::Undecidable, Lookup::Undecidable),
        ] {
            assert_eq!(root, Root::Undecidable);
            assert_eq!(rendered(&root), ":: cerulion_core");
            assert!(
                !unresolved_diagnostic_applies(&root),
                "an undecidable manifest must carry no diagnostic"
            );
        }
    }

    /// Whether `unresolved_diagnostic` would emit for this root, without
    /// needing a real manifest underneath.
    fn unresolved_diagnostic_applies(root: &Root) -> bool {
        *root == Root::Unresolved
    }

    /// Every macro this crate exports must route its expansion past the
    /// missing-dependency diagnostic.
    ///
    /// The fallback in `Root::path` is what makes a macro that skips it
    /// silent rather than broken: it would emit the pre-resolver spelling and
    /// the user would get a resolution error against a path they never wrote,
    /// which is the whole condition this module exists to name. A source walk
    /// rather than a hand-written list, so a macro added later is covered the
    /// day it is added.
    ///
    /// Read over a COMMENT-STRIPPED view, and matched on the body's TAIL
    /// expression rather than anywhere in it: a comment naming the call, or a
    /// call on one error arm while the success path skips it, must not
    /// satisfy this.
    #[test]
    fn every_exported_macro_routes_past_the_missing_dependency_diagnostic() {
        let lib = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/lib.rs"),
        )
        .expect("read this crate's lib.rs");

        // The entry points are the `pub fn`s directly under a `#[proc_macro`
        // attribute. Each body runs to the next line that is a closing brace
        // in column zero, which is how this file is formatted.
        let lines: Vec<&str> = lib.lines().collect();
        let mut checked = Vec::new();
        for (i, line) in lines.iter().enumerate() {
            if !line.starts_with("#[proc_macro") {
                continue;
            }
            let signature = lines[i + 1..]
                .iter()
                .position(|l| l.starts_with("pub fn "))
                .map(|offset| i + 1 + offset)
                .expect("a proc-macro attribute is followed by its function");
            let end = lines[signature..]
                .iter()
                .position(|l| *l == "}")
                .map(|offset| signature + offset)
                .expect("the function body closes at column zero");
            let name = lines[signature]
                .trim_start_matches("pub fn ")
                .split('(')
                .next()
                .expect("a function signature names the function")
                .to_owned();

            let code = code_only(&lines[signature..=end].join("\n"));
            let tail = code
                .lines()
                .rev()
                .find(|l| !l.trim().is_empty() && l.trim() != "}")
                .unwrap_or("")
                .to_owned();
            // Either the tail expression is the helper, or the body's last
            // statement is a `match` on the diagnostic. Statement level is
            // indentation exactly four: an error arm deeper inside the body
            // would not satisfy this, and neither would a comment.
            let routed_at_statement_level = code
                .lines()
                .any(|l| l.starts_with("    match crate_root::unresolved_diagnostic"));
            assert!(
                tail.contains("with_root_diagnostic") || routed_at_statement_level,
                "`{name}` returns its expansion without passing it through the \
                 crate-root diagnostic, so a package that names neither \
                 `{UMBRELLA}` nor `{RUNTIME}` would get a resolution error \
                 against a generated path instead of being told which \
                 dependency is missing. Its body tail reads: {tail}"
            );
            checked.push(name);
        }

        // Anti-tautology: a walk that found nothing would pass silently.
        assert_eq!(
            checked,
            vec![
                "derive_cerulion_state".to_owned(),
                "cerulion_node".to_owned(),
                "cerulion_node_impl".to_owned(),
            ],
            "the walk must reach every exported macro"
        );
    }

    /// Strip `//` line comments and `/* */` block comments, which nest.
    ///
    /// String literals are deliberately not modelled: nothing in the walked
    /// bodies carries a comment marker inside a literal, and the anti-vacuity
    /// arm below would catch a stripper that ate real code.
    fn code_only(src: &str) -> String {
        let bytes: Vec<char> = src.chars().collect();
        let mut out = String::with_capacity(src.len());
        let mut depth = 0usize;
        let mut i = 0;
        while i < bytes.len() {
            if depth == 0 && bytes[i] == '/' && bytes.get(i + 1) == Some(&'/') {
                while i < bytes.len() && bytes[i] != '\n' {
                    i += 1;
                }
                continue;
            }
            if bytes[i] == '/' && bytes.get(i + 1) == Some(&'*') {
                depth += 1;
                i += 2;
                continue;
            }
            if depth > 0 && bytes[i] == '*' && bytes.get(i + 1) == Some(&'/') {
                depth -= 1;
                i += 2;
                continue;
            }
            if depth == 0 {
                out.push(bytes[i]);
            } else if bytes[i] == '\n' {
                out.push('\n');
            }
            i += 1;
        }
        out
    }

    #[test]
    fn the_comment_stripper_removes_both_syntaxes_and_keeps_the_code() {
        assert_eq!(
            code_only("let a = 1; // gone\nlet b = 2;"),
            "let a = 1; \nlet b = 2;"
        );
        assert_eq!(code_only("a/* x */b"), "ab");
        assert_eq!(code_only("a/* /* deep */ still */b"), "ab");
        // A `//` inside a block comment is part of the comment, not a new one.
        assert_eq!(code_only("a/* // */b"), "ab");
        // Anti-vacuity: a stripper that ate everything would make every
        // absence assertion above pass for the wrong reason.
        assert_eq!(
            code_only("with_root_diagnostic(x)"),
            "with_root_diagnostic(x)"
        );
    }

    #[test]
    fn the_diagnostic_names_both_manifests_and_the_reason() {
        let text = unresolved_message();
        assert!(
            text.contains(&format!("cerulion = \"{VERSION}\"")),
            "{text}"
        );
        assert!(
            text.contains(&format!("cerulion_core = \"{VERSION}\"")),
            "{text}"
        );
        assert!(
            text.contains("transitive dependencies"),
            "the message must say why a re-export cannot stand in: {text}"
        );
    }

    /// The version in the advice is this crate's own, not a literal.
    ///
    /// A literal would tell a user to add an outdated dependency from the
    /// first workspace bump onwards, and the assertion above would keep
    /// passing because it would be reading the same literal back.
    #[test]
    fn the_recommended_version_follows_this_crate() {
        assert_eq!(VERSION, env!("CARGO_PKG_VERSION"));
        assert!(
            unresolved_message().contains(env!("CARGO_PKG_VERSION")),
            "the message must recommend the version this crate ships at"
        );
    }
}
