// SPDX-License-Identifier: AGPL-3.0-only
//! The **Team & access page**.
//!
//! The owner-facing surface for a robot's access list: which accounts and devices
//! are revoked, this account's registered desks, and the revoke actions — served by
//! the account service and rendered INSIDE Cerulion Studio's in-app webview.
//!
//! ## Why hosted, not bundled
//!
//! The page is a HOSTED web surface, not a Studio asset that re-implements the data.
//! Two consequences that are the whole point:
//!
//! 1. It **survives the shell swap** — the shell only has to open
//!    a webview, so replacing the shell never rewrites this page.
//! 2. Access logic lives with the service that **owns** access, so a change reaches
//!    every desk without shipping a Studio release.
//!
//! ## The document is credential-free; the DATA is session-authed
//!
//! `GET /team` is **not** session-gated, deliberately: a browser navigation cannot
//! carry an `Authorization` header, and the document contains **no account data** —
//! it is inert chrome plus the script that fetches it. Everything of value comes
//! from the existing session-authed JSON endpoints
//! (`/v1/me`, `/v1/devices`, `/v1/robots`, `/v1/robots/{id}/access`,
//! `POST /v1/robots/{id}/revoke`, `POST /v1/devices/{id}/revoke`), each of which
//! authenticates the bearer and enforces the owner-only rule on its own. So the
//! open document leaks nothing an unauthenticated caller could not already fetch:
//! the page's own HTML.
//!
//! The token reaches the page over the **host handshake**, never a URL: the page
//! posts `{"cmd":"team_ready"}` over the host's IPC bridge and the host answers by
//! calling `window.__cerulion_session({session_token, refresh_token, …})`. No
//! credential is ever placed in a URL, a query string, a redirect, or this
//! service's access log. Opened in a plain browser (no host) the page renders a
//! loud "open this from Cerulion Studio" state.
//!
//! ## Self-containment is ENFORCED, not just documented
//!
//! The asset inlines all CSS + JS and references no external host. That claim is
//! backed by a `Content-Security-Policy` (`default-src 'none'` with a same-origin
//! `connect-src`) served with the document, so a future edit that adds a CDN font
//! or script is BLOCKED at runtime rather than silently shipping a page that
//! phones out — plus a unit test that fails on any `http(s)://` in the asset.

use axum::http::header::{
    HeaderName, HeaderValue, CACHE_CONTROL, CONTENT_SECURITY_POLICY, CONTENT_TYPE,
    X_CONTENT_TYPE_OPTIONS,
};
use axum::response::{IntoResponse, Response};

/// The Team & access page: one self-contained HTML document (inline CSS + JS, no
/// external references). Served verbatim by the crate's `GET /team` handler.
pub const TEAM_PAGE_HTML: &str = include_str!("../assets/team.html");

/// `Referrer-Policy` — not a typed constant in `http`, so name it once here.
const REFERRER_POLICY: &str = "referrer-policy";

/// The **complete** page → host verb vocabulary of the Studio handshake.
///
/// # Cross-repo contract
///
/// The other half of this vocabulary lives in a DIFFERENT repository —
/// `cerulion-studio/native/studio-shell/src/team_panel.rs`
/// (`TEAM_PAGE_TO_HOST_VERBS`), parsed by that repo's
/// `src/bin/shell.rs::parse_js_command`. Neither repo can `include_str!` the
/// other, so this constant + its twin are hand-mirrored **on purpose** and each
/// side pins its own list EXACTLY (not merely "contains"):
///
/// * here — the served asset emits exactly these verbs and no others,
/// * there — the shell parses exactly these verbs and no others.
///
/// Exactness is what makes the mirror hold: a unilateral addition or rename on
/// either side fails THAT side's test, which names this constant and its twin,
/// so the author is sent to the other repo before the drift can ship. (A verb
/// silently DELETED from both would pass both, which is why each list is also
/// transcribed into a hand oracle in the tests below.)
///
/// # A third surface speaks a SUBSET
///
/// The Studio panel's webview can also hold Studio's OWN offline chrome,
/// `cerulion-studio/native/studio-shell/assets/team_fallback.html`, shown when
/// this page cannot be (no desk sign-in, an unreadable auth store, an unreachable
/// or misconfigured service). It emits a strict subset — `team_close` and
/// `team_retry`, never the session verbs — pinned exactly on the Studio side by
/// `the_local_fallback_chrome_speaks_a_subset_of_the_declared_vocabulary`.
///
/// So renaming a verb that BOTH documents speak (today: `team_close`,
/// `team_retry`) touches THREE files — `assets/team.html`, the Studio mirror of
/// this constant, and that fallback asset — and each has its own failing pin.
/// `team_ready` and `team_session_update` are this page's alone.
pub const TEAM_PAGE_TO_HOST_VERBS: [&str; 4] = [
    "team_close",
    "team_ready",
    "team_retry",
    "team_session_update",
];

/// The **complete** host → page entry points of the Studio handshake: globals
/// this page installs for the shell to call. Mirrored in the Studio repo exactly
/// as [`TEAM_PAGE_TO_HOST_VERBS`] is — see that constant's cross-repo note.
pub const TEAM_HOST_TO_PAGE_FNS: [&str; 3] = [
    "__cerulion_request_close",
    "__cerulion_session",
    "__cerulion_session_saved",
];

/// Every session-authed endpoint the page calls, as the EXACT source expression
/// the asset uses to build each URL.
///
/// Pinning the whole expression (not the `/v1/robots/` prefix) is the point: a
/// rename of a SUFFIX — `/access` → `/acl`, `/revoke` → `/deny` — is invisible to
/// a prefix pin, and the failure mode is a page that renders an empty or
/// permanently-erroring view against a service that changed underneath it.
pub const TEAM_PAGE_ENDPOINT_EXPRESSIONS: [&str; 6] = [
    "\"/v1/me\"",
    "\"/v1/devices\"",
    "\"/v1/robots\"",
    "\"/v1/robots/\" + encodeURIComponent(rb.robot_id) + \"/access\"",
    "\"/v1/robots/\" + encodeURIComponent(rb.robot_id) + \"/revoke\"",
    "\"/v1/devices/\" + encodeURIComponent(d.device_id) + \"/revoke\"",
];

/// The `Content-Security-Policy` served with the page — the runtime ENFORCEMENT of
/// its self-containment contract.
///
/// * `default-src 'none'` — nothing loads by default; a CDN script/font/image added
///   by a future edit is blocked instead of silently phoning out.
/// * `style-src`/`script-src 'unsafe-inline'` — the page's own inline `<style>` +
///   `<script>` (the only code it has). No nonce/hash is used, so `'unsafe-inline'`
///   is honoured; and because `default-src` is `'none'`, no *external* script or
///   stylesheet can load even so.
/// * `connect-src 'self'` — the same-origin XHR to this service's session-authed
///   JSON endpoints, and nothing else. A token can never be posted off-origin.
/// * `frame-ancestors 'none'` + `base-uri 'none'` + `form-action 'none'` — the page
///   cannot be framed, cannot have its relative URLs re-based, and submits nothing.
pub const TEAM_PAGE_CSP: &str = "default-src 'none'; \
     style-src 'unsafe-inline'; \
     script-src 'unsafe-inline'; \
     connect-src 'self'; \
     base-uri 'none'; \
     form-action 'none'; \
     frame-ancestors 'none'";

/// `GET /team` — serve the Team & access page.
///
/// Deliberately NOT session-gated (see the module docs): the document carries no
/// account data, and every datum it renders comes from a session-authed endpoint.
/// `no-store` keeps a credentialed surface's shell out of every cache and makes a
/// page fix reach desks on the next open.
pub async fn team_page() -> Response {
    let mut resp = TEAM_PAGE_HTML.into_response();
    let headers = resp.headers_mut();
    headers.insert(
        CONTENT_TYPE,
        HeaderValue::from_static("text/html; charset=utf-8"),
    );
    headers.insert(CACHE_CONTROL, HeaderValue::from_static("no-store"));
    headers.insert(
        CONTENT_SECURITY_POLICY,
        HeaderValue::from_static(TEAM_PAGE_CSP),
    );
    headers.insert(X_CONTENT_TYPE_OPTIONS, HeaderValue::from_static("nosniff"));
    headers.insert(
        HeaderName::from_static(REFERRER_POLICY),
        HeaderValue::from_static("no-referrer"),
    );
    resp
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every `--cer-*` token the page defines, against a HAND oracle transcribed
    /// from `cerulion-studio/native/studio-shell/palette.toml` — the single source
    /// of truth for the Studio design language.
    ///
    /// This page cannot `include_str!` across repos, so those values are duplicated
    /// in `assets/team.html` on purpose. This test is what keeps the duplication
    /// IN STEP: an edit that quietly re-tints the page (or the studio palette moving
    /// without this page following) fails here instead of shipping a Team page that
    /// looks foreign next to the rest of Studio.
    #[test]
    fn palette_tokens_match_the_studio_shell_palette() {
        // (token, hex) — transcribed by hand from studio-shell/palette.toml.
        let oracle = [
            ("bg-stage", "#10161f"),
            ("bg-header", "#0c1219"),
            ("bg-hover", "#141d2a"),
            ("bg-elevated", "#182338"),
            ("bg-badge", "#1a2636"),
            ("bg-active", "#223252"),
            ("border", "#223047"),
            ("border-strong", "#2c3f5c"),
            ("border-subtle", "#1a2536"),
            ("text-primary", "#e6edf6"),
            ("text-bright", "#cfe0f5"),
            ("text-muted", "#7c93b3"),
            ("text-faint", "#6a7f9e"),
            ("text-dim", "#55688a"),
            ("accent", "#4b86e0"),
            ("accent-soft", "#9db8e6"),
            ("accent-bright", "#b9d5ff"),
            ("accent-hover", "#5c95ec"),
            ("accent-press", "#3d76cf"),
            ("success", "#6fd39a"),
            ("success-bright", "#4ad07f"),
            ("error", "#ff8f8f"),
            ("warn", "#e6b45a"),
            ("scrollbar", "#22314a"),
            ("scrollbar-hover", "#2c3f5c"),
            ("focus-ring", "#4b86e0"),
        ];
        for (token, hex) in oracle {
            let decl = format!("--cer-{token}: {hex};");
            assert!(
                TEAM_PAGE_HTML.contains(&decl),
                "assets/team.html must declare `{decl}` to match the Studio shell palette \
                 (cerulion-studio/native/studio-shell/palette.toml). If the studio palette \
                 changed, update BOTH the asset and this oracle."
            );
        }
        // Anti-tautology: a token that is NOT in the studio palette must not be
        // silently invented, and the assert above must be able to FAIL.
        assert!(
            !TEAM_PAGE_HTML.contains("--cer-bg-stage: #000000;"),
            "the oracle above would pass vacuously if any hex matched"
        );
    }

    /// The non-colour design scale mirrors `studio.css` (spacing 4/8 grid, radii,
    /// type ramp, motion) — the other half of "looks like the rest of Studio".
    #[test]
    fn design_scale_mirrors_the_shared_studio_stylesheet() {
        for decl in [
            "--font-sans:",
            "--font-mono:",
            "--fs-meta: 11px;",
            "--fs-body: 12.5px;",
            "--fs-title: 13px;",
            "--sp-4: 8px;",
            "--sp-6: 12px;",
            "--sp-8: 16px;",
            "--r-sm: 5px;",
            "--r-md: 7px;",
            "--r-lg: 9px;",
            "--r-full: 999px;",
            "--ease-out: cubic-bezier(.2,.8,.2,1);",
            "--dur-fast: 120ms;",
        ] {
            assert!(
                TEAM_PAGE_HTML.contains(decl),
                "assets/team.html is missing the shared design token `{decl}` \
                 (mirror of cerulion-studio/native/studio-shell/assets/studio.css)"
            );
        }
        // Dark-first + reduced-motion + keyboard focus, like every Studio surface.
        assert!(TEAM_PAGE_HTML.contains("prefers-reduced-motion"));
        assert!(TEAM_PAGE_HTML.contains(":focus-visible"));
        assert!(TEAM_PAGE_HTML.contains("content=\"dark\""));
    }

    /// The asset is SELF-CONTAINED: no external host, no external subresource. The
    /// served CSP enforces this at runtime; this test catches it at `cargo test`.
    #[test]
    fn page_is_self_contained_with_no_external_references() {
        assert!(
            !TEAM_PAGE_HTML.contains("http://") && !TEAM_PAGE_HTML.contains("https://"),
            "assets/team.html must not reference any external URL — it is served as a \
             self-contained document (and the CSP's `default-src 'none'` would block it)"
        );
        for tag in ["<link ", "<img ", "<iframe", "@import", "<object", "<embed"] {
            assert!(
                !TEAM_PAGE_HTML.contains(tag),
                "assets/team.html must not use `{tag}` — all CSS/JS/graphics are inline"
            );
        }
        // Inline style + script ARE present (the page is not accidentally empty).
        assert!(TEAM_PAGE_HTML.contains("<style>") && TEAM_PAGE_HTML.contains("<script>"));
    }

    // ── JS source helpers (used by the structural pins below) ────────────────

    /// The body of `function <name>(…) { … }` in `src`, brace-matched with string
    /// literals and line comments skipped, WITHOUT the outer braces.
    ///
    /// Deliberately does NOT understand regex literals — no function this is
    /// applied to contains one, and a silent mis-parse is impossible: an
    /// unbalanced scan panics rather than returning a short body that would make
    /// the caller's "no gate here" assertion pass vacuously.
    fn js_function_body(src: &str, name: &str) -> String {
        js_body_after(
            src,
            &format!("function {name}("),
            &format!("function {name}"),
        )
    }

    /// The body of the function ASSIGNED at `window.<name> = function …` — the
    /// host → page entry points, which are expressions rather than declarations.
    fn js_assigned_function_body(src: &str, name: &str) -> String {
        js_body_after(
            src,
            &format!("window.{name} = function"),
            &format!("window.{name}"),
        )
    }

    /// The brace-matched `{ … }` block following the first `needle`, WITHOUT its
    /// outer braces. `label` names the construct in panics.
    fn js_body_after(src: &str, needle: &str, label: &str) -> String {
        let start = src
            .find(needle)
            .unwrap_or_else(|| panic!("assets/team.html must define `{needle}`"));
        let open = start
            + src[start..]
                .find('{')
                .unwrap_or_else(|| panic!("`{label}` has no body"));
        let bytes = src.as_bytes();
        let mut depth = 0usize;
        let mut i = open;
        let mut quote: Option<u8> = None;
        while i < bytes.len() {
            let c = bytes[i];
            if let Some(q) = quote {
                if c == b'\\' {
                    i += 2;
                    continue;
                }
                if c == q {
                    quote = None;
                }
                i += 1;
                continue;
            }
            match c {
                b'"' | b'\'' => quote = Some(c),
                b'/' if bytes.get(i + 1) == Some(&b'/') => {
                    while i < bytes.len() && bytes[i] != b'\n' {
                        i += 1;
                    }
                    continue;
                }
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        return src[open + 1..i].to_owned();
                    }
                }
                _ => {}
            }
            i += 1;
        }
        panic!("unbalanced braces in `{label}` — assets/team.html is malformed");
    }

    /// `s` with every run of whitespace collapsed to one space, trimmed.
    fn squash(s: &str) -> String {
        s.split_whitespace().collect::<Vec<_>>().join(" ")
    }

    /// The first offset in `hay` where `ident` appears as a WHOLE JavaScript
    /// identifier — i.e. bounded by a non-identifier byte on both sides.
    ///
    /// A plain `hay.find(ident)` is a SUBSTRING scan, and the revoke entry points
    /// are ordinary lowercase words (`submit` above all): a confirmation gate
    /// hidden in a wrapper whose name merely CONTAINS one — `submitAfterConfirm(…)`
    /// — scored as a bare immediately-calling handler and walked straight past the
    /// call-site oracle below, while every interior pin stayed green (the
    /// capitalized twin `confirmThenSubmit` was caught, so the hole needed only a
    /// lowercase spelling). Whole-identifier matching closes it: a wrapper name is
    /// by definition a DIFFERENT identifier.
    fn find_identifier(hay: &str, ident: &str) -> Option<usize> {
        debug_assert!(
            ident
                .bytes()
                .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'$'),
            "find_identifier takes an identifier, not a pattern"
        );
        fn is_ident_byte(b: u8) -> bool {
            b.is_ascii_alphanumeric() || b == b'_' || b == b'$'
        }
        let bytes = hay.as_bytes();
        let mut from = 0usize;
        while let Some(rel) = hay[from..].find(ident) {
            let at = from + rel;
            let end = at + ident.len();
            let before_free = at == 0 || !is_ident_byte(bytes[at - 1]);
            let after_free = end >= bytes.len() || !is_ident_byte(bytes[end]);
            if before_free && after_free {
                return Some(at);
            }
            // `ident` is ASCII, so `at + 1` is always a char boundary.
            from = at + 1;
        }
        None
    }

    /// Whether `s` is nothing but the opening of a zero-statement function literal
    /// (`function (…) {`) — i.e. the handler invokes its target immediately, with
    /// no statement of its own in between.
    fn is_bare_handler_open(s: &str) -> bool {
        let Some(rest) = s.strip_prefix("function (") else {
            return false;
        };
        let Some(params) = rest.strip_suffix(") {") else {
            return false;
        };
        params
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == ',' || c == ' ')
    }

    /// Every DISTINCT first-argument expression handed to the page's `api(` helper,
    /// sorted — i.e. the COMPLETE set of endpoints the page calls.
    ///
    /// Extracted by scanning to the first top-level `,`/`)` (strings and nested
    /// parens skipped), so a concatenated URL (`"/v1/robots/" + … + "/revoke"`)
    /// comes back whole. The helper's own definition (`function api(`) is skipped.
    fn api_call_expressions(src: &str) -> Vec<String> {
        let bytes = src.as_bytes();
        let mut out: Vec<String> = Vec::new();
        let mut from = 0usize;
        while let Some(rel) = src[from..].find("api(") {
            let at = from + rel;
            from = at + "api(".len();
            // Not a suffix of a longer identifier, and not the definition.
            let prev = src[..at].chars().next_back().unwrap_or(' ');
            if prev.is_ascii_alphanumeric() || prev == '_' || prev == '.' {
                continue;
            }
            if src[..at].trim_end().ends_with("function") {
                continue;
            }
            let mut i = from;
            let mut depth = 0i32;
            let mut quote: Option<u8> = None;
            while i < bytes.len() {
                let c = bytes[i];
                if let Some(q) = quote {
                    if c == b'\\' {
                        i += 2;
                        continue;
                    }
                    if c == q {
                        quote = None;
                    }
                    i += 1;
                    continue;
                }
                match c {
                    b'"' | b'\'' => quote = Some(c),
                    b'(' | b'[' | b'{' => depth += 1,
                    b')' | b']' | b'}' => {
                        if depth == 0 {
                            break;
                        }
                        depth -= 1;
                    }
                    b',' if depth == 0 => break,
                    _ => {}
                }
                i += 1;
            }
            assert!(
                i < bytes.len(),
                "unterminated `api(` call in assets/team.html"
            );
            let expr = squash(&src[from..i]);
            if !out.contains(&expr) {
                out.push(expr);
            }
        }
        out.sort();
        out
    }

    /// Every distinct `cmd: "<verb>"` literal the asset emits, sorted.
    fn emitted_verbs(src: &str) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        let mut rest = src;
        while let Some(i) = rest.find("cmd: \"") {
            let after = &rest[i + 6..];
            let end = after.find('"').expect("an unterminated cmd literal");
            let verb = after[..end].to_owned();
            if !out.contains(&verb) {
                out.push(verb);
            }
            rest = &after[end..];
        }
        out.sort();
        out
    }

    /// Every distinct `window.__cerulion_*` global named anywhere in the asset
    /// (definitions and prose alike), sorted — the host → page surface.
    fn host_entry_points(src: &str) -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        let mut rest = src;
        while let Some(i) = rest.find("window.__cerulion") {
            let after = &rest[i + "window.".len()..];
            let end = after
                .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
                .unwrap_or(after.len());
            let name = after[..end].to_owned();
            if !out.contains(&name) {
                out.push(name);
            }
            rest = &after[end..];
        }
        out.sort();
        out
    }

    // ── Handshake vocabulary (the cross-repo mirror) ─────────────────────────

    /// The host handshake contract, pinned from the accountd side EXACTLY — the
    /// asset emits every verb in [`TEAM_PAGE_TO_HOST_VERBS`] and *no others*, and
    /// installs every global in [`TEAM_HOST_TO_PAGE_FNS`] and *no others*.
    ///
    /// Exactness (not "contains") is the cross-repo drift catcher: the twin lists
    /// live in `cerulion-studio/native/studio-shell/src/team_panel.rs`, and an
    /// added or renamed verb on THIS side now fails here — naming the twin — so
    /// the author cannot ship half a rename. The oracle is transcribed by hand so
    /// a verb deleted from the constant is caught too.
    #[test]
    fn page_speaks_exactly_the_studio_handshake_vocabulary() {
        // Hand oracle — keep byte-identical to the Studio repo's mirror.
        let want_verbs = [
            "team_close",
            "team_ready",
            "team_retry",
            "team_session_update",
        ];
        let mut declared = TEAM_PAGE_TO_HOST_VERBS.to_vec();
        declared.sort_unstable();
        assert_eq!(
            declared, want_verbs,
            "TEAM_PAGE_TO_HOST_VERBS drifted from the hand oracle — if this is intentional, \
             update `cerulion-studio/native/studio-shell/src/team_panel.rs` \
             (TEAM_PAGE_TO_HOST_VERBS) in the same change"
        );
        assert_eq!(
            emitted_verbs(TEAM_PAGE_HTML),
            want_verbs,
            "assets/team.html must emit EXACTLY the handshake verbs — the Studio shell \
             (native/studio-shell/src/bin/shell.rs::parse_js_command) parses exactly these; \
             an extra verb is dropped on the floor and a missing one breaks the page"
        );

        let want_fns = [
            "__cerulion_request_close",
            "__cerulion_session",
            "__cerulion_session_saved",
        ];
        assert_eq!(TEAM_HOST_TO_PAGE_FNS, want_fns);
        assert_eq!(
            host_entry_points(TEAM_PAGE_HTML),
            want_fns,
            "assets/team.html must install EXACTLY the host->page globals the shell calls \
             (see `eval_in_team_webview` in the Studio shell)"
        );
        // Each one is really DEFINED here, not merely mentioned in prose.
        for f in TEAM_HOST_TO_PAGE_FNS {
            assert!(
                TEAM_PAGE_HTML.contains(&format!("window.{f} = function")),
                "assets/team.html must define `window.{f}`"
            );
        }
        // The bridge the host installs.
        assert!(TEAM_PAGE_HTML.contains("window.ipc.postMessage"));
        // No credential may ride a URL: the page must never build a token query param.
        assert!(
            !TEAM_PAGE_HTML.contains("session_token="),
            "the session token must never be placed in a URL/query string"
        );
    }

    /// The page consumes EXACTLY the session-authed access endpoints — pinned
    /// as the FULL URL expression, so a renamed *suffix* (`/access`, `/revoke`)
    /// fails here instead of silently emptying the page against a moved route.
    ///
    /// "Only" is meant literally: the assertion is set EQUALITY against every
    /// first argument the page hands `api(`, not containment. That is what makes
    /// [`TEAM_PAGE_ENDPOINT_EXPRESSIONS`] a usable machine input rather than
    /// prose — `acceptance_test.rs` DERIVES its unauthenticated-401 floor from
    /// the same constant, so a seventh endpoint added to the page fails HERE
    /// (the constant is stale) and, once added there, is automatically probed
    /// for its auth gate. Under a plain containment assertion both could drift.
    #[test]
    fn page_calls_only_the_session_authed_endpoints() {
        let mut want: Vec<String> = TEAM_PAGE_ENDPOINT_EXPRESSIONS
            .iter()
            .map(|s| (*s).to_owned())
            .collect();
        want.sort();
        assert_eq!(
            api_call_expressions(TEAM_PAGE_HTML),
            want,
            "assets/team.html must call EXACTLY the endpoints in \
             TEAM_PAGE_ENDPOINT_EXPRESSIONS — if a route moved or one was added, update BOTH \
             the asset and the constant (the 401 floor in tests/acceptance_test.rs is derived \
             from it, so an unlisted endpoint ships with its auth gate unproven)"
        );
        // Plus the unauthenticated refresh exchange (bearer-less by design).
        assert!(TEAM_PAGE_HTML.contains("\"/v1/auth/refresh\""));
        // Anti-tautology: the pin is on the WHOLE expression, so a suffix-only
        // rename is caught. (A prefix pin like `/v1/robots/` would not be.)
        assert!(
            !TEAM_PAGE_HTML.contains("+ \"/acl\""),
            "the oracle above would pass vacuously if suffixes were unpinned"
        );
        // Every data call goes out bearer-authed.
        assert!(TEAM_PAGE_HTML.contains("\"Bearer \" + SESSION.session_token"));
        // Cookies are never used (accountd has no cookie session; be explicit).
        assert!(TEAM_PAGE_HTML.contains("credentials: \"omit\""));
    }

    // ── Rule: revoke has no confirmation prompt ──────────────────────────

    /// Rule: revoking — even the last device — carries no
    /// confirmation prompt. Pinned STRUCTURALLY, on the code path rather than on
    /// spelling: between entering a revoke function and issuing its request there
    /// is *nothing at all* except disabling the button.
    ///
    /// The previous version of this test was a three-string blacklist
    /// (`confirm(`, `window.confirm`, `<dialog`), which a hand-rolled modal —
    /// build a div, wait for its "Yes" click, then revoke — walks straight past.
    /// The exact-prefix oracle below cannot be walked past: any such modal has to
    /// put DOM construction and a listener between the entry and the request.
    #[test]
    fn the_revoke_path_reaches_the_request_with_no_user_gate() {
        for func in ["doRobotRevoke", "doDeviceRevoke"] {
            let body = js_function_body(TEAM_PAGE_HTML, func);
            let call = body.find("api(").unwrap_or_else(|| {
                panic!("`{func}` must issue its revoke through the shared `api(` helper")
            });
            assert_eq!(
                squash(&body[..call]),
                "btn.disabled = true;",
                "`{func}` must go straight from entry to the revoke request — the ONLY \
                 statement before it may be disabling the button. Anything else (a modal, a \
                 listener, a timer, an awaited promise) is a confirmation gate, which the \
                 project rules out: account-keyed recovery exists, so the prompt buys nothing \
                 and costs friction on the intended action."
            );
            assert!(
                body[call..].contains("/revoke"),
                "`{func}`'s request must target a /revoke route"
            );
        }

        // The free-form "Revoke by id" form routes through `submit`, whose only
        // pre-request step is the empty-input validation (not a user gate).
        let submit = js_function_body(TEAM_PAGE_HTML, "submit");
        let to_revoke = submit
            .find("doRobotRevoke(")
            .expect("`submit` must call doRobotRevoke");
        for gate in [
            "addEventListener",
            "appendChild",
            "insertBefore",
            "replaceChild",
            "showModal",
            "setTimeout",
            "setInterval",
            "requestAnimationFrame",
            "new Promise",
            ".then(",
        ] {
            assert!(
                !submit[..to_revoke].contains(gate),
                "`submit` must not `{gate}` before revoking — that is a confirmation gate"
            );
        }

        // Every destructive button leads DIRECTLY to one of those entry points,
        // pinned on the CALL SITE with the same exact-prefix discipline as the
        // function interiors above: the click handler must be either a bare
        // reference to an entry point (`addEventListener("click", submit)`) or a
        // zero-statement function literal that invokes one immediately.
        //
        // The previous form asked only whether an entry point
        // appeared ANYWHERE in the 600-byte region, so a gate wired at the call
        // site — `addEventListener("click", function () { if (await ok()) doRobotRevoke(…) })`
        // — walked straight past it while every interior pin stayed green. The
        // prelude oracle below cannot be walked past: any gate has to put a
        // statement between the handler's `{` and the entry point.
        //
        // Two further holes were closed in the same spirit:
        //
        //  * the entry point is matched as a WHOLE IDENTIFIER (see
        //    `find_identifier`), so a wrapper merely CONTAINING one — the
        //    lowercase `submitAfterConfirm(…)` — no longer scores as a bare
        //    immediately-calling handler;
        //  * each button's region runs to a STRUCTURAL boundary rather than a
        //    magic 600 bytes. The free-form form's handler already sat at
        //    offset 435 (its whole `function submit()` body lies between the
        //    class literal and the listener), i.e. 165 bytes from a failure that
        //    would have read "registers no click handler" while the real cause
        //    was a window too small.
        let entry_points = ["doRobotRevoke", "doDeviceRevoke", "submit"];
        let sites: Vec<usize> = TEAM_PAGE_HTML
            .match_indices("\"btn danger\"")
            .map(|(i, _)| i)
            .collect();
        assert_eq!(
            sites.len(),
            3,
            "expected exactly 3 destructive buttons (revoke a device from a robot, revoke by \
             id, revoke a device) — a new one must be added to this pin"
        );
        // The end of the page's dynamic render code and the start of its static
        // bottom-of-file wiring — itself pinned by
        // `the_panel_is_dismissible_in_every_state`, so this is an anchored
        // boundary rather than another magic number. Every destructive button is
        // built above it, so a region can never run off the end of the document.
        const WIRING: &str = "document.getElementById(\"close\").addEventListener";
        let wiring_at = TEAM_PAGE_HTML
            .find(WIRING)
            .expect("assets/team.html must carry the static close-button wiring");
        const CLICK: &str = "addEventListener(\"click\",";
        for (n, &site) in sites.iter().enumerate() {
            // This button's territory: from its class literal to whichever comes
            // first of the NEXT destructive button and the static wiring. Both
            // bounds are ASCII anchors, so the slice needs no char-boundary
            // widening — the em-dash hazard a byte window carries is gone by
            // construction rather than handled.
            let end = sites
                .get(n + 1)
                .copied()
                .unwrap_or(wiring_at)
                .min(wiring_at);
            assert!(
                site < end,
                "destructive button #{} sits at or after the page's static wiring — the region \
                 boundary assumed by this pin no longer holds",
                n + 1
            );
            let region = &TEAM_PAGE_HTML[site..end];
            let click = region.find(CLICK).unwrap_or_else(|| {
                panic!(
                    "destructive button #{} registers NO click handler anywhere in its own \
                     region (from its `\"btn danger\"` literal to the next destructive button \
                     or the page's static wiring): {}",
                    n + 1,
                    squash(region)
                )
            });
            let after = &region[click + CLICK.len()..];
            let at = entry_points
                .iter()
                .filter_map(|f| find_identifier(after, f))
                .min()
                .unwrap_or_else(|| {
                    panic!(
                        "a destructive button's click handler must reach a revoke entry point \
                         (one of {entry_points:?}) — as a WHOLE identifier, so a wrapper that \
                         merely contains one does not count; got: {}",
                        squash(after)
                    )
                });
            let prelude = squash(&after[..at]);
            assert!(
                prelude.is_empty() || is_bare_handler_open(&prelude),
                "a destructive button's click handler must reach its revoke entry point with \
                 NOTHING in between — it may be a bare reference or an immediately-calling \
                 `function (…) {{` literal, and nothing else. Found `{prelude}`, which is a \
                 confirmation gate at the call site (excluded by design: account-keyed \
                 recovery exists, so the prompt buys nothing and costs friction)."
            );
        }

        // Belt-and-suspenders: no confirmation UI primitive anywhere in the page.
        for banned in [
            "confirm(",
            "window.confirm",
            "<dialog",
            "showModal",
            "prompt(",
            "Are you sure",
        ] {
            assert!(
                !TEAM_PAGE_HTML.contains(banned),
                "assets/team.html must not use `{banned}` (project rule: no revoke prompt)"
            );
        }
        // …but the irreversibility IS stated, in the always-visible scope footer.
        assert!(
            TEAM_PAGE_HTML.contains("cannot be undone"),
            "the scope note must still say revocations cannot be undone"
        );
    }

    // ── The panel is always dismissible ──────────────────────────────────────

    /// The page can ALWAYS be left: Escape is bound on the document and the close
    /// affordance is synced from the first paint, not gated behind a completed
    /// handshake.
    ///
    /// Both halves were real dead ends. Escape was bound only in the Studio
    /// shell's egui `raw_input_hook`, which never sees a key press while this
    /// webview holds focus — so Escape did nothing on the hosted page. And the
    /// close button was un-hidden only inside `boot()`, i.e. only AFTER the
    /// handshake, leaving the pre-handshake / "no session handed over" / "open
    /// from Studio" states with no exit at all.
    #[test]
    fn the_panel_is_dismissible_in_every_state() {
        // Escape → close, bound on the document (not on a state-local node).
        assert!(
            TEAM_PAGE_HTML.contains("document.addEventListener(\"keydown\""),
            "assets/team.html must bind keydown on the document"
        );
        assert!(
            TEAM_PAGE_HTML.contains("if (e.key !== \"Escape\") return;")
                && TEAM_PAGE_HTML.contains("requestClose();"),
            "Escape must dismiss the panel — while this webview has focus the Studio shell's \
             own egui-side Escape handler never runs"
        );
        // One dismissal path, shared by Escape and the button — and the ONE place
        // the availability predicate is applied, so the two can never disagree.
        let close = js_function_body(TEAM_PAGE_HTML, "requestClose");
        assert!(
            close.contains("if (!closeIsAvailable()) return;"),
            "requestClose must honour `closeIsAvailable()` — Escape bypasses the hidden \
             button, so without the shared predicate the key could act in a state where the \
             affordance is deliberately not offered: {close}"
        );
        assert!(
            close.contains("postHost({ cmd: \"team_close\" });"),
            "requestClose must post the close verb: {close}"
        );
        assert!(TEAM_PAGE_HTML.contains(
            "document.getElementById(\"close\").addEventListener(\"click\", requestClose);"
        ));

        // The affordance is synced at STARTUP (before any handshake) — the old
        // un-hide inside boot() is gone.
        assert!(
            !TEAM_PAGE_HTML.contains("closeBtn.hidden"),
            "the close button must not be un-hidden only inside boot() — that leaves every \
             pre-handshake state with no exit"
        );
        // Host presence is the WHOLE predicate. Pinned as exact equality,
        // not containment: the clause deleted from `closeIsAvailable()`
        // (`SESSION.embedded === false`) was an opt-out no host could ever satisfy —
        // it read as a real condition while being hardcoded off, and a containment
        // pin would let that class straight back in.
        let visibility = js_function_body(TEAM_PAGE_HTML, "closeIsAvailable");
        assert_eq!(
            squash(&visibility),
            "return hostAvailable();",
            "the close affordance must key off the presence of a host bridge and NOTHING \
             else — any further condition here is a state the user cannot leave: {visibility}"
        );
        // Synced from the first paint AND on every handshake outcome.
        assert!(TEAM_PAGE_HTML.matches("syncCloseAffordance();").count() >= 4);
    }

    /// Escape is a DOCUMENT handler, so it fires wherever the caret is. Two
    /// things follow, and both were wrong when the binding first landed:
    ///
    /// 1. It must not hijack TEXT ENTRY. While the caret is in the "Revoke by id"
    ///    field, Escape is the universal "get me out of this field" key — and a
    ///    handler that closed the whole panel instead fired exactly when a user
    ///    was mid-thought about a destructive action. It blurs there; a second
    ///    Escape, now outside the field, dismisses.
    /// 2. It must route through `requestClose`, the ONE place `closeIsAvailable()`
    ///    and the in-flight-revoke guard are applied. Posting `team_close`
    ///    directly would bypass both — and the revoke guard is the one that can
    ///    actually refuse, so bypassing it silently discards the outcome the user
    ///    came for.
    #[test]
    fn escape_is_scoped_away_from_text_entry_and_routed_through_the_close_predicate() {
        let at = TEAM_PAGE_HTML
            .find("document.addEventListener(\"keydown\"")
            .expect("assets/team.html must bind keydown on the document");
        let rest = &TEAM_PAGE_HTML[at..];
        let end = rest
            .find("\n});")
            .expect("the keydown handler must be a terminated statement")
            + "\n});".len();
        let handler = squash(&rest[..end]);

        assert!(
            handler.contains("if (isTextEntry(e.target)) { e.target.blur(); return; }"),
            "Escape must blur a focused text-entry control instead of dismissing the \
             panel from under the user's caret: {handler}"
        );
        assert!(
            handler.contains("requestClose();"),
            "Escape must dismiss through the shared path: {handler}"
        );
        assert!(
            !handler.contains("postHost("),
            "Escape must NOT post `team_close` itself — that bypasses \
             `closeIsAvailable()` and the in-flight-revoke guard: {handler}"
        );

        // The predicate covers every control this page can put a caret in.
        let is_text = js_function_body(TEAM_PAGE_HTML, "isTextEntry");
        for tag in ["input", "textarea", "select"] {
            assert!(
                is_text.contains(&format!("\"{tag}\"")),
                "isTextEntry must cover <{tag}> (the Revoke-by-id form uses input + select)"
            );
        }
        assert!(
            is_text.contains("isContentEditable"),
            "isTextEntry must cover contenteditable hosts"
        );
    }

    /// The HOST's dismissal entry point routes through `requestClose` — the THIRD
    /// and last way this panel can be asked to close, driven by the Studio shell's
    /// stage-side Escape (`native/studio-shell/src/bin/shell.rs::request_team_close`,
    /// which evals the name held in that repo's `team_panel::TEAM_REQUEST_CLOSE_FN`).
    /// Routing it here is the WHOLE point of the entry point: it inherits both guards
    /// — the shared `closeIsAvailable()` predicate and the in-flight-revoke refusal
    /// (the one that can actually refuse) — so the button, this document's Escape, and
    /// the host's Escape are one behaviour rather than three.
    ///
    /// Pinning it by NAME is not enough.
    /// [`page_speaks_exactly_the_studio_handshake_vocabulary`] asserts only that
    /// `window.__cerulion_request_close = function` EXISTS;
    /// [`the_panel_is_dismissible_in_every_state`]'s bare `requestClose();` substring
    /// is already satisfied by the document's own keydown handler; and the keydown
    /// slice in the test above stops before this line. So re-spelling the body as
    /// `postHost({ cmd: "team_close" });` — the direct bypass, which
    /// drops BOTH guards — left the entire crate green. The body oracle below is the
    /// mutation kill; the count oracle keeps a SECOND emitter from re-opening the
    /// same hole somewhere else in the document.
    #[test]
    fn the_host_close_entry_point_routes_through_the_shared_dismissal_path() {
        let host_close = js_assigned_function_body(TEAM_PAGE_HTML, "__cerulion_request_close");
        assert_eq!(
            squash(&host_close),
            "requestClose();",
            "`window.__cerulion_request_close` must do NOTHING but call `requestClose()` — \
             the one place `closeIsAvailable()` and the in-flight-revoke guard are applied. \
             Posting `team_close` (or closing) directly here is exactly the bypass this \
             entry point was introduced to remove: {host_close}"
        );
        // …and the close verb leaves the page from exactly ONE place — `requestClose`
        // — so no other route can skip the guards either.
        assert_eq!(
            TEAM_PAGE_HTML
                .matches("postHost({ cmd: \"team_close\" });")
                .count(),
            1,
            "`team_close` must be posted from exactly one site (inside `requestClose`); \
             every dismissal route funnels through it, which is what makes the guards \
             unskippable"
        );
    }

    /// A failed credential write-back is reported in a PERSISTENT in-page
    /// surface, and the page keeps running.
    ///
    /// This is the one failure whose report may NOT cost the user their document.
    /// accountd's refresh is single-use — it rotates both tokens under a CAS — so
    /// the moment the refresh succeeds the old pair is dead and the rotated pair
    /// exists ONLY in this page's memory (the host deliberately keeps its own copy
    /// at the OLD pair precisely because the write failed, and the staging temp is
    /// removed on both failure branches). Swapping this document for the host's
    /// local chrome would therefore destroy the desk's only live credential AND
    /// the in-flight user action that provoked the refresh — and the "Retry" that
    /// chrome offered was provably futile, since retrying re-reads the same stale
    /// `auth.json` and re-presents the already-spent refresh token.
    ///
    /// So the report lives here, in a sticky bar OUTSIDE `#content` (a re-render
    /// never wipes it, a toast's 8 seconds cannot swallow it), and it clears only
    /// when a later save actually succeeds.
    #[test]
    fn a_failed_persist_is_reported_in_a_persistent_in_page_surface() {
        // The surface exists, sits outside the re-rendered content, and is styled
        // (a class with no rule renders as bare text).
        assert!(
            TEAM_PAGE_HTML
                .contains("<div id=\"alert\" role=\"alert\" aria-live=\"assertive\" hidden></div>"),
            "assets/team.html must carry the persistent `#alert` bar"
        );
        assert!(TEAM_PAGE_HTML.contains("#alert {"));
        let main_at = TEAM_PAGE_HTML
            .find("<main id=\"main\">")
            .expect("the main region");
        let alert_at = TEAM_PAGE_HTML
            .find("<div id=\"alert\"")
            .expect("the alert bar");
        assert!(
            alert_at < main_at,
            "the alert bar must live OUTSIDE `#content`, or `render()` would wipe it"
        );

        // It is populated as TEXT — the reason is an untrusted host/service string.
        let show = js_function_body(TEAM_PAGE_HTML, "showAlert");
        assert!(
            show.contains("a.textContent = String(text);") && !show.contains("innerHTML"),
            "the alert's text is untrusted and must never be parsed as HTML: {show}"
        );

        // The host's failure ack goes to the STICKY alert (not only a toast), and
        // a success clears it.
        let saved = js_assigned_function_body(TEAM_PAGE_HTML, "__cerulion_session_saved");
        assert!(
            saved.contains("showAlert(msg, true);"),
            "a failed persist must raise the STICKY alert: {saved}"
        );
        assert!(
            saved.contains("clearAlert(true);"),
            "a successful persist must clear it: {saved}"
        );
        assert!(
            saved.contains("cerulion login"),
            "the alert must carry the repair: {saved}"
        );
        // A sticky alert is never displaced by a transient one.
        assert!(js_function_body(TEAM_PAGE_HTML, "showAlert")
            .contains("if (ALERT_STICKY && !sticky) return;"));
        assert!(js_function_body(TEAM_PAGE_HTML, "clearAlert")
            .contains("if (ALERT_STICKY && !force) return;"));
    }

    /// `find_identifier` matches WHOLE identifiers, against a hand oracle.
    ///
    /// This helper is the load-bearing half of the call-site oracle in
    /// `the_revoke_path_reaches_the_request_with_no_user_gate`: with a plain
    /// substring scan, a confirmation gate wrapped in `submitAfterConfirm(…)`
    /// passes that test. The cases below are the exact
    /// shapes that matters for — a token that is a prefix, a suffix, or an
    /// infix of a longer identifier must NOT match, while every real call and
    /// bare-reference spelling must.
    #[test]
    fn find_identifier_matches_whole_identifiers_only() {
        // (haystack, needle, expected offset)
        let hits: [(&str, &str, usize); 6] = [
            (" submit);", "submit", 1),                    // a bare reference
            ("function () { submit(); }", "submit", 14),   // an immediate call
            ("submit()", "submit", 0),                     // at the very start
            ("x=submit", "submit", 2),                     // at the very end
            (" doRobotRevoke(rb, t)", "doRobotRevoke", 1), // the real call site
            ("f(submitted); submit()", "submit", 14),      // skips the prefix hit
        ];
        for (hay, needle, want) in hits {
            assert_eq!(
                find_identifier(hay, needle),
                Some(want),
                "`{needle}` must match as a whole identifier in `{hay}`"
            );
        }
        // Every one of these CONTAINS the needle and must still not match — a
        // plain substring scan accepts all of them.
        let misses: [(&str, &str); 6] = [
            (" submitAfterConfirm(d, btn);", "submit"), // a substring scan accepts this
            (" resubmit();", "submit"),                 // a suffix
            (" x_submit_y();", "submit"),               // an infix, `_`-joined
            (" submit2();", "submit"),                  // a digit suffix
            (" submit$fn();", "submit"),                // a `$` suffix (a JS ident byte)
            (" confirmThendoRobotRevoke();", "doRobotRevoke"),
        ];
        for (hay, needle) in misses {
            assert_eq!(
                find_identifier(hay, needle),
                None,
                "`{needle}` must NOT match inside a longer identifier in `{hay}`"
            );
        }
        assert_eq!(find_identifier("", "submit"), None);
    }

    /// Closing never SILENTLY abandons an in-flight revoke.
    ///
    /// Closing does not cancel the request — it completes server-side either way —
    /// so what a mid-revoke dismissal throws away is the OUTCOME, which is the one
    /// thing the user came here for. The first close attempt while a destructive
    /// call is in flight says so and stays; a second leaves anyway, so a hung
    /// request can never wedge the panel.
    ///
    /// This is NOT a confirmation gate on the destructive action (that gate was
    /// decided against, and `the_revoke_path_reaches_the_request_with_no_user_gate` still
    /// pins its absence): the revoke fires immediately and unprompted. The guard is
    /// on LEAVING, after the fact — which is why the counter is maintained inside
    /// `api`, where it cannot put a statement on the revoke path.
    #[test]
    fn closing_never_silently_abandons_an_in_flight_revoke() {
        let close = js_function_body(TEAM_PAGE_HTML, "requestClose");
        assert!(
            close.contains("if (PENDING_DESTRUCTIVE > 0 && !CLOSE_ARMED) {"),
            "requestClose must notice an in-flight revoke: {close}"
        );
        assert!(
            close.contains("CLOSE_ARMED = true;"),
            "a second attempt must always leave — a hung request may not wedge the panel: \
             {close}"
        );

        // …and the refusal is actually SEEN. The branch arms `CLOSE_ARMED`, so a
        // surface that renders nothing turns the user's first Escape/click into a
        // silent no-op — and the one state where that happens is the compound one
        // this page can genuinely reach: `showAlert` early-returns whenever a
        // STICKY alert is already up (`if (ALERT_STICKY && !sticky) return;`),
        // which is exactly the persist-failure bar `__cerulion_session_saved`
        // raises when a mid-revoke refresh cannot be written to disk.
        //
        // So the oracle is not "some surface is called" (a bare `showAlert` passes
        // that) but "a surface that a sticky alert CANNOT swallow is called" —
        // derived from the surfaces' own bodies, so it re-decides itself if the
        // stickiness rules move rather than encoding today's answer as a literal.
        let branch = js_body_after(
            &close,
            "if (PENDING_DESTRUCTIVE > 0 && !CLOSE_ARMED)",
            "requestClose's in-flight refusal branch",
        );
        const SURFACES: [&str; 2] = ["showAlert", "toast"];
        let reached: Vec<&str> = SURFACES
            .into_iter()
            .filter(|s| branch.contains(&format!("{s}(")))
            .collect();
        assert!(
            !reached.is_empty(),
            "the in-flight refusal must TELL the user — it arms CLOSE_ARMED, so rendering \
             nothing makes the first close attempt a silent no-op: {branch}"
        );
        let unswallowable: Vec<&str> = reached
            .iter()
            .copied()
            .filter(|s| !js_function_body(TEAM_PAGE_HTML, s).contains("ALERT_STICKY"))
            .collect();
        assert!(
            !unswallowable.is_empty(),
            "every surface the in-flight refusal uses ({reached:?}) yields to an already-raised \
             STICKY alert, so with the persist-failure bar up the refusal renders NOTHING while \
             still arming CLOSE_ARMED — a silent no-op on a user action. Add one that a sticky \
             alert cannot swallow (`toast`), as `__cerulion_session_saved` already does. Branch: \
             {branch}"
        );

        let api = js_function_body(TEAM_PAGE_HTML, "api");
        assert!(
            api.contains("if (destructive) PENDING_DESTRUCTIVE++;"),
            "the in-flight counter is maintained in `api`: {api}"
        );
        assert_eq!(
            api.matches("settle();").count(),
            2,
            "`api` must settle the counter on BOTH the success and the failure arm — a \
             missed decrement wedges the panel after one failed revoke: {api}"
        );
        // …and NOT in the revoke functions, whose entry→request path must stay a
        // straight line (see `the_revoke_path_reaches_the_request_with_no_user_gate`).
        for f in ["doRobotRevoke", "doDeviceRevoke"] {
            assert!(
                !js_function_body(TEAM_PAGE_HTML, f).contains("PENDING_DESTRUCTIVE"),
                "`{f}` must not touch the in-flight counter — that would put a statement \
                 between its entry and its request"
            );
        }
    }

    // ── Failure causes are persistent, never toast-only ──────────────────────

    /// A failure state renders its CAUSE into the persistent state block. The
    /// load-failure copy must not say "the reason is shown below" while the reason
    /// lives in a toast that self-clears after 8 seconds — a user reading the
    /// state carefully would be told to look at nothing.
    ///
    /// The cause is untrusted (a service error string), so it goes in as TEXT.
    #[test]
    fn a_failed_load_shows_its_cause_in_the_persistent_state_not_a_toast() {
        let render = js_function_body(TEAM_PAGE_HTML, "renderState");
        // The detail block is rendered via `el(...)`, whose third argument is set
        // with `textContent` — never `innerHTML`.
        assert!(
            render.contains("el(\"div\", \"detail\", String(spec.detail))"),
            "renderState must render `detail` as TEXT into a `.detail` block"
        );
        assert!(
            render.contains("sub.innerHTML = spec.body;"),
            "only the static author-written body may be inserted as HTML"
        );
        // The load-failure path supplies the service's own reason.
        let boot = js_function_body(TEAM_PAGE_HTML, "boot");
        assert!(
            boot.contains("detail: e && e.message ? e.message : String(e)"),
            "the load-failure state must carry the service's reason as its detail"
        );
        assert!(
            boot.contains("The reason is below"),
            "the copy must point at the persistent block, not a toast"
        );
        assert!(
            !TEAM_PAGE_HTML.contains("The reason is shown below \\u2014 if it mentions"),
            "the toast-pointing copy must not appear"
        );
        // The block is styled (a class with no rule would render as bare text).
        assert!(TEAM_PAGE_HTML.contains(".detail {"));
    }

    /// The scope note is present and matches `docs/revocation.md`: grants are
    /// desk-carried (not cloud-enumerable) and revocation is online-enforced.
    #[test]
    fn page_states_the_honest_scope() {
        for phrase in [
            "online-enforced",
            "syncs",
            "cannot enumerate them",
            "revocation set",
        ] {
            assert!(
                TEAM_PAGE_HTML.contains(phrase),
                "the scope footer must state `{phrase}` (see docs/revocation.md — the page \
                 must not imply it shows cloud-recorded grants, which stay off-cloud by design)"
            );
        }
    }

    /// The CSP is the runtime half of self-containment: it must deny by default,
    /// allow the page's own inline code, and confine XHR to this origin.
    #[test]
    fn csp_denies_by_default_and_confines_xhr_to_this_origin() {
        assert!(TEAM_PAGE_CSP.contains("default-src 'none'"));
        assert!(TEAM_PAGE_CSP.contains("connect-src 'self'"));
        assert!(TEAM_PAGE_CSP.contains("frame-ancestors 'none'"));
        assert!(TEAM_PAGE_CSP.contains("base-uri 'none'"));
        assert!(TEAM_PAGE_CSP.contains("form-action 'none'"));
        // Inline style/script must be permitted or the page renders blank.
        assert!(TEAM_PAGE_CSP.contains("style-src 'unsafe-inline'"));
        assert!(TEAM_PAGE_CSP.contains("script-src 'unsafe-inline'"));
        // No wildcard host may creep in.
        assert!(
            !TEAM_PAGE_CSP.contains('*'),
            "the Team page CSP must never allow a wildcard source"
        );
        // It must be a valid header value (the handler uses `from_static`, which
        // panics on an invalid byte — assert here so the failure is a test, not a
        // 500 in production).
        assert!(HeaderValue::from_static(TEAM_PAGE_CSP).to_str().is_ok());
    }

    /// The served response shape: HTML content type, uncached, hardened headers.
    #[tokio::test]
    async fn team_page_serves_html_uncached_and_hardened() {
        let resp = team_page().await;
        assert_eq!(resp.status(), axum::http::StatusCode::OK);
        let h = resp.headers();
        assert_eq!(
            h.get(CONTENT_TYPE).unwrap(),
            "text/html; charset=utf-8",
            "the page must be served as HTML (not the `&str` default of text/plain)"
        );
        assert_eq!(h.get(CACHE_CONTROL).unwrap(), "no-store");
        assert_eq!(h.get(CONTENT_SECURITY_POLICY).unwrap(), TEAM_PAGE_CSP);
        assert_eq!(h.get(X_CONTENT_TYPE_OPTIONS).unwrap(), "nosniff");
        assert_eq!(h.get(REFERRER_POLICY).unwrap(), "no-referrer");
    }
}
