// SPDX-License-Identifier: AGPL-3.0-only
//! The caller a tool call is receipted and journaled under.
//!
//! A call that arrives over the streamable HTTP transport may carry a
//! [`CALLER_HEADER`]. The HTTP layer admits a request only after its bearer
//! token matches, so the header comes from a holder of the token: on a robot,
//! `cerulion-remoted`, which sets it to the paired identity of the remote desk
//! it forwards for. [`RcpServer`](crate::server::RcpServer) runs the call inside
//! [`scope`]; while it runs, [`current`] returns that caller, and every receipt
//! the call writes names it, as does the call frame published on the journal.
//! Outside a scope (stdio, or HTTP without the header) [`current`] returns
//! [`DEFAULT_CALLER`] and the call frame carries no `caller` field.

use std::future::Future;

/// The HTTP request header naming the remote caller.
pub const CALLER_HEADER: &str = "x-cerulion-caller";

/// The caller a receipt names when no remote caller is in scope.
pub const DEFAULT_CALLER: &str = crate::tools::CALLER;

/// The longest accepted caller, bytes.
pub const MAX_CALLER_LEN: usize = 200;

tokio::task_local! {
    static CALLER: String;
}

/// Whether `caller` is 1 to [`MAX_CALLER_LEN`] bytes from
/// `[A-Za-z0-9:._@/-]`.
pub fn valid_caller(caller: &str) -> bool {
    !caller.is_empty()
        && caller.len() <= MAX_CALLER_LEN
        && caller.bytes().all(|b| {
            b.is_ascii_alphanumeric() || matches!(b, b':' | b'.' | b'_' | b'@' | b'/' | b'-')
        })
}

/// Run `fut` with `caller` as the remote caller.
pub async fn scope<F: Future>(caller: String, fut: F) -> F::Output {
    CALLER.scope(caller, fut).await
}

/// The remote caller in scope, if any.
pub fn remote() -> Option<String> {
    CALLER.try_with(Clone::clone).ok()
}

/// The remote caller in scope, else [`DEFAULT_CALLER`].
pub fn current() -> String {
    remote().unwrap_or_else(|| DEFAULT_CALLER.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn caller_charset_and_length_are_enforced() {
        assert!(valid_caller("iroh:ab12/acct:cd34"));
        assert!(valid_caller(&"a".repeat(MAX_CALLER_LEN)));
        assert!(!valid_caller(""));
        assert!(!valid_caller(&"a".repeat(MAX_CALLER_LEN + 1)));
        for bad in ["a b", "a\nb", "a\"b", "a,b", "é", "a;b"] {
            assert!(!valid_caller(bad), "{bad:?}");
        }
    }

    #[tokio::test]
    async fn current_is_the_scoped_caller_inside_and_the_default_outside() {
        assert_eq!(current(), DEFAULT_CALLER);
        assert_eq!(remote(), None);
        let inside = scope("iroh:x".to_string(), async { (current(), remote()) }).await;
        assert_eq!(inside, ("iroh:x".to_string(), Some("iroh:x".to_string())));
        assert_eq!(current(), DEFAULT_CALLER);
    }
}
