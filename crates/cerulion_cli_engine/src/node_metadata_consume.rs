// SPDX-License-Identifier: AGPL-3.0-only
//! The `#[input(trigger, consume = fifo | latest)]` grammar as
//! `node_metadata`'s input-attribute walk checks it.
//!
//! The CLI must not accept a form the macro refuses, nor refuse one it
//! accepts, so this mirrors `cerulion_macros::parse`: `consume` takes the
//! single identifier `fifo` or `latest`, may appear once, and needs
//! `trigger`. The refusal texts are the macro's own, word for word.

use proc_macro2::TokenTree;

use cerulion_core::graph::node::InputConsume;

/// The accepted values, quoted in every `consume` refusal (the macro's
/// `CONSUME_VALUES`).
const CONSUME_VALUES: &str = "`consume = fifo` (the default) or `consume = latest`";

/// The refusal for `consume` on a non-trigger input (the macro's
/// `CONSUME_NEEDS_TRIGGER`).
pub(crate) const CONSUME_NEEDS_TRIGGER: &str =
    "`consume` applies only to a trigger input: write `#[input(trigger, consume = latest)]`. \
     A non-trigger input never fires the node; it is a latest-value read that already \
     serves the newest frame on every fire, so it has no consumption order to choose.";

/// The refusal for `consume = latest` on an input of a `sync_window_ms` /
/// `unbounded_sync` node (the macro's `SYNC_CONSUME_LATEST_REFUSED`).
pub(crate) const SYNC_CONSUME_LATEST_REFUSED: &str =
    "`consume = latest` is not accepted on a trigger input of a `sync_window_ms` or \
     `unbounded_sync` node: a Sync node fires once per aligned set of its trigger inputs, \
     in set order, and does not take a per-input consumption mode. `consume = latest` \
     applies to the one trigger input of a data-triggered node.";

/// Where the walk is inside a `consume = <mode>`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Not inside a `consume` key.
    #[default]
    Idle,
    /// Just saw `consume`; the next token must be `=`.
    NeedEq,
    /// Just saw `consume =`; the next token must be the mode identifier.
    NeedValue,
    /// Just saw the mode; the next token must be `,` or the attribute ends.
    NeedSeparator,
}

/// The `consume` key seen in one `#[input(...)]`.
#[derive(Debug, Default)]
pub(crate) struct ConsumeGrammar {
    seen: bool,
    mode: Option<InputConsume>,
    state: State,
}

impl ConsumeGrammar {
    /// Is the walk inside a `consume = <mode>`, so the next token belongs to
    /// it?
    pub(crate) fn wants_token(&self) -> bool {
        self.state != State::Idle
    }

    /// Record a top-level key. Returns `true` when it is `consume`.
    pub(crate) fn on_key(&mut self, ident: &str) -> Result<bool, String> {
        if ident != "consume" {
            return Ok(false);
        }
        if self.seen {
            return Err(format!("`consume` given twice. Write {CONSUME_VALUES}."));
        }
        self.seen = true;
        self.state = State::NeedEq;
        Ok(true)
    }

    /// Take the next token of a `consume = <mode>`.
    pub(crate) fn on_token(&mut self, tt: &TokenTree) -> Result<(), String> {
        match (self.state, tt) {
            (State::NeedEq, TokenTree::Punct(p)) if p.as_char() == '=' => {
                self.state = State::NeedValue;
                Ok(())
            }
            (State::NeedValue, TokenTree::Ident(mode)) => {
                self.mode = Some(match mode.to_string().as_str() {
                    "fifo" => InputConsume::Fifo,
                    "latest" => InputConsume::Latest,
                    other => {
                        return Err(format!(
                            "unknown consume mode `{other}`, expected {CONSUME_VALUES}."
                        ))
                    }
                });
                self.state = State::NeedSeparator;
                Ok(())
            }
            (State::NeedSeparator, TokenTree::Punct(p)) if p.as_char() == ',' => {
                self.state = State::Idle;
                Ok(())
            }
            _ => Err(format!("`consume` takes {CONSUME_VALUES}.")),
        }
    }

    /// The declared mode once every token is fed, or the refusal. `None`
    /// when `consume` was not written.
    pub(crate) fn finish(&self, trigger: bool) -> Result<Option<InputConsume>, String> {
        if matches!(self.state, State::NeedEq | State::NeedValue) {
            return Err(format!("`consume` takes {CONSUME_VALUES}."));
        }
        if self.seen && !trigger {
            return Err(CONSUME_NEEDS_TRIGGER.to_string());
        }
        Ok(self.mode)
    }
}
