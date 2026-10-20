// SPDX-License-Identifier: AGPL-3.0-only
//! The `#[input(accumulate)]` / `#[input(accumulate, cap = N)]` grammar as
//! `node_metadata`'s input-attribute walk checks it.
//!
//! The CLI must not accept a form the macro refuses, so this mirrors the
//! macro's rules (`cerulion_macros::input_accumulate`): `cap` needs
//! `accumulate`; `accumulate` refuses `trigger`, `backpressure = block`,
//! `backpressure = sample(N)` and `depth`; `cap` is an integer in
//! `1..=MAX_CAP`; neither key may repeat. The walk feeds tokens in; the
//! verdict comes from [`AccumulateGrammar::finish`].

use proc_macro2::TokenTree;

use cerulion_core::graph::node::BackpressurePolicy;
use cerulion_core::transport::accumulate::MAX_ACCUMULATE_CAP;

/// The keys an accumulate input accepts, quoted in every refusal.
const ACCEPTED: &str = "With `accumulate` the accepted keys are `cap = N`, \
                        `backpressure = drop_oldest` and `expect_within_ms = N`.";

/// Where the walk is inside a `cap = N`.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
enum CapState {
    /// Not inside a `cap` key.
    #[default]
    Idle,
    /// Just saw `cap`; the next token must be `=`.
    NeedEq,
    /// Just saw `cap =`; the next token must be the integer.
    NeedValue,
}

/// The accumulate keys seen in one `#[input(...)]`.
#[derive(Debug, Default)]
pub(crate) struct AccumulateGrammar {
    accumulate: bool,
    cap_key: bool,
    cap: Option<usize>,
    depth: bool,
    state: CapState,
}

impl AccumulateGrammar {
    /// Is the walk inside a `cap = N`, so the next token belongs to it?
    pub(crate) fn wants_token(&self) -> bool {
        self.state != CapState::Idle
    }

    /// Take the next token of a `cap = N`.
    pub(crate) fn on_token(&mut self, tt: &TokenTree) -> Result<(), String> {
        match (self.state, tt) {
            (CapState::NeedEq, TokenTree::Punct(p)) if p.as_char() == '=' => {
                self.state = CapState::NeedValue;
                Ok(())
            }
            (CapState::NeedValue, TokenTree::Literal(lit)) => {
                self.state = CapState::Idle;
                // Parsed exactly the way the macro parses it, so `cap = 0x10`
                // and `cap = 8usize` agree on both sides.
                let parsed = syn::parse2::<syn::LitInt>(TokenTree::Literal(lit.clone()).into())
                    .and_then(|l| l.base10_parse::<usize>())
                    .map_err(|e| format!("`cap` requires an integer, got `{lit}`: {e}."))?;
                self.cap = Some(parsed);
                Ok(())
            }
            _ => Err(format!("`cap` requires a value: `cap = N`. {ACCEPTED}")),
        }
    }

    /// Record an `accumulate`, `cap` or `depth` key. Returns `true` for the
    /// two accumulate keys (the walk needs nothing else for them).
    pub(crate) fn on_key(&mut self, ident: &str) -> Result<bool, String> {
        match ident {
            "accumulate" => {
                if self.accumulate {
                    return Err(format!("`accumulate` given twice. {ACCEPTED}"));
                }
                self.accumulate = true;
                Ok(true)
            }
            "cap" => {
                if self.cap_key {
                    return Err(format!("`cap` given twice. {ACCEPTED}"));
                }
                self.cap_key = true;
                self.state = CapState::NeedEq;
                Ok(true)
            }
            "depth" => {
                self.depth = true;
                Ok(false)
            }
            _ => Ok(false),
        }
    }

    /// The verdict once the attribute's tokens are all fed, against the
    /// rest of the input's keys.
    pub(crate) fn finish(
        &self,
        trigger: bool,
        backpressure: &BackpressurePolicy,
    ) -> Result<(), String> {
        if self.state != CapState::Idle {
            return Err(format!("`cap` requires a value: `cap = N`. {ACCEPTED}"));
        }
        if !self.accumulate {
            if self.cap_key {
                return Err(
                    "`cap` applies only to `#[input(accumulate)]`. For the queue size \
                            of a plain input, use `depth = N`."
                        .to_string(),
                );
            }
            return Ok(());
        }
        if trigger {
            return Err(format!(
                "`#[input(accumulate)]` cannot be combined with `trigger`: a trigger input \
                 already serves every frame, one fire per frame. {ACCEPTED}"
            ));
        }
        match backpressure {
            BackpressurePolicy::Block => {
                return Err(format!(
                    "`#[input(accumulate)]` cannot be combined with `backpressure = block`: an \
                     accumulate input drops its oldest frames at the cap. {ACCEPTED}"
                ));
            }
            BackpressurePolicy::Sample(_) => {
                return Err(format!(
                    "`#[input(accumulate)]` cannot be combined with `backpressure = sample(N)`: \
                     decimation would drop the frames accumulate keeps. {ACCEPTED}"
                ));
            }
            BackpressurePolicy::DropOldest => {}
        }
        if self.depth {
            return Err(format!(
                "`#[input(accumulate)]` takes `cap = N`, not `depth`: the cap is the input's \
                 queue depth. {ACCEPTED}"
            ));
        }
        if let Some(cap) = self.cap {
            if cap == 0 || cap > MAX_ACCUMULATE_CAP {
                return Err(format!(
                    "`cap` must be in 1..={MAX_ACCUMULATE_CAP} (got {cap}). {ACCEPTED}"
                ));
            }
        }
        Ok(())
    }
}
