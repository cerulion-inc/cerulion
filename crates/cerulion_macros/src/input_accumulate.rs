// SPDX-License-Identifier: AGPL-3.0-only
//! `#[input(accumulate)]` and `#[input(accumulate, cap = N)]`: parse,
//! validation and the generated code that reads the input.
//!
//! An accumulate input serves every frame since the node's last fire, in
//! the order and under the cap `cerulion_core::transport::accumulate`
//! documents. The cap is also the input's transport queue depth, so it is
//! written into the input's `depth` once validated, and the rest of the
//! macro treats the depth as it always does.

use proc_macro2::{Span, TokenStream};
use quote::{format_ident, quote};
use syn::meta::ParseNestedMeta;
use syn::{Ident, LitInt};

use crate::parse::{FieldInputAttr, ParsedBackpressurePolicy};

/// The keys an accumulate input accepts, quoted in every refusal.
const ACCEPTED: &str = "With `accumulate` the accepted keys are `cap = N`, \
                        `backpressure = drop_oldest` and `expect_within_ms = N`.";

/// The largest accepted cap. Mirrors `MAX_ACCUMULATE_CAP` (the queue depth
/// ceiling) in `cerulion_core`; the generated code passes the cap to the
/// runtime, which checks it again.
const MAX_CAP: usize = 64;

/// The cap of an `accumulate` without `cap = N`. Mirrors
/// `DEFAULT_ACCUMULATE_CAP` in `cerulion_core`.
const DEFAULT_CAP: usize = 10;

/// The accumulate keys seen while parsing one `#[input(...)]`.
#[derive(Default)]
pub struct AccumulateKeys {
    accumulate: Option<Span>,
    cap: Option<(usize, Span)>,
}

/// Parse one `accumulate` or `cap = N` key.
pub fn parse_key(
    ident: &Ident,
    meta: &ParseNestedMeta<'_>,
    keys: &mut AccumulateKeys,
) -> syn::Result<()> {
    if ident == "accumulate" {
        keys.accumulate = Some(ident.span());
    } else {
        let lit: LitInt = meta.value()?.parse()?;
        keys.cap = Some((lit.base10_parse()?, lit.span()));
    }
    Ok(())
}

/// Validate the accumulate keys against the rest of the input's keys and
/// resolve the cap into `input.accumulate` and `input.depth`.
pub fn finish(keys: AccumulateKeys, input: &mut FieldInputAttr) -> syn::Result<()> {
    let Some(span) = keys.accumulate else {
        if let Some((_, cap_span)) = keys.cap {
            return Err(syn::Error::new(
                cap_span,
                "`cap` applies only to `#[input(accumulate)]`. For the queue size of a plain \
                 input, use `depth = N`.",
            ));
        }
        return Ok(());
    };
    let refuse = |why: &str| Err(syn::Error::new(span, format!("{why} {ACCEPTED}")));
    if input.trigger {
        return refuse(
            "`#[input(accumulate)]` cannot be combined with `trigger`: a trigger input already \
             serves every frame, one fire per frame.",
        );
    }
    match input.backpressure {
        Some(ParsedBackpressurePolicy::Block) => {
            return refuse(
                "`#[input(accumulate)]` cannot be combined with `backpressure = block`: an \
                 accumulate input drops its oldest frames at the cap, while `block` paces the \
                 producer instead.",
            );
        }
        Some(ParsedBackpressurePolicy::Sample(_)) => {
            return refuse(
                "`#[input(accumulate)]` cannot be combined with `backpressure = sample(N)`: \
                 decimation would drop the frames accumulate keeps.",
            );
        }
        Some(ParsedBackpressurePolicy::DropOldest) | None => {}
    }
    if input.depth.is_some() {
        return refuse(
            "`#[input(accumulate)]` takes `cap = N`, not `depth`: the cap is the input's queue \
             depth.",
        );
    }
    let cap = match keys.cap {
        None => DEFAULT_CAP,
        Some((cap, cap_span)) if cap == 0 || cap > MAX_CAP => {
            return Err(syn::Error::new(
                cap_span,
                format!("`cap` must be in 1..={MAX_CAP} (got {cap}). {ACCEPTED}"),
            ));
        }
        Some((cap, _)) => cap,
    };
    input.accumulate = Some(cap);
    input.depth = Some(cap);
    Ok(())
}

/// The `init` statements that register each accumulate input on the
/// entry's `AccumulatorSet` (`self.__cer_accumulators`) before the user's
/// `init` runs.
pub fn register_calls(inputs: &[FieldInputAttr]) -> TokenStream {
    let calls = inputs.iter().filter_map(|i| {
        let cap = i.accumulate?;
        let name = i.field_name.to_string();
        let ty = &i.field_type;
        Some(quote! {
            self.__cer_accumulators.register::<#ty>(&mut context, #name, #cap)?;
        })
    });
    quote! { #(#calls)* }
}

/// One accumulate port as the tick rewriter sees it.
pub struct AccumulatePort<'a> {
    pub name: &'a Ident,
    pub ty: &'a syn::Type,
}

/// The statements that bind each accumulate port's tick local
/// (`__cer_<port>: &Accumulated<'_, T>`) inside the tick, just before the
/// user body runs. All serves are marked first, then every view is taken
/// as a shared borrow, so several accumulate ports can be in scope at once.
pub fn serve_bindings(ports: &[AccumulatePort<'_>]) -> TokenStream {
    let marks = ports.iter().map(|p| {
        let name = p.name.to_string();
        quote! { __cer_acc.mark_served(#name); }
    });
    let views = ports.iter().map(|p| {
        let name = p.name.to_string();
        let ty = p.ty;
        let owned = format_ident!("__cer_{}_frames", p.name);
        let local = format_ident!("__cer_{}", p.name);
        quote! {
            let #owned = __cer_acc.view::<#ty>(#name);
            let #local: &::cerulion_core::transport::accumulate::Accumulated<'_, #ty> = &#owned;
        }
    });
    quote! { #(#marks)* #(#views)* }
}

#[cfg(test)]
mod tests {
    use crate::parse::extract_field_attrs;
    use syn::{parse_quote, DeriveInput};

    fn input(src: DeriveInput) -> syn::Result<crate::parse::FieldInputAttr> {
        extract_field_attrs(&src).map(|a| a.inputs[0].clone())
    }

    fn refusal(src: DeriveInput) -> String {
        input(src).expect_err("refused").to_string()
    }

    #[test]
    fn bare_accumulate_gets_the_default_cap_as_its_depth() {
        let i = input(parse_quote! { struct N { #[input(accumulate)] tf: TFMessage } }).unwrap();
        assert_eq!(i.accumulate, Some(10));
        assert_eq!(i.depth, Some(10));
    }

    #[test]
    fn an_explicit_cap_sets_the_depth() {
        let i = input(parse_quote! {
            struct N { #[input(accumulate, cap = 32, expect_within_ms = 5)] tf: TFMessage }
        })
        .unwrap();
        assert_eq!(i.accumulate, Some(32));
        assert_eq!(i.depth, Some(32));
        assert_eq!(i.expect_within_ms, Some(5));
    }

    #[test]
    fn the_cap_bounds_are_inclusive() {
        let one = input(parse_quote! { struct N { #[input(accumulate, cap = 1)] tf: T } }).unwrap();
        let max =
            input(parse_quote! { struct N { #[input(accumulate, cap = 64)] tf: T } }).unwrap();
        assert_eq!((one.accumulate, max.accumulate), (Some(1), Some(64)));
    }

    #[test]
    fn explicit_drop_oldest_is_accepted() {
        let i = input(parse_quote! {
            struct N { #[input(accumulate, backpressure = drop_oldest)] tf: T }
        })
        .unwrap();
        assert_eq!(i.accumulate, Some(10));
    }

    const TAIL: &str = "With `accumulate` the accepted keys are `cap = N`, \
                        `backpressure = drop_oldest` and `expect_within_ms = N`.";

    #[test]
    fn every_refused_combination_names_the_accepted_set() {
        let cases: [(DeriveInput, &str); 6] = [
            (
                parse_quote! { struct N { #[input(accumulate, trigger)] tf: T } },
                "`#[input(accumulate)]` cannot be combined with `trigger`",
            ),
            (
                parse_quote! { struct N { #[input(accumulate, backpressure = block)] tf: T } },
                "`#[input(accumulate)]` cannot be combined with `backpressure = block`",
            ),
            (
                parse_quote! { struct N { #[input(accumulate, backpressure = sample(5))] tf: T } },
                "`#[input(accumulate)]` cannot be combined with `backpressure = sample(N)`",
            ),
            (
                parse_quote! { struct N { #[input(accumulate, depth = 4)] tf: T } },
                "`#[input(accumulate)]` takes `cap = N`, not `depth`",
            ),
            (
                parse_quote! { struct N { #[input(accumulate, cap = 0)] tf: T } },
                "`cap` must be in 1..=64 (got 0).",
            ),
            (
                parse_quote! { struct N { #[input(accumulate, cap = 65)] tf: T } },
                "`cap` must be in 1..=64 (got 65).",
            ),
        ];
        for (src, head) in cases {
            let msg = refusal(src);
            assert!(msg.starts_with(head), "want `{head}`, got: {msg}");
            assert!(
                msg.ends_with(TAIL),
                "must name the accepted set, got: {msg}"
            );
        }
    }

    #[test]
    fn cap_without_accumulate_is_refused() {
        assert_eq!(
            refusal(parse_quote! { struct N { #[input(cap = 4)] tf: T } }),
            "`cap` applies only to `#[input(accumulate)]`. For the queue size of a plain input, \
             use `depth = N`."
        );
    }

    #[test]
    fn a_plain_input_is_not_accumulate() {
        let i = input(parse_quote! { struct N { #[input(depth = 3)] tf: T } }).unwrap();
        assert_eq!(i.accumulate, None);
        assert_eq!(i.depth, Some(3));
    }
}
