// SPDX-License-Identifier: AGPL-3.0-only
//! Node-local copies of the common tick and init rewrite.
//!
//! Port access and helper argument ordering are decided by `impl_macro` once.
//! This adapter changes only the local plumbing types and method names, so
//! native and plugin ticks keep the same write shims and success gates.

use std::collections::BTreeSet;

use proc_macro2::TokenStream;
use quote::{format_ident, quote};
use syn::visit::Visit;
use syn::visit_mut::{self, VisitMut};
use syn::{ImplItem, ItemImpl, Path};

use crate::crate_root;

pub(crate) fn generate(common: &ItemImpl, port_helpers: &BTreeSet<String>) -> TokenStream {
    let mut plugin = common.clone();
    let mut names = port_helpers.clone();
    for item in &common.items {
        if let ImplItem::Fn(method) = item {
            let mut context = ContextType(false);
            context.visit_signature(&method.sig);
            if context.0 {
                names.insert(method.sig.ident.to_string());
            }
        }
    }
    names.insert("__cer_zero_copy_tick".into());
    names.insert("__cer_user_init".into());
    plugin.items.retain(|item| {
        matches!(item, ImplItem::Fn(method) if names.contains(&method.sig.ident.to_string()))
    });
    let mut adapter = Adapter { names: &names };
    adapter.visit_item_impl_mut(&mut plugin);
    quote! {
        #[allow(unexpected_cfgs)]
        #[cfg(feature = "cdylib")]
        #plugin
    }
}

struct Adapter<'a> {
    names: &'a BTreeSet<String>,
}

struct ContextType(bool);
impl<'ast> syn::visit::Visit<'ast> for ContextType {
    fn visit_type_path(&mut self, path: &'ast syn::TypePath) {
        self.0 |= path
            .path
            .segments
            .last()
            .is_some_and(|part| part.ident == "NodeContext");
        syn::visit::visit_type_path(self, path);
    }
}

fn plugin_name(name: &syn::Ident) -> syn::Ident {
    match name.to_string().as_str() {
        "__cer_zero_copy_tick" => format_ident!("__cer_plugin_tick"),
        "__cer_user_init" => format_ident!("__cer_plugin_init"),
        _ => format_ident!("__cer_plugin_{}", name.to_string().trim_start_matches("r#")),
    }
}

impl VisitMut for Adapter<'_> {
    fn visit_impl_item_fn_mut(&mut self, method: &mut syn::ImplItemFn) {
        if self.names.contains(&method.sig.ident.to_string()) {
            for input in &mut method.sig.inputs {
                let syn::FnArg::Typed(input) = input else {
                    continue;
                };
                let syn::Type::Reference(reference) = &mut *input.ty else {
                    continue;
                };
                let syn::Type::Path(context) = &mut *reference.elem else {
                    continue;
                };
                if context
                    .path
                    .segments
                    .last()
                    .is_some_and(|p| p.ident == "NodeContext")
                {
                    let root = crate_root::root();
                    context.path = syn::parse_quote!(#root::graph::node_plugin_abi::NodeContext);
                }
            }
        }
        if self.names.contains(&method.sig.ident.to_string()) {
            method.sig.ident = plugin_name(&method.sig.ident);
        }
        visit_mut::visit_impl_item_fn_mut(self, method);
    }

    fn visit_expr_method_call_mut(&mut self, call: &mut syn::ExprMethodCall) {
        if matches!(&*call.receiver, syn::Expr::Path(receiver) if receiver.path.is_ident("self"))
            && self.names.contains(&call.method.to_string())
        {
            call.method = plugin_name(&call.method);
        }
        visit_mut::visit_expr_method_call_mut(self, call);
    }

    fn visit_expr_path_mut(&mut self, expression: &mut syn::ExprPath) {
        if expression.qself.is_none()
            && expression
                .path
                .segments
                .first()
                .is_some_and(|part| part.ident == "Self")
        {
            if let Some(last) = expression.path.segments.last_mut() {
                if self.names.contains(&last.ident.to_string()) {
                    last.ident = plugin_name(&last.ident);
                }
            }
        }
        visit_mut::visit_expr_path_mut(self, expression);
    }

    fn visit_path_mut(&mut self, path: &mut Path) {
        let root = crate_root::root();
        for (name, module) in [
            ("NodeContext", quote!(graph::node)),
            ("AnyPublisher", quote!(graph::node)),
            ("AnySubscriber", quote!(graph::node)),
            ("LazyOutput", quote!(graph::node)),
            ("InputView", quote!(transport::input_view)),
            ("AccumulatorSet", quote!(transport::accumulate)),
            ("Accumulated", quote!(transport::accumulate)),
        ] {
            let ident = format_ident!("{name}");
            let native: Path = syn::parse_quote!(#root::#module::#ident);
            let count = native.segments.len();
            if path.leading_colon != native.leading_colon
                || path.segments.len() < count
                || !path
                    .segments
                    .iter()
                    .zip(&native.segments)
                    .all(|(a, b)| a.ident == b.ident)
            {
                continue;
            }
            // Include associated constructors as well as type positions.
            // Preserve generic arguments on the type and the untouched tail.
            let args = path.segments[count - 1].arguments.clone();
            let tail = path
                .segments
                .iter()
                .skip(count)
                .cloned()
                .collect::<Vec<_>>();
            *path = syn::parse_quote!(#root::graph::node_plugin_abi::#ident);
            path.segments
                .last_mut()
                .expect("adapter path has a name")
                .arguments = args;
            path.segments.extend(tail);
            break;
        }
        visit_mut::visit_path_mut(self, path);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn compact(tokens: TokenStream) -> String {
        tokens.to_string().split_whitespace().collect()
    }

    #[test]
    fn plugin_uses_local_ports_and_preserves_the_success_gate() {
        let common: ItemImpl = syn::parse_quote! {
            impl Node {
                fn __cer_zero_copy_tick(&mut self, ctx: &mut ::cerulion_core::graph::node::NodeContext,
                    acc: &mut ::cerulion_core::transport::accumulate::AccumulatorSet) {
                    let mut output: ::cerulion_core::graph::node::LazyOutput<'_, Message> = setup(ctx);
                    let result = self.write(&mut output);
                    if result.is_ok() { output.__cer_arm_if_loaned(); }
                }
                fn write(&mut self, out: &mut ::cerulion_core::graph::node::LazyOutput<'_, Message>) {
                    self.observe(); out.__cer_loan()?.__cer_assign_x(7)?;
                }
                fn observe(&self) {}
                fn __cer_user_init(&mut self, ctx: &mut NodeContext) { ctx.env_str("MODE", ""); }
                fn __cer_user_shutdown(&mut self) {}
            }
        };
        let helpers = BTreeSet::from(["write".into()]);
        let original = compact(quote!(#common));
        let emitted = compact(generate(&common, &helpers));
        assert!(emitted.contains("self.__cer_plugin_write(&mutoutput)"));
        assert!(emitted.contains("graph::node_plugin_abi::LazyOutput<'_,Message>"));
        assert!(emitted.contains("graph::node_plugin_abi::NodeContext"));
        assert!(emitted.contains("ifresult.is_ok(){output.__cer_arm_if_loaned();}"));
        assert!(emitted.contains("self.observe();out.__cer_loan()?.__cer_assign_x(7)?;"));
        assert!(!emitted.contains("fnobserve"));
        assert!(!emitted.contains("fn__cer_user_shutdown"));
        assert!(!emitted.contains("graph::node::"));
        assert_eq!(compact(quote!(#common)), original);
    }

    #[test]
    fn adaptation_is_repeatable_and_keeps_nested_reader_lifetimes() {
        let common: ItemImpl = syn::parse_quote! {
            impl Node {
                fn __cer_zero_copy_tick(&mut self, ctx: &mut ::cerulion_core::graph::node::NodeContext,
                    acc: &mut ::cerulion_core::transport::accumulate::AccumulatorSet) {
                    self.read(view);
                }
                fn read(&self, view: &::cerulion_core::transport::input_view::InputView<'_, Message>) { self.read_other(view); }
                fn read_other(&self, view: &::cerulion_core::transport::input_view::InputView<'_, Message>) { view.x(); }
                fn __cer_user_init(&mut self, ctx: &mut NodeContext) {}
            }
        };
        let helpers = BTreeSet::from(["read".into(), "read_other".into()]);
        let first = compact(generate(&common, &helpers));
        assert_eq!(first, compact(generate(&common, &helpers)));
        assert!(first.contains("self.__cer_plugin_read_other(view)"));
        assert_eq!(
            first
                .matches("graph::node_plugin_abi::InputView<'_,Message>")
                .count(),
            2
        );
    }

    #[test]
    fn associated_constructors_and_context_helpers_use_the_same_local_types() {
        let common: ItemImpl = syn::parse_quote! {
            impl Node {
                fn __cer_zero_copy_tick(&mut self, ctx: &mut ::cerulion_core::graph::node::NodeContext) {
                    let output = ::cerulion_core::graph::node::LazyOutput::<Message>::new(publisher);
                    self.r#match(output);
                }
                fn r#match(&mut self, output: ::cerulion_core::graph::node::LazyOutput<'_, Message>) {}
                fn __cer_user_init(&mut self, ctx: &mut NodeContext) { Self::configure(ctx); }
                fn configure(ctx: &mut NodeContext) { ctx.env_str("MODE", "normal"); }
            }
        };
        let emitted = compact(generate(&common, &BTreeSet::from(["r#match".into()])));
        assert!(emitted.contains("graph::node_plugin_abi::LazyOutput::<Message>::new(publisher)"));
        assert!(emitted.contains("self.__cer_plugin_match(output)"));
        assert!(emitted.contains("Self::__cer_plugin_configure(ctx)"));
        assert!(emitted.contains("fn__cer_plugin_configure(ctx:&mut::cerulion_core::graph::node_plugin_abi::NodeContext)"));
        assert!(!emitted.contains("graph::node::"));
    }
}
