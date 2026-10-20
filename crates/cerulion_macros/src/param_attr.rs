// SPDX-License-Identifier: AGPL-3.0-only
//! `#[param(...)]`: a typed knob on a plain node field.
//!
//! ```text
//! #[cerulion_node(period_ms = 100)]
//! struct Controller {
//!     #[output]
//!     cmd: Vector3,
//!     #[param(default = 2.0, min = 0.0, max = 5.0, description = "loop gain")]
//!     gain: f64,
//!     #[param(default = "base_link")]
//!     frame_id: String,
//! }
//! ```
//!
//! The field type IS the param type, so a name cannot be typoed and a value
//! cannot be coerced: the node reads `self.gain`. Accepted field types:
//! `bool`, `i8` to `i64`, `u8` to `u64`, `f32`, `f64`, `String`. Accepted
//! keys: `default` (required), `min`, `max` (numeric fields only),
//! `description`. `live` is reserved for the live tier and refused.
//!
//! Every check a graph build would make on a default is made HERE, at
//! expansion: the default has the field's type and lies within the bounds,
//! `min <= max`, bounds only on numeric fields, a float default is a float
//! literal (`2.0`, never `2`). A node that compiles has a spec the graph
//! build can trust.
//!
//! This module owns the attribute's parse, its checks and the three pieces
//! of code it adds to the node: the spec list for `NodeInfo`, the JSON
//! fragment for the cdylib info document, and the `__cer_apply_param_defaults`
//! / `__cer_apply_params` methods on the user struct that write the resolved
//! values into the fields before the node's `init` runs.

use proc_macro2::{Literal, Span, TokenStream};
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{Ident, Token};

/// The accepted keys, in the order the error text lists them.
const ACCEPTED_KEYS: &str = "`default`, `min`, `max`, `description`";

/// The accepted field types, in the order the error text lists them.
const ACCEPTED_TYPES: &str = "bool, i8, i16, i32, i64, u8, u16, u32, u64, f32, f64, String";

/// The Rust type of a `#[param]` field. Mirrors `cerulion_core::graph::params::ParamType`
/// (the macro crate cannot depend on core; the two are kept in step by
/// `as_str`, which spells each variant exactly as the core type serializes it).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamKind {
    Bool,
    I8,
    I16,
    I32,
    I64,
    U8,
    U16,
    U32,
    U64,
    F32,
    F64,
    String,
}

impl ParamKind {
    fn from_type(ty: &syn::Type) -> Option<Self> {
        let syn::Type::Path(path) = ty else {
            return None;
        };
        if path.qself.is_some() {
            return None;
        }
        let last = path.path.segments.last()?;
        if !last.arguments.is_empty() {
            return None;
        }
        Some(match last.ident.to_string().as_str() {
            "bool" => Self::Bool,
            "i8" => Self::I8,
            "i16" => Self::I16,
            "i32" => Self::I32,
            "i64" => Self::I64,
            "u8" => Self::U8,
            "u16" => Self::U16,
            "u32" => Self::U32,
            "u64" => Self::U64,
            "f32" => Self::F32,
            "f64" => Self::F64,
            "String" => Self::String,
            _ => return None,
        })
    }

    /// The Rust spelling, and the `ParamType` serialization in core.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Bool => "bool",
            Self::I8 => "i8",
            Self::I16 => "i16",
            Self::I32 => "i32",
            Self::I64 => "i64",
            Self::U8 => "u8",
            Self::U16 => "u16",
            Self::U32 => "u32",
            Self::U64 => "u64",
            Self::F32 => "f32",
            Self::F64 => "f64",
            Self::String => "String",
        }
    }

    /// The core `ParamType` variant name.
    fn core_variant(self) -> Ident {
        let name = match self {
            Self::Bool => "Bool",
            Self::I8 => "I8",
            Self::I16 => "I16",
            Self::I32 => "I32",
            Self::I64 => "I64",
            Self::U8 => "U8",
            Self::U16 => "U16",
            Self::U32 => "U32",
            Self::U64 => "U64",
            Self::F32 => "F32",
            Self::F64 => "F64",
            Self::String => "String",
        };
        Ident::new(name, Span::call_site())
    }

    fn is_numeric(self) -> bool {
        !matches!(self, Self::Bool | Self::String)
    }

    fn is_float(self) -> bool {
        matches!(self, Self::F32 | Self::F64)
    }

    fn is_unsigned(self) -> bool {
        matches!(self, Self::U8 | Self::U16 | Self::U32 | Self::U64)
    }

    /// The inclusive integer range of an integer kind.
    fn int_range(self) -> Option<(i128, i128)> {
        Some(match self {
            Self::I8 => (i128::from(i8::MIN), i128::from(i8::MAX)),
            Self::I16 => (i128::from(i16::MIN), i128::from(i16::MAX)),
            Self::I32 => (i128::from(i32::MIN), i128::from(i32::MAX)),
            Self::I64 => (i128::from(i64::MIN), i128::from(i64::MAX)),
            Self::U8 => (0, i128::from(u8::MAX)),
            Self::U16 => (0, i128::from(u16::MAX)),
            Self::U32 => (0, i128::from(u32::MAX)),
            Self::U64 => (0, i128::from(u64::MAX)),
            Self::Bool | Self::F32 | Self::F64 | Self::String => return None,
        })
    }
}

/// A literal value as the author wrote it: `default = -5`, `min = 0.0`,
/// `default = "base_link"`, `default = true`.
#[derive(Debug, Clone, PartialEq)]
pub enum ParamLit {
    Bool(bool),
    Int(i128),
    Float(f64),
    Str(String),
}

impl ParamLit {
    /// The value as the user would write it, for diagnostics.
    fn render(&self) -> String {
        match self {
            Self::Bool(b) => b.to_string(),
            Self::Int(i) => i.to_string(),
            Self::Float(x) => format!("{x:?}"),
            Self::Str(s) => format!("{s:?}"),
        }
    }

    /// The literal as a JSON scalar: the shape `ParamValue` deserializes.
    fn json(&self) -> String {
        match self {
            Self::Bool(b) => b.to_string(),
            Self::Int(i) => i.to_string(),
            Self::Float(x) => format!("{x:?}"),
            Self::Str(s) => format!("\"{}\"", crate::codegen::json_escape(s)),
        }
    }

    /// The literal as a typed Rust expression of `kind`'s type, for the
    /// generated default write.
    fn typed_tokens(&self, kind: ParamKind) -> TokenStream {
        match (self, kind) {
            (Self::Bool(b), _) => quote! { #b },
            (Self::Str(s), _) => quote! { ::std::string::String::from(#s) },
            (Self::Float(x), ParamKind::F32) => {
                let lit = Literal::f32_suffixed(*x as f32);
                quote! { #lit }
            }
            (Self::Float(x), _) => {
                let lit = Literal::f64_suffixed(*x);
                quote! { #lit }
            }
            (Self::Int(i), kind) => {
                let lit = match kind {
                    ParamKind::I8 => Literal::i8_suffixed(*i as i8),
                    ParamKind::I16 => Literal::i16_suffixed(*i as i16),
                    ParamKind::I32 => Literal::i32_suffixed(*i as i32),
                    ParamKind::I64 => Literal::i64_suffixed(*i as i64),
                    ParamKind::U8 => Literal::u8_suffixed(*i as u8),
                    ParamKind::U16 => Literal::u16_suffixed(*i as u16),
                    ParamKind::U32 => Literal::u32_suffixed(*i as u32),
                    ParamKind::U64 => Literal::u64_suffixed(*i as u64),
                    // Unreachable after `check_literal`: an integer literal is
                    // only accepted on an integer kind. Emit it as i64 so a
                    // future kind cannot compile to a silent `0`.
                    ParamKind::Bool | ParamKind::F32 | ParamKind::F64 | ParamKind::String => {
                        Literal::i64_suffixed(*i as i64)
                    }
                };
                quote! { #lit }
            }
        }
    }

    /// The literal as a `ParamValue` expression of the canonical variant
    /// for `kind`: unsigned kinds carry `UInt`, signed kinds `Int`.
    fn param_value_tokens(&self, kind: ParamKind) -> TokenStream {
        let root = quote! { ::cerulion_core::graph::params::ParamValue };
        match self {
            Self::Bool(b) => quote! { #root::Bool(#b) },
            Self::Str(s) => quote! { #root::Str(::std::string::String::from(#s)) },
            Self::Float(x) => {
                let lit = Literal::f64_suffixed(*x);
                quote! { #root::Float(#lit) }
            }
            Self::Int(i) if kind.is_unsigned() => {
                let lit = Literal::u64_suffixed(*i as u64);
                quote! { #root::UInt(#lit) }
            }
            Self::Int(i) => {
                let lit = Literal::i64_suffixed(*i as i64);
                quote! { #root::Int(#lit) }
            }
        }
    }

    fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Int(i) => Some(*i as f64),
            Self::Float(x) => Some(*x),
            Self::Bool(_) | Self::Str(_) => None,
        }
    }
}

/// A parsed `#[param(...)]` field attribute, checked and ready for codegen.
#[derive(Debug, Clone)]
pub struct ParamAttr {
    pub field_name: Ident,
    pub field_type: syn::Type,
    pub kind: ParamKind,
    pub default: ParamLit,
    pub min: Option<ParamLit>,
    pub max: Option<ParamLit>,
    pub description: Option<String>,
}

/// One `key = value` item inside `#[param(...)]`, or a bare `live`.
enum ParamItem {
    Default(ParamLit, Span),
    Min(ParamLit, Span),
    Max(ParamLit, Span),
    Description(String, Span),
}

impl Parse for ParamItem {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let key: Ident = input.parse()?;
        match key.to_string().as_str() {
            "live" => Err(syn::Error::new(
                key.span(),
                "`#[param(live)]` is reserved for the live tier and not yet supported; remove \
                 `live`. A static param is fixed at graph load for the whole run.",
            )),
            "default" | "min" | "max" => {
                input.parse::<Token![=]>()?;
                let (lit, span) = parse_value(input)?;
                Ok(match key.to_string().as_str() {
                    "default" => Self::Default(lit, span),
                    "min" => Self::Min(lit, span),
                    _ => Self::Max(lit, span),
                })
            }
            "description" => {
                input.parse::<Token![=]>()?;
                let lit: syn::Lit = input.parse()?;
                match lit {
                    syn::Lit::Str(s) => Ok(Self::Description(s.value(), s.span())),
                    other => Err(syn::Error::new(
                        other.span(),
                        "`description` takes a string literal",
                    )),
                }
            }
            other => Err(syn::Error::new(
                key.span(),
                format!("unknown param attribute `{other}`. Supported: {ACCEPTED_KEYS}."),
            )),
        }
    }
}

/// A scalar literal, with an optional leading `-` on a number.
fn parse_value(input: ParseStream) -> syn::Result<(ParamLit, Span)> {
    let negative = input.peek(Token![-]);
    let minus_span = if negative {
        Some(input.parse::<Token![-]>()?.span)
    } else {
        None
    };
    let lit: syn::Lit = input.parse()?;
    let span = minus_span.unwrap_or_else(|| lit.span());
    let value = match &lit {
        syn::Lit::Int(i) => {
            let v: i128 = i.base10_parse()?;
            ParamLit::Int(if negative { -v } else { v })
        }
        syn::Lit::Float(f) => {
            let v: f64 = f.base10_parse()?;
            ParamLit::Float(if negative { -v } else { v })
        }
        syn::Lit::Bool(b) if !negative => ParamLit::Bool(b.value),
        syn::Lit::Str(s) if !negative => ParamLit::Str(s.value()),
        syn::Lit::Bool(_) | syn::Lit::Str(_) => {
            return Err(syn::Error::new(
                span,
                "`-` applies to a number literal only",
            ));
        }
        _ => {
            return Err(syn::Error::new(
                lit.span(),
                "a param value is a bool, integer, float or string literal",
            ));
        }
    };
    Ok((value, span))
}

/// Parse and check one `#[param(...)]` attribute on `field_name: field_type`.
pub fn parse_param_attr(
    attr: &syn::Attribute,
    field_name: &Ident,
    field_type: &syn::Type,
) -> syn::Result<ParamAttr> {
    let Some(kind) = ParamKind::from_type(field_type) else {
        return Err(syn::Error::new_spanned(
            field_type,
            format!(
                "`#[param]` field `{field_name}` has an unsupported type; a param is one of: \
                 {ACCEPTED_TYPES}"
            ),
        ));
    };
    let list = match &attr.meta {
        syn::Meta::List(list) => list,
        syn::Meta::Path(_) | syn::Meta::NameValue(_) => {
            return Err(syn::Error::new_spanned(
                attr,
                format!(
                    "`#[param]` on `{field_name}` needs `default = <value>`: write \
                     `#[param(default = ..)]`"
                ),
            ));
        }
    };
    use syn::punctuated::Punctuated;
    let items: Punctuated<ParamItem, Token![,]> =
        list.parse_args_with(Punctuated::<ParamItem, Token![,]>::parse_terminated)?;

    let mut default: Option<(ParamLit, Span)> = None;
    let mut min: Option<(ParamLit, Span)> = None;
    let mut max: Option<(ParamLit, Span)> = None;
    let mut description: Option<String> = None;
    for item in items {
        match item {
            ParamItem::Default(lit, span) => set_once(&mut default, lit, span, "default", attr)?,
            ParamItem::Min(lit, span) => set_once(&mut min, lit, span, "min", attr)?,
            ParamItem::Max(lit, span) => set_once(&mut max, lit, span, "max", attr)?,
            ParamItem::Description(text, span) => {
                if description.is_some() {
                    return Err(syn::Error::new(span, "duplicate `description` in #[param]"));
                }
                description = Some(text);
            }
        }
    }
    let Some((default, default_span)) = default else {
        return Err(syn::Error::new_spanned(
            attr,
            format!("`#[param]` on `{field_name}` needs `default = <value>`"),
        ));
    };

    check_literal(&default, default_span, kind, "default", field_name)?;
    for (label, bound) in [("min", &min), ("max", &max)] {
        if let Some((lit, span)) = bound {
            if !kind.is_numeric() {
                return Err(syn::Error::new(
                    *span,
                    format!(
                        "`{label}` is not accepted on the {} param `{field_name}`; bounds apply \
                         to integer and float params only",
                        kind.as_str()
                    ),
                ));
            }
            check_literal(lit, *span, kind, label, field_name)?;
        }
    }
    if let (Some((lo, _)), Some((hi, hi_span))) = (&min, &max) {
        if let (Some(lo_v), Some(hi_v)) = (lo.as_f64(), hi.as_f64()) {
            let inverted = match (lo, hi) {
                (ParamLit::Int(a), ParamLit::Int(b)) => a > b,
                _ => lo_v > hi_v,
            };
            if inverted {
                return Err(syn::Error::new(
                    *hi_span,
                    format!(
                        "`min` {} is greater than `max` {} on param `{field_name}`",
                        lo.render(),
                        hi.render()
                    ),
                ));
            }
        }
    }
    let below = min
        .as_ref()
        .is_some_and(|(lo, _)| lit_less_than(&default, lo));
    let above = max
        .as_ref()
        .is_some_and(|(hi, _)| lit_less_than(hi, &default));
    if below || above {
        let lo = min.as_ref().map(|(l, _)| l.render());
        let hi = max.as_ref().map(|(h, _)| h.render());
        return Err(syn::Error::new(
            default_span,
            format!(
                "`default` {} is outside [{}, {}] on param `{field_name}`",
                default.render(),
                lo.unwrap_or_else(|| "..".to_string()),
                hi.unwrap_or_else(|| "..".to_string())
            ),
        ));
    }

    Ok(ParamAttr {
        field_name: field_name.clone(),
        field_type: field_type.clone(),
        kind,
        default,
        min: min.map(|(l, _)| l),
        max: max.map(|(l, _)| l),
        description,
    })
}

fn set_once(
    slot: &mut Option<(ParamLit, Span)>,
    lit: ParamLit,
    span: Span,
    key: &str,
    attr: &syn::Attribute,
) -> syn::Result<()> {
    if slot.is_some() {
        return Err(syn::Error::new_spanned(
            attr,
            format!("duplicate `{key}` in #[param]"),
        ));
    }
    *slot = Some((lit, span));
    Ok(())
}

/// Integers compare exactly; anything else compares as `f64`.
fn lit_less_than(a: &ParamLit, b: &ParamLit) -> bool {
    match (a, b) {
        (ParamLit::Int(x), ParamLit::Int(y)) => x < y,
        _ => match (a.as_f64(), b.as_f64()) {
            (Some(x), Some(y)) => x < y,
            _ => false,
        },
    }
}

/// The literal under `key` must have the field's type exactly, and an
/// integer must fit the field's width.
fn check_literal(
    lit: &ParamLit,
    span: Span,
    kind: ParamKind,
    key: &str,
    field_name: &Ident,
) -> syn::Result<()> {
    let ty = kind.as_str();
    let shown = lit.render();
    // Ordered, unguarded arms so the match is provably exhaustive: the two
    // non-numeric kinds first, then every literal shape against a numeric
    // kind.
    let message = match (kind, lit) {
        (ParamKind::Bool, ParamLit::Bool(_)) => return Ok(()),
        (ParamKind::String, ParamLit::Str(_)) => return Ok(()),
        (ParamKind::Bool, _) => {
            format!(
                "`{key}` for the bool param `{field_name}` must be `true` or `false`, got {shown}"
            )
        }
        (ParamKind::String, _) => {
            format!(
                "`{key}` for the String param `{field_name}` must be a string literal, got {shown}"
            )
        }
        (k, ParamLit::Int(i)) => match k.int_range() {
            Some((lo, hi)) if (lo..=hi).contains(i) => return Ok(()),
            Some(_) => format!("`{key}` {shown} does not fit {ty} on param `{field_name}`"),
            // A float kind: no coercion, the literal must carry a point.
            None => format!(
                "`{key}` for the {ty} param `{field_name}` must be a float literal such as \
                 {shown}.0, got `{shown}`"
            ),
        },
        (k, ParamLit::Float(x)) => {
            if !k.is_float() {
                format!(
                    "`{key}` for the {ty} param `{field_name}` must be an integer literal, got {shown}"
                )
            } else if !x.is_finite() || (k == ParamKind::F32 && !(*x as f32).is_finite()) {
                format!("`{key}` {shown} is not a finite {ty} on param `{field_name}`")
            } else {
                return Ok(());
            }
        }
        (_, ParamLit::Str(_) | ParamLit::Bool(_)) => {
            format!("`{key}` for the {ty} param `{field_name}` must be a number, got {shown}")
        }
    };
    Err(syn::Error::new(span, message))
}

/// A `#[param]` field is plain node state: it cannot also be a port.
pub fn validate_params_are_not_ports(
    params: &[ParamAttr],
    port_names: impl Iterator<Item = String>,
    errors: &mut Vec<syn::Error>,
) {
    let ports: Vec<String> = port_names.collect();
    for param in params {
        if ports.contains(&param.field_name.to_string()) {
            errors.push(syn::Error::new(
                param.field_name.span(),
                format!(
                    "`#[param]` cannot be combined with `#[input]` or `#[output]` on field \
                     `{}`: a param is plain node state, a port is a message",
                    param.field_name
                ),
            ));
        }
    }
}

/// `ParamSpec` expressions for the in-process `NodeInfo::with_params(vec![..])`.
pub fn gen_spec_exprs(params: &[ParamAttr]) -> Vec<TokenStream> {
    params
        .iter()
        .map(|p| {
            let name = p.field_name.to_string();
            let variant = p.kind.core_variant();
            let default = p.default.param_value_tokens(p.kind);
            let bound = |b: &Option<ParamLit>| match b {
                Some(lit) => {
                    let v = lit.param_value_tokens(p.kind);
                    quote! { ::std::option::Option::Some(#v) }
                }
                None => quote! { ::std::option::Option::None },
            };
            let min = bound(&p.min);
            let max = bound(&p.max);
            let description = match &p.description {
                Some(d) => quote! { ::std::option::Option::Some(::std::string::String::from(#d)) },
                None => quote! { ::std::option::Option::None },
            };
            quote! {
                ::cerulion_core::graph::params::ParamSpec {
                    name: ::std::string::String::from(#name),
                    ty: ::cerulion_core::graph::params::ParamType::#variant,
                    default: #default,
                    min: #min,
                    max: #max,
                    description: #description,
                }
            }
        })
        .collect()
}

/// The `,"params":[..]` fragment of the cdylib info JSON: one object per
/// param in declaration order, the same shape `ParamSpec` deserializes.
/// Empty when the node declares no params, so the key is absent and the
/// host defaults it to an empty list.
pub fn gen_params_json(params: &[ParamAttr]) -> String {
    if params.is_empty() {
        return String::new();
    }
    let objects: Vec<String> = params
        .iter()
        .map(|p| {
            let mut s = format!(
                r#"{{"name":"{}","type":"{}","default":{}"#,
                crate::codegen::json_escape(&p.field_name.to_string()),
                p.kind.as_str(),
                p.default.json()
            );
            if let Some(min) = &p.min {
                s.push_str(&format!(r#","min":{}"#, min.json()));
            }
            if let Some(max) = &p.max {
                s.push_str(&format!(r#","max":{}"#, max.json()));
            }
            if let Some(d) = &p.description {
                s.push_str(&format!(
                    r#","description":"{}""#,
                    crate::codegen::json_escape(d)
                ));
            }
            s.push('}');
            s
        })
        .collect();
    format!(r#","params":[{}]"#, objects.join(","))
}

/// The two apply methods on the user struct.
///
/// `__cer_apply_param_defaults` writes every declared default; the wrapper
/// calls it at construction so a node built with no graph values holds its
/// defaults. `__cer_apply_params` writes the graph's resolved values; the
/// wrapper calls it in `init`, before the user's own `init`, so the user
/// code and the first tick both read the resolved value. Both are one plain
/// field write per param: nothing here runs on the tick path.
pub fn gen_apply_impl(
    struct_name: &Ident,
    impl_generics: &syn::ImplGenerics<'_>,
    ty_generics: &syn::TypeGenerics<'_>,
    where_clause: Option<&syn::WhereClause>,
    params: &[ParamAttr],
) -> TokenStream {
    let default_writes: Vec<TokenStream> = params
        .iter()
        .map(|p| {
            let field = &p.field_name;
            let value = p.default.typed_tokens(p.kind);
            quote! { self.#field = #value; }
        })
        .collect();
    let apply_arms: Vec<TokenStream> = params
        .iter()
        .map(|p| {
            let field = &p.field_name;
            let name = p.field_name.to_string();
            let ty = &p.field_type;
            let ty_name = p.kind.as_str();
            quote! {
                #name => {
                    self.#field = <#ty as ::cerulion_core::graph::params::FromParamValue>
                        ::from_param_value(__cer_value)
                        .ok_or_else(|| ::std::format!(
                            "param '{}' expects {}, got {}", #name, #ty_name, __cer_value,
                        ))?;
                }
            }
        })
        .collect();
    quote! {
        #[automatically_derived]
        impl #impl_generics #struct_name #ty_generics #where_clause {
            /// Write every `#[param]` default into its field.
            #[doc(hidden)]
            pub fn __cer_apply_param_defaults(&mut self) {
                #(#default_writes)*
            }

            /// Write the graph's resolved values into their fields. The graph
            /// build has already checked names, types and bounds; a mismatch
            /// here is a host and node disagreeing, and is refused rather
            /// than written.
            #[doc(hidden)]
            pub fn __cer_apply_params(
                &mut self,
                __cer_values: &::cerulion_core::graph::params::ParamValues,
            ) -> ::std::result::Result<(), ::std::string::String> {
                for (__cer_name, __cer_value) in __cer_values {
                    match __cer_name.as_str() {
                        #(#apply_arms)*
                        __cer_other => {
                            return ::std::result::Result::Err(::std::format!(
                                "unknown param '{}'", __cer_other,
                            ));
                        }
                    }
                }
                ::std::result::Result::Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use syn::parse_quote;

    fn parse(attr: syn::Attribute, ty: syn::Type) -> syn::Result<ParamAttr> {
        let name: Ident = parse_quote!(gain);
        parse_param_attr(&attr, &name, &ty)
    }

    fn err_text(attr: syn::Attribute, ty: syn::Type) -> String {
        parse(attr, ty)
            .expect_err("attribute must be rejected")
            .to_string()
    }

    #[test]
    fn a_full_float_param_parses_with_its_bounds_and_description() {
        let p = parse(
            parse_quote!(#[param(default = 2.0, min = 0.0, max = 5.0, description = "loop gain")]),
            parse_quote!(f64),
        )
        .unwrap();
        assert_eq!(p.kind, ParamKind::F64);
        assert_eq!(p.default, ParamLit::Float(2.0));
        assert_eq!(p.min, Some(ParamLit::Float(0.0)));
        assert_eq!(p.max, Some(ParamLit::Float(5.0)));
        assert_eq!(p.description.as_deref(), Some("loop gain"));
    }

    #[test]
    fn a_negative_integer_default_parses_on_a_signed_field() {
        let p = parse(parse_quote!(#[param(default = -5)]), parse_quote!(i32)).unwrap();
        assert_eq!(p.default, ParamLit::Int(-5));
    }

    #[test]
    fn string_and_bool_defaults_parse() {
        let s = parse(
            parse_quote!(#[param(default = "base_link")]),
            parse_quote!(String),
        )
        .unwrap();
        assert_eq!(s.default, ParamLit::Str("base_link".into()));
        let b = parse(parse_quote!(#[param(default = true)]), parse_quote!(bool)).unwrap();
        assert_eq!(b.default, ParamLit::Bool(true));
    }

    #[test]
    fn an_unknown_key_lists_the_accepted_set() {
        assert_eq!(
            err_text(
                parse_quote!(#[param(default = 1.0, step = 0.1)]),
                parse_quote!(f64)
            ),
            "unknown param attribute `step`. Supported: `default`, `min`, `max`, `description`."
        );
    }

    #[test]
    fn live_is_refused_as_reserved() {
        let text = err_text(
            parse_quote!(#[param(default = 1.0, live)]),
            parse_quote!(f64),
        );
        assert!(
            text.contains("reserved") && text.contains("not yet supported"),
            "{text}"
        );
    }

    #[test]
    fn a_missing_default_is_refused() {
        let text = err_text(parse_quote!(#[param(min = 0.0)]), parse_quote!(f64));
        assert!(text.contains("needs `default = <value>`"), "{text}");
        let bare = err_text(parse_quote!(#[param]), parse_quote!(f64));
        assert!(bare.contains("needs `default = <value>`"), "{bare}");
    }

    #[test]
    fn an_integer_default_on_a_float_field_names_the_float_form() {
        assert_eq!(
            err_text(parse_quote!(#[param(default = 2)]), parse_quote!(f64)),
            "`default` for the f64 param `gain` must be a float literal such as 2.0, got `2`"
        );
    }

    #[test]
    fn a_default_that_does_not_fit_the_width_is_refused() {
        assert_eq!(
            err_text(parse_quote!(#[param(default = 300)]), parse_quote!(u8)),
            "`default` 300 does not fit u8 on param `gain`"
        );
        assert_eq!(
            err_text(parse_quote!(#[param(default = -1)]), parse_quote!(u32)),
            "`default` -1 does not fit u32 on param `gain`"
        );
    }

    #[test]
    fn min_above_max_is_refused() {
        assert_eq!(
            err_text(
                parse_quote!(#[param(default = 1.0, min = 5.0, max = 0.0)]),
                parse_quote!(f64)
            ),
            "`min` 5.0 is greater than `max` 0.0 on param `gain`"
        );
    }

    #[test]
    fn a_default_outside_the_bounds_is_refused() {
        assert_eq!(
            err_text(
                parse_quote!(#[param(default = 9.0, min = 0.0, max = 5.0)]),
                parse_quote!(f64)
            ),
            "`default` 9.0 is outside [0.0, 5.0] on param `gain`"
        );
        assert_eq!(
            err_text(
                parse_quote!(#[param(default = 0, min = 1)]),
                parse_quote!(u8)
            ),
            "`default` 0 is outside [1, ..] on param `gain`"
        );
    }

    #[test]
    fn bounds_on_a_non_numeric_field_are_refused() {
        assert_eq!(
            err_text(parse_quote!(#[param(default = "x", min = 0)]), parse_quote!(String)),
            "`min` is not accepted on the String param `gain`; bounds apply to integer and float params only"
        );
        assert_eq!(
            err_text(parse_quote!(#[param(default = true, max = 1)]), parse_quote!(bool)),
            "`max` is not accepted on the bool param `gain`; bounds apply to integer and float params only"
        );
    }

    #[test]
    fn an_unsupported_field_type_lists_the_accepted_types() {
        let text = err_text(parse_quote!(#[param(default = 1)]), parse_quote!(usize));
        assert!(
            text.contains("unsupported type") && text.contains(ACCEPTED_TYPES),
            "{text}"
        );
        let text = err_text(parse_quote!(#[param(default = 1)]), parse_quote!(Vec<u8>));
        assert!(text.contains("unsupported type"), "{text}");
    }

    #[test]
    fn a_default_of_the_wrong_literal_kind_is_refused() {
        assert_eq!(
            err_text(parse_quote!(#[param(default = "no")]), parse_quote!(bool)),
            "`default` for the bool param `gain` must be `true` or `false`, got \"no\""
        );
        assert_eq!(
            err_text(parse_quote!(#[param(default = 1)]), parse_quote!(String)),
            "`default` for the String param `gain` must be a string literal, got 1"
        );
        assert_eq!(
            err_text(parse_quote!(#[param(default = 1.5)]), parse_quote!(i32)),
            "`default` for the i32 param `gain` must be an integer literal, got 1.5"
        );
    }

    #[test]
    fn duplicate_keys_are_refused() {
        let text = err_text(
            parse_quote!(#[param(default = 1.0, default = 2.0)]),
            parse_quote!(f64),
        );
        assert_eq!(text, "duplicate `default` in #[param]");
    }

    #[test]
    fn the_json_fragment_matches_the_core_spec_shape() {
        let p = parse(
            parse_quote!(#[param(default = 2.0, min = 0.0, max = 5.0, description = "loop \"g\"")]),
            parse_quote!(f64),
        )
        .unwrap();
        let name: Ident = parse_quote!(frame_id);
        let s = parse_param_attr(
            &parse_quote!(#[param(default = "base_link")]),
            &name,
            &parse_quote!(String),
        )
        .unwrap();
        assert_eq!(
            gen_params_json(&[p, s]),
            r#","params":[{"name":"gain","type":"f64","default":2.0,"min":0.0,"max":5.0,"description":"loop \"g\""},{"name":"frame_id","type":"String","default":"base_link"}]"#
        );
        assert_eq!(gen_params_json(&[]), "");
    }

    #[test]
    fn an_unsigned_default_emits_the_uint_variant_and_a_signed_one_the_int_variant() {
        let u = parse(parse_quote!(#[param(default = 7)]), parse_quote!(u16)).unwrap();
        let i = parse(parse_quote!(#[param(default = -7)]), parse_quote!(i16)).unwrap();
        let [u_expr, i_expr] = <[TokenStream; 2]>::try_from(gen_spec_exprs(&[u, i])).unwrap();
        assert!(u_expr.to_string().contains("UInt (7u64)"), "{u_expr}");
        assert!(i_expr.to_string().contains("Int (- 7i64)"), "{i_expr}");
    }
}
