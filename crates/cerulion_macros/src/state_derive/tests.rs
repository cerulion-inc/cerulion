//! Oracle tests for the derive's PURE decisions.
//!
//! What a proc-macro unit test can see is the classification and the emitted
//! token stream; what it CANNOT see is whether the emission type-checks, what
//! bytes it produces, or how many diagnostics rustc renders. Those three live
//! in `cerulion_core`: `tests/ui/pass/` compiles the happy shapes,
//! `tests/state_derive_test.rs` drives real captures against hand oracles, and
//! `tests/state_derive_diagnostic_test.rs` counts `compiler-message` records.
//! Assertions here are about the DECISION, never about a second run of the
//! generator.

use super::*;
use syn::parse_quote;

/// The escape a field is classified as, through the production path.
fn escape_of(field: &Field) -> Escape {
    classify(field).expect("classifies").escape
}

fn field(tokens: proc_macro2::TokenStream) -> Field {
    Field::parse_named.parse2(tokens).expect("field parses")
}

use syn::parse::Parser;

#[test]
fn an_ordinary_field_is_captured_and_an_explicit_escape_wins() {
    assert_eq!(escape_of(&field(quote! { pose: Pose })), Escape::Captured);
    assert_eq!(
        escape_of(&field(
            quote! { #[cerulion(reconstruct)] cuda: CudaContext }
        )),
        Escape::Reconstruct
    );
    assert_eq!(
        escape_of(&field(quote! { #[cerulion(serde)] iso: Isometry3<f64> })),
        Escape::Serde
    );
    assert_eq!(
        escape_of(&field(
            quote! { #[cerulion(unordered)] grid: HashMap<Cell, u8> }
        )),
        Escape::Unordered
    );
}

#[test]
fn a_recognised_resource_needs_no_attribute() {
    // The whole point is that this field carries no tag.
    assert_eq!(
        escape_of(&field(quote! { transport: Option<Arc<TransportManager>> })),
        Escape::Reconstruct
    );
    assert_eq!(
        escape_of(&field(quote! { pump: Option<Box<dyn Shutdown>> })),
        Escape::Reconstruct
    );
    // An EXPLICIT attribute reaches the same verdict, deliberately: the two
    // provenances mean the same thing, so they must produce the same code and
    // the same shape.
    assert_eq!(
        escape_of(&field(quote! { #[cerulion(reconstruct)] h: MyOwnHandle })),
        Escape::Reconstruct
    );
    // ANTI-TAUTOLOGY: an ordinary field is still captured.
    assert_eq!(escape_of(&field(quote! { pose: Pose })), Escape::Captured);
}

#[test]
fn an_explicit_attribute_overrides_the_inventory() {
    // A user who says `serde` about a type the inventory would reconstruct is
    // making a deliberate claim; the attribute is the stronger signal.
    assert_eq!(
        escape_of(&field(quote! { #[cerulion(serde)] c: Command })),
        Escape::Serde
    );
}

#[test]
fn a_typoed_key_is_refused_rather_than_silently_meaning_capture_it() {
    // The no-silent-inference rule applied literally: `#[cerulion(recontsruct)]` silently meaning
    // "capture it" would restore a stale handle.
    let f = field(quote! { #[cerulion(recontsruct)] cuda: CudaContext });
    let err = parse_field_escape(&f).expect_err("a typo must be refused");
    let msg = err.to_string();
    assert!(
        msg.contains("recontsruct"),
        "names the offending key: {msg}"
    );
    assert!(msg.contains("reconstruct"), "lists the real keys: {msg}");
    assert!(msg.contains("serde") && msg.contains("unordered"), "{msg}");
}

#[test]
fn two_escapes_on_one_field_are_refused() {
    let f = field(quote! { #[cerulion(reconstruct)] #[cerulion(serde)] x: Foo });
    let err = parse_field_escape(&f).expect_err("two keys must be refused");
    assert!(err.to_string().contains("only one"), "{err}");
}

#[test]
fn the_unordered_escape_reads_the_container_it_was_put_on() {
    let map: Type = parse_quote!(HashMap<Cell, Occupancy>);
    let plan = unordered_plan(&map).expect("a map is supported");
    assert!(matches!(plan.kind, UnorderedKind::Map));
    assert_eq!(plan.args.len(), 2);
    assert_eq!(plan.container, "HashMap");

    let set: Type = parse_quote!(std::collections::HashSet<Cell>);
    let plan = unordered_plan(&set).expect("a set is supported");
    assert!(matches!(plan.kind, UnorderedKind::Set));
    assert_eq!(plan.args.len(), 1);
    assert_eq!(plan.container, "HashSet", "keyed on the LAST path segment");

    let index: Type = parse_quote!(IndexMap<K, V>);
    assert!(unordered_plan(&index).is_ok());
}

#[test]
fn unordered_on_something_that_is_not_a_container_is_refused_with_the_list() {
    let plain: Type = parse_quote!(Vec<u8>);
    let err = unordered_plan(&plain).expect_err("a Vec has no unordered form");
    let msg = err.to_string();
    assert!(msg.contains("Vec"), "names what it found: {msg}");
    assert!(msg.contains("HashMap") && msg.contains("IndexSet"), "{msg}");

    // The offered list must not advertise a container the escape cannot serve.
    assert!(
        !msg.contains("BTree"),
        "a BTree container cannot take this escape, so it must not be offered:\n{msg}"
    );

    // An alias hides the element types the encoder must name.
    let aliased: Type = parse_quote!(MyGrid);
    assert!(unordered_plan(&aliased).is_err());
}

#[test]
fn unordered_on_a_btree_container_is_refused_at_the_attribute_not_inside_std() {
    // The escape's whole claim is "no `Ord` on the key". Rebuilding a BTree
    // goes through `insert`, whose bound IS `Ord`, so accepting the attribute
    // here promised something it could not deliver: MEASURED before the
    // refusal existed, `#[cerulion(unordered)] BTreeMap<f64, u8>` failed with
    // "the trait bound `f64: Ord` is not satisfied ... required by a bound in
    // `BTreeMap::<K, V, A>::insert`" — an error pointing into `std`.
    for ty in [
        parse_quote!(BTreeMap<Cell, u8>),
        parse_quote!(std::collections::BTreeMap<Cell, u8>),
        parse_quote!(BTreeSet<Cell>),
        parse_quote!(std::collections::BTreeSet<Cell>),
    ] {
        let ty: Type = ty;
        let err = unordered_plan(&ty).expect_err("a BTree container cannot take this escape");
        let msg = err.to_string();
        assert!(
            msg.contains("BTree") && msg.contains("Ord") && msg.contains("insert"),
            "names the container, the bound, and where it comes from:\n{msg}"
        );
        // The remedy has to be reachable: dropping the attribute WORKS, because
        // a BTree container is already deterministic and already refuses
        // duplicates. A message that only listed the hash containers would tell
        // a user to change their data structure for no reason. (It is the FIRST
        // remedy inside the std-scoped branch — that it is scoped there rather
        // than led with unscoped is pinned by the sibling test below.)
        assert!(
            msg.contains("Drop the attribute"),
            "the cheap remedy is offered:\n{msg}"
        );
    }

    // The containers the escape CAN serve are untouched — without this the
    // refusal above is satisfied by a plan that rejects everything.
    assert!(unordered_plan(&parse_quote!(HashMap<Cell, u8>)).is_ok());
    assert!(unordered_plan(&parse_quote!(IndexSet<Cell>)).is_ok());
}

#[test]
fn the_btree_refusal_tells_a_same_named_custom_container_what_to_do_instead() {
    // A macro sees TOKENS, not types, so the arm above catches a container of
    // the user's OWN that merely shares the name. MEASURED both directions
    // before this clause was written: under the pre-refusal arm a
    // `foo::BTreeMap<u32, u8>` whose `insert` needs no `Ord` derived cleanly
    // AND round-tripped, and today it is refused by a message explaining
    // itself entirely in terms of `std`'s `insert` — false of their type —
    // whose remedies are wrong for them (dropping the attribute leaves a
    // container with no `CerulionState`).
    //
    // The refusal stays name-keyed (see `unordered_plan`'s docs: a
    // `std::`-qualified-only rule would miss `use std::collections::BTreeMap;`,
    // the spelling the compile-fail fixture itself uses, and so would close
    // none of the class). What has to hold is that the message names the
    // KEYING and a remedy that EXISTS — both named remedies were driven
    // against a custom `BTreeMap`-named container before being offered here.
    //
    // Every form carries the clause, deliberately: a BARE `BTreeMap` is
    // itself ambiguous (`use foo::BTreeMap;` renders exactly like the `std`
    // import), so no form of this refusal can know which type it is looking
    // at and none of them may claim to.
    for ty in [
        parse_quote!(foo::BTreeMap<Cell, u8>),
        parse_quote!(my_crate::collections::BTreeSet<Cell>),
        parse_quote!(BTreeMap<Cell, u8>),
        parse_quote!(std::collections::BTreeSet<Cell>),
    ] {
        let ty: Type = ty;
        let err = unordered_plan(&ty).expect_err("keyed on the LAST path segment");
        let msg = err.to_string();
        assert!(
            msg.contains("matched on the NAME you wrote"),
            "the message must say what it is really keyed on:\n{msg}"
        );
        assert!(
            msg.contains("a container of your OWN that merely shares the name"),
            "and that a same-named custom container lands here too:\n{msg}"
        );
        // The remedy has to be one that EXISTS for a container the derive
        // knows nothing about. `Drop the attribute` does NOT: it leaves a field
        // whose type has no `CerulionState`, i.e. `E0277`. These two do.
        assert!(
            msg.contains("`#[cerulion(serde)]`"),
            "names the escape that captures an opaque container:\n{msg}"
        );
        assert!(
            msg.contains("implement `CerulionState` for it"),
            "and the hand-impl route:\n{msg}"
        );

        // ORDER is part of the contract, not presentation. A remedy that is a
        // DEAD END for this reader must not stand in front of the clause
        // written for them: led with unscoped, `Drop the attribute` is the
        // first instruction a custom container's owner reads and follows, and
        // they hit `E0277` before ever reaching the two remedies above. So each
        // remedy has to sit INSIDE the branch it serves, and the keying — the
        // sentence that tells the reader which branch is theirs — has to come
        // before either branch.
        let at = |needle: &str| {
            msg.find(needle)
                .unwrap_or_else(|| panic!("message must carry {needle:?}:\n{msg}"))
        };
        let keying = at("matched on the NAME you wrote");
        let std_branch = at("If this is std's");
        let own_branch = at("If instead this is a container of your OWN");
        let drop_attribute = at("Drop the attribute");
        let serde_escape = at("`#[cerulion(serde)]`");
        let hand_impl = at("implement `CerulionState` for it");

        assert!(
            keying < std_branch && std_branch < own_branch,
            "the keying must be stated before either branch:\n{msg}"
        );
        assert!(
            std_branch < drop_attribute && drop_attribute < own_branch,
            "`Drop the attribute` must sit INSIDE the std-scoped branch — as an \
             unscoped leading imperative it is a dead end for the reader this \
             clause exists for:\n{msg}"
        );
        assert!(
            own_branch < serde_escape && own_branch < hand_impl,
            "both custom-container remedies must be reachable without passing \
             through the std-only one:\n{msg}"
        );
    }
}

#[test]
fn a_tuple_struct_is_refused_because_the_shape_is_keyed_by_field_name() {
    let input: DeriveInput = parse_quote! {
        struct Pose(f64, f64);
    };
    let err = expand(&input).expect_err("a tuple struct must be refused");
    let msg = err.to_string();
    assert!(msg.contains("tuple struct"), "{msg}");
    assert!(msg.contains("transposition"), "states WHY: {msg}");
}

#[test]
fn a_union_is_refused() {
    let input: DeriveInput = parse_quote! {
        union Raw { a: u32, b: f32 }
    };
    let err = expand(&input).expect_err("a union must be refused");
    assert!(err.to_string().contains("union"), "{err}");
}

#[test]
fn an_escape_inside_an_enum_variant_is_refused_with_the_reason() {
    let input: DeriveInput = parse_quote! {
        enum Sample {
            Cloud {
                #[cerulion(reconstruct)]
                handle: File,
            },
        }
    };
    let err = expand(&input).expect_err("an escape in a variant must be refused");
    let msg = err.to_string();
    assert!(msg.contains("enum variant"), "{msg}");
    assert!(
        msg.contains("replacing the whole value"),
        "states WHY: {msg}"
    );
}

#[test]
fn a_reconstruct_in_a_variant_leads_with_the_remedy_that_restores() {
    // The refusal used to end, for ALL THREE keys, "Move the field into a
    // struct that carries the escape, and hold that struct in the variant".
    // For `reconstruct` that is a DEAD END: the wrapper's `cer_read` returns
    // `Unrestorable`, and an enum restores by calling `cer_read`, so the
    // variant captures and can never be restored (MEASURED: 5 bytes captured,
    // then ``restore -> `ConnState` can be captured but not reconstructed``).
    // Led with, it is the first instruction the reader follows.
    let input: DeriveInput = parse_quote! {
        enum Link {
            Connected {
                #[cerulion(reconstruct)]
                sock: TcpStream,
            },
        }
    };
    let msg = expand(&input)
        .expect_err("an escape in a variant must be refused")
        .to_string();

    let hoist = msg
        .find("OUTSIDE the enum")
        .unwrap_or_else(|| panic!("names the remedy that actually restores: {msg}"));
    assert!(
        msg.contains("restored (& mut self)") || msg.contains("restored(&mut self)"),
        "says where the handle is re-opened: {msg}"
    );

    // The wrap is still offered — it compiles, and it fails LOUDLY rather than
    // silently — but only AFTER the working remedy and only with its cost
    // stated. Ordering is the contract, so it is asserted as an ordering.
    let wrap = msg
        .find("Wrapping the handle")
        .unwrap_or_else(|| panic!("still offers the wrap: {msg}"));
    assert!(
        hoist < wrap,
        "the remedy that restores must come FIRST: {msg}"
    );
    assert!(
        msg.contains("Unrestorable"),
        "names the cost of the wrap by the error it produces: {msg}"
    );
}

#[test]
fn a_serde_or_unordered_in_a_variant_keeps_the_wrap_remedy_that_works_for_it() {
    // The other half of the branch, and the anti-overreach control: the wrap
    // is a DEAD END only for `reconstruct`. A wrapper carrying a `serde` or
    // `unordered` field reads back, so the variant round-trips and the wrap is
    // the whole remedy — telling those users to hoist the field out of the
    // enum would be wrong.
    for tokens in [
        quote! {
            enum Payload {
                Blob {
                    #[cerulion(serde)]
                    body: Config,
                },
            }
        },
        quote! {
            enum Payload {
                Cells {
                    #[cerulion(unordered)]
                    grid: HashMap<Cell, u8>,
                },
            }
        },
    ] {
        let input: DeriveInput = syn::parse2(tokens).expect("parses");
        let msg = expand(&input)
            .expect_err("an escape in a variant must be refused")
            .to_string();
        assert!(
            msg.contains("hold that struct in the variant"),
            "the wrap is the remedy here: {msg}"
        );
        assert!(
            msg.contains("round-trips"),
            "says the wrap actually reads back: {msg}"
        );
        assert!(
            !msg.contains("OUTSIDE the enum"),
            "must NOT send a capturable field out of the enum: {msg}"
        );
        assert!(
            !msg.contains("Unrestorable"),
            "the wrap does not fail for this key: {msg}"
        );
    }
}

#[test]
fn an_enum_variants_members_are_bound_through_the_diagnostic_that_is_true_there() {
    // `CerulionState`'s own `on_unimplemented` names `#[cerulion(reconstruct)]`
    // as the handle fix — and the derive REFUSES that attribute on a variant's
    // field, so an enum holding an unrecognised handle sent the user round a
    // closed loop. The fix is `CerulionVariantMember`, whose message is true
    // there. EVERY member use must name it: one `CerulionState` use left in
    // the emission raises its own obligation and prints the misleading note as
    // a second `E0277` block.
    // A type the inventory does NOT recognise, so both halves below turn on
    // the trait the derive picks and nothing else. (A recognised handle takes
    // a different struct path — see the sibling test.)
    let input: DeriveInput = parse_quote! {
        enum Link {
            Idle,
            Connected(CudaContext),
            Named { ctx: CudaContext },
        }
    };
    let text = emitted(input);
    assert!(
        text.contains("CudaContext : :: cerulion_core :: state :: CerulionVariantMember"),
        "the variant member obligation is the one that is true here:\n{text}"
    );

    // Scoped to MEMBER uses. Two `CerulionState` mentions must survive and are
    // asserted below: the `impl .. for Link` header (the trait being derived)
    // and `<Self as CerulionState>::INLINE_SAFE` in the probe's short circuit.
    // Neither names a member type, so neither can raise a member obligation.
    assert!(
        !text.contains("CudaContext as :: cerulion_core :: state :: CerulionState"),
        "a member TYPE use would raise the misleading obligation:\n{text}"
    );
    for call in [
        "CerulionState :: cer_capture",
        "CerulionState :: cer_probe",
        "CerulionState :: cer_read",
    ] {
        assert!(
            !text.contains(call),
            "a member VALUE use through `{call}` would raise it too:\n{text}"
        );
    }
    assert!(
        text.contains("impl :: cerulion_core :: state :: CerulionState for Link"),
        "the enum still implements the real trait:\n{text}"
    );
    assert!(
        text.contains("< Self as :: cerulion_core :: state :: CerulionState > :: INLINE_SAFE"),
        "the probe short circuit still reads Self's own const:\n{text}"
    );

    // The STRUCT path is untouched — it is where `#[cerulion(reconstruct)]` is
    // genuinely available, so its message is the right one. Without this the
    // assertion above could be satisfied by moving every path onto the variant
    // trait, which would make the struct diagnostic false instead.
    let struct_input: DeriveInput = parse_quote! {
        struct Link { ctx: CudaContext }
    };
    let struct_text = emitted(struct_input);
    assert!(
        struct_text.contains("CudaContext : :: cerulion_core :: state :: CerulionState"),
        "a struct field keeps the trait whose fix it can actually use:\n{struct_text}"
    );
    assert!(
        !struct_text.contains("CerulionVariantMember"),
        "a struct field is not a variant member:\n{struct_text}"
    );
}

#[test]
fn a_recognised_handle_auto_reconstructs_in_a_struct_and_is_bound_inside_a_variant() {
    // The asymmetry item 1 is about, pinned rather than left implicit. A
    // struct field holding an inventory name auto-classifies `reconstruct`
    // by design; the same type inside a variant does not, and cannot:
    //
    //  1. `reconstruct` means "restore walks past this field", which needs an
    //     in-place field. An enum restores by REPLACING the value, so its read
    //     must MINT the variant, and a socket cannot be minted. Defaulting one
    //     would fabricate state and silently choose which variant is live.
    //  2. The inventory is NAME-keyed, so a user type sharing a name is a
    //     known false positive. In a struct that fails OPEN (silently
    //     reconstructed — the documented cost). In a variant it would have to
    //     fail CLOSED, so `enum E { V(MyFile) }` with a capturable `MyFile`
    //     would stop compiling. A rule whose failure direction flips between
    //     the two paths is worse than no rule.
    //
    // So the variant member keeps an ordinary obligation, and what changes is
    // WHICH trait carries it — the one whose message does not name an escape
    // this path refuses.
    let struct_input: DeriveInput = parse_quote! {
        struct Link { sock: TcpStream }
    };
    let struct_text = emitted(struct_input);
    assert!(
        struct_text.contains("\"cerulion::reconstruct\""),
        "a recognised handle needs no attribute in a struct:\n{struct_text}"
    );

    let enum_input: DeriveInput = parse_quote! {
        enum Link { Connected(TcpStream) }
    };
    let enum_text = emitted(enum_input);
    assert!(
        !enum_text.contains("\"cerulion::reconstruct\""),
        "a variant member must NOT be silently reconstructed:\n{enum_text}"
    );
    assert!(
        enum_text.contains("TcpStream : :: cerulion_core :: state :: CerulionVariantMember"),
        "it carries an ordinary obligation, under the accurate diagnostic:\n{enum_text}"
    );
}

#[test]
fn a_named_variants_member_folds_its_name_while_a_tuple_variants_stays_positional() {
    // The transposition trap inside an enum: `V { a: u32, b: u32 }` and
    // `V { b: u32, a: u32 }` capture and read in DECLARATION order, so a
    // positional fold leaves them sharing a shape and an old recording
    // restores transposed. The runtime oracle is
    // `state_derive_test::a_named_variants_field_names_are_folded_...`; this
    // is the token-level half that says WHICH builder call is emitted.
    let named: DeriveInput = parse_quote! {
        enum Goal { Point { x: f64, y: f64 } }
    };
    let text = emitted(named);
    assert!(
        text.contains("field (\"x\""),
        "a named variant member folds its NAME:\n{text}"
    );
    assert!(
        text.contains("field (\"y\""),
        "a named variant member folds its NAME:\n{text}"
    );

    // A tuple variant has no names, so there is nothing to fold that would
    // separate `V(f64, f64)` from itself with the slots swapped. It keeps
    // `.element(..)`, and a regression that synthesised `"0"`/`"1"` names —
    // which would move every tuple-variant shape for no gain — fails here.
    let tuple: DeriveInput = parse_quote! {
        enum Goal { Point(f64, f64) }
    };
    let tuple_text = emitted(tuple);
    assert!(
        tuple_text.contains("element ("),
        "a tuple variant member stays positional:\n{tuple_text}"
    );
    assert!(
        !tuple_text.contains("field (\"0\"") && !tuple_text.contains("field (\"1\""),
        "a tuple variant must not fabricate member names:\n{tuple_text}"
    );
}

#[test]
fn an_enum_past_the_tag_width_is_refused_rather_than_silently_widened() {
    let variants = (0..=MAX_ENUM_VARIANTS)
        .map(|i| {
            let id = format_ident!("V{}", i);
            quote! { #id }
        })
        .collect::<Vec<_>>();
    let input: DeriveInput = parse_quote! {
        enum Wide { #(#variants),* }
    };
    let err = expand(&input).expect_err("past the tag width must be refused");
    let msg = err.to_string();
    assert!(msg.contains("256"), "names the limit: {msg}");
    assert!(
        msg.contains("already recorded"),
        "states the cost of widening: {msg}"
    );

    // BOUNDARY, the other side: exactly at the limit is accepted.
    let variants = (0..MAX_ENUM_VARIANTS)
        .map(|i| {
            let id = format_ident!("V{}", i);
            quote! { #id }
        })
        .collect::<Vec<_>>();
    let input: DeriveInput = parse_quote! {
        enum AtLimit { #(#variants),* }
    };
    assert!(expand(&input).is_ok(), "exactly at the limit is fine");
}

// ---------------------------------------------------------------------------
// emission shape — what the token stream must and must not contain
// ---------------------------------------------------------------------------

fn emitted(input: DeriveInput) -> String {
    expand(&input).expect("expands").to_string()
}

#[test]
fn a_captured_field_gets_exactly_one_where_predicate_not_one_per_use() {
    // The 1-vs-2 diagnostic result (MEASURED — see the module docs) rests on
    // this: the obligation is named in the `where` clause, so the body's uses
    // inherit it instead of each raising their own. A regression to per-use
    // obligations shows up as a missing predicate, which is what this counts.
    // This is the CI-GATED half of that pin; the exact rendered count lives in
    // the `#[ignore]`d trybuild fixture, whose snapshot drifts with rustc.
    let input: DeriveInput = parse_quote! {
        struct Pose { x: f64, y: f64 }
    };
    let text = emitted(input);
    let predicates = text
        .matches("f64 : :: cerulion_core :: state :: CerulionState")
        .count();
    assert_eq!(
        predicates, 2,
        "expected exactly one predicate per captured field, got {predicates} in:\n{text}"
    );
}

#[test]
fn every_type_parameter_is_bounded_like_serde_does_it() {
    // A prototype measured the alternative: resolution inside `impl<T>`
    // where nothing about `T` is known loses a capturable payload SILENTLY at
    // every instantiation.
    let input: DeriveInput = parse_quote! {
        struct Holder<T, U> { a: T, b: U }
    };
    let text = emitted(input);
    assert!(text.contains("T : :: cerulion_core :: state :: CerulionState"));
    assert!(text.contains("U : :: cerulion_core :: state :: CerulionState"));
}

#[test]
fn a_reconstruct_field_is_absent_from_capture_and_restore_but_present_in_the_shape() {
    let input: DeriveInput = parse_quote! {
        struct Slam {
            pose: Pose,
            #[cerulion(reconstruct)]
            cuda: CudaContext,
        }
    };
    let text = emitted(input);
    // It contributes its NAME and its ESCAPE KIND — so ADDING the attribute
    // changes STATE_SHAPE, which is what stops an old bag decoding
    // "successfully" against a node that no longer captures the field.
    assert!(text.contains("\"cuda\""), "the name is folded:\n{text}");
    assert!(
        text.contains("\"cerulion::reconstruct\""),
        "the escape kind is folded:\n{text}"
    );
    // ... and nothing touches the field itself.
    assert!(
        !text.contains("self . cuda"),
        "an escaped field must never be read or written:\n{text}"
    );
    // The CAPTURED sibling still is.
    assert!(text.contains("self . pose"));
}

#[test]
fn a_struct_with_a_reconstruct_field_reports_that_it_cannot_be_constructed() {
    let input: DeriveInput = parse_quote! {
        struct Slam {
            pose: Pose,
            #[cerulion(reconstruct)]
            cuda: CudaContext,
        }
    };
    assert!(
        emitted(input).contains("Unrestorable"),
        "cer_read must report rather than invent a value for the escaped field"
    );

    // ANTI-TAUTOLOGY: a struct with no escape constructs normally.
    let input: DeriveInput = parse_quote! {
        struct Pose { x: f64 }
    };
    let text = emitted(input);
    assert!(!text.contains("Unrestorable"), "{text}");
    assert!(text.contains("Ok (Self {"), "{text}");
}

#[test]
fn a_serde_field_forces_inline_safe_false() {
    // A user-written encoder can block, so the sink's byte bound stops
    // being a time bound and the node must take the fork carrier.
    let input: DeriveInput = parse_quote! {
        struct Node {
            #[cerulion(serde)]
            iso: Isometry3<f64>,
        }
    };
    let text = emitted(input);
    assert!(
        text.contains("INLINE_SAFE : bool = true && false"),
        "a serde field must fold INLINE_SAFE to false:\n{text}"
    );

    // ANTI-TAUTOLOGY: an ordinary field folds the type's own answer instead.
    let input: DeriveInput = parse_quote! {
        struct Node { x: f64 }
    };
    assert!(!emitted(input).contains("&& false"));
}

#[test]
fn an_unordered_map_refuses_duplicates_rather_than_letting_insert_swallow_one() {
    // `read_unordered_entries` carries no `Eq`/`Ord` bound, so the refusal is
    // the COLLECTING caller's job — i.e. this derive's. A duplicate-tolerant
    // restore silently holds FEWER entries than the blob declared.
    let input: DeriveInput = parse_quote! {
        struct Grid {
            #[cerulion(unordered)]
            cells: HashMap<Cell, Occupancy>,
        }
    };
    let text = emitted(input);
    assert!(text.contains("DuplicateEntry"), "{text}");
    assert!(
        text.contains("is_some ()"),
        "map insert is checked:\n{text}"
    );
    assert!(text.contains("\"HashMap\""), "names the container:\n{text}");
    // It must NOT name the whole map type in a CerulionState bound — that
    // bound is the `K: Ord` obligation the escape exists to avoid.
    assert!(
        !text.contains("HashMap < Cell , Occupancy > : :: cerulion_core :: state :: CerulionState"),
        "the unordered escape must not re-impose the ordered bound:\n{text}"
    );
}

#[test]
fn an_unordered_set_checks_its_own_insert_shape() {
    // A set's `insert` returns `bool`, not `Option` — using the map shape
    // would not compile, and using no check at all would swallow duplicates.
    let input: DeriveInput = parse_quote! {
        struct Seen {
            #[cerulion(unordered)]
            ids: HashSet<Key>,
        }
    };
    let text = emitted(input);
    assert!(text.contains("DuplicateEntry"), "{text}");
    assert!(text.contains("! __cer_out . insert"), "{text}");
}

#[test]
fn the_decode_the_derive_emits_is_budgeted() {
    // Anything blob-driven the generated code LOOPS over
    // must be charged. The derive's own loops are the unordered readers, and
    // both `read_unordered_*` draw from `read_element_count`. A struct's
    // fields are a fixed list, so they need no count at all.
    let input: DeriveInput = parse_quote! {
        struct Grid {
            #[cerulion(unordered)]
            cells: HashMap<Cell, Occupancy>,
        }
    };
    let text = emitted(input);
    assert!(text.contains("read_unordered_entries"), "{text}");
    assert!(
        !text.contains("read_payload_len"),
        "the derive must never drive a loop from an UNCHARGED length:\n{text}"
    );
}

#[test]
fn an_enum_tags_its_variants_and_refuses_an_unknown_tag() {
    let input: DeriveInput = parse_quote! {
        enum PadEvent {
            Button { id: u8, down: bool },
            Axis(u8, f32),
            Disconnected,
        }
    };
    let text = emitted(input);
    // One byte, matching Option/Result.
    assert!(text.contains("take_array :: < 1 > ()"), "{text}");
    assert!(text.contains("InvalidTag"), "{text}");
    // The variant NAME and ARITY are folded, so reordering or adding a member
    // moves the shape.
    assert!(
        text.contains("\"Button\"") && text.contains("\"Axis\""),
        "{text}"
    );
    assert!(text.contains("count (2usize)"), "arity is folded:\n{text}");
    assert!(
        text.contains("count (3usize)"),
        "variant count is folded:\n{text}"
    );
    // An enum must NOT override cer_restore: its variant can change, so the
    // default whole-value replacement is the only correct behaviour.
    assert!(
        !text.contains("fn cer_restore"),
        "an enum must inherit the default restore:\n{text}"
    );
}

#[test]
fn a_struct_does_override_cer_restore_so_escaped_fields_are_left_alone() {
    // The mirror of the assertion above — without it, "an enum has no
    // cer_restore" could be satisfied by a derive that emits one for nothing.
    let input: DeriveInput = parse_quote! {
        struct Pose { x: f64 }
    };
    assert!(emitted(input).contains("fn cer_restore"));
}

#[test]
fn a_unit_struct_captures_nothing_and_still_round_trips() {
    let input: DeriveInput = parse_quote! {
        struct Marker;
    };
    let text = emitted(input);
    assert!(
        text.contains("MIN_ENCODED_BYTES : usize = 0usize"),
        "{text}"
    );
    assert!(text.contains("INLINE_SAFE : bool = true"), "{text}");
    assert!(text.contains("Ok (Self { })"), "{text}");
}

// ---------------------------------------------------------------------------
// the `#[cerulion_node]` FOLD-IN
// ---------------------------------------------------------------------------

/// Drive the fold-in's entry point the way `codegen::generate` does.
fn node_emitted(input: DeriveInput) -> String {
    expand_node(&input).expect("expands").to_string()
}

/// A node struct as the ATTRIBUTE macro sees it: port attrs still attached,
/// `__cer_rt` not yet injected (that happens after `expand_node` runs).
fn node_input() -> DeriveInput {
    parse_quote! {
        struct SlamNode {
            #[input(trigger)]
            scan: LaserScan,
            #[output]
            map_out: OccupancyGrid,
            pose: Pose,
            frames: u64,
        }
    }
}

#[test]
fn a_nodes_walk_excludes_its_ports_and_names_only_its_state() {
    // The load-bearing exclusion. A port's declared type is a zero-sized SHM
    // marker with no `CerulionState` impl and never will have one, so an
    // emission that walked it would make EVERY node in the system a compile
    // error — not a worse report, a broken build.
    let text = node_emitted(node_input());

    assert!(
        text.contains("\"pose\"") && text.contains("\"frames\""),
        "both state fields must be in the shape:\n{text}"
    );
    assert!(
        !text.contains("\"scan\"") && !text.contains("\"map_out\""),
        "a PORT must contribute nothing — no shape term, no bound:\n{text}"
    );
    assert!(
        !text.contains("LaserScan") && !text.contains("OccupancyGrid"),
        "a port's TYPE must never appear in a predicate or a term:\n{text}"
    );
}

#[test]
fn a_node_names_one_where_predicate_per_captured_field_and_none_per_port() {
    // The diagnostic half, CI-gated. The rendered count lives in the
    // `#[ignore]`d `tests/ui/type_error/node_uncapturable_field.rs`, whose
    // snapshot drifts with rustc; this counts the predicate that produces it.
    let text = node_emitted(node_input());
    let bound = ": :: cerulion_core :: state :: CerulionState";
    assert_eq!(
        text.matches(&format!("Pose {bound}")).count(),
        1,
        "exactly one predicate for the captured `pose`:\n{text}"
    );
    assert_eq!(
        text.matches(&format!("u64 {bound}")).count(),
        1,
        "exactly one predicate for the captured `frames`:\n{text}"
    );
}

#[test]
fn a_field_declaring_both_input_and_output_is_excluded_exactly_once() {
    // Such a field lands in BOTH of `FieldAttrs`' vectors, so
    // a partition driven off those vectors would try to remove it twice (or,
    // depending on how, leave it in). Asking the FIELD makes the question
    // idempotent by construction.
    let input: DeriveInput = parse_quote! {
        struct Weird {
            #[input]
            #[output]
            both: Vector3,
            keep: u32,
        }
    };
    let text = node_emitted(input);
    assert!(!text.contains("\"both\""), "{text}");
    assert!(text.contains("\"keep\""), "{text}");
}

#[test]
fn a_nodes_cer_read_reports_rather_than_constructing_a_partial_self() {
    // A node's walk excludes its ports, so `Self { .. }` would be missing
    // fields rustc requires (E0063) — the refusal is FORCED, not a policy
    // choice. Pinned here because a regression to `Ok(Self { .. })` would fail
    // to COMPILE in a user's crate rather than in this macro's own tests.
    let text = node_emitted(node_input());
    assert!(
        text.contains("Unrestorable"),
        "cer_read must report on a node:\n{text}"
    );
    assert!(
        !text.contains("Ok (Self {"),
        "a node must never construct a partial Self:\n{text}"
    );
}

/// The body of one emitted method, sliced out of the token text.
///
/// The guards below are asserted PER METHOD rather than over the whole
/// emission, because a ports-only node emits `let _ = src;` TWICE (once in
/// `cer_read`, once in `cer_restore`) and a bare `text.contains(..)` is
/// therefore satisfied by either one alone — which is exactly how the earlier
/// version of this test stayed green with a guard deleted.
fn method_body<'a>(text: &'a str, name: &str, next: Option<&str>) -> &'a str {
    let start = text
        .find(&format!("fn {name} ("))
        .unwrap_or_else(|| panic!("`fn {name}` is not in the emission:\n{text}"));
    let rest = &text[start..];
    match next {
        Some(next) => {
            let end = rest
                .find(&format!("fn {next} ("))
                .unwrap_or_else(|| panic!("`fn {next}` does not follow `fn {name}`:\n{text}"));
            &rest[..end]
        }
        None => rest,
    }
}

#[test]
fn a_node_with_no_state_at_all_still_emits_a_usable_impl() {
    // The `let _ = out;` / `let _ = src;` arms matter MORE here than on the
    // derive: `unused_variables = "deny"` is set by every scaffolded node
    // crate, so an unused parameter in emitted code is a hard error in the
    // USER's build, and a ports-only node is an ordinary shape.
    //
    // A ports-only node emits THREE such guards — capture's `out`, and `src`
    // in BOTH `cer_read` (a node always takes the `Unrestorable` arm) and
    // `cer_restore` — so all three are pinned, each inside its own method. The
    // per-method scoping is the whole point: over the flat text, deleting the
    // `cer_restore` guard leaves `cer_read`'s occurrence behind and a
    // `contains` check never notices.
    let input: DeriveInput = parse_quote! {
        struct Sink {
            #[input(trigger)]
            only: Vector3,
        }
    };
    let text = node_emitted(input);

    let capture = method_body(&text, "cer_capture", Some("cer_read"));
    assert!(
        capture.contains("let _ = out ;"),
        "cer_capture writes nothing, so `out` needs its guard:\n{capture}"
    );

    let read = method_body(&text, "cer_read", Some("cer_restore"));
    assert!(
        read.contains("let _ = src ;"),
        "cer_read reports Unrestorable without touching `src`, so it needs its guard:\n{read}"
    );

    let restore = method_body(&text, "cer_restore", None);
    assert!(
        restore.contains("let _ = src ;"),
        "cer_restore restores nothing, so `src` needs its guard:\n{restore}"
    );

    assert!(
        text.contains("MIN_ENCODED_BYTES : usize = 0usize"),
        "{text}"
    );
    assert!(text.contains("INLINE_SAFE : bool = true"), "{text}");
}

#[test]
fn a_nodes_escape_attribute_is_honoured_exactly_as_the_derives_is() {
    // The fold-in and the derive share ONE emission, so this is really a pin
    // on that sharing: if a node ever grew its own copy of the field walk,
    // this is where the two would start to disagree.
    let input: DeriveInput = parse_quote! {
        struct Bridge {
            #[output]
            out: Vector3,
            #[cerulion(reconstruct)]
            transport: Handle,
            seen: u64,
        }
    };
    let text = node_emitted(input);
    assert!(text.contains("cerulion::reconstruct"), "{text}");
    assert!(
        !text.contains("Handle : :: cerulion_core"),
        "an escaped field must carry no capture bound:\n{text}"
    );
    assert!(text.contains("\"seen\""), "{text}");
}

#[test]
fn a_node_and_a_derive_over_the_same_state_fields_agree_on_the_shape() {
    // The whole reason the fold-in calls into this module rather than growing
    // its own walk. Same struct name, same state fields, one carrying ports:
    // the SHAPE terms must be identical, or a node and a hand-derived helper
    // would disagree about what a field means — the two-copies class.
    let with_ports: DeriveInput = parse_quote! {
        struct Same {
            #[input(trigger)]
            scan: LaserScan,
            pose: Pose,
            frames: u64,
        }
    };
    let plain: DeriveInput = parse_quote! {
        struct Same {
            pose: Pose,
            frames: u64,
        }
    };
    let node = node_emitted(with_ports);
    let derived = emitted(plain);

    let shape_of = |text: &str| {
        let start = text.find("STATE_SHAPE").expect("shape present");
        let end = text[start..].find("INLINE_SAFE").expect("next const") + start;
        text[start..end].to_string()
    };
    assert_eq!(
        shape_of(&node),
        shape_of(&derived),
        "node and derive must fold the same state fields identically"
    );
}

// ---------------------------------------------------------------------------
// the REDUNDANT-DERIVE detectors
// ---------------------------------------------------------------------------
//
// `#[cerulion_node]` emits the state impl itself, so an explicit
// `#[derive(CerulionState)]` beside it is a conflicting implementation
// (`E0119`). rustc reports that, loudly — but it reports it as a duplicate,
// not as "the derive you wrote last week is now redundant", and it also lets
// the derive expand over the port fields and drag a second failure along.
//
// The two orders are caught by two DIFFERENT detectors (neither can see both;
// see `redundant_state_derive`'s docs), so both are pinned here, against hand
// oracles rather than against each other.

/// The struct-level attribute list of a parsed item.
fn attrs_of(input: DeriveInput) -> Vec<syn::Attribute> {
    input.attrs
}

#[test]
fn an_explicit_state_derive_is_found_under_every_spelling_that_resolves_to_it() {
    // Matched on the LAST PATH SEGMENT, the same rule `user_derives_default`
    // uses, so a qualified import is not a way to slip past the check.
    for spelling in [
        quote::quote! { #[derive(CerulionState)] struct S; },
        quote::quote! { #[derive(Default, CerulionState)] struct S; },
        quote::quote! { #[derive(CerulionState, Default)] struct S; },
        quote::quote! { #[derive(cerulion_core::state::CerulionState)] struct S; },
        quote::quote! { #[derive(::cerulion_macros::CerulionState)] struct S; },
        // Two derive attributes rather than one list — a shape rustc accepts
        // and a single-attribute scan would miss.
        quote::quote! { #[derive(Debug)] #[derive(CerulionState)] struct S; },
    ] {
        let input: DeriveInput = syn::parse2(spelling.clone()).expect("parses");
        assert!(
            redundant_state_derive(&attrs_of(input)).is_some(),
            "not detected: {spelling}"
        );
    }
}

#[test]
fn an_ordinary_node_struct_is_never_accused_of_deriving_the_state_impl() {
    // The ANTI-TAUTOLOGY arm. Without it a detector returning `Some` for
    // everything would satisfy every assertion above — and would refuse to
    // compile the 651 nodes that carry no derive at all.
    for spelling in [
        quote::quote! { struct S; },
        quote::quote! { #[derive(Default)] struct S; },
        quote::quote! { #[derive(Debug, Clone, Default)] struct S; },
        quote::quote! { #[serde(rename_all = "snake_case")] struct S; },
        // A field-level escape is not a derive.
        quote::quote! { struct S { #[cerulion(reconstruct)] h: Handle } },
        // The documented RESIDUAL, asserted in the direction it fails: a
        // renamed import is tokens that cannot be resolved, so it falls through to
        // rustc's own E0119 rather than to a silent accept of two impls.
        quote::quote! { #[derive(Capturable)] struct S; },
    ] {
        let input: DeriveInput = syn::parse2(spelling.clone()).expect("parses");
        assert!(
            redundant_state_derive(&attrs_of(input)).is_none(),
            "falsely detected: {spelling}"
        );
    }
}

#[test]
fn the_derive_side_detector_sees_the_node_attribute_the_other_order_leaves_behind() {
    // The mirror half. When the derive is written ABOVE `#[cerulion_node]`,
    // rustc expands the derive FIRST and strips the `derive` attribute before
    // the attribute macro runs — so the attribute macro cannot see the
    // collision, and only the derive can.
    for spelling in [
        quote::quote! { #[cerulion_node] struct S; },
        quote::quote! { #[cerulion_node(period_ms = 10)] struct S; },
        quote::quote! { #[cerulion_core::prelude::cerulion_node(external)] struct S; },
    ] {
        let input: DeriveInput = syn::parse2(spelling.clone()).expect("parses");
        assert!(
            node_attr_span(&attrs_of(input)).is_some(),
            "not detected: {spelling}"
        );
    }
    // Anti-tautology: an ordinary helper struct, which is what this derive is
    // FOR, must expand normally.
    for spelling in [
        quote::quote! { #[derive(Default)] struct Pose { x: f64 } },
        quote::quote! { #[cerulion_node_impl] struct S; },
    ] {
        let input: DeriveInput = syn::parse2(spelling.clone()).expect("parses");
        assert!(
            node_attr_span(&attrs_of(input)).is_none(),
            "falsely detected: {spelling}"
        );
    }
}

#[test]
fn the_derive_side_detector_falls_back_to_shape_when_the_attribute_is_aliased() {
    // `use ... cerulion_node as my_node;` is ordinary Rust and defeats the
    // name match above — a proc macro resolves no names. In THIS order the
    // struct still carries its PORT attributes (the attribute macro that
    // strips them has not run yet), and a `#[cerulion_node]` must declare at
    // least one, so the port is a name-free proof that this is a node.
    for spelling in [
        quote::quote! { #[my_node(period_ms = 10)] struct S { #[output] out: V, n: u64 } },
        quote::quote! { #[node(external)] struct S { #[input(trigger)] scan: V } },
        // No struct-level attribute AT ALL: the shape still decides.
        quote::quote! { struct S { #[input] a: V, #[output] b: V } },
    ] {
        let input: DeriveInput = syn::parse2(spelling.clone()).expect("parses");
        assert!(
            node_attr_span(&input.attrs).is_none(),
            "precondition: the NAME match must miss these, or the fallback is \
             not what is being tested: {spelling}"
        );
        assert!(
            node_evidence_span(&input).is_some(),
            "not detected by shape: {spelling}"
        );
    }

    // Anti-tautology: the helper structs this derive EXISTS for declare no
    // ports, so nothing about them looks like a node.
    for spelling in [
        quote::quote! { #[derive(Default)] struct Pose { x: f64, y: f64 } },
        quote::quote! { struct Stats { #[cerulion(reconstruct)] h: H, n: u64 } },
        quote::quote! { enum Mode { Idle, Run(u64) } },
        quote::quote! { struct Unit; },
    ] {
        let input: DeriveInput = syn::parse2(spelling.clone()).expect("parses");
        assert!(
            node_evidence_span(&input).is_none(),
            "falsely detected: {spelling}"
        );
    }

    // The NAME match still wins where it applies, because it points at the
    // attribute rather than at the struct name — a strictly better span, and
    // the reason `state_derive_before_node_attr.stderr` is unchanged.
    let canonical: DeriveInput =
        parse_quote! { #[cerulion_node(period_ms = 10)] struct S { #[output] out: V } };
    assert_eq!(
        format!("{:?}", node_evidence_span(&canonical).expect("detected")),
        format!(
            "{:?}",
            node_attr_span(&canonical.attrs).expect("name match")
        ),
        "the name match must keep providing the span when it is available"
    );
}

#[test]
fn the_derive_on_a_node_struct_emits_the_refusal_and_no_impl_at_all() {
    // Emitting an impl ALONGSIDE the error would put the conflict back: the
    // attribute macro still runs afterwards and still emits the fold-in, so
    // the derive must contribute nothing but the message.
    let input: DeriveInput = parse_quote! {
        #[cerulion_node(period_ms = 10)]
        struct Counter {
            count: u64,
        }
    };
    let text = derive(&input).to_string();
    assert!(
        text.contains("compile_error"),
        "the derive must refuse:\n{text}"
    );
    // Matched on the marker the emission stamps on every impl, never on the
    // bare word `impl` — the refusal MESSAGE says "emits this impl itself", so
    // a substring check on `impl` would be satisfied by the message and pass no
    // matter what the derive emitted.
    assert!(
        !text.contains("automatically_derived"),
        "the derive must emit NO impl beside its refusal:\n{text}"
    );
    assert!(
        !text.contains("STATE_SHAPE"),
        "the derive must emit NO impl beside its refusal:\n{text}"
    );
}

#[test]
fn both_detectors_tell_the_user_the_same_story() {
    // One message constant, two call sites. Two texts for one mistake would
    // mean a user who moves the derive one line up gets a different
    // explanation of the same thing.
    let node_first: DeriveInput = parse_quote! {
        #[derive(CerulionState)]
        struct S { a: u64 }
    };
    let derive_first: DeriveInput = parse_quote! {
        #[cerulion_node]
        struct S { a: u64 }
    };
    let from_attr = syn::Error::new(
        redundant_state_derive(&node_first.attrs).expect("detected"),
        REDUNDANT_DERIVE_MSG,
    );
    let from_derive = syn::Error::new(
        node_attr_span(&derive_first.attrs).expect("detected"),
        REDUNDANT_DERIVE_MSG,
    );
    assert_eq!(from_attr.to_string(), from_derive.to_string());
    // And it names the remedy, not merely the symptom.
    assert!(
        REDUNDANT_DERIVE_MSG.contains("remove `#[derive(CerulionState)]`"),
        "{REDUNDANT_DERIVE_MSG}"
    );
}
