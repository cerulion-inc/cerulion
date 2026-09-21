//! The curated resource inventory.
//!
//! A field holding a resource (an fd, a socket, a thread handle, a framework
//! handle) is not state: capturing it would record a number that means nothing
//! in another process, and restoring it would point the node at the wrong
//! device. Such a field wants `#[cerulion(reconstruct)]`, and by design
//! the user should not have to write that for a type we have already seen.
//!
//! So this module answers ONE question: *is this field's declared type a
//! resource we recognise?* A `true` classifies the field exactly as an
//! explicit `#[cerulion(reconstruct)]` would — nothing captured, nothing
//! restored, the escape kind folded into `STATE_SHAPE` so the decision is
//! visible in the recording rather than silent.
//!
//! # Why this is a TOKEN list and not a trait
//!
//! Three type-system routes were tried and each is closed:
//!
//! 1. **`impl CerulionState for File` with no-op semantics.** `cer_read` must
//!    return `Self`, and a `File` cannot be minted from bytes. It would also
//!    contradict the shipped `on_unimplemented` diagnostic, which tells the
//!    user these types deliberately carry no impl.
//! 2. **A marker trait plus a blanket bridge** (`impl<R: CerulionResource>
//!    CerulionState for R`). Overlaps every concrete impl in the inventory;
//!    rustc rejects it without negative reasoning.
//! 3. **Autoref specialization** to ask "does this type implement it?".
//!    Forbidden by design: a prototype measured it unsound (its
//!    verdict moves with cargo feature unification and is silently wrong
//!    inside a generic).
//!
//! # The residual, stated in the direction it fails
//!
//! A proc macro sees TOKENS. So:
//!
//! - **A newtype or alias over a resource is NOT recognised** (`type Handle =
//!   File`). That direction is FAIL-CLOSED and therefore safe: the field falls
//!   through to the ordinary bound, `File: CerulionState` does not hold, and
//!   the user gets the required `E0277` naming the field and the one-line fix.
//!   This is exactly the project's rule: "a never-seen type fails compile, loudly".
//! - **A user type whose LAST PATH SEGMENT collides with an inventory name is
//!   recognised** (`struct File { .. }` of your own, held as state). That
//!   direction FAILS OPEN — the field is silently reconstructed rather than
//!   captured. It is the accepted cost of a token list, bounded three ways: the list is
//!   deliberately short and handle-shaped, every entry is a name a data struct
//!   is unlikely to want, and the decision is RECORDED (the field folds
//!   `cerulion::reconstruct` into `STATE_SHAPE`, so a bag captured before and
//!   after a rename does not silently compare equal). The remedy for a
//!   collision is to rename the type or wrap it — there is deliberately no
//!   fourth attribute to un-say it, because a per-field opt-out is exactly the
//!   tag the inventory exists to remove.
//!
//! # `dyn` is STRUCTURAL, not curated
//!
//! A trait object, a raw pointer and a function pointer can never implement
//! `CerulionState` in a way that means anything, and recognising them needs no
//! list at all — they are syntactic categories. That is what makes a field
//! like `Box<dyn FnMut(..)>` zero-tag without anybody adding a name.

use syn::Type;

/// Names recognised as resources — std first, then Cerulion's own handles.
///
/// Matched against the type's LAST PATH SEGMENT, so both `File` and
/// `std::fs::File` hit. Keep this list handle-shaped: every addition widens
/// the fail-open direction documented above.
const RESOURCE_NAMES: &[&str] = &[
    // ---- std: files and OS handles -------------------------------------
    "File",
    "OpenOptions",
    "DirEntry",
    "ReadDir",
    "OwnedFd",
    "BorrowedFd",
    "RawFd",
    "OwnedHandle",
    "BorrowedHandle",
    "RawHandle",
    "Stdin",
    "Stdout",
    "Stderr",
    "StdinLock",
    "StdoutLock",
    "StderrLock",
    // ---- std: sockets ---------------------------------------------------
    "TcpStream",
    "TcpListener",
    "UdpSocket",
    "UnixStream",
    "UnixListener",
    "UnixDatagram",
    // ---- std: processes --------------------------------------------------
    "Child",
    "ChildStdin",
    "ChildStdout",
    "ChildStderr",
    "Command",
    // ---- std: threads and channels ---------------------------------------
    "JoinHandle",
    "Thread",
    "Sender",
    "SyncSender",
    "Receiver",
    "Condvar",
    "Barrier",
    // ---- Cerulion's own handles ------------------------------------------
    "TransportManager",
    "NodeContext",
    "CerNodeRuntimeFields",
    "AnyPublisher",
    "AnySubscriber",
    "CerulionPublisher",
    "CerulionSubscriber",
    "DataOnlySubscriber",
    "OutputProxy",
    "InputView",
    "IngressInjector",
    "WakeSource",
    "WakeSet",
    "NetdClient",
];

/// Single-type-argument wrappers this walk looks THROUGH.
///
/// Deliberately conservative: only containers with exactly one meaningful type
/// argument. A two-argument container (`HashMap<String, File>`) is NOT peeled,
/// so it falls through to the ordinary bound and fails loudly — the
/// fail-closed direction.
const TRANSPARENT_WRAPPERS: &[&str] = &[
    "Option", "Box", "Rc", "Arc", "Mutex", "RwLock", "RefCell", "Cell", "Vec", "VecDeque",
];

/// Is this field's declared type a resource to rebuild rather than restore?
pub fn is_resource(ty: &Type) -> bool {
    match ty {
        // STRUCTURAL: no list needed, and no false positive is possible.
        Type::TraitObject(_) | Type::Ptr(_) | Type::BareFn(_) => true,
        Type::ImplTrait(_) => true,
        Type::Reference(r) => is_resource(&r.elem),
        Type::Paren(p) => is_resource(&p.elem),
        Type::Group(g) => is_resource(&g.elem),
        Type::Array(a) => is_resource(&a.elem),
        Type::Slice(s) => is_resource(&s.elem),
        Type::Path(path) => {
            let Some(segment) = path.path.segments.last() else {
                return false;
            };
            let name = segment.ident.to_string();
            if RESOURCE_NAMES.contains(&name.as_str()) {
                return true;
            }
            if !TRANSPARENT_WRAPPERS.contains(&name.as_str()) {
                return false;
            }
            let syn::PathArguments::AngleBracketed(generics) = &segment.arguments else {
                return false;
            };
            let mut types = generics.args.iter().filter_map(|a| match a {
                syn::GenericArgument::Type(t) => Some(t),
                _ => None,
            });
            // Peel ONLY when there is exactly one type argument, so a
            // two-argument container never silently inherits a verdict from
            // one of its halves.
            match (types.next(), types.next()) {
                (Some(inner), None) => is_resource(inner),
                _ => false,
            }
        }
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use syn::parse_quote;

    fn resource(ty: Type) -> bool {
        is_resource(&ty)
    }

    #[test]
    fn a_bare_std_handle_is_recognised_under_either_spelling() {
        assert!(resource(parse_quote!(File)));
        assert!(resource(parse_quote!(std::fs::File)));
        assert!(resource(parse_quote!(::std::fs::File)));
        assert!(resource(parse_quote!(TcpStream)));
        assert!(resource(parse_quote!(JoinHandle<()>)));
        assert!(resource(parse_quote!(std::sync::mpsc::Receiver<u32>)));
    }

    #[test]
    fn a_handle_is_recognised_through_the_wrappers_it_actually_ships_in() {
        // The shape the Go2 bridge holds today.
        assert!(resource(parse_quote!(Option<Arc<TransportManager>>)));
        assert!(resource(parse_quote!(Arc<Mutex<File>>)));
        assert!(resource(parse_quote!(Vec<JoinHandle<()>>)));
        assert!(resource(parse_quote!(Box<TcpStream>)));
    }

    #[test]
    fn ordinary_state_is_never_mistaken_for_a_resource() {
        // The ANTI-TAUTOLOGY arm: without it, a predicate returning `true`
        // unconditionally would pass every assertion above.
        assert!(!resource(parse_quote!(u64)));
        assert!(!resource(parse_quote!(String)));
        assert!(!resource(parse_quote!(Vec<u8>)));
        assert!(!resource(parse_quote!(Option<Arc<Mutex<Costmap>>>)));
        assert!(!resource(parse_quote!(HashMap<Cell2D, Occupancy>)));
        assert!(!resource(parse_quote!(PadState)));
        // A lock around ordinary state is state, not a resource — the whole
        // `Arc<Mutex<T>>` decision depends on this staying false.
        assert!(!resource(parse_quote!(Arc<Mutex<Vec<PadEvent>>>)));
    }

    #[test]
    fn a_trait_object_is_structural_so_it_needs_no_list_entry() {
        assert!(resource(parse_quote!(Box<dyn FnMut() -> bool + Send>)));
        assert!(resource(parse_quote!(Option<Box<dyn JpegTranscoder>>)));
        assert!(resource(parse_quote!(Arc<dyn Clock>)));
        assert!(resource(parse_quote!(*const u8)));
        assert!(resource(parse_quote!(*mut Foo)));
        assert!(resource(parse_quote!(fn(u32) -> u32)));
    }

    #[test]
    fn a_two_argument_container_is_never_peeled_so_it_fails_closed() {
        // `File` is in the list, but a map is not peeled — the field falls
        // through to the ordinary bound and the user gets the required E0277
        // rather than a silent reconstruct of a whole map.
        assert!(!resource(parse_quote!(HashMap<String, File>)));
        assert!(!resource(parse_quote!(BTreeMap<u32, TcpStream>)));
    }

    #[test]
    fn an_alias_over_a_resource_fails_closed_which_is_the_safe_direction() {
        // A proc macro cannot resolve `type Handle = File;`. The verdict is
        // "not a resource", so the field takes the ordinary bound and fails to
        // compile with the fix named — never a silent reconstruct.
        assert!(!resource(parse_quote!(Handle)));
        assert!(!resource(parse_quote!(MyOwnFdWrapper)));
    }

    #[test]
    fn every_declared_name_is_matched_by_the_predicate_that_reads_the_list() {
        // Guards against an entry that is present but unreachable — e.g. one
        // added with a `::`-qualified spelling, which the last-segment match
        // would never see.
        for name in RESOURCE_NAMES {
            let ident = syn::Ident::new(name, proc_macro2::Span::call_site());
            let ty: Type = parse_quote!(#ident);
            assert!(
                is_resource(&ty),
                "`{name}` is declared in RESOURCE_NAMES but the predicate does not match it"
            );
        }
    }

    #[test]
    fn every_declared_wrapper_really_peels() {
        // Same guard for the other list: a wrapper that does not peel would
        // silently stop recognising handles held inside it.
        for name in TRANSPARENT_WRAPPERS {
            let ident = syn::Ident::new(name, proc_macro2::Span::call_site());
            let ty: Type = parse_quote!(#ident<File>);
            assert!(
                is_resource(&ty),
                "`{name}` is declared transparent but a `File` inside it is not recognised"
            );
        }
    }
}
