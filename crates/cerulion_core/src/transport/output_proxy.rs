// SPDX-License-Identifier: AGPL-3.0-only
//! Zero-copy SHM-backed output proxy.
//!
//! `OutputProxy<'loan, T: ShmMessage>` is the handle a message is published
//! through. A node author never constructs one. Inside a node's `tick`, an
//! `#[output] cmd: Twist` field IS this proxy: `#[cerulion_node_impl]`
//! rewrites `self.cmd` to a proxy that is loaned on the port's first write,
//! so `self.cmd.linear.x = 0.3;` is the whole user-side API.
//!
//! What the proxy does:
//!
//! 1. The runtime loans it from the port's publisher
//!    (`CerulionPublisher::loan_proxy::<Twist>()`, framework-internal).
//! 2. The proxy `Deref`s to `T::Writer<'loan>` (the SHM-backed `TwistShm`)
//!    so a field assignment writes directly into shared memory.
//! 3. On drop the proxy validates that all declared variable fields were
//!    written, finalizes `WireHeader::total_size` AND consumes +
//!    stamps the wire `sequence` (commit-time, so discarded loans burn no
//!    sequence number), calls iceoryx2 `send()` on the sample, and notifies
//!    subscribers with `SentSample`.
//!
//! A tick that returns `Err` publishes nothing: the loan is released
//! without a send.
//!
//! On a missing variable field the proxy logs `tracing::error!` (once per
//! regime; repeats are demoted to `debug!`) and skips the publish. Drop must
//! not panic. Every discard is counted, whatever the log level:
//! `CerulionPublisher::output_discard_count()` on the port, and
//! `NodeHandle::output_discard_count(output)` from outside the node. When this runs inside
//! a macro-generated cdylib node (`#[cerulion_node]`), that `error!` lands on
//! stderr via the cdylib-local subscriber the generated `cerulion_node_init`
//! installs
//! ([`install_cdylib_stderr_tracing`](crate::graph::node::install_cdylib_stderr_tracing)) —
//! it is not a silent black hole. A hand-written raw-FFI cdylib gets that
//! subscriber only if it calls the installer itself.
//!

use std::marker::PhantomData;

use crate::message::ShmMessage;
use crate::wire::WireHeader;

use super::events::PubSubEvent;
use super::output_discard_latch::DiscardLogLevel;
use super::shm_sample::{ProxyPublisher, SampleHandle};

/// Zero-copy SHM-backed publish handle.
///
/// `'loan` is the borrow of the publisher; the proxy holds it for its
/// entire lifetime so a single port cannot have two concurrent loans
/// (multi-publish-per-port-per-tick is not supported).
///
/// `T: ShmMessage` is the schema being published. `T::Writer<'loan>` —
/// the SHM-backed `<Name><'loan>` — is what the proxy `Deref`s to.
///
/// # Send / Sync
///
/// `OutputProxy` is intentionally `!Send` and `!Sync`, enforced by the
/// `_not_send: PhantomData<*mut ()>` marker field below. This keeps the
/// "tick is single-threaded" invariant a type-system fact rather than a
/// doc-comment hope — user code cannot smuggle a held loan across threads.
///
/// NOTE: the marker is the SOLE enforcer. The held iceoryx2 sample
/// is itself `Send`
/// (`ipc_threadsafe::Service`; only the Rc-backed `ipc::Service`
/// sample is `!Send`), so the `PhantomData` marker is load-bearing — do
/// NOT remove it as "redundant with the sample's `!Send`-ness".
//
// `sample` and `publisher` are held to keep the borrow + the SHM slot
// alive for the proxy's lifetime; both are consumed at drop time to
// finalize the publish. `writer` is the payload accessor that user code
// reaches via Deref.
pub struct OutputProxy<'loan, T: ShmMessage + 'loan> {
    /// Held outbound iceoryx2 SHM sample. On drop the proxy `take()`s the
    /// sample out and calls `send()` on it.
    sample: SampleHandle<'loan>,

    /// SHM-backed writer — the codegen-emitted `<Name>Shm` (`<'loan>` for
    /// variable schemas). Constructed from a `&'loan mut [u8]` slice into
    /// the loaned payload (after the 32-byte WireHeader prefix).
    writer: T::Writer<'loan>,

    /// Borrow of the iceoryx2 publisher backend. Held for `'loan` so a
    /// single port cannot vend two concurrent proxies.
    publisher: ProxyPublisher<'loan>,

    /// Idempotence flag. Set to true when the sample has been sent so
    /// drop becomes a no-op. Manual `publish()` (if added later) and
    /// drop both consult this.
    sent: bool,

    /// Publish-deferral flag — when true, `Drop` releases the loan
    /// WITHOUT sending (checked FIRST, before the staged flush and the
    /// all-variables gate).
    ///
    /// Two regimes share it (the publish-on-success inversion):
    ///
    /// * **Macro path** (`#[cerulion_node_impl]`): the generated tick
    ///   preamble calls [`__cer_defer_publish`](Self::__cer_defer_publish)
    ///   IMMEDIATELY after each loan, and ONLY the generated tail's
    ///   [`__cer_arm_publish`](Self::__cer_arm_publish) — on a fully-Ok
    ///   tick outcome — clears it. Publishing therefore requires
    ///   affirmative tick completion: EVERY early exit (a sibling output's
    ///   loan failure in the preamble, a transport error from the try_view
    ///   chain, a user-body `Err`, and any future leg) discards by
    ///   construction, fixed-only schemas included (which the
    ///   variable-field gate never covered).
    /// * **Direct path** (`loan_proxy` called by closure-node bodies,
    ///   raw-FFI cdylib nodes, tools, and tests): stays `false` from
    ///   construction — drop-publish remains those callers' documented
    ///   contract (they have no generated tail; their success point is
    ///   inside user code the runtime cannot reach, so a
    ///   constructor-level default flip would break them un-wireably).
    ///
    /// Subordinate to `sent`: an already-sent frame stays sent.
    discarded: bool,

    /// `OutputProxy` must be `!Send` and `!Sync`.
    /// Lifetime-tied raw pointer is the standard idiom for that.
    _not_send: PhantomData<*mut ()>,
}

impl<'loan, T: ShmMessage + 'loan> OutputProxy<'loan, T> {
    /// Construct an `OutputProxy` from an outbound sample, a writer over
    /// its payload, and the publisher borrow.
    ///
    /// Crate-private: only `CerulionPublisher::loan_proxy` may call this.
    /// User code reaches the proxy only through `loan_proxy`.
    ///
    /// The `sample` must hold an outbound iceoryx2 sample
    /// that has had the WireHeader written to bytes `[0..32]` (with
    /// `total_size` AND `sequence` provisionally set — drop rewrites
    /// `total_size` from `T::payload_wire_size` and stamps the
    /// committed sequence consumed via `commit_sequence`). The offset
    /// table (variable schemas) must already be zero-initialized.
    /// `writer` must be a `T::Writer<'loan>` constructed over the
    /// payload bytes `[32..]` of that same sample (via
    /// `T::build_writer`).
    pub(crate) fn new(
        sample: SampleHandle<'loan>,
        writer: T::Writer<'loan>,
        publisher: ProxyPublisher<'loan>,
    ) -> Self {
        Self {
            sample,
            writer,
            publisher,
            sent: false,
            discarded: false,
            _not_send: PhantomData,
        }
    }

    /// Whether this proxy has already published.
    ///
    /// True after the proxy has been dropped successfully (or, in a future
    /// extension, after an explicit `publish()` call). False during normal
    /// use of the proxy.
    pub fn sent(&self) -> bool {
        self.sent
    }

    /// DEFER publishing — until re-armed, `Drop` releases the
    /// loaned SHM slot without sending (iceoryx2 reclaims an unsent sample
    /// on drop; no `mem::forget` anywhere on this path).
    ///
    /// Doc-hidden macro plumbing (the `__cer` namespace keeps it from
    /// shadowing any schema accessor reachable through Deref): the
    /// `#[cerulion_node_impl]` tick PREAMBLE calls this immediately after
    /// each output loan, making discard the default and publishing an
    /// affirmative act — see [`__cer_arm_publish`](Self::__cer_arm_publish).
    /// Partial writes, complete writes, and staged nested writes alike are
    /// dropped on a non-completed tick (staging without flush and without
    /// the child-gate error; the tick error itself is the loud signal).
    #[doc(hidden)]
    #[inline]
    pub fn __cer_defer_publish(&mut self) {
        self.discarded = true;
    }

    /// ARM publishing — the inverse of
    /// [`__cer_defer_publish`](Self::__cer_defer_publish). Called ONLY by
    /// the macro-generated tick tail on a fully-Ok tick outcome; `Drop`
    /// then publishes through the normal gates (staged flush +
    /// all-variables). An explicit pre-error send (`sent == true`) wins
    /// over everything — `Drop` checks `sent` first, unchanged.
    #[doc(hidden)]
    #[inline]
    pub fn __cer_arm_publish(&mut self) {
        self.discarded = false;
    }
}

impl<'loan, T: ShmMessage + 'loan> std::ops::Deref for OutputProxy<'loan, T> {
    type Target = T::Writer<'loan>;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.writer
    }
}

impl<'loan, T: ShmMessage + 'loan> std::ops::DerefMut for OutputProxy<'loan, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.writer
    }
}

impl<'loan, T: ShmMessage + 'loan> Drop for OutputProxy<'loan, T> {
    fn drop(&mut self) {
        // Idempotent: if already sent (e.g. by a future explicit publish()),
        // skip. Mark sent up-front so a panic inside this body cannot
        // double-publish (Drop is also called during unwinding).
        if self.sent {
            return;
        }
        self.sent = true;

        // REPLAY: if replay is suppressing this fire (the scheduler set
        // the node's shared flag from the recorded discard marker), release the
        // loan WITHOUT publishing — checked FIRST, above `commit_sequence`, so no
        // sequence is burned and no frame is delivered: the byte-identical mirror
        // of the live discard this fire took at record time (the +seq-shift fix).
        // The `sent` guard above still wins (an explicitly-published frame stays
        // published). No-op on the live/record path (the flag is never set).
        if self.publisher.replay_suppress_active() {
            tracing::debug!(
                topic = self.publisher.topic(),
                schema_hash = T::SCHEMA_HASH,
                "OutputProxy: replay suppressing a live-discarded fire's publish (no seq burned)"
            );
            return;
        }

        // A NON-COMPLETED tick publishes NOTHING (publish-on-
        // success inversion: the macro preamble defers every output at
        // loan time; only the tick tail's arm-on-Ok clears it — so a
        // sibling-loan failure, a transport error from the input chain, a
        // user-body Err, and any future early exit all land here by
        // construction). Checked FIRST — before the staged flush (step 0)
        // and the all-variables gate — so the discard also preempts
        // staged-nested flushing (staging drops without flush and without
        // NestedChildIncomplete noise) and covers fixed-only schemas
        // (which have no variable-field gate to trip). DEBUG-level
        // breadcrumb only: the tick error itself is the loud signal, and
        // double-alarming every errored tick at error level would drown
        // it. The `sent` guard above still wins — an explicitly-published
        // frame stays published (idempotence unchanged). Dropping
        // `self.sample` unsent releases the iceoryx2 loan back to the
        // pool (same release path as every discard below).
        if self.discarded {
            // RECORD: this output committed 0 bytes, so signal the node's
            // discard counter so `fire_node_into`'s delta marks this fire's trace
            // record (the discard bit). Burns no sequence (returns above
            // `commit_sequence`), so replay can byte-exactly reproduce it.
            self.publisher.bump_discard_signal();
            tracing::debug!(
                topic = self.publisher.topic(),
                schema_hash = T::SCHEMA_HASH,
                "OutputProxy: tick did not complete — output discarded, loan released without publish"
            );
            return;
        }

        // 0. Flush staged complex-nested writes (the
        //    `self.<port>.<nested>.<leaf> = …` / `with_<nested>` sugar)
        //    into the real payload BEFORE the all-variables gate — each
        //    touched staged field lands via the schema's own
        //    `set_<f>_bytes`, which marks it written. Fixed schemas and
        //    unstaged fields are untouched (trait default is a no-op, so
        //    the optimizer folds this out for fixed instantiations). A
        //    flush failure (capacity) is the same loud-discard class as a
        //    missing field: log + skip publish, never a silent partial
        //    frame (and never a panic — Drop must not panic).
        if let Err(e) = T::flush_staged_nested(&mut self.writer) {
            // Flood-latch this per-port discard. The FIRST failure of
            // a regime is loud (`error!`, unchanged message + fields); sustained
            // failures downgrade to `debug!` with a running suppressed count so a
            // persistently broken node does not emit ~875 errors/s. Recovery
            // (a subsequent complete publish) re-arms the loud path.
            match self.publisher.record_output_discard() {
                DiscardLogLevel::Error => {
                    tracing::error!(
                        topic = self.publisher.topic(),
                        schema_hash = T::SCHEMA_HASH,
                        error = %e,
                        "OutputProxy: staged nested-field flush failed; skipping publish"
                    );
                }
                DiscardLogLevel::Debug { suppressed } => {
                    tracing::debug!(
                        topic = self.publisher.topic(),
                        schema_hash = T::SCHEMA_HASH,
                        error = %e,
                        suppressed_count = suppressed,
                        "OutputProxy discard suppressed: staged nested-field flush recurred as \
                         a failure; skipping publish — first occurrence logged loudly at error"
                    );
                }
            }
            return;
        }

        // 1. Validate that every declared variable field was written.
        //    For fixed schemas this is vacuously true. For variable
        //    schemas the bitset on `WriterState` records each set/loan/push
        //    call; missing fields would emit a partially-valid frame.
        if T::VARIABLE_FIELD_COUNT > 0 && !T::all_variables_written(&self.writer) {
            // Flood-latch this per-port discard (the dominant flood
            // source — a broken node that never writes a declared variable field
            // trips this every publish). FIRST discard of a regime stays loud
            // (`error!`, byte-identical message + fields: the loud-first-signal contract);
            // sustained discards downgrade to `debug!` with a running suppressed
            // count; a subsequent complete publish reports recovery once (at the
            // send-success site below) and re-arms the loud path.
            match self.publisher.record_output_discard() {
                DiscardLogLevel::Error => {
                    tracing::error!(
                        topic = self.publisher.topic(),
                        schema_hash = T::SCHEMA_HASH,
                        "OutputProxy dropped without writing all declared variable fields; skipping publish"
                    );
                }
                DiscardLogLevel::Debug { suppressed } => {
                    tracing::debug!(
                        topic = self.publisher.topic(),
                        schema_hash = T::SCHEMA_HASH,
                        suppressed_count = suppressed,
                        "OutputProxy discard suppressed: incomplete output (missing declared \
                         variable field) recurred; skipping publish — first occurrence logged \
                         loudly at error"
                    );
                }
            }
            // Drop the sample without sending. The held SampleHandle / Vec
            // is released when `self` is dropped after this function returns.
            return;
        }

        // 1a. Detect overflow spill. Variable schemas that grew
        //     their payload past the adaptive loan size during this tick
        //     have a heap buffer attached to the writer. The original
        //     loaned SHM sample's post-header bytes are stale (writes
        //     past the spill point went to the heap, not back to SHM);
        //     instead we re-loan a fresh sample sized exactly to fit and
        //     memcpy header + heap-payload in.
        //
        //     `T::has_overflow` returns `false` for fixed schemas (cannot
        //     overflow), so the optimizer can constant-fold this branch
        //     out for fixed-schema instantiations.
        if T::has_overflow(&self.writer) {
            let payload_size = T::payload_wire_size(&self.writer);
            let total_size = WireHeader::SIZE + payload_size;
            // Pin the total_size cast invariant. It is bounded by
            // max_capacity:u32 + WireHeader::SIZE, but the assert pins it
            // against future changes.
            debug_assert!(
                total_size <= u32::MAX as usize,
                "OutputProxy::Drop overflow: total_size {} exceeds u32::MAX",
                total_size
            );
            // Copy the WireHeader from the original sample's [0..32]
            // (stamped at loan time with provisional total_size) and
            // patch total_size to the final wire size before forwarding.
            let original_bytes = self.sample.bytes();
            // This guard is unreachable in production: `loan_proxy`
            // stamps a 32-byte header before constructing the proxy; the
            // check defends against a future invariant break + buggy
            // hand-written `impl ShmMessage`. There is
            // deliberately no
            // `debug_assert!(false, ...)` here: Drop must not panic, and
            // a debug-assert would panic in debug
            // builds + violate unwind-safety. The `tracing::error!` is
            // the correct fail-loud mechanism for a Drop-time invariant
            // break.
            if original_bytes.len() < WireHeader::SIZE {
                // Invariant violation: `loan_proxy` stamps a 32-byte
                // header before proxy construction; reaching this
                // branch means a future refactor broke that
                // invariant. Bump `frames_dropped_invariant_violation`
                // (NOT `frames_dropped_overflow` — different semantic
                // class; operator action is "file a bug", not
                // "scale SHM pool").
                match &mut self.publisher {
                    ProxyPublisher::Iceoryx2(pubr) => pubr.record_invariant_violation_drop(),
                }
                tracing::error!(
                    topic = self.publisher.topic(),
                    buf_len = original_bytes.len(),
                    "OutputProxy: original sample too small to host WireHeader on overflow drop \
                     (frames_dropped_invariant_violation counter incremented)"
                );
                return;
            }
            let mut header_bytes = [0u8; WireHeader::SIZE];
            header_bytes.copy_from_slice(&original_bytes[..WireHeader::SIZE]);
            // Patch total_size at [8..12]. Other header fields (schema_hash,
            // offset_table_offset/count, timestamp_ns) carry over unchanged
            // so subscribers see consistent metadata. `sequence` at [20..24]
            // is PROVISIONAL in the loan-time copy — it is
            // committed just before `send_overflow_frame` below, after the
            // spill-view invariant guard, so an invariant-violation return
            // burns no sequence number.
            header_bytes[8..12].copy_from_slice(&(total_size as u32).to_le_bytes());

            // Read the spill bytes via the trait accessor.
            let Some(spill_view) = T::overflow_view_bytes(&self.writer) else {
                // `has_overflow == true` && `overflow_view_bytes == None`
                // is a contract violation by a hand-written `impl ShmMessage`
                // (the two trait methods must agree). Drop must not panic,
                // so the only fail-loud mechanism is
                // `tracing::error!`. A `debug_assert!(false, ...)`
                // here would be wrong: it would panic in
                // debug builds — Drop is reached during unwinding so even
                // debug-only panic-in-Drop is unwind-unsafe.
                //
                // Bump `frames_dropped_invariant_violation` so operators
                // see the drop in metrics. Operator action: file bug
                // against the misbehaving `impl ShmMessage`.
                match &mut self.publisher {
                    ProxyPublisher::Iceoryx2(pubr) => pubr.record_invariant_violation_drop(),
                }
                tracing::error!(
                    topic = self.publisher.topic(),
                    "OutputProxy: has_overflow=true but overflow_view_bytes=None (contract violation; \
                     frames_dropped_invariant_violation counter incremented)"
                );
                return;
            };

            // Backend-specific re-loan + send.
            //
            // Convergence-on-failure: on the Err
            // arm, record `total_size` into the adaptive sizer BEFORE
            // bumping the dropped-frame counter. Rationale: the growth
            // happened — the writer demanded a `total_size`-byte payload
            // this tick. Whether the re-loan + send succeeded is
            // orthogonal to whether the sliding window should learn the
            // demand size. Skipping record on Err means the next tick
            // attempts the same payload, hits the same too-small loan,
            // and spills again — perpetual overflow on a single oversized
            // publisher whose first overflow happened to coincide with
            // SHM exhaustion. Recording on Err lets the sizer converge
            // even when delivery fails: the next loan will be sized to
            // accommodate that payload, so subsequent ticks fit in the
            // steady-state path. The Ok path records inside
            // `send_overflow_frame` itself (publisher.rs); this is the
            // symmetric Err-arm record.
            match &mut self.publisher {
                ProxyPublisher::Iceoryx2(pubr) => {
                    // Consume + stamp the wire sequence at COMMIT.
                    // Mutually exclusive with the steady-state stamp below
                    // (this arm `return`s), so exactly one consume per
                    // committed frame. A send_overflow_frame Err after this
                    // burns the number for real — the frame was genuinely
                    // attempted and dropped (counter + error! below).
                    let seq = pubr.commit_sequence();
                    header_bytes[20..24].copy_from_slice(&seq.to_le_bytes());
                    if let Err(e) = pubr.send_overflow_frame(&header_bytes, spill_view) {
                        pubr.record_payload_size(total_size as u32);
                        // Increment the dropped-frame counter for
                        // metrics visibility. The tracing::error! is
                        // grep-only; the counter is the metrics surface
                        // operators monitor.
                        pubr.record_dropped_overflow_frame();
                        tracing::error!(
                            topic = %pubr.topic(),
                            error = ?e,
                            total_size,
                            "OutputProxy: overflow re-loan or send failed; frame DROPPED \
                             (frames_dropped_overflow counter incremented; adaptive sizer recorded the size)"
                        );
                    } else {
                        // A complete (overflowed) frame went out —
                        // clear the discard flood-latch (no-op unless a discard
                        // regime was open on this port).
                        if let Some(suppressed) = pubr.record_output_complete() {
                            tracing::info!(
                                topic = %pubr.topic(),
                                schema_hash = T::SCHEMA_HASH,
                                suppressed_count = suppressed,
                                "OutputProxy: output recovered — complete publish after a \
                                 discard regime (further discards on this port will log \
                                 loudly again)"
                            );
                        }
                    }
                }
            }
            // Skip the regular steady-state path below. The original SHM
            // sample is released when `self` drops after this function
            // returns.
            return;
        }

        // 2. Finalize WireHeader::total_size + `sequence` in the
        //    loaned bytes. The payload size comes from the writer (cursor
        //    for variable, WIRE_FIXED_SIZE for fixed); add the 32-byte
        //    header to get the wire frame total size.
        let payload_size = T::payload_wire_size(&self.writer);
        let total_size = WireHeader::SIZE + payload_size;
        // total_size lives at bytes [8..12] in the WireHeader (after
        // schema_hash: u64 at [0..8]); `sequence` at [20..24]. Rewrite both
        // in place — every other field was set at loan time and is still
        // correct (`timestamp_ns` deliberately stays loan-time: publisher-
        // side time, see `loan_proxy` step 4). The sequence
        // counter is CONSUMED here — at commit, not loan — so every discard
        // path above returned without burning a number and published
        // streams are gap-free (no phantom losses in bagd / drop_oldest
        // gap detectors). Every discard/return above this point must stay
        // above the `commit_sequence` call.
        {
            let buf = self.sample.bytes_mut();
            // Guard covers every header byte rewritten at commit:
            // total_size at [8..12] and sequence at [20..24].
            let buf_too_small = buf.len() < 24;
            if !buf_too_small {
                buf[8..12].copy_from_slice(&(total_size as u32).to_le_bytes());
                let seq = self.publisher.commit_sequence();
                buf[20..24].copy_from_slice(&seq.to_le_bytes());
            } else {
                // Capture buf.len() before the else-branch's mutable
                // borrow of `self.publisher`. `buf` is a `&mut [u8]`
                // (already borrowed mutably from `self.sample`), and
                // we need a separate mutable borrow of `self.publisher`
                // below. Capture-then-shadow-out the buf reference by
                // limiting its scope: the `buf_len` capture is the
                // last use, so the borrow ends here.
                let buf_len = buf.len();
                // Invariant violation: a publisher returned a buffer
                // smaller than 12 bytes — undersized sample. Bump
                // `frames_dropped_invariant_violation`. Operator action:
                // file a bug.
                match &mut self.publisher {
                    ProxyPublisher::Iceoryx2(pubr) => pubr.record_invariant_violation_drop(),
                }
                tracing::error!(
                    topic = self.publisher.topic(),
                    buf_len,
                    "OutputProxy: loaned buffer too small to host WireHeader; skipping publish \
                     (frames_dropped_invariant_violation counter incremented)"
                );
                return;
            }
        }

        // 3. Hand the finalized frame off to the publisher backend. Each
        //    branch is best-effort: errors log via `tracing::error!` and
        //    are swallowed so Drop never panics.
        match &mut self.publisher {
            ProxyPublisher::Iceoryx2(pubr) => {
                // History is NATIVE (iceoryx2 retains the SHM
                // frame by offset on `send()` below — zero-copy, no Cerulion
                // buffer). There is NO post-send network fan-out here
                // either — a graph process is network-free, so the plain
                // pub/sub path is UNCONDITIONALLY capture-free (the separate
                // gateway taps produced topics for egress).
                let Some(sample_mut) = self.sample.take_outbound() else {
                    // The wire sequence was already CONSUMED at the
                    // commit above — this frame burned its number but reaches no
                    // subscriber queue. Bump the send-fail counter so the loss is
                    // not a counter-less accounting hole (this is the send-side
                    // half of the reconciliation identity).
                    pubr.record_send_fail_drop();
                    tracing::error!(
                        topic = pubr.topic(),
                        "OutputProxy: outbound iceoryx2 sample missing at drop \
                         (frames_dropped_send_fail counter incremented)"
                    );
                    return;
                };
                // Truncate the loaned slice to the actual wire frame size
                // before sending. iceoryx2 supports a smaller-than-loan send
                // via `assume_init()` returning the consumed sample, so we
                // simply call `send()` here. The header carries the real
                // size for the receiver to slice on.
                // SAFETY: every byte is initialized — the WireHeader covers
                // [0..32], `T::build_writer` initialized the fixed section
                // and zeroed the offset table, and the user's writes
                // initialized the variable payload region (or the bytes
                // are unused dead space past total_size).
                // Scope boundary: this `send` failure runs
                // AFTER `commit_sequence` above — it has already BURNED a wire
                // sequence, so the recorded stream has a REAL gap at that seq
                // and subsequent seqs are unshifted. That is a DIFFERENT bug from
                // the no-seq-burn pre-commit discard class (which the discard path
                // above marks + suppresses): its replay divergence would be "replay's
                // send succeeds → one localized extra with a MATCHING seq" and
                // needs "replay must burn the seq but skip delivery" semantics.
                // The discard marker is deliberately scoped to "committed 0
                // outputs (no seq burned)"; the post-commit send-failure class is
                // NOT handled here.
                if let Err(e) = sample_mut.send() {
                    // The sequence was consumed at commit but the frame
                    // never reached any subscriber queue. Bump the send-fail
                    // counter (a bare `error!` + `return` would be the one counter-less
                    // silent-loss arm in the commit path).
                    pubr.record_send_fail_drop();
                    tracing::error!(
                        topic = pubr.topic(),
                        error = %e,
                        "OutputProxy: iceoryx2 send() failed at drop \
                         (frames_dropped_send_fail counter incremented)"
                    );
                    return;
                }
                // A COMPLETE publish went out — clear the discard
                // flood-latch. `record_output_complete` is a single branch
                // returning `None` on the healthy steady-state path (no alloc,
                // no lock); it returns `Some` only on the cold recovery
                // transition (the first complete publish after a discard
                // regime), where one `info!` reports the total suppressed count
                // and the port re-arms so a fresh breakage is loud again.
                if let Some(suppressed) = pubr.record_output_complete() {
                    tracing::info!(
                        topic = pubr.topic(),
                        schema_hash = T::SCHEMA_HASH,
                        suppressed_count = suppressed,
                        "OutputProxy: output recovered — complete publish after a discard \
                         regime (further discards on this port will log loudly again)"
                    );
                }
                // `block`: the frame is now in every subscriber's
                // queue — bump each registered consumer's outstanding
                // counter (no-op unless this topic is wired all-`block`).
                // Mirrors the queue depth the producer's pre-fire reads.
                pubr.record_block_published();
                // The frame went out — reset this output's
                // `promise_within_ms` watchdog window (no-op unless wired).
                pubr.record_promise_within_published();
                // Adaptive loan sizing: feed the actual published
                // payload size into the publisher's sliding window so
                // the next `loan_proxy` call can pick a tighter loan
                // size. Recorded only after the SHM send succeeds —
                // partial / failed ticks do NOT contribute to the
                // window (the convergence policy; overflow ticks record
                // separately, see `send_overflow_frame`).
                //
                // Record vs notify_sent_sample ordering:
                // record runs BEFORE `notify_sent_sample` below.
                // The sliding window tracks "how big are this
                // publisher's payloads" — not "did subscribers wake
                // up" — so the order is semantically correct for
                // sizing purposes. Be precise about subscriber
                // recovery semantics under notify failure though:
                //   - `try_view` is unaffected: it calls
                //     `subscriber.receive()` directly and only
                //     touches the listener via non-blocking
                //     `try_wait_one` for stale-event drainage —
                //     no blocking wait, so polling-style
                //     subscribers always see the frame on their
                //     next call regardless of notify state.
                //   - `wait_for_message` recovers ONLY via its
                //     pre-drain at call-start; a subscriber already
                //     blocked inside `timed_wait_one` when notify
                //     fails stays blocked until either the timeout
                //     expires or another notifier on the same
                //     `<topic>/event` service wakes it (any
                //     publisher's `notify_sent_sample`, or any
                //     subscriber's `SubscriberConnected`/
                //     `SubscriberDisconnected` on the same topic).
                //     Notify failures therefore extend the
                //     `wait_for_message` user's latency up to its
                //     `timeout` argument — loud at the metrics
                //     layer, but not a data-loss event (the SHM
                //     frame is committed atomically; only the wake
                //     is lost). Polling-style users (`try_view`)
                //     and bench-style spin-loops are unaffected.
                // Moving record after notify would mis-classify
                // data-delivered ticks as "didn't happen" for window
                // purposes, shrinking the next loan and increasing
                // overflow risk. So the order stays.
                pubr.record_payload_size(total_size as u32);
                // Best-effort SentSample notification — data is already in
                // shared memory. Subscribers can still read it on their next
                // try_view / wait_for_message even if this fails.
                if let Err(e) = pubr.notify_sent_sample() {
                    tracing::trace!(
                        topic = pubr.topic(),
                        error = %e,
                        "OutputProxy: SentSample notification failed (data already delivered)"
                    );
                }
                // No post-send network fan-out — a graph process is
                // network-free (the separate gateway taps produced topics for
                // egress), so the publish path is unconditionally capture-free.
            }
        }

        // notify_with_custom_event_id was called inside the iceoryx2 branch
        // above. No further work here.
        let _ = PubSubEvent::SentSample; // doc anchor only
    }
}
