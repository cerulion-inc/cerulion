// SPDX-License-Identifier: AGPL-3.0-only
//! `create_ingress_publisher` provisions a SLICE-AWARE receive-queue
//! depth, and a recorder-style tap really inherits it.
//!
//! # Why the depth is slice-aware
//!
//! A `ros2 attach` bridge route is created by `create_ingress_publisher`. Under
//! `default_topic_config()` it would get `DEFAULT_SUBSCRIBER_BUFFER_SIZE`
//! = 16 frames — a general-purpose default, never a considered choice for a
//! foreign stream at an undeclared rate. Measured on a Go2, that depth costs two
//! 1.66 kHz Point-LIO outputs **70.9 %** and **70.8 %** of their frames, while a
//! 700 Hz topic on the same recorder loses 0.13 %. The entire difference is that
//! 16 frames absorbs 9.6 ms at one rate and 22.9 ms at the other.
//!
//! It cannot be set anywhere else: iceoryx2 pins
//! `subscriber_max_buffer_size` at service CREATE, rejects an opener that asks
//! for more (`DoesNotSupportRequestedMinBufferSize`), and silently hands an
//! opener that asks for less the existing ceiling. The bridge's first
//! `create_ingress_publisher` call is the only moment this is settable.
//!
//! # The five pins
//!
//! 1. the ceiling is EXACTLY `ingress_route_buffer_depth(max_slice_len)` at the
//!    bridge's own slice size — an opener requiring it attaches, `+1` is
//!    rejected by iceoryx2 itself, so the first assertion is not vacuous;
//! 2. the CAP arm — a small-slice route gets the full cap, so the rule is a
//!    scaling one and not a single hard-coded number;
//! 3. the FLOOR arm — a huge-slice route keeps the stock ceiling, so the rule
//!    can only ever RAISE (the never-make-it-worse guarantee);
//! 4. the DEGRADED-ATTACH arm — a route whose service already exists at the
//!    stock ceiling still comes up, publishes, and says so, in the wording that
//!    BLAMES an earlier opener. Setting a ceiling also arms iceoryx2's open-time
//!    verification, so a bare raise would turn a working degraded
//!    attach into a hard refusal; a route that cannot START is worse than one
//!    that loses frames. The fallback catches every error class but only the
//!    buffer-ceiling refusal licenses naming a culprit, so the warn has two
//!    arms; this is the arm with a REAL shallow incumbent behind it, and the
//!    cause-neutral twin is pinned purely (no black-box test can drive the
//!    fallback into it — see `transport::tests::
//!    the_ingress_fallback_blames_an_earlier_opener_only_when_one_is_proven`);
//! 5. the BEHAVIORAL arm — a burst larger than the stock depth, published into a
//!    real ingress publisher and drained by the recorder's exact tap type
//!    (`create_data_only_subscriber`, which passes `buffer_size: None` and so
//!    inherits the service ceiling), recovers MORE than the stock 16 frames.
//!    That is the pin that fails if the raise is wired to nothing, and its
//!    control is the same burst on a hand-provisioned stock-depth service,
//!    which must recover exactly 16.
//!
//! Isolated per-test SHM roots, so parallel-safe — no `#[serial]`.

use std::sync::Arc;

use cerulion_core::testing::iceoryx_test_config;
use cerulion_core::transport::{
    ingress_route_buffer_depth, PublisherProvisioning, TopicServiceConfig,
    INGRESS_ROUTE_BUFFER_DEPTH,
};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_core::{TransportConfig, TransportManager};

/// `dds_bridge`'s `DEFAULT_RAW_MAX_SLICE_LEN` — what EVERY `ros2 attach` route
/// gets, because the generated bridge config never emits `max_slice_len:`.
const BRIDGE_SLICE: u32 = 1 << 20;

/// The stock ceiling of an unprovisioned route. Not imported from `cerulion_core`
/// (the constant is private) — spelled out here on purpose, so this file states
/// the number it is about rather than tracking whatever the default
/// becomes.
const STOCK_DEPTH: usize = 16;

static NEXT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

fn manager() -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: "ingress_depth".to_string(),
            subscriber_buffer_size: STOCK_DEPTH,
            ..Default::default()
        },
        iceoryx_test_config(),
    )
    .expect("test transport manager")
}

fn topic_name(tag: &str) -> String {
    let n = NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    format!("/{tag}/{n}")
}

/// An opener REQUIRING exactly `ceiling` on the buffer axis — the iceoryx2
/// open-requirement probe. Cribs `recording_provisioning_iox2_test`'s helper,
/// but with `PublisherProvisioning::External`, which is what an ingress route
/// declares.
fn attach_requiring(
    mgr: &TransportManager,
    topic: &str,
    ceiling: usize,
) -> Result<(), cerulion_core::TransportError> {
    mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            ceiling,
            0,
            PublisherProvisioning::External,
            0,
            0,
        ),
        ceiling,
    )
    .map(|_| ())
}

fn frame(seq: u32, body: &[u8]) -> Vec<u8> {
    let total = WireHeader::SIZE + body.len();
    let mut h = WireHeader::new(0xC1230, seq, seq as u64);
    h.total_size = total as u32;
    let mut f = vec![0u8; total];
    h.write_to_buf(&mut f[..WireHeader::SIZE]);
    f[WireHeader::SIZE..].copy_from_slice(body);
    f
}

// ============================================================
// Pin 1 — the ceiling is EXACTLY the scaled depth, at the bridge's slice.
// ============================================================

#[test]
fn an_ingress_route_ceiling_is_exactly_the_scaled_depth() {
    let expected = ingress_route_buffer_depth(BRIDGE_SLICE as usize);
    assert_eq!(
        expected, 64,
        "the oracle for the bridge's own 1 MiB slice — 64 frames, 38.6 ms at \
         1.66 kHz against the stock 16's 9.6 ms"
    );
    assert!(
        expected > STOCK_DEPTH,
        "anti-tautology: if the rule did not RAISE at the bridge's slice size \
         there would be nothing to pin"
    );

    let mgr = manager();
    let topic = topic_name("exact");
    let _pubr = mgr
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(BRIDGE_SLICE))
        .expect("create_ingress_publisher");

    // A DEFAULT opener still attaches — the raise preserves dominance.
    mgr.create_subscriber(&topic)
        .expect("a default opener must still attach to a raised ingress route");
    // An opener requiring the FULL scaled depth attaches: the service really
    // was created at that ceiling, which is what a `buffer_size: None` tap
    // inherits.
    attach_requiring(&mgr, &topic, expected)
        .expect("an opener requiring the scaled depth must attach");
    // ...and depth+1 is rejected BY ICEORYX2, which is what makes the attach
    // above a pin on the EXACT value rather than on "at least something".
    assert!(
        attach_requiring(&mgr, &topic, expected + 1).is_err(),
        "an opener requiring depth+1 must be rejected — the ceiling must be \
         EXACTLY the scaled depth, not merely at least it"
    );
}

// ============================================================
// Pin 2 — the CAP arm: a small slice buys the full cap.
// ============================================================

#[test]
fn a_small_slice_ingress_route_reaches_the_depth_cap() {
    const SMALL: u32 = 4 * 1024;
    let expected = ingress_route_buffer_depth(SMALL as usize);
    assert_eq!(
        expected, INGRESS_ROUTE_BUFFER_DEPTH,
        "a 4 KiB route is budget-cheap enough for the full cap — the property \
         that makes right-sizing `max_slice_len` REWARDING rather than merely \
         permitted"
    );

    let mgr = manager();
    let topic = topic_name("cap");
    let _pubr = mgr
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(SMALL))
        .expect("create_ingress_publisher");

    attach_requiring(&mgr, &topic, expected).expect("the cap must be provisioned");
    assert!(
        attach_requiring(&mgr, &topic, expected + 1).is_err(),
        "the cap must be a ceiling, not a floor"
    );
}

// ============================================================
// Pin 3 — the FLOOR arm: the rule may only ever RAISE.
// ============================================================

#[test]
fn a_huge_slice_ingress_route_keeps_the_stock_ceiling() {
    // 16 MiB: the budget cannot buy even one frame past the floor here, so the
    // rule is INERT and the route must be left at the stock ceiling.
    const HUGE: u32 = 16 * 1024 * 1024;
    assert_eq!(
        ingress_route_buffer_depth(HUGE as usize),
        STOCK_DEPTH,
        "the floor is the stock ceiling, so a huge-slice route is never made \
         WORSE than the stock depth"
    );

    let mgr = manager();
    let topic = topic_name("floor");
    let _pubr = mgr
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(HUGE))
        .expect("create_ingress_publisher");

    attach_requiring(&mgr, &topic, STOCK_DEPTH).expect("the stock ceiling is provisioned");
    assert!(
        attach_requiring(&mgr, &topic, STOCK_DEPTH + 1).is_err(),
        "a huge-slice route must NOT be silently deepened — the budget exists \
         precisely so a multi-MiB slot count stays bounded"
    );
}

// ============================================================
// Pin 4 — DEGRADED ATTACH: a shallower pre-existing service is TOLERATED.
// ============================================================

/// A route whose service ALREADY EXISTS at the stock ceiling must still be
/// created — degraded, with a warn — never refused.
///
/// # Why this arm exists
///
/// Raising the ceiling on the config handed to
/// `open_topic_services` is not enough: setting a buffer ceiling ALSO arms iceoryx2's
/// open-time at-least verification. So any service that already exists at 16 —
/// a `cerulion topic echo` left running, a consumer that opened first, a
/// leftover from a previous run — would turn from a working degraded attach into a
/// hard `DoesNotSupportRequestedMinBufferSize` refusal. A bridge route that will
/// not START is strictly worse than one that loses frames, and it breaks the
/// contract this repo states on `TopicServiceConfig::for_topology`'s borrow
/// field: owners TOLERATE + WARN, they do not refuse.
///
/// Four ingress byte-identity tests exercise it as well
/// (`network_ingress_test`, `network_ingress_e2e_test`, `network_tf_e2e_test`,
/// `gateway_iox2_test`), since every one of them opens a local subscriber BEFORE the
/// ingress publisher. This is the arm that names the property directly, so the
/// provisioning rule is stated here rather than inferred
/// from four unrelated failures.
#[test]
#[tracing_test::traced_test]
fn a_route_whose_service_already_exists_shallow_attaches_instead_of_refusing() {
    let mgr = manager();
    let topic = topic_name("degraded");

    // A default opener CREATES the service at the stock ceiling first — the
    // exact shape a running `cerulion topic echo` leaves behind.
    let _incumbent = mgr
        .create_subscriber(&topic)
        .expect("a default opener creates the service at the stock ceiling");

    // The route must still come up.
    let mut pubr = mgr
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(BRIDGE_SLICE))
        .expect(
            "a shallower pre-existing service must DEGRADE the route, never refuse it — a \
             bridge route that cannot start is worse than one that loses frames",
        );

    // ...and it must be FUNCTIONAL, not merely constructed.
    let body = vec![0x5Au8; 32];
    pubr.publish_raw(&frame(0, &body))
        .expect("a degraded route still publishes");

    // ...at the incumbent's depth, which is the correct outcome: the ceiling is
    // fixed at create and cannot be raised afterwards.
    attach_requiring(&mgr, &topic, STOCK_DEPTH).expect("the stock ceiling is what exists");
    assert!(
        attach_requiring(
            &mgr,
            &topic,
            ingress_route_buffer_depth(BRIDGE_SLICE as usize)
        )
        .is_err(),
        "the route did NOT get the deeper queue — iceoryx2 pins the ceiling at \
         create, so an incumbent shallow service wins"
    );

    // And it must SAY so. A silent degrade would leave an operator with a route
    // that quietly cannot absorb a stall, which is the failure itself.
    assert!(
        logs_contain("ingress route attached to a SHALLOWER pre-existing service"),
        "the degrade must be loud — a silent one hides exactly the condition \
         the slice-aware depth exists to close"
    );
    // ...in the arm that BLAMES, because here there really IS an incumbent: the
    // test created it two statements ago. This is the half a pure test cannot
    // reach — that the production classification says `true` when a shallow
    // service is genuinely in the way.
    assert!(
        logs_contain("stop the earlier opener and let the route create the service first"),
        "a CONFIRMED shallow incumbent must carry the remedy — the operator can \
         actually act on this one"
    );
    assert!(
        !logs_contain("ingress route attached at the STOCK buffer ceiling"),
        "a confirmed shallow incumbent must NOT take the cause-neutral arm — \
         that arm exists for refusals with no incumbent behind them, and using \
         it here would withhold a remedy that applies"
    );
}

// ============================================================
// Pin 5 — BEHAVIORAL: the recorder's own tap type inherits the depth.
// ============================================================

/// A burst larger than the stock depth, published with NO drain, then drained
/// by the recorder's exact tap type.
///
/// iceoryx2 keeps the newest `ceiling` samples, so the count recovered IS the
/// provisioned ceiling, measured rather than asserted from the config. This is
/// the arm that fails if the raise is computed and then never reaches
/// `open_topic_services` — every other pin in this file goes through
/// `attach_requiring`, which probes the SERVICE, while this one probes what a
/// recorder actually gets.
#[test]
fn a_data_only_tap_inherits_the_raised_ingress_depth() {
    let mgr = manager();
    let expected = ingress_route_buffer_depth(BRIDGE_SLICE as usize);

    // --- the raised route -------------------------------------------------
    let topic = topic_name("behavioral");
    let mut pubr = mgr
        .create_ingress_publisher(&topic, MaxSliceLen::const_new(BRIDGE_SLICE))
        .expect("create_ingress_publisher");
    let mut tap = mgr
        .create_data_only_subscriber(&topic)
        .expect("the recorder's tap type");

    // Comfortably past any plausible ceiling, so the queue provably laps and
    // the recovered count is the ceiling rather than the burst.
    let burst = expected * 4 + 128;
    let body = vec![0xABu8; 64];
    for k in 0..burst {
        pubr.publish_raw(&frame(k as u32, &body)).expect("publish");
    }
    let mut scratch = Vec::with_capacity(1);
    let mut recovered = 0usize;
    loop {
        let n = tap.drain_owned(1, &mut scratch).expect("drain");
        scratch.clear();
        if n == 0 {
            break;
        }
        recovered += n;
    }

    assert!(
        recovered < burst,
        "anti-vacuity: the burst must have OVERFLOWED ({recovered} of {burst} \
         recovered) — otherwise this measured the burst, not the queue"
    );
    assert_eq!(
        recovered, expected,
        "the tap inherits the ROUTE's ceiling: a data-only tap opens with \
         `buffer_size: None`, so what it can hold IS what \
         `create_ingress_publisher` provisioned"
    );
    assert!(
        recovered > STOCK_DEPTH,
        "THE pin: a recorder tapping a bridge route must hold more than the \
         {STOCK_DEPTH} frames that cost a Go2 71 % of two kHz streams"
    );

    // --- the control: the same burst at the stock depth -------------------
    //
    // Without this, the assertion above would also pass on a build where every
    // service happened to be deep for some unrelated reason.
    let ctrl_topic = topic_name("behavioral_ctrl");
    let mut ctrl_cfg = mgr.default_topic_config();
    ctrl_cfg.subscriber_max_buffer_size = STOCK_DEPTH;
    let mut ctrl_pub = mgr
        .create_publisher_with_topic_config(
            &ctrl_topic,
            MaxSliceLen::const_new(BRIDGE_SLICE),
            0,
            ctrl_cfg,
        )
        .expect("stock-depth control publisher");
    let mut ctrl_tap = mgr
        .create_data_only_subscriber(&ctrl_topic)
        .expect("control tap");
    for k in 0..burst {
        ctrl_pub
            .publish_raw(&frame(k as u32, &body))
            .expect("publish");
    }
    let mut ctrl_recovered = 0usize;
    loop {
        let n = ctrl_tap.drain_owned(1, &mut scratch).expect("drain");
        scratch.clear();
        if n == 0 {
            break;
        }
        ctrl_recovered += n;
    }
    assert_eq!(
        ctrl_recovered, STOCK_DEPTH,
        "the control must show the stock-depth behaviour on the same burst — that is \
         what makes the {expected}-frame reading above attributable to the raise"
    );
}
