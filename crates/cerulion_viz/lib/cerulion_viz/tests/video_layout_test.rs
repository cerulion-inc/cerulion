// SPDX-License-Identifier: AGPL-3.0-only
//! An INTERLEAVED H.264 topic gets ONE `spatial2d` view — its DEFAULT
//! RENDITION — in BOTH layout producers, and no empty status pane beside it.
//!
//! # Why this is a defect and not a nicety
//!
//! A video topic renders each rendition at its OWN child entity
//! (`<entity>/viz-video/<WxH>`). A rerun 0.34 `spatial2d` view anchors to a SINGLE
//! `space_origin`, so two renditions placed in one view OVERLAY — both decode
//! correctly and one is invisible. That is the exact failure `build_image_views`
//! was written for (the smart-layout finding that N sibling cameras must not share a 2D
//! view); two renditions of ONE camera are the same problem one level down.
//!
//! The H.264 route answered that with one view PER rendition, which is mechanically
//! correct and reads to a user as the same camera checked twice. A live
//! report showed it: attaching `/go2/camera/h264` rendered the video plus a
//! second tile titled the same topic showing `(empty)`. So the ENTITIES stay
//! split (decode correctness is untouched) while the default LAYOUT shows one of
//! them, and the others stay reachable in the entity tree.
//!
//! Two things produced that duplication and both are covered here: the
//! per-rendition view split and the `text_document` companion
//! every degradable kind carries — which is the `(empty)` tile, and which is
//! RETAINED on every path where a dump can still be the only thing to render.
//!
//! # The plumb these tests cover
//!
//! The demux that knows the rendition set lives in the WORKER's `SinkState`; the
//! layout is assembled on the daemon's CONTROL thread. The path between them is
//! `VizWorkerCounters::video_renditions`, mirrored after each processed batch
//! beside `coalesced_frames`. These tests cover BOTH halves:
//!
//! * the layout SELECTION, over hand-built `AttachedRender`s (pure, no transport);
//! * the MIRROR being populated by a real `VideoDemux` and surviving the trip
//!   through the counters — so the field cannot ship inert.
//!
//! The daemon-side read (`Ctx::video_renditions_for`) is a two-line lookup on that
//! same map, pinned by `cerulion_vizd`'s own `daemon.rs` test.
//!
//! No iceoryx2, no rerun sink — pure functions plus one `VideoDemux`.
//! Parallel-safe.

use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::blueprint::{
    compose_layout, default_layout, views_for_render, AttachedRender, ComposeGroup, ComposeInput,
    ComposeStrategy, PlanNode, PlanView, ViewKind,
};
use cerulion_viz::representation::Representation;
use cerulion_viz::sink::ArchetypeKind;
use cerulion_viz::sink::RenderProof;
use cerulion_viz::sink::SinkState;
use cerulion_viz::worker::{InputFrames, VizLogWorker};

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// The REAL Go2 360p SPS (see `video_h264_test.rs` for full provenance) — bytes
/// 0x18..0x32 of the committed `go2_frontvideostream_360p_idr.cdr` capture.
const GO2_REAL_SPS: &[u8] = &[
    0x67, 0x64, 0x10, 0x28, 0xac, 0x1b, 0x1a, 0xa0, 0xa0, 0x2f, 0xf9, 0x61, 0x00, 0x00, 0x03, 0x00,
    0x01, 0x00, 0x00, 0x03, 0x00, 0x3c, 0x8f, 0x08, 0x84, 0x6a,
];
const GO2_REAL_PPS: &[u8] = &[0x68, 0xee, 0x31, 0xb2, 0x1b];
const GO2_REAL_IDR_HEAD: &[u8] = &[0x65, 0xb8, 0x00, 0x01, 0x40, 0x00, 0x01, 0x3f];

fn annex_b(nals: &[&[u8]]) -> Vec<u8> {
    let mut out = Vec::new();
    for n in nals {
        out.extend_from_slice(&[0, 0, 0, 1]);
        out.extend_from_slice(n);
    }
    out
}

/// A minimal MSB-first Exp-Golomb writer, enough for a Baseline SPS (crib:
/// `video_h264_test.rs::build_sps`).
struct BitWriter {
    bits: Vec<bool>,
}

impl BitWriter {
    fn new() -> Self {
        Self { bits: Vec::new() }
    }
    fn u(&mut self, value: u32, n: u32) {
        for i in (0..n).rev() {
            self.bits.push((value >> i) & 1 == 1);
        }
    }
    fn ue(&mut self, value: u32) {
        let code = value + 1;
        let len = 32 - code.leading_zeros();
        self.u(0, len - 1);
        self.u(code, len);
    }
    fn finish(mut self) -> Vec<u8> {
        self.bits.push(true);
        while !self.bits.len().is_multiple_of(8) {
            self.bits.push(false);
        }
        self.bits
            .chunks(8)
            .map(|c| c.iter().fold(0u8, |acc, b| (acc << 1) | u8::from(*b)))
            .collect()
    }
}

fn build_sps(width_mbs: u32, height_mbs: u32) -> Vec<u8> {
    let mut w = BitWriter::new();
    w.u(66, 8); // Baseline
    w.u(0, 8);
    w.u(30, 8);
    w.ue(0); // sps_id
    w.ue(0);
    w.ue(2);
    w.ue(1);
    w.u(0, 1);
    w.ue(width_mbs - 1);
    w.ue(height_mbs - 1);
    w.u(1, 1);
    w.u(1, 1);
    w.u(0, 1);
    w.u(0, 1);
    let mut nal = vec![0x67u8];
    nal.extend_from_slice(&w.finish());
    nal
}

fn go2_360_keyframe() -> Vec<u8> {
    annex_b(&[GO2_REAL_SPS, GO2_REAL_PPS, GO2_REAL_IDR_HEAD])
}

fn video_topic(topic: &str, renditions: &[&str]) -> AttachedRender {
    AttachedRender {
        topic: topic.to_string(),
        entity: format!("world{topic}"),
        archetype: Some(ArchetypeKind::VideoStream),
        producer_count: Some(1),
        video_renditions: renditions.iter().map(|s| s.to_string()).collect(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    }
}

/// Every leaf view in a plan, flattened.
fn leaf_views(node: &PlanNode, out: &mut Vec<PlanView>) {
    match node {
        PlanNode::View(v) => out.push(v.clone()),
        PlanNode::Container(c) => {
            for child in &c.children {
                leaf_views(child, out);
            }
        }
    }
}

/// The `spatial2d` view ORIGINS a plan produces, sorted.
fn spatial2d_origins(root: &PlanNode) -> Vec<String> {
    let mut views = Vec::new();
    leaf_views(root, &mut views);
    let mut origins: Vec<String> = views
        .into_iter()
        .filter(|v| v.kind == ViewKind::Spatial2d)
        .filter_map(|v| v.origin)
        .collect();
    origins.sort();
    origins
}

// ===========================================================================
// 1. The DEFAULT layout producer
// ===========================================================================

#[test]
fn the_default_layout_gives_an_interleaved_topic_one_view_at_its_default_rendition() {
    // THE headline (replacing the H.264 route's one-view-per-rendition). Two
    // renditions ⇒ ONE `spatial2d` view, rooted at the HIGHEST rendition's own
    // child entity.
    //
    // Rooted at the CHILD, never at the topic: the topic's subtree holds both
    // renditions, so a topic-rooted view overlays them and one is invisible —
    // the overlay failure this must not re-open while collapsing the pane count.
    let plan = default_layout(&[video_topic("/go2/camera", &["640x360", "1280x720"])]);
    assert_eq!(
        spatial2d_origins(&plan.root),
        vec!["world/go2/camera/viz-video/1280x720".to_string()],
        "one checked topic is ONE pane, showing the highest rendition"
    );

    // The pane names its rendition: it shows one of several streams the topic
    // carries, and a bare topic title would claim it is the whole of it.
    let mut views = Vec::new();
    leaf_views(&plan.root, &mut views);
    let names: Vec<String> = views
        .iter()
        .filter(|v| v.kind == ViewKind::Spatial2d)
        .filter_map(|v| v.name.clone())
        .collect();
    assert_eq!(names.len(), 1, "exactly one 2D pane, got {names:?}");
    assert!(
        names[0].contains("1280x720"),
        "the pane must name the rendition it shows, got {names:?}"
    );
}

#[test]
fn the_default_rendition_is_chosen_by_pixel_count_not_by_the_order_segments_arrive_in() {
    // The ONE arm that separates "read the resolution" from the two cheap proxies,
    // and the Go2's own pair is where both of them break.
    //
    // `video_renditions` reaches the layout as OPAQUE `WxH` strings across a
    // thread boundary. Today they arrive numerically ascending only because
    // `StreamKey`'s derived `Ord` sorts a `BTreeMap` by `(width, height)` — a
    // producer-side re-ordering would silently demote the operator's camera to its
    // thumbnail, and nothing else in the suite would notice.
    //
    // Driven in BOTH orders against ONE oracle, so neither "take the first" nor
    // "take the last" can pass:
    //   ascending  ["640x360", "1280x720"] — "first" would pick 640x360
    //   descending ["1280x720", "640x360"] — "last"  would pick 640x360
    // Sorting the strings is wrong too: "1280x720" < "640x360" lexicographically.
    for order in [vec!["640x360", "1280x720"], vec!["1280x720", "640x360"]] {
        let plan = default_layout(&[video_topic("/go2/camera", &order)]);
        assert_eq!(
            spatial2d_origins(&plan.root),
            vec!["world/go2/camera/viz-video/1280x720".to_string()],
            "{order:?}: the pane must show the 720p rendition whatever the order"
        );
    }

    // A segment nobody can parse must not out-rank a real resolution: it is not
    // evidence of a bigger picture, and letting it win hands the pane to the
    // stream we understand least. Deterministic either way — a layout that
    // flickered between renditions run to run would be worse than either answer.
    let plan = default_layout(&[video_topic("/go2/camera", &["garbage", "640x360"])]);
    assert_eq!(
        spatial2d_origins(&plan.root),
        vec!["world/go2/camera/viz-video/640x360".to_string()],
        "an unparseable segment must not out-rank a real resolution"
    );
}

#[test]
fn the_default_layout_leaves_a_single_rendition_topic_with_one_view() {
    // NO REGRESSION on the common case. One rendition keeps the single
    // topic-rooted view it has always had: its lone child is already inside that
    // view's subtree, and rooting at the child would make the view origin depend
    // on the resolution.
    for renditions in [vec![], vec!["640x360"]] {
        let plan = default_layout(&[video_topic("/cam", &renditions)]);
        assert_eq!(
            spatial2d_origins(&plan.root),
            vec!["world/cam".to_string()],
            "{renditions:?}: one view, rooted at the topic"
        );
    }
}

#[test]
fn the_default_layout_is_unchanged_for_a_non_video_topic() {
    // ANTI-TAUTOLOGY: an ordinary image topic must be untouched by the split, and
    // an empty `video_renditions` is what every non-video topic carries.
    let image = AttachedRender {
        topic: "/cam/image_raw".to_string(),
        entity: "world/cam/image_raw".to_string(),
        archetype: Some(ArchetypeKind::Image),
        producer_count: Some(1),
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    };
    let plan = default_layout(&[image]);
    assert_eq!(
        spatial2d_origins(&plan.root),
        vec!["world/cam/image_raw".to_string()]
    );
}

#[test]
fn each_attached_topic_contributes_exactly_one_2d_pane_when_two_cameras_are_attached() {
    // MULTI-TOPIC must not regress: a video topic with two renditions BESIDE a
    // plain image topic is TWO 2D views — one each — and the plain one is
    // untouched. Collapsing the renditions must not collapse the TOPICS.
    let plan = default_layout(&[
        video_topic("/go2/camera", &["640x360", "1280x720"]),
        AttachedRender {
            topic: "/wrist/image_raw".to_string(),
            entity: "world/wrist/image_raw".to_string(),
            archetype: Some(ArchetypeKind::Image),
            producer_count: Some(1),
            video_renditions: Vec::new(),
            representation: Representation::Auto,
            render_proof: RenderProof::default(),
        },
    ]);
    assert_eq!(
        spatial2d_origins(&plan.root),
        vec![
            "world/go2/camera/viz-video/1280x720".to_string(),
            "world/wrist/image_raw".to_string(),
        ]
    );
}

// ===========================================================================
// 2. The COMPOSE layout producer
// ===========================================================================

fn bare(topics: Vec<AttachedRender>) -> ComposeInput {
    ComposeInput {
        groups: vec![ComposeGroup {
            role: None,
            title: None,
            topics,
        }],
        strategy: ComposeStrategy::Auto,
        include_robot: false,
    }
}

#[test]
fn compose_gives_an_interleaved_topic_one_view_at_its_default_rendition() {
    // The SECOND producer, which reaches the same decision by a DIFFERENT
    // mechanism: `compose_layout` builds one 2D view per image ENTITY
    // (`build_image_views`), so it is fed the ONE chosen rendition child rather
    // than the topic entity or all of them.
    //
    // It gets its own arm because the two producers share no code path — the
    // `>= 2` threshold and the child-entity spelling are written twice, and a fix
    // applied to one does not propagate to the other.
    let composed = compose_layout(&bare(vec![video_topic(
        "/go2/camera",
        &["640x360", "1280x720"],
    )]))
    .expect("composes");
    assert_eq!(
        spatial2d_origins(&composed.plan.root),
        vec!["world/go2/camera/viz-video/1280x720".to_string()]
    );

    // The order-independence the default producer is pinned on holds here too —
    // separately, because this path selects at a different call site.
    let composed = compose_layout(&bare(vec![video_topic(
        "/go2/camera",
        &["1280x720", "640x360"],
    )]))
    .expect("composes");
    assert_eq!(
        spatial2d_origins(&composed.plan.root),
        vec!["world/go2/camera/viz-video/1280x720".to_string()],
        "compose must read the resolution too, not the arrival order"
    );
}

#[test]
fn compose_leaves_a_single_rendition_topic_with_one_view() {
    for renditions in [vec![], vec!["640x360"]] {
        let composed = compose_layout(&bare(vec![video_topic("/cam", &renditions)])).expect("ok");
        assert_eq!(
            spatial2d_origins(&composed.plan.root),
            vec!["world/cam".to_string()],
            "{renditions:?}"
        );
    }
}

// ===========================================================================
// 2b. The EMPTY status pane — the other half of "duplicated"
// ===========================================================================

/// Every leaf view kind a plan produces, sorted — the pane INVENTORY.
fn view_kinds(root: &PlanNode) -> Vec<ViewKind> {
    let mut views = Vec::new();
    leaf_views(root, &mut views);
    let mut kinds: Vec<ViewKind> = views.into_iter().map(|v| v.kind).collect();
    kinds.sort_by_key(|k| k.as_str());
    kinds
}

#[test]
fn a_decoding_video_topic_is_not_given_a_permanently_empty_status_pane() {
    // The reported screenshot, at the layer that produces it: attaching
    // `/go2/camera/h264` rendered the live video AND a second tile titled the same
    // topic showing `(empty)`.
    //
    // That tile is the `text_document` companion every degradable kind
    // carries so a fallback dump has somewhere to land. Its stated cost was "one
    // extra (empty until it degrades) status pane"; this fix refuses that cost on a
    // video topic that is provably decoding.
    //
    // Asserted on the KIND inventory, not just the count, so a pane surviving
    // under another name cannot pass.
    let plan = default_layout(&[video_topic("/go2/camera/h264", &["640x360", "1280x720"])]);
    assert_eq!(
        view_kinds(&plan.root),
        vec![ViewKind::Spatial2d, ViewKind::Spatial3d],
        "a decoding camera is the Scene plus ONE video pane — no empty status tile"
    );

    // The SAME quantity the production log prints at the apply site
    // ("applied a runtime blueprint (layout) views=N"), so this arm is directly
    // comparable to the reported live capture. That session logged views=4 for
    // this exact topic and rendition pair — Scene + 640x360 + 1280x720 + the empty
    // status tile. Two of those four were the duplication.
    assert_eq!(
        plan.view_count(),
        2,
        "the reported live session logged views=4 for this input"
    );

    // …and the same answer one layer down, where the decision is actually made.
    assert_eq!(
        views_for_render(&video_topic("/go2/camera/h264", &["640x360", "1280x720"])),
        vec![ViewKind::Spatial2d]
    );
}

#[test]
fn a_video_topic_that_has_decoded_nothing_keeps_the_status_pane() {
    // THE anti-tautology, and the arm deliberately RETAINED. A rendition
    // segment exists only because the demux opened a sub-stream from a real SPS,
    // so an EMPTY set is a video topic that has decoded nothing — exactly the
    // never-seen-robot path the dump ladder exists for. Its pane stays.
    //
    // Without this arm, "drop the companion" and "drop it only once decode is
    // proven" are indistinguishable, and the refusal would be a silent-dump regression
    // wearing a passing test.
    assert_eq!(
        views_for_render(&video_topic("/go2/camera/h264", &[])),
        vec![ViewKind::Spatial2d, ViewKind::TextDocument],
        "a video topic with no observed rendition must keep its dump pane"
    );

    // One rendition is already proof of decode, so the companion goes — while the
    // VIEW stays topic-rooted (the single-rendition rule, untouched).
    let plan = default_layout(&[video_topic("/go2/camera/h264", &["640x360"])]);
    assert_eq!(
        view_kinds(&plan.root),
        vec![ViewKind::Spatial2d, ViewKind::Spatial3d]
    );
    assert_eq!(
        spatial2d_origins(&plan.root),
        vec!["world/go2/camera/h264".to_string()]
    );
}

#[test]
fn a_non_video_degradable_topic_keeps_its_status_pane() {
    // SCOPE. The refusal is video-only: an occupancy grid whose cell buffer is
    // short, and every other degradable kind, still dump into a pane that exists.
    // This is the arm that fails if the refusal is widened past the video
    // case.
    let grid = AttachedRender {
        topic: "/map".to_string(),
        entity: "world/map".to_string(),
        archetype: Some(ArchetypeKind::OccupancyGrid),
        producer_count: Some(1),
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    };
    assert_eq!(
        views_for_render(&grid),
        vec![ViewKind::Spatial2d, ViewKind::TextDocument],
        "a non-video degradable kind keeps the companion pane"
    );
}

#[test]
fn the_refusal_is_keyed_on_the_archetype_not_merely_on_a_populated_rendition_set() {
    // The SCOPE conjunct, isolated. `video_renditions` is a `pub` field on a `pub`
    // struct, and today only a video topic's demux ever fills it — which is
    // exactly why "non-empty" ALONE cannot carry the rule: the two agree on every
    // input the production path produces, so the sibling scope arm above (an
    // occupancy grid, whose set is empty) passes with the archetype check deleted.
    //
    // Written against a hand-built input the demux cannot mint precisely so the
    // stated scope is enforced rather than inferred from a coincidence.
    let mut grid = AttachedRender {
        topic: "/map".to_string(),
        entity: "world/map".to_string(),
        archetype: Some(ArchetypeKind::OccupancyGrid),
        producer_count: Some(1),
        video_renditions: Vec::new(),
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    };
    grid.video_renditions = vec!["640x360".to_string()];
    assert_eq!(
        views_for_render(&grid),
        vec![ViewKind::Spatial2d, ViewKind::TextDocument],
        "a non-video kind keeps its dump pane however its rendition set was filled"
    );
}

#[test]
fn an_operator_who_asked_for_the_dump_still_gets_it_on_a_decoding_video_topic() {
    // The choice OUTRANKS the refusal: `plan.dump` is what the operator
    // asked for, and dropping it would make `representation text`/`both` a silent
    // no-op on the one topic class the refusal touches.
    for choice in [Representation::Both, Representation::Text] {
        let mut topic = video_topic("/go2/camera/h264", &["640x360", "1280x720"]);
        topic.representation = choice;
        assert!(
            views_for_render(&topic).contains(&ViewKind::TextDocument),
            "{choice:?}: a FORCED dump must still get its pane"
        );
    }
}

// ===========================================================================
// 3. The PLUMB — the field must be POPULATED by the REAL worker, not inert
// ===========================================================================

/// The Go2's corrected `/frontvideostream` shape, used to build REAL
/// wire frames the production walker decodes on the worker thread.
const PROBE_MSG: &str = "\
uint64 time_frame
uint32 video_height
uint8[] video_data
";
const PROBE_QNAME: &str = "layout_probe/VideoProbe";

fn probe_schema() -> MessageSchema {
    parse_rosmsg(PROBE_MSG, "VideoProbe", Some("layout_probe")).expect("probe schema")
}

fn all_schemas() -> Vec<MessageSchema> {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas.push(probe_schema());
    schemas
}

/// A real `layout_probe/VideoProbe` wire frame carrying `au`.
fn build_probe_frame(time_frame: u64, video_height: u32, au: &[u8]) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    let layout = resolver.layout_of(PROBE_QNAME).expect("probe layout");
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let mut payload = vec![0u8; fixed + table];
    let off = |field: &str| -> usize {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == field)
            .expect("fixed field")
            .offset
    };
    let tf = off("time_frame");
    let vh = off("video_height");
    payload[tf..tf + 8].copy_from_slice(&time_frame.to_le_bytes());
    payload[vh..vh + 4].copy_from_slice(&video_height.to_le_bytes());
    write_offset_entry(
        &mut payload,
        fixed,
        0,
        (fixed + table) as u32,
        au.len() as u32,
    );
    payload.extend_from_slice(au);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: probe_schema().schema_hash(),
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns: 1_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn the_real_worker_mirrors_its_rendition_set_and_the_layout_picks_the_default_from_it() {
    // NO INERT SHIPPING, and the mutation target: this drives the REAL
    // `VizLogWorker` (its own thread, the real `dispatch_or_stage`, the real
    // `SinkState` demux) with REAL Go2 wire frames, then reads the mirror the way
    // the DAEMON does. Deleting the mirror at the worker's refresh point makes
    // this fail — nothing else in the suite would notice, because every other
    // layout assertion is pure over hand-built inputs.
    let input = "/go2/camera";
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("layout")
        .recording_id("worker_mirror")
        .memory()
        .expect("memory sink");
    let (walker, _warn) = FrameWalker::new(all_schemas());
    let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
    let counters = worker.counters();

    // Two REAL keyframes: the captured Go2 360p one, and a built 720p one.
    let key_360 = go2_360_keyframe();
    let key_720 = annex_b(&[&build_sps(80, 45), &[0x65, 0x77, 0xb8, 0x00]]);
    worker.try_enqueue(vec![InputFrames {
        name: input.to_string(),
        frames: vec![
            build_probe_frame(1, 360, &key_360),
            build_probe_frame(2, 720, &key_720),
        ],
    }]);
    worker.sync();

    // The DAEMON's read (`Ctx::video_renditions_for`), verbatim.
    let renditions: Vec<String> = counters
        .video_renditions
        .lock()
        .expect("lock")
        .get(input)
        .cloned()
        .unwrap_or_default();
    assert_eq!(
        renditions,
        vec!["640x360".to_string(), "1280x720".to_string()],
        "the WORKER must publish its demux's rendition set — without the mirror the \
         layout plane can never learn a topic is interleaved"
    );

    // …and the layout assembled from it shows ONE pane — the 720p rendition — in
    // BOTH producers. This is the reported live shape end to end: real frames in,
    // one video tile out.
    let attached = AttachedRender {
        topic: input.to_string(),
        entity: format!("world{input}"),
        archetype: Some(ArchetypeKind::VideoStream),
        producer_count: Some(1),
        video_renditions: renditions,
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    };
    let expected = vec!["world/go2/camera/viz-video/1280x720".to_string()];
    assert_eq!(
        spatial2d_origins(&default_layout(std::slice::from_ref(&attached)).root),
        expected,
        "default layout"
    );
    assert_eq!(
        spatial2d_origins(
            &compose_layout(&bare(vec![attached.clone()]))
                .expect("ok")
                .plan
                .root
        ),
        expected,
        "compose layout"
    );

    // …and no empty status tile beside it (the reported second pane), on a topic
    // whose decode was driven by REAL frames rather than a hand-set field.
    assert_eq!(
        views_for_render(&attached),
        vec![ViewKind::Spatial2d],
        "a topic the real worker decoded needs no dump pane"
    );
}

#[test]
fn a_single_rendition_worker_publishes_one_segment_and_the_layout_does_not_split() {
    // NO REGRESSION on the common case, through the same REAL worker: one rendition
    // ⇒ one mirrored segment ⇒ the topic keeps its single topic-rooted view.
    let input = "/cam/h264";
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("layout")
        .recording_id("worker_single")
        .memory()
        .expect("memory sink");
    let (walker, _warn) = FrameWalker::new(all_schemas());
    let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
    let counters = worker.counters();

    let key_360 = go2_360_keyframe();
    worker.try_enqueue(vec![InputFrames {
        name: input.to_string(),
        frames: vec![build_probe_frame(1, 360, &key_360)],
    }]);
    worker.sync();

    let renditions: Vec<String> = counters
        .video_renditions
        .lock()
        .expect("lock")
        .get(input)
        .cloned()
        .unwrap_or_default();
    assert_eq!(renditions, vec!["640x360".to_string()]);

    let attached = AttachedRender {
        topic: input.to_string(),
        entity: format!("world{input}"),
        archetype: Some(ArchetypeKind::VideoStream),
        producer_count: Some(1),
        video_renditions: renditions,
        representation: Representation::Auto,
        render_proof: RenderProof::default(),
    };
    assert_eq!(
        spatial2d_origins(&default_layout(&[attached]).root),
        vec!["world/cam/h264".to_string()],
        "one rendition keeps the topic-rooted view"
    );
}

#[test]
fn a_non_video_worker_publishes_no_renditions() {
    // ANTI-TAUTOLOGY for the mirror: a topic the demux never opened a sub-stream
    // for contributes NOTHING, so a non-video topic can never be split.
    let input = "/not/video";
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("layout")
        .recording_id("worker_nonvideo")
        .memory()
        .expect("memory sink");
    let (walker, _warn) = FrameWalker::new(all_schemas());
    let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
    let counters = worker.counters();

    let junk: &[u8] = &[0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x11, 0x22, 0x33];
    worker.try_enqueue(vec![InputFrames {
        name: input.to_string(),
        frames: vec![build_probe_frame(7, 0, junk)],
    }]);
    worker.sync();

    assert!(
        !counters
            .video_renditions
            .lock()
            .expect("lock")
            .contains_key(input),
        "a non-video topic must publish no rendition set"
    );
}

// ===========================================================================
// 4. The render-proof fix — the LAYOUT-SIGNAL generation
// ===========================================================================

/// The mid-GOP camera attach, which is what makes the two live layout signals
/// come apart — and the reason the drain loop cannot watch the render-proof map
/// alone.
///
/// `render_is_proven` reads the RENDITION SET for `VideoStream` and the render
/// proof for every other archetype. On a real camera attach the first access unit
/// is almost never an IDR (one keyframe per 30-60 frame GOP at ~30 fps), so the
/// worker takes `VideoRoute::Drop(BeforeKeyframe)`, which is not a degradation —
/// `note_render_native` runs and the proof reaches its TERMINAL value
/// (`{rendered_without_dumping: true, degraded: false}`, both flags sticky) a
/// second or more BEFORE the SPS opens the first rendition. So across the ONE
/// transition the video layout decision turns on, the proof map is byte-identical
/// and only the rendition set moves.
///
/// The generation must therefore move too, and that is what this pins: the two
/// readings measured on a rendition-only transition
/// (`proof_changed_on_the_flip = false`, `views_changed_on_the_flip = true`),
/// plus the generation that carries the change across.
#[test]
fn a_rendition_only_transition_bumps_the_layout_signal_generation() {
    let input = "/go2/camera/h264";
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("layout_signal")
        .recording_id("midgop")
        .memory()
        .expect("memory sink");
    let (walker, _warn) = FrameWalker::new(all_schemas());
    let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
    let counters = worker.counters();

    let read_proofs = || counters.render_proofs.lock().expect("lock").clone();
    let read_renditions = || counters.video_renditions.lock().expect("lock").clone();
    let read_generation = || {
        counters
            .layout_signal_generation
            .load(std::sync::atomic::Ordering::Relaxed)
    };
    let views_now = || {
        views_for_render(&AttachedRender {
            topic: input.to_string(),
            entity: format!("world{input}"),
            archetype: Some(ArchetypeKind::VideoStream),
            producer_count: Some(1),
            video_renditions: read_renditions().get(input).cloned().unwrap_or_default(),
            representation: Representation::Auto,
            render_proof: read_proofs().get(input).copied().unwrap_or_default(),
        })
    };

    // BATCH 1 — a mid-GOP access unit: one non-IDR slice NAL, NO parameter sets.
    worker.try_enqueue(vec![InputFrames {
        name: input.to_string(),
        frames: vec![build_probe_frame(
            1,
            360,
            &annex_b(&[&[0x41, 0x9a, 0x00, 0x20]]),
        )],
    }]);
    worker.sync();

    let proofs_midgop = read_proofs();
    let generation_midgop = read_generation();
    assert!(
        read_renditions()
            .get(input)
            .map(|r| r.is_empty())
            .unwrap_or(true),
        "PRECONDITION: no parameter set has been seen, so no rendition may exist — \
         got {:?}",
        read_renditions()
    );
    assert_eq!(
        proofs_midgop.get(input).copied(),
        Some(RenderProof {
            rendered_without_dumping: true,
            degraded: false,
        }),
        "PRECONDITION: a BeforeKeyframe drop is not a degradation, so the proof \
         reaches its terminal value here — if this stops holding, the flip below is \
         no longer rendition-only and this test proves nothing"
    );
    assert!(
        generation_midgop > 0,
        "the first signal any topic produces is itself a change"
    );
    assert_eq!(
        views_now(),
        vec![ViewKind::Spatial2d, ViewKind::TextDocument],
        "with no rendition the video render is NOT proven, so the companion pane \
         is KEPT — the state the flip below leaves"
    );

    // BATCH 2 — the REAL Go2 360p keyframe. The demux opens its sub-stream, the
    // rendition set goes non-empty, and the layout decision flips.
    worker.try_enqueue(vec![InputFrames {
        name: input.to_string(),
        frames: vec![build_probe_frame(2, 360, &go2_360_keyframe())],
    }]);
    worker.sync();

    assert_eq!(
        read_renditions().get(input).cloned(),
        Some(vec!["640x360".to_string()]),
        "the keyframe must open a rendition"
    );
    assert_eq!(
        read_proofs(),
        proofs_midgop,
        "THE POINT: the render-proof map is BYTE-IDENTICAL across this transition, \
         so a watcher that compares only the proofs is blind to it"
    );
    assert_eq!(
        views_now(),
        vec![ViewKind::Spatial2d],
        "…while the layout decision DID change — the companion is now refused"
    );
    assert!(
        read_generation() > generation_midgop,
        "so the layout-signal generation MUST carry the change: {} -> {}",
        generation_midgop,
        read_generation()
    );
}

/// The other half of the boundedness claim: a signal that did NOT change must not
/// bump the generation, however many batches are drained.
///
/// Without it the generation could be "bump every batch", which reflows the whole
/// default layout at frame rate — the shape `Ctx::poll_layout_signal_reflows`'s
/// doc calls BOUNDED, and which only this arm asserts.
#[test]
fn a_steady_stream_of_identical_signals_never_bumps_the_generation() {
    let input = "/cam/steady";
    let (rec, _storage) = rerun::RecordingStreamBuilder::new("layout_signal")
        .recording_id("steady")
        .memory()
        .expect("memory sink");
    let (walker, _warn) = FrameWalker::new(all_schemas());
    let mut worker = VizLogWorker::spawn(rec, walker, SinkState::new()).expect("spawn worker");
    let counters = worker.counters();
    let read_generation = || {
        counters
            .layout_signal_generation
            .load(std::sync::atomic::Ordering::Relaxed)
    };

    // Settle: one keyframe opens the rendition and latches the proof.
    worker.try_enqueue(vec![InputFrames {
        name: input.to_string(),
        frames: vec![build_probe_frame(1, 360, &go2_360_keyframe())],
    }]);
    worker.sync();
    let settled = read_generation();
    assert!(settled > 0, "the settling batch itself is a change");

    // …then 40 more batches of the SAME keyframe. Both flags are sticky and the
    // rendition set only grows, so nothing here is new.
    for seq in 0..40u64 {
        worker.try_enqueue(vec![InputFrames {
            name: input.to_string(),
            frames: vec![build_probe_frame(2 + seq, 360, &go2_360_keyframe())],
        }]);
    }
    worker.sync();
    assert_eq!(
        read_generation(),
        settled,
        "a batch that changes NEITHER mirror must not bump the generation — \
         otherwise the daemon re-derives and re-sends the whole default layout at \
         frame rate"
    );
}
