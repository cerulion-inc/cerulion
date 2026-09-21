// SPDX-License-Identifier: AGPL-3.0-only
//! ON: the tier table's own sizing rule, MEASURED.
//!
//! # Why a measurement and not another literal
//!
//! `variable_schema_max_slice_len` states its placement rule in one place:
//!
//! > **Worst case × 4 picks the tier.** The realistic maximum payload for the
//! > type, times four for headroom, must fit; the next tier down must not.
//!
//! Every arm's comment then applies that rule with a byte count. Those byte
//! counts were computed once, by hand, by whoever wrote the arm — and nothing
//! re-checks them. `max_slice_len_test.rs` pins the ANSWER (the tier), which
//! catches a retier but not a wrong premise; a type whose real envelope is 10×
//! what its comment claims passes every existing test in this repo.
//!
//! This file closes that: it rebuilds each scenario named in an arm's comment
//! as REAL BYTES and asserts the rule against the shipped tier — so a wrong
//! number in a comment, or a schema edit that changes an envelope, fails here
//! rather than on a robot.
//!
//! # The envelope is built by the production encoder, not modelled
//!
//! [`envelope`] assembles each frame with [`CanonicalBodyBuilder`] — the same
//! type `CdrCodec::decode` uses to build a bridged ingress frame, and the same
//! canonical element framing the `FrameWalker` decodes. So the number
//! is what the wire carries, not a re-implementation of the layout rules that
//! could drift into agreeing with itself.
//!
//! One deliberate difference from the generated SHM writer: the canonical
//! builder pads each variable payload to its natural alignment (up to 7 bytes
//! per field), where the SHM writer packs at the running cursor. The canonical
//! form is therefore the LARGER of the two, which is the right side to be on
//! for a capacity bound — and it is the exact form on the ingress path, which
//! is the path where the tier decides queue depth.
//!
//! # Two bases, because the table has two rules
//!
//! An arm is placed by SIZE (rule 1: worst × 4 fits, the next tier down does
//! not) or by FLOOR (rule 2: a container is at least its largest variable
//! member, so its own size would have allowed something smaller). Each row
//! below declares which, and gets the matching assertion — a FLOOR row is
//! additionally required to sit EXACTLY at its floor, so "we rounded up for
//! comfort" cannot hide behind rule 2.
//!
//! # Scope
//!
//! The rows are the arms this follow-on added or moved: the 24 catch-all
//! schemas and `control_msgs/JointTrajectoryControllerState`. The container
//! rule itself is checked over the WHOLE corpus by
//! `tier_container_rule_test.rs`; the tier VALUES by `tier_catch_all_test.rs`.

use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::{
    parse_rosmsg, resolve_fixed_nested, variable_schema_max_slice_len, CanonicalBodyBuilder,
    FieldType, MessageSchema,
};
use cerulion_core::wire::WireHeader;
use std::collections::BTreeMap;
use std::path::PathBuf;

const HUGE: usize = 128 * 1024 * 1024;
const LARGE: usize = 16 * 1024 * 1024;
const MEDIUM: usize = 4 * 1024 * 1024;
const SMALL: usize = 256 * 1024;
const TINY: usize = 16 * 1024;

/// The tier immediately below `t` — the one rule 1 requires NOT to fit.
fn next_lower(t: usize) -> Option<usize> {
    match t {
        HUGE => Some(LARGE),
        LARGE => Some(MEDIUM),
        MEDIUM => Some(SMALL),
        SMALL => Some(TINY),
        TINY => None,
        other => panic!("{other} is not one of the five tiers"),
    }
}

// ───────────────────────────── corpus + envelope ─────────────────────────────

/// Every vendored `.msg`, parsed and nested-resolved as one set — the shape
/// `build.rs` compiles, so "variable" here means what it means at codegen time.
fn corpus() -> Vec<MessageSchema> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("msg");
    let mut pkgs: Vec<PathBuf> = std::fs::read_dir(&root)
        .unwrap_or_else(|e| panic!("read {}: {e}", root.display()))
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir())
        .collect();
    pkgs.sort();

    let mut schemas = Vec::new();
    for pkg_dir in pkgs {
        let pkg = pkg_dir
            .file_name()
            .expect("package dir name")
            .to_string_lossy()
            .replace('-', "_");
        let mut files: Vec<PathBuf> = std::fs::read_dir(&pkg_dir)
            .unwrap_or_else(|e| panic!("read {}: {e}", pkg_dir.display()))
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "msg"))
            .collect();
        files.sort();
        for f in files {
            let name = f
                .file_stem()
                .expect("msg file stem")
                .to_string_lossy()
                .to_string();
            let text = std::fs::read_to_string(&f).unwrap_or_else(|e| panic!("read {f:?}: {e}"));
            // FAIL CLOSED: a file this walk cannot parse is a file whose
            // envelope goes unmeasured.
            let schema = parse_rosmsg(&text, &name, Some(&pkg))
                .unwrap_or_else(|e| panic!("cannot parse vendored {pkg}/{name}: {e}"));
            schemas.push(schema);
        }
    }
    resolve_fixed_nested(&mut schemas);
    schemas
}

/// How full each array is and how long each string is, for ONE scenario.
///
/// Counts are keyed `"pkg/Type.field"` so a nested type can be populated
/// differently from its container (an autoware object carries six predicted
/// paths of fifty poses each, not six of six). A field with no entry is EMPTY —
/// silence means zero, never "some default", so nothing rides in unnamed.
#[derive(Clone)]
struct Scenario {
    counts: BTreeMap<&'static str, usize>,
    strlen: usize,
}

impl Scenario {
    fn new(strlen: usize, counts: &[(&'static str, usize)]) -> Self {
        Self {
            counts: counts.iter().copied().collect(),
            strlen,
        }
    }
    fn n(&self, qualified: &str, field: &str) -> usize {
        *self
            .counts
            .get(format!("{qualified}.{field}").as_str())
            .unwrap_or(&0)
    }
}

fn prim_size(ft: &FieldType) -> Option<usize> {
    Some(match ft {
        FieldType::I8 | FieldType::U8 | FieldType::Bool | FieldType::Bytes => 1,
        FieldType::I16 | FieldType::U16 => 2,
        FieldType::I32 | FieldType::U32 | FieldType::F32 => 4,
        FieldType::I64 | FieldType::U64 | FieldType::F64 => 8,
        _ => return None,
    })
}

struct Corpus {
    resolver: LayoutResolver,
    variable: BTreeMap<String, bool>,
}

impl Corpus {
    fn load() -> Self {
        let schemas = corpus();
        let variable = schemas
            .iter()
            .map(|s| (s.qualified_name(), !s.is_definitely_fixed()))
            .collect();
        let (resolver, warnings) = LayoutResolver::new(schemas);
        assert!(
            warnings.is_empty(),
            "layout resolution warned, so the envelopes below are not trustworthy: {warnings:?}"
        );
        Self { resolver, variable }
    }

    /// Resolve a nested reference the way `resolve_fixed_nested` does: the
    /// corpus is uniformly qualified (held at zero by
    /// `upstream_drift_test::the_vendored_corpus_declares_no_bare_nested_refs`),
    /// with a same-package then `std_msgs/Header` fallback for the bare case.
    fn resolve(&mut self, pkg: &Option<String>, name: &str, parent_pkg: &str) -> String {
        if let Some(p) = pkg {
            return format!("{p}/{name}");
        }
        let same = format!("{parent_pkg}/{name}");
        if self.resolver.layout_of(&same).is_some() {
            return same;
        }
        let std = format!("std_msgs/{name}");
        assert!(
            self.resolver.layout_of(&std).is_some(),
            "cannot resolve bare nested reference `{name}` from {parent_pkg}"
        );
        std
    }

    fn is_variable(&self, qualified: &str) -> bool {
        *self
            .variable
            .get(qualified)
            .unwrap_or_else(|| panic!("{qualified} is not in the vendored corpus"))
    }

    /// The canonical body for one variable field's payload.
    fn payload(
        &mut self,
        container: &str,
        parent_pkg: &str,
        field: &str,
        ft: &FieldType,
        sc: &Scenario,
    ) -> Vec<u8> {
        match ft {
            FieldType::String => vec![b'x'; sc.strlen],
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let q = self.resolve(package, schema_name, parent_pkg);
                self.body(&q, sc)
            }
            FieldType::DynamicArray { element_type } => {
                let n = sc.n(container, field);
                match element_type.as_ref() {
                    // `string[]`: u32 count, then u32 len + UTF-8 each.
                    FieldType::String => {
                        let mut v = (n as u32).to_le_bytes().to_vec();
                        for _ in 0..n {
                            v.extend_from_slice(&(sc.strlen as u32).to_le_bytes());
                            v.extend(std::iter::repeat_n(b'x', sc.strlen));
                        }
                        v
                    }
                    e if prim_size(e).is_some() => {
                        vec![0u8; n * prim_size(e).expect("checked by the guard")]
                    }
                    FieldType::Nested {
                        schema_name,
                        package,
                        ..
                    } => {
                        let q = self.resolve(package, schema_name, parent_pkg);
                        if self.is_variable(&q) {
                            // Canonical variable element: u32 count, then u32 len
                            // + the element's headerless sub-frame each.
                            let mut v = (n as u32).to_le_bytes().to_vec();
                            for _ in 0..n {
                                let body = self.body(&q, sc);
                                v.extend_from_slice(&(body.len() as u32).to_le_bytes());
                                v.extend_from_slice(&body);
                            }
                            v
                        } else {
                            // Canonical recursively-FIXED element: back-to-back
                            // fixed sections at the padded stride, no count.
                            let l = self
                                .resolver
                                .layout_of(&q)
                                .unwrap_or_else(|| panic!("layout {q}"));
                            let stride = l.fixed_size.next_multiple_of(l.fixed_align.max(1));
                            vec![0u8; n * stride]
                        }
                    }
                    // FAIL CLOSED: an element shape this file does not model is
                    // an envelope it would silently under-count.
                    other => panic!("{container}.{field}: unmodelled array element {other:?}"),
                }
            }
            other => panic!("{container}.{field}: unmodelled variable field {other:?}"),
        }
    }

    /// The canonical body for one message, populated per `sc`.
    fn body(&mut self, qualified: &str, sc: &Scenario) -> Vec<u8> {
        let layout = self
            .resolver
            .layout_of(qualified)
            .unwrap_or_else(|| panic!("layout {qualified}"));
        let parent_pkg = qualified
            .split_once('/')
            .unwrap_or_else(|| panic!("{qualified} is not package-qualified"))
            .0
            .to_string();
        let fields: Vec<(String, FieldType)> = layout
            .variable_fields
            .iter()
            .map(|v| (v.name.clone(), v.field_type.clone()))
            .collect();

        let mut payloads = Vec::with_capacity(fields.len());
        for (name, ft) in &fields {
            payloads.push(self.payload(qualified, &parent_pkg, name, ft, sc));
        }

        let layout = self
            .resolver
            .layout_of(qualified)
            .expect("layout resolved above");
        let mut builder = CanonicalBodyBuilder::new(&layout);
        for p in payloads {
            builder.push_variable(p).expect("one payload per field");
        }
        builder.finish().expect("canonical body")
    }

    /// The whole wire frame: 32-byte `WireHeader` + canonical body. This is the
    /// number `MAX_SLICE_LEN` must cover.
    fn envelope(&mut self, qualified: &str, sc: &Scenario) -> usize {
        WireHeader::SIZE + self.body(qualified, sc).len()
    }

    /// The container floor: the largest tier among a schema's DIRECT variable
    /// members, or `None` when it embeds none.
    fn floor(&mut self, qualified: &str) -> Option<usize> {
        let layout = self
            .resolver
            .layout_of(qualified)
            .unwrap_or_else(|| panic!("layout {qualified}"));
        let parent_pkg = qualified.split_once('/').expect("qualified").0.to_string();
        let types: Vec<FieldType> = layout
            .variable_fields
            .iter()
            .map(|v| v.field_type.clone())
            .collect();

        let mut floor = None;
        for ft in types {
            let mut cursor = &ft;
            while let FieldType::DynamicArray { element_type }
            | FieldType::FixedArray { element_type, .. } = cursor
            {
                cursor = element_type;
            }
            if let FieldType::Nested {
                schema_name,
                package,
                ..
            } = cursor
            {
                let q = self.resolve(package, schema_name, &parent_pkg);
                if self.is_variable(&q) {
                    let t = variable_schema_max_slice_len(&q);
                    floor = Some(floor.map_or(t, |f: usize| f.max(t)));
                }
            }
        }
        floor
    }
}

// ────────────────────────────────── the rows ──────────────────────────────────

/// Why a row sits where it does — the table has two rules, and a row must
/// declare which one it is claiming.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Basis {
    /// Rule 1: worst × 4 fits this tier, and does NOT fit the next one down.
    Size,
    /// Rule 2: the container floor forced it here. Its own worst × 4 would have
    /// fitted something smaller, and the tier must equal the floor EXACTLY.
    Floor,
    /// Rule 1's LOWER bound is deliberately waived: the realistic frame × 4
    /// would fit a smaller tier, but that smaller tier hard-fails a CONFORMING
    /// producer, so the arm takes the larger one.
    ///
    /// This is a real escape from the table's own rule and it is granted to
    /// exactly one arm, so the row does not get to assert its own
    /// justification: it carries the name of the test that PROVES the smaller
    /// tier is unsafe, and that test is required to exist and to be about this
    /// schema. Without that, `Ceiling` would be a way to mark any over-tier as
    /// intentional.
    Ceiling { proved_by: &'static str },
}

struct Row {
    schema: &'static str,
    basis: Basis,
    /// The scenario the arm's comment names as the realistic worst case.
    scenario: Scenario,
    /// What that scenario is, for the failure message.
    what: &'static str,
}

fn rows() -> Vec<Row> {
    use Basis::{Ceiling, Floor, Size};

    // Shorthands for the nested populations a scenario needs to reach down into.
    const POSE: &str = "autoware_perception_msgs/PredictedPath.path";
    const PATHS: &str = "autoware_perception_msgs/PredictedObjectKinematics.predicted_paths";
    const CLASSES: &str = "autoware_perception_msgs/PredictedObject.classification";
    const FOOTPRINT: &str = "geometry_msgs/Polygon.points";
    const PRIMS: &str = "autoware_planning_msgs/LaneletSegment.primitives";

    vec![
        // ── autoware_perception_msgs ──
        Row {
            schema: "autoware_perception_msgs/PredictedObjects",
            // Rule 1 on the REALISTIC frame would say LARGE (1,851,693 x 4 =
            // 7,406,772, which fits 16 MiB). The arm takes HUGE anyway because
            // LARGE hard-fails at 29 objects filled to the IDL's own per-object
            // bound — inside the object count a dense urban scene reaches. That
            // is the one waiver of rule 1's lower bound in this table, and it
            // is proved, not asserted.
            basis: Ceiling {
                proved_by: "large_is_not_a_safe_home_for_predicted_objects",
            },
            scenario: Scenario::new(
                25,
                &[
                    ("autoware_perception_msgs/PredictedObjects.objects", 100),
                    (PATHS, 6),
                    (POSE, 50),
                    (CLASSES, 8),
                    (FOOTPRINT, 32),
                ],
            ),
            what: "a dense-urban frame: 100 objects, 6 predicted paths x 50 poses each",
        },
        Row {
            schema: "autoware_perception_msgs/PredictedObject",
            basis: Size,
            scenario: Scenario::new(
                25,
                &[(PATHS, 100), (POSE, 100), (CLASSES, 8), (FOOTPRINT, 100)],
            ),
            what: "the IDL maximum: PredictedPath[<=100] x Pose[<=100]",
        },
        Row {
            schema: "autoware_perception_msgs/PredictedObjectKinematics",
            basis: Size,
            scenario: Scenario::new(25, &[(PATHS, 100), (POSE, 100)]),
            what: "the IDL maximum: PredictedPath[<=100] x Pose[<=100]",
        },
        Row {
            schema: "autoware_perception_msgs/PredictedPath",
            basis: Size,
            scenario: Scenario::new(25, &[(POSE, 100)]),
            what: "the IDL maximum: Pose[<=100] over a 56 B fixed element",
        },
        Row {
            schema: "autoware_perception_msgs/Shape",
            basis: Floor,
            scenario: Scenario::new(25, &[(FOOTPRINT, 100)]),
            what: "a 100-vertex object footprint polygon",
        },
        // ── autoware_planning_msgs ──
        Row {
            schema: "autoware_planning_msgs/LaneletRoute",
            basis: Size,
            scenario: Scenario::new(
                25,
                &[
                    ("autoware_planning_msgs/LaneletRoute.segments", 500),
                    (PRIMS, 4),
                ],
            ),
            what: "a long urban route: 500 segments x 4 candidate primitives",
        },
        Row {
            schema: "autoware_planning_msgs/Trajectory",
            basis: Size,
            scenario: Scenario::new(25, &[("autoware_planning_msgs/Trajectory.points", 10_000)]),
            what: "the IDL maximum: TrajectoryPoint[<=10000] over an 88 B fixed element",
        },
        Row {
            schema: "autoware_planning_msgs/LaneletSegment",
            basis: Size,
            scenario: Scenario::new(25, &[(PRIMS, 20)]),
            what: "20 candidate primitives at one segment (real roads carry <= 8 lanes)",
        },
        Row {
            schema: "autoware_planning_msgs/LaneletPrimitive",
            basis: Size,
            scenario: Scenario::new(25, &[]),
            what: "an int64 id and a short primitive-type string",
        },
        // ── grid_map_msgs ──
        Row {
            schema: "grid_map_msgs/GridMap",
            basis: Size,
            scenario: Scenario::new(
                25,
                &[
                    ("grid_map_msgs/GridMap.layers", 8),
                    ("grid_map_msgs/GridMap.basic_layers", 8),
                    ("grid_map_msgs/GridMap.data", 8),
                    ("std_msgs/Float32MultiArray.data", 1_000_000),
                    ("std_msgs/MultiArrayLayout.dim", 2),
                ],
            ),
            what: "an 8-layer 100 m elevation map at 10 cm (1000x1000 cells)",
        },
        // ── radar_msgs ──
        Row {
            schema: "radar_msgs/RadarScan",
            basis: Size,
            scenario: Scenario::new(25, &[("radar_msgs/RadarScan.returns", 30_000)]),
            what: "a 4D imaging radar frame: 30,000 detections",
        },
        Row {
            schema: "radar_msgs/RadarTracks",
            basis: Size,
            scenario: Scenario::new(25, &[("radar_msgs/RadarTracks.tracks", 255)]),
            what: "a full sensor track table: 255 tracks",
        },
        // ── vision_msgs ──
        Row {
            schema: "vision_msgs/Detection3DArray",
            basis: Size,
            scenario: Scenario::new(
                32,
                &[
                    ("vision_msgs/Detection3DArray.detections", 500),
                    ("vision_msgs/Detection3D.results", 1),
                ],
            ),
            what: "the nuScenes per-frame cap: 500 detections x 1 hypothesis",
        },
        Row {
            schema: "vision_msgs/Detection2DArray",
            basis: Size,
            scenario: Scenario::new(
                32,
                &[
                    ("vision_msgs/Detection2DArray.detections", 300),
                    ("vision_msgs/Detection2D.results", 1),
                ],
            ),
            what: "YOLO's max_det default: 300 detections x 1 hypothesis",
        },
        Row {
            schema: "vision_msgs/LabelInfo",
            basis: Size,
            scenario: Scenario::new(64, &[("vision_msgs/LabelInfo.class_map", 1_000)]),
            what: "a 1,000-class label map with 64-character class names",
        },
        Row {
            schema: "vision_msgs/Classification",
            basis: Size,
            scenario: Scenario::new(32, &[("vision_msgs/Classification.results", 1_000)]),
            what: "a full 1,000-class softmax dump",
        },
        Row {
            schema: "vision_msgs/BoundingBox2DArray",
            basis: Size,
            scenario: Scenario::new(25, &[("vision_msgs/BoundingBox2DArray.boxes", 1_000)]),
            what: "1,000 boxes over a 40 B fixed element",
        },
        Row {
            schema: "vision_msgs/BoundingBox3DArray",
            basis: Size,
            scenario: Scenario::new(25, &[("vision_msgs/BoundingBox3DArray.boxes", 500)]),
            what: "the nuScenes per-frame cap: 500 boxes over an 80 B fixed element",
        },
        Row {
            schema: "vision_msgs/Detection2D",
            basis: Size,
            scenario: Scenario::new(32, &[("vision_msgs/Detection2D.results", 100)]),
            what: "100 class hypotheses on one detection (standard detectors emit 1)",
        },
        Row {
            schema: "vision_msgs/Detection3D",
            basis: Size,
            scenario: Scenario::new(32, &[("vision_msgs/Detection3D.results", 100)]),
            what: "100 class hypotheses on one detection",
        },
        Row {
            schema: "vision_msgs/ObjectHypothesisWithPose",
            basis: Floor,
            scenario: Scenario::new(32, &[]),
            what: "a 32-character class id, a score, and a fixed PoseWithCovariance",
        },
        Row {
            schema: "vision_msgs/ObjectHypothesis",
            basis: Size,
            scenario: Scenario::new(32, &[]),
            what: "a 32-character class id and a score",
        },
        Row {
            schema: "vision_msgs/VisionClass",
            basis: Size,
            scenario: Scenario::new(64, &[]),
            what: "a uint16 and a 64-character class name",
        },
        Row {
            schema: "vision_msgs/VisionInfo",
            basis: Size,
            scenario: Scenario::new(512, &[]),
            what: "512-character method and database-location strings",
        },
        // ── the other follow-on ──
        Row {
            schema: "control_msgs/JointTrajectoryControllerState",
            basis: Size,
            scenario: Scenario::new(25, &[]).with_jtcs(60),
            what: "a 60-DOF humanoid whole-body controller group",
        },
    ]
}

// ────────────────────────────────── the gate ──────────────────────────────────

#[test]
fn every_followon_arm_satisfies_the_tier_tables_own_sizing_rule() {
    let mut corpus = Corpus::load();
    let rows = rows();

    // Anti-tautology: an empty (or silently shrunken) row set would pass every
    // assertion below. 25 = the 24 catch-all schemas + JTCS.
    assert_eq!(
        rows.len(),
        25,
        "the follow-on moved 25 arms (24 catch-all schemas + JointTrajectoryControllerState); \
         a row set of {} means a row was dropped and its arithmetic is now unchecked",
        rows.len()
    );

    let mut report = Vec::new();
    for row in &rows {
        // The SHIPPED tier is the only source. A row deliberately does NOT
        // restate it: an expected-tier column here would duplicate the literal
        // pins in `tier_catch_all_test.rs` and, worse, SHADOW the rule — a
        // retier would fail on the duplication before the rule was consulted,
        // and a retier that updated both in lockstep would then face no rule
        // check at all. Reading the table means every placement is judged by
        // the rule, always.
        let tier = variable_schema_max_slice_len(row.schema);

        let bytes = corpus.envelope(row.schema, &row.scenario);
        let with_headroom = bytes * 4;
        report.push(format!(
            "  {:52} {:>10} B x4 = {:>11} B  ({})",
            row.schema, bytes, with_headroom, row.what
        ));

        // BOTH bases require the worst case itself to fit. A tier that cannot
        // hold one message of the type is a hard `PayloadTooLarge`, whatever
        // rule put it there.
        assert!(
            with_headroom <= tier,
            "{}: {} measures {bytes} B, and {bytes} x 4 = {with_headroom} does NOT fit its tier \
             ({}). The tier table's rule 1 requires the realistic worst case plus 4x headroom to \
             fit — either the arm is under-tiered (a reachable PayloadTooLarge; `Static` pools \
             cannot grow) or this row's scenario is no longer the realistic worst.",
            row.schema,
            row.what,
            tier
        );

        match row.basis {
            Basis::Size => {
                // The other half of rule 1, and the half that actually stops
                // drift: without it, every arm could be raised a tier "for
                // safety" and nothing would notice.
                if let Some(lower) = next_lower(tier) {
                    assert!(
                        with_headroom > lower,
                        "{}: {} measures {bytes} B, and {bytes} x 4 = {with_headroom} ALSO fits \
                         the next tier down ({lower}). Rule 1 says the next tier down must not \
                         fit, so this arm is over-tiered — it is reserving a tier it does not \
                         need and, below MEDIUM, giving up 4x of ingress queue depth for it. \
                         Move it down, or mark this row `Basis::Floor` if a container member is \
                         what actually holds it up.",
                        row.schema,
                        row.what
                    );
                }
            }
            Basis::Ceiling { proved_by } => {
                // The row cannot certify its own escape. All it does here is
                // name the proof and require it to be a real test in this file
                // about this schema; the proof itself runs separately.
                let src = include_str!("tier_capacity_test.rs");
                assert!(
                    src.contains(&format!("fn {proved_by}(")),
                    "{}: marked `Basis::Ceiling`, whose escape from rule 1 is only granted \
                     against a proof — but `{proved_by}` is not a test in this file",
                    row.schema
                );
                let body = src
                    .split_once(&format!("fn {proved_by}("))
                    .expect("checked above")
                    .1;
                assert!(
                    body.contains(row.schema),
                    "{}: its `Basis::Ceiling` proof `{proved_by}` never mentions this schema, so \
                     it is not proving anything about it",
                    row.schema
                );
            }
            Basis::Floor => {
                let floor = corpus.floor(row.schema).unwrap_or_else(|| {
                    panic!(
                        "{}: marked `Basis::Floor`, but it embeds NO variable member, so there is \
                         no floor to be held up by — it must be justified by size",
                        row.schema
                    )
                });
                assert_eq!(
                    tier, floor,
                    "{}: marked `Basis::Floor`, so it must sit EXACTLY at its container floor \
                     ({floor} — the largest tier among its variable members). It ships at \
                     {tier}. Rule 2 licenses a container to be as large as its biggest member, \
                     not larger; a tier above the floor needs a size argument, which means this \
                     row should be `Basis::Size`.",
                    row.schema
                );
            }
        }
    }

    // Printed so a reader of a CI log can see the numbers the arms claim,
    // rather than only learning that they held.
    println!("follow-on measured envelopes:\n{}", report.join("\n"));
}

/// The one place the table takes a tier its realistic-frame arithmetic alone
/// would not justify, proved rather than asserted.
///
/// `PredictedObjects` sits at HUGE. A realistic dense-urban frame × 4 says
/// LARGE, and the row above only checks that such a frame fits. The reason the
/// arm goes further is that LARGE hard-fails well inside the object count a
/// dense scene reaches, once each object is filled to the IDL's OWN per-object
/// bound — which a conforming `map_based_prediction` may do. That claim is
/// arithmetic on a measured per-object envelope, so it is measured here.
#[test]
fn large_is_not_a_safe_home_for_predicted_objects() {
    let mut corpus = Corpus::load();

    // One object at the IDL maximum its own message declares.
    let at_idl_max = Scenario::new(
        25,
        &[
            (
                "autoware_perception_msgs/PredictedObjectKinematics.predicted_paths",
                100,
            ),
            ("autoware_perception_msgs/PredictedPath.path", 100),
            ("autoware_perception_msgs/PredictedObject.classification", 8),
            ("geometry_msgs/Polygon.points", 100),
        ],
    );
    let per_object = corpus.envelope("autoware_perception_msgs/PredictedObject", &at_idl_max);

    // Elements ride the container's payload, so the object count a container
    // tier admits is bounded above by tier / per-object (the count prefix and
    // per-element length words only make it smaller).
    let at_large = LARGE / per_object;
    let at_huge = HUGE / per_object;

    assert!(
        at_large < 50,
        "LARGE would admit {at_large} IDL-maximum objects. The arm's justification for taking \
         HUGE is that LARGE fails inside the object count a dense urban scene reaches; if LARGE \
         now holds 50+, that justification is gone and PredictedObjects should be re-audited."
    );
    assert!(
        at_huge >= 200,
        "HUGE admits only {at_huge} IDL-maximum objects, which is not the comfortable ceiling the \
         arm claims. Re-audit."
    );
    assert_eq!(
        variable_schema_max_slice_len("autoware_perception_msgs/PredictedObjects"),
        HUGE,
        "PredictedObjects keeps 128 MiB deliberately — see its arm"
    );

    println!(
        "PredictedObject at its IDL maximum = {per_object} B; LARGE admits {at_large}, \
         HUGE admits {at_huge}"
    );
}

/// The breakage boundaries the arms state, SEARCHED and pinned.
///
/// A tier move is a promise about what a publisher can still fit, and each arm
/// spends a paragraph on where that stops. Those numbers are what an operator
/// reads before deciding whether they need `max_slice_len:` on a topic, so they
/// are not described here — they are found by bisection over real encoded
/// frames and compared to the number the arm states.
///
/// Bisection rather than a two-sided spot check because the failure message
/// then carries the RIGHT answer. A boundary that drifts is a doc bug either
/// way, but one that tells you the correct number gets fixed correctly.
///
/// Each boundary is stated for the string length its row names, because these
/// messages carry strings and the boundary moves with them. The `chars` column
/// below is part of the claim, not a test detail.
#[test]
fn the_stated_breakage_boundaries_hold_where_the_arms_say_they_do() {
    let mut corpus = Corpus::load();

    struct Boundary {
        schema: &'static str,
        tier: usize,
        /// The last element count that still fits, per the arm's comment.
        stated: usize,
        unit: &'static str,
        /// String length the boundary is quoted at.
        chars: usize,
        build: fn(usize, usize) -> Scenario,
    }

    let boundaries = [
        Boundary {
            schema: "control_msgs/JointTrajectoryControllerState",
            tier: SMALL,
            stated: 1_666,
            unit: "joints",
            chars: 25,
            build: |c, n| Scenario::new(c, &[]).with_jtcs(n),
        },
        Boundary {
            schema: "radar_msgs/RadarTracks",
            tier: SMALL,
            stated: 1_213,
            unit: "tracks",
            chars: 25,
            build: |c, n| Scenario::new(c, &[]).with_count("radar_msgs/RadarTracks.tracks", n),
        },
        Boundary {
            schema: "vision_msgs/Detection2D",
            tier: SMALL,
            stated: 648,
            unit: "hypotheses",
            chars: 32,
            build: |c, n| Scenario::new(c, &[]).with_count("vision_msgs/Detection2D.results", n),
        },
        Boundary {
            schema: "vision_msgs/BoundingBox2DArray",
            tier: SMALL,
            stated: 6_551,
            unit: "boxes",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[]).with_count("vision_msgs/BoundingBox2DArray.boxes", n)
            },
        },
        Boundary {
            schema: "vision_msgs/BoundingBox3DArray",
            tier: SMALL,
            stated: 3_275,
            unit: "boxes",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[]).with_count("vision_msgs/BoundingBox3DArray.boxes", n)
            },
        },
        Boundary {
            schema: "autoware_perception_msgs/PredictedPath",
            tier: SMALL,
            stated: 4_680,
            unit: "poses",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[]).with_count("autoware_perception_msgs/PredictedPath.path", n)
            },
        },
        Boundary {
            schema: "autoware_planning_msgs/Trajectory",
            tier: MEDIUM,
            stated: 47_661,
            unit: "points",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[]).with_count("autoware_planning_msgs/Trajectory.points", n)
            },
        },
        Boundary {
            schema: "autoware_planning_msgs/LaneletSegment",
            tier: TINY,
            stated: 362,
            unit: "primitives",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[])
                    .with_count("autoware_planning_msgs/LaneletSegment.primitives", n)
            },
        },
        Boundary {
            schema: "control_msgs/JointTrajectoryControllerState",
            tier: SMALL,
            stated: 410,
            unit: "multi-DOF joints",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[])
                    .with_count(
                        "control_msgs/JointTrajectoryControllerState.multi_dof_joint_names",
                        n,
                    )
                    .with_count("trajectory_msgs/MultiDOFJointTrajectoryPoint.transforms", n)
                    .with_count("trajectory_msgs/MultiDOFJointTrajectoryPoint.velocities", n)
                    .with_count(
                        "trajectory_msgs/MultiDOFJointTrajectoryPoint.accelerations",
                        n,
                    )
            },
        },
        Boundary {
            schema: "vision_msgs/Detection3D",
            tier: SMALL,
            stated: 648,
            unit: "hypotheses",
            chars: 32,
            build: |c, n| Scenario::new(c, &[]).with_count("vision_msgs/Detection3D.results", n),
        },
        Boundary {
            schema: "vision_msgs/Classification",
            tier: SMALL,
            stated: 5039,
            unit: "hypotheses",
            chars: 32,
            build: |c, n| Scenario::new(c, &[]).with_count("vision_msgs/Classification.results", n),
        },
        Boundary {
            schema: "vision_msgs/LabelInfo",
            tier: MEDIUM,
            stated: 53771,
            unit: "classes",
            chars: 64,
            build: |c, n| Scenario::new(c, &[]).with_count("vision_msgs/LabelInfo.class_map", n),
        },
        Boundary {
            schema: "vision_msgs/Detection2DArray",
            tier: SMALL,
            stated: 471,
            unit: "single-hypothesis detections (at SMALL, the tier NOT taken)",
            chars: 32,
            build: |c, n| {
                Scenario::new(c, &[])
                    .with_count("vision_msgs/Detection2DArray.detections", n)
                    .with_count("vision_msgs/Detection2D.results", 1)
            },
        },
        Boundary {
            schema: "vision_msgs/Detection3DArray",
            tier: SMALL,
            stated: 439,
            unit: "single-hypothesis detections (at SMALL, the tier NOT taken)",
            chars: 32,
            build: |c, n| {
                Scenario::new(c, &[])
                    .with_count("vision_msgs/Detection3DArray.detections", n)
                    .with_count("vision_msgs/Detection3D.results", 1)
            },
        },
        Boundary {
            schema: "autoware_planning_msgs/LaneletRoute",
            tier: SMALL,
            stated: 1069,
            unit: "segments (at SMALL, the tier NOT taken)",
            chars: 25,
            build: |c, n| {
                Scenario::new(c, &[])
                    .with_count("autoware_planning_msgs/LaneletRoute.segments", n)
                    .with_count("autoware_planning_msgs/LaneletSegment.primitives", 4)
            },
        },
        Boundary {
            schema: "radar_msgs/RadarScan",
            tier: MEDIUM,
            stated: 209710,
            unit: "returns",
            chars: 25,
            build: |c, n| Scenario::new(c, &[]).with_count("radar_msgs/RadarScan.returns", n),
        },
    ];

    let mut drift = Vec::new();
    for b in boundaries {
        // Bracket, then bisect on "does n still fit". The envelope is monotone
        // in the element count (each element only adds bytes), so the predicate
        // has exactly one crossing and bisection finds it.
        let fits =
            |c: &mut Corpus, n: usize| c.envelope(b.schema, &(b.build)(b.chars, n)) <= b.tier;
        assert!(
            fits(&mut corpus, 1),
            "{}: a ONE-element message does not fit its own tier, so there is no boundary to find",
            b.schema
        );
        let mut lo = 1usize;
        let mut hi = 2usize;
        while fits(&mut corpus, hi) {
            lo = hi;
            hi *= 2;
            assert!(
                hi < 1 << 28,
                "{}: no boundary below 2^28 elements",
                b.schema
            );
        }
        while lo + 1 < hi {
            let mid = lo + (hi - lo) / 2;
            if fits(&mut corpus, mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        if lo != b.stated {
            drift.push(format!(
                "  {}: arm states {} {} @ {}-char strings, MEASURED {lo}",
                b.schema, b.stated, b.unit, b.chars
            ));
        }
        println!(
            "  {:52} {:>8} {} @ {}-char strings",
            b.schema, lo, b.unit, b.chars
        );
    }

    // Every drifted boundary at once: fixing them one panic at a time invites
    // re-blessing each in isolation, which is how a set of numbers stops
    // meaning anything together.
    assert!(
        drift.is_empty(),
        "Tier follow-on: {} stated breakage boundary(ies) disagree with the measurement:\n{}\n\n\
         These numbers are what an operator reads to decide whether a topic needs \
         `max_slice_len:`, so correct the ARM in \
         `cerulion_core/src/codegen/generator/wire_impl.rs` — do NOT re-bless these lines from \
         the measurement without reading why they moved. A boundary that fell is a capacity \
         regression somewhere upstream; one that rose is a schema that shrank.",
        drift.len(),
        drift.join("\n")
    );
}

// Small builder conveniences for the boundary sweep, kept next to their one use.
impl Scenario {
    fn with_count(mut self, key: &'static str, n: usize) -> Self {
        self.counts.insert(key, n);
        self
    }
    /// A JTCS carrying `n` revolute joints and no multi-DOF joints — the shape
    /// a ros2_control `JointTrajectoryController` publishes.
    fn with_jtcs(self, n: usize) -> Self {
        self.with_count("control_msgs/JointTrajectoryControllerState.joint_names", n)
            .with_count("trajectory_msgs/JointTrajectoryPoint.positions", n)
            .with_count("trajectory_msgs/JointTrajectoryPoint.velocities", n)
            .with_count("trajectory_msgs/JointTrajectoryPoint.accelerations", n)
            .with_count("trajectory_msgs/JointTrajectoryPoint.effort", n)
    }
}
