// SPDX-License-Identifier: AGPL-3.0-only
//! `bag play` HANDS the bag's schema definitions to a running viewer.
//!
//! The daemon half — that a side-loaded definition really makes a topic
//! decodable — is pinned over the real daemon by
//! `cerulion_vizd/tests/vizd_e2e_test.rs::side_loaded_schemas_make_a_local_custom_topic_decodable`.
//! What THAT test cannot see is whether the player ever offers anything, or what
//! it offers, so this file drives the production `bag_play_with_manager` against
//! a STAND-IN daemon: a `UnixListener` speaking the real hello banner and
//! recording the request lines it receives.
//!
//! A stand-in rather than a real vizd is deliberate and is not a weaker oracle
//! for this question: `cerulion_cli_engine` cannot depend on `cerulion_vizd`
//! (that crate pulls the whole rerun SDK — the same decoupling `VIZD_SOCKET_ENV`
//! exists for), and the assertion here is about the BYTES the player sends,
//! which a listener observes exactly.
//!
//! Oracles are hand-written: the definition text is a string chosen in this
//! file, and it must come back out of the wire byte-identically.
//!
//! `VIZD_SOCKET_ENV` is process-global, so the tests in this file share a mutex.

#![cfg(unix)]

use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixListener;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use cerulion_bag::{BagReader, BagSchemaCatalog, BagWriter, BagWriterConfig, TopicSchema};
use cerulion_cli_engine::bag_cmd::{self, PlayOptions};
use cerulion_cli_engine::viz_client::VIZD_SOCKET_ENV;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::WireHeader;
use cerulion_core::{SchemaDoc, SchemaEncoding, SchemaHashName};

const WIDGET_TEXT: &str = "# a definition only this bag carries\nfloat64 x\nfloat64 y\n";
const WIDGET_Q: &str = "play/Widget";
const WIDGET_HASH: u64 = 0x0952_0952_0952_0952;

/// `VIZD_SOCKET_ENV` is process-global.
fn env_lock() -> MutexGuard<'static, ()> {
    static L: Mutex<()> = Mutex::new(());
    L.lock().unwrap_or_else(|p| p.into_inner())
}

fn unique() -> u64 {
    static N: AtomicU64 = AtomicU64::new(0);
    (std::process::id() as u64) << 20 | N.fetch_add(1, Ordering::Relaxed)
}

/// RAII restore of `VIZD_SOCKET_ENV` — a panicking test must not leave the
/// process pointing at a dead socket.
struct SocketEnv(Option<String>);
impl SocketEnv {
    fn set(path: &Path) -> Self {
        let prev = std::env::var(VIZD_SOCKET_ENV).ok();
        std::env::set_var(VIZD_SOCKET_ENV, path);
        SocketEnv(prev)
    }
}
impl Drop for SocketEnv {
    fn drop(&mut self) {
        match &self.0 {
            Some(v) => std::env::set_var(VIZD_SOCKET_ENV, v),
            None => std::env::remove_var(VIZD_SOCKET_ENV),
        }
    }
}

/// A stand-in vizd: sends the real hello banner, then answers every request with
/// a plausible `schemas` reply, recording each request line it saw.
struct StandInVizd {
    seen: Arc<Mutex<Vec<String>>>,
    stop: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
    _dir: tempfile::TempDir,
    socket: PathBuf,
}

impl StandInVizd {
    fn start() -> Self {
        let dir = tempfile::tempdir().expect("tempdir");
        let socket = dir.path().join("vizd.sock");
        let listener = UnixListener::bind(&socket).expect("bind stand-in socket");
        listener
            .set_nonblocking(true)
            .expect("nonblocking so the accept loop can observe the stop flag");
        let seen: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let (seen_t, stop_t) = (Arc::clone(&seen), Arc::clone(&stop));
        let thread = std::thread::spawn(move || {
            while !stop_t.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        stream.set_nonblocking(false).ok();
                        let mut writer = stream.try_clone().expect("clone");
                        // The hello banner the client parses on connect.
                        let _ = writeln!(writer, r#"{{"protocol":1,"rerun_url":null}}"#);
                        let mut reader = BufReader::new(stream);
                        let mut line = String::new();
                        while reader.read_line(&mut line).unwrap_or(0) > 0 {
                            seen_t.lock().unwrap().push(line.trim().to_string());
                            let _ = writeln!(
                                writer,
                                r#"{{"id":1,"ok":true,"offered":1,"accepted":1,"known":1}}"#
                            );
                            line.clear();
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                        std::thread::sleep(Duration::from_millis(5));
                    }
                    Err(_) => break,
                }
            }
        });
        StandInVizd {
            seen,
            stop,
            thread: Some(thread),
            _dir: dir,
            socket,
        }
    }

    /// The `schemas` request lines received so far.
    fn schema_requests(&self) -> Vec<String> {
        self.seen
            .lock()
            .unwrap()
            .iter()
            .filter(|l| l.contains(r#""method":"schemas""#))
            .cloned()
            .collect()
    }

    fn wait_for_schema_request(&self, bound: Duration) -> Option<String> {
        let deadline = Instant::now() + bound;
        while Instant::now() < deadline {
            if let Some(l) = self.schema_requests().into_iter().next() {
                return Some(l);
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        None
    }
}

impl Drop for StandInVizd {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn frame(seq: u32) -> Vec<u8> {
    let payload = [1.5f64.to_le_bytes(), 2.5f64.to_le_bytes()].concat();
    let total = WireHeader::SIZE + payload.len();
    let header = WireHeader {
        schema_hash: WIDGET_HASH,
        total_size: total as u32,
        offset_table_offset: 0,
        offset_table_count: 0,
        sequence: seq,
        timestamp_ns: 1_000_000_000 + u64::from(seq) * 10_000_000,
    };
    let mut buf = vec![0u8; total];
    header.write_to_buf(&mut buf[..WireHeader::SIZE]);
    buf[WireHeader::SIZE..].copy_from_slice(&payload);
    buf
}

/// Craft a finalized bag on `topic`, carrying `catalog` when `Some`.
fn write_bag(path: &Path, topic: &str, catalog: Option<&BagSchemaCatalog>) {
    write_bag_with(path, topic, catalog, 4);
}

/// [`write_bag`] with an explicit frame count (`0` = a finalized bag with a
/// declared channel and nothing on it — which `bag play` REFUSES, after the
/// point where the offer thread is spawned).
fn write_bag_with(path: &Path, topic: &str, catalog: Option<&BagSchemaCatalog>, frames: u32) {
    let frames: Vec<Vec<u8>> = (0..frames).map(frame).collect();
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[TopicSchema {
            topic: topic.to_string(),
            schema_name: WIDGET_Q.to_string(),
            schema_hash: WIDGET_HASH,
            wire_fixed_size: 16,
        }],
    )
    .expect("create bag");
    if let Some(c) = catalog {
        w.write_schema_catalog(c).expect("write catalog");
    }
    w.write_chunk(|c| {
        for (i, f) in frames.iter().enumerate() {
            let ts = 1_000_000_000 + i as u64 * 10_000_000;
            c.write_message(topic, i as u32, ts, ts, &[&f[..]])?;
        }
        Ok(())
    })
    .expect("write frames");
    w.finalize().expect("finalize");
}

fn catalog() -> BagSchemaCatalog {
    BagSchemaCatalog::new(
        vec![SchemaDoc {
            qualified: WIDGET_Q.to_string(),
            encoding: SchemaEncoding::Msg,
            text: WIDGET_TEXT.to_string(),
            deps: vec![],
        }],
        vec![SchemaHashName {
            schema_hash: WIDGET_HASH,
            qualified: WIDGET_Q.to_string(),
        }],
    )
}

/// Start a playback thread, returning its stop flag and a handle that reports
/// whether the play actually RAN.
///
/// The outcome matters: an absence assertion ("the viewer was never contacted")
/// passes for the wrong reason if playback failed immediately, so every such
/// assertion is paired in-body with a positive from this handle.
fn play(path: &Path, tag: &str) -> (Arc<AtomicBool>, std::sync::mpsc::Receiver<bool>) {
    play_topics(path, tag, Vec::new())
}

/// [`play`] with an explicit `--topics` filter.
fn play_topics(
    path: &Path,
    tag: &str,
    filter: Vec<String>,
) -> (Arc<AtomicBool>, std::sync::mpsc::Receiver<bool>) {
    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_{tag}_{}", unique()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let running = Arc::new(AtomicBool::new(true));
    let flag = Arc::clone(&running);
    let bag = path.to_path_buf();
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let outcome = bag_cmd::bag_play_with_manager(
            &manager,
            &bag,
            PlayOptions {
                rate: 1.0,
                repeat: true,
                topics: filter,
                ..Default::default()
            },
            flag,
            &mut Vec::new(),
        );
        let _ = tx.send(outcome.map(|s| s.total_injected() > 0).unwrap_or(false));
    });
    (running, rx)
}

/// THE headline: playing a bag that carries a definition offers it to the
/// running viewer, verbatim.
#[test]
fn playing_a_bag_offers_its_schema_definitions_to_a_running_viewer() {
    let _guard = env_lock();
    let vizd = StandInVizd::start();
    let _env = SocketEnv::set(&vizd.socket);

    let dir = tempfile::tempdir().expect("tempdir");
    let bag = dir.path().join("vendor.mcap");
    let topic = format!("/widget/{}", unique());
    write_bag(&bag, &topic, Some(&catalog()));

    let (running, _played) = play(&bag, "offer");
    let line = vizd
        .wait_for_schema_request(Duration::from_secs(10))
        .expect("the player must offer the bag's definitions to a running viewer");
    running.store(false, Ordering::Relaxed);

    // The DEFINITION crossed the wire, byte-identically — a viewer decodes from
    // exactly these bytes, so a re-serialised or truncated copy is a defect.
    let v: serde_json::Value = serde_json::from_str(&line).expect("the request line is JSON");
    let docs = v["docs"].as_array().expect("docs array");
    assert_eq!(docs.len(), 1, "one definition, as the bag carries: {line}");
    assert_eq!(docs[0]["qualified"].as_str(), Some(WIDGET_Q), "{line}");
    assert_eq!(docs[0]["text"].as_str(), Some(WIDGET_TEXT), "{line}");
    assert_eq!(docs[0]["encoding"].as_str(), Some("msg"), "{line}");
}

/// ANTI-TAUTOLOGY + the zero-cost promise: a bag carrying NO definitions offers
/// nothing at all. Without this, the test above would pass a player that
/// unconditionally pinged the viewer with an empty list.
#[test]
fn a_bag_with_no_definitions_offers_nothing() {
    let _guard = env_lock();
    let vizd = StandInVizd::start();
    let _env = SocketEnv::set(&vizd.socket);

    let dir = tempfile::tempdir().expect("tempdir");
    let bag = dir.path().join("plain.mcap");
    let topic = format!("/plain/{}", unique());
    write_bag(&bag, &topic, None);

    let (running, played) = play(&bag, "silent");
    // Well past the offer interval: whatever the player was going to send, it
    // has had several chances to send it.
    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);

    assert!(
        vizd.schema_requests().is_empty(),
        "a bag carrying no definitions must not talk to the viewer at all: {:?}",
        vizd.schema_requests()
    );
    // The absence above proves nothing unless playback actually RAN — a play
    // that errored out immediately would contact no viewer either.
    assert_eq!(
        played.recv_timeout(Duration::from_secs(10)),
        Ok(true),
        "the playback must have really published, or the silence proves nothing"
    );
}

/// The player RE-offers while it plays, so a viewer started AFTER playback began
/// still learns the bag's types.
///
/// That ordering is not exotic, it is the normal one: `cerulion viz <topic>` can
/// only attach to a topic that already exists, so the player necessarily starts
/// first — and `cerulion viz` is what spawns the daemon. A single startup offer
/// would miss the very daemon that ends up rendering.
#[test]
fn a_viewer_that_appears_mid_playback_is_still_offered_the_definitions() {
    let _guard = env_lock();
    let dir = tempfile::tempdir().expect("tempdir");
    let bag = dir.path().join("late.mcap");
    let topic = format!("/late/{}", unique());
    write_bag(&bag, &topic, Some(&catalog()));

    // Point at a socket path that does NOT exist yet, and start playing.
    let sock_dir = tempfile::tempdir().expect("tempdir");
    let socket = sock_dir.path().join("vizd.sock");
    let _env = SocketEnv::set(&socket);
    let (running, _played) = play(&bag, "late");

    // Give the player time to try and fail against the absent daemon.
    std::thread::sleep(Duration::from_millis(500));

    // NOW the viewer appears at that path — the mid-playback case.
    let listener = UnixListener::bind(&socket).expect("bind late listener");
    listener.set_nonblocking(true).expect("nonblocking");
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut got: Option<String> = None;
    while got.is_none() && Instant::now() < deadline {
        match listener.accept() {
            Ok((stream, _)) => {
                stream.set_nonblocking(false).ok();
                let mut writer = stream.try_clone().expect("clone");
                let _ = writeln!(writer, r#"{{"protocol":1,"rerun_url":null}}"#);
                let mut reader = BufReader::new(stream);
                let mut line = String::new();
                if reader.read_line(&mut line).unwrap_or(0) > 0 && line.contains("schemas") {
                    got = Some(line.trim().to_string());
                }
            }
            Err(_) => std::thread::sleep(Duration::from_millis(20)),
        }
    }
    running.store(false, Ordering::Relaxed);

    let line = got.expect(
        "a viewer that starts DURING playback must still be offered the bag's definitions — \
         a one-shot offer at player startup would have missed it",
    );
    assert!(line.contains(WIDGET_Q), "{line}");
}

/// A REFUSED playback reaps its offer thread instead of leaving it re-connecting
/// to the viewer socket for the life of the process.
///
/// `bag_play_with_manager` has SIX fallible exits between the spawn point and
/// the end (two `return Err` — the not-finalized refusal and the
/// no-playable-channel refusal — three banner `writeln!(…)?`, and the `?` on the
/// frame walk). A reap that any of them skips is unbounded when the function is
/// called as a LIBRARY, which is how these tests and any embedder drive it — so
/// the thread is owned by an RAII guard rather than a reap call. This pins that:
/// a refused play must go QUIET.
///
/// The fixture is a FINALIZED bag with a catalog and ZERO frames, refused at the
/// "nothing to publish" gate. A truncated bag would ALSO be refused, and would
/// prove nothing: a non-finalized bag has no readable attachment index, so its
/// catalog reads `None` and no thread is spawned at all — measured, by watching
/// a broken variant survive that version of this test.
///
/// R1: that fixture fix was not the whole hazard. This test asserts a SILENCE,
/// so it is only meaningful if a thread existed to be silenced — and the
/// original class (no catalog ⇒ no thread ⇒ asserting the silence of nothing)
/// is re-created by any future drift in what the fixture carries, with no test
/// failing. `spawn_schema_offer` returns `None` WITHOUT spawning when the
/// `SchemaEncoding::Msg` partition leaves `docs` empty, so the spawn
/// precondition is asserted on the fixture below rather than assumed.
#[test]
fn a_refused_playback_does_not_leave_an_offer_thread_running() {
    let _guard = env_lock();
    let vizd = StandInVizd::start();
    let _env = SocketEnv::set(&vizd.socket);

    let dir = tempfile::tempdir().expect("tempdir");
    let bag = dir.path().join("empty.mcap");
    let topic = format!("/empty/{}", unique());
    write_bag_with(&bag, &topic, Some(&catalog()), 0);

    // ANTI-VACUITY (R1): the bag must really carry a `.msg`-encoded doc, or
    // `spawn_schema_offer` never spawns and every assertion below is `0 == 0`.
    // Read it back through the SAME reader the player uses, so a fixture that
    // silently stopped carrying a catalog fails HERE with a diagnosis rather
    // than passing as a green absence.
    let carried = BagReader::open(&bag)
        .expect("the fixture bag opens")
        .schema_catalog()
        .expect(
            "PRECONDITION: the fixture must carry a schema catalog — without one no offer \
             thread is ever spawned and this test asserts the silence of nothing",
        );
    assert!(
        carried
            .docs
            .iter()
            .any(|d| d.encoding == SchemaEncoding::Msg),
        "PRECONDITION: the catalog must carry at least one `.msg` doc — \
         `spawn_schema_offer` returns None without spawning when the Msg partition \
         leaves `docs` empty: {:?}",
        carried.docs
    );

    let manager = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("play_refuse_{}", unique()),
            ..Default::default()
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let err = bag_cmd::bag_play_with_manager(
        &manager,
        &bag,
        PlayOptions {
            rate: 1.0,
            repeat: false,
            topics: Vec::new(),
            ..Default::default()
        },
        Arc::new(AtomicBool::new(true)),
        &mut Vec::new(),
    );
    assert!(
        err.is_err(),
        "PRECONDITION: this bag must be REFUSED, or the test exercises the happy path"
    );

    // Whatever it managed to send before the refusal is fine; what must not
    // happen is a thread that keeps offering afterwards.
    let before = vizd.schema_requests().len();
    std::thread::sleep(Duration::from_secs(3)); // > SCHEMA_OFFER_INTERVAL
    assert_eq!(
        vizd.schema_requests().len(),
        before,
        "the offer thread must be reaped by the refusal — it is still contacting the viewer"
    );
}

// ── The offer is scoped to what is actually playing ──

const OTHER_Q: &str = "play/Other";
const OTHER_TEXT: &str = "play/Nested nested\nint32 n\n";
const OTHER_HASH: u64 = 0x0952_0952_0952_1111;
const NESTED_Q: &str = "play/Nested";
const NESTED_TEXT: &str = "float64 v\n";
/// A nested custom hanging off the SELECTED type.
///
/// Without it the filtered arm proved only EXCLUSION: `WIDGET_Q` had no `deps`,
/// so a variant that scoped per-channel WITHOUT walking the closure still produced
/// exactly `[WIDGET_Q]` and only the whole-play arm caught it. A dep on the
/// selected side makes the filtered arm self-sufficient — it now fails if the
/// scoping stops walking the closure.
const WIDGET_PART_Q: &str = "play/WidgetPart";
const WIDGET_PART_TEXT: &str = "int32 part\n";

/// A bag with TWO topics of DIFFERENT types, and a catalog whose second type
/// carries a nested `deps` closure (so the scoping below is proven to pull a
/// closure, not just a single doc).
fn write_two_topic_bag(path: &Path, widget_topic: &str, other_topic: &str) {
    let catalog = BagSchemaCatalog::new(
        vec![
            SchemaDoc {
                qualified: WIDGET_Q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: WIDGET_TEXT.to_string(),
                deps: vec![WIDGET_PART_Q.to_string()],
            },
            SchemaDoc {
                qualified: WIDGET_PART_Q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: WIDGET_PART_TEXT.to_string(),
                deps: vec![],
            },
            SchemaDoc {
                qualified: OTHER_Q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: OTHER_TEXT.to_string(),
                deps: vec![NESTED_Q.to_string()],
            },
            SchemaDoc {
                qualified: NESTED_Q.to_string(),
                encoding: SchemaEncoding::Msg,
                text: NESTED_TEXT.to_string(),
                deps: vec![],
            },
        ],
        vec![
            SchemaHashName {
                schema_hash: WIDGET_HASH,
                qualified: WIDGET_Q.to_string(),
            },
            SchemaHashName {
                schema_hash: OTHER_HASH,
                qualified: OTHER_Q.to_string(),
            },
        ],
    );
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[
            TopicSchema {
                topic: widget_topic.to_string(),
                schema_name: WIDGET_Q.to_string(),
                schema_hash: WIDGET_HASH,
                wire_fixed_size: 16,
            },
            TopicSchema {
                topic: other_topic.to_string(),
                schema_name: OTHER_Q.to_string(),
                schema_hash: OTHER_HASH,
                wire_fixed_size: 16,
            },
        ],
    )
    .expect("create bag");
    w.write_schema_catalog(&catalog).expect("write catalog");
    let frames: Vec<Vec<u8>> = (0..4u32).map(frame).collect();
    w.write_chunk(|c| {
        for (i, f) in frames.iter().enumerate() {
            let ts = 1_000_000_000 + i as u64 * 10_000_000;
            c.write_message(widget_topic, i as u32, ts, ts, &[&f[..]])?;
            c.write_message(other_topic, i as u32, ts, ts, &[&f[..]])?;
        }
        Ok(())
    })
    .expect("write frames");
    w.finalize().expect("finalize");
}

/// The qualified names in a captured `schemas` request line, sorted.
fn offered_names(line: &str) -> Vec<String> {
    let v: serde_json::Value = serde_json::from_str(line).expect("the request line is JSON");
    let mut names: Vec<String> = v["docs"]
        .as_array()
        .expect("docs array")
        .iter()
        .filter_map(|d| d["qualified"].as_str().map(str::to_string))
        .collect();
    names.sort();
    names
}

/// `bag play --topics /one` offers only the selected topics'
/// schema closure — never the whole catalog.
///
/// A side-load is DAEMON-WIDE and the layout resolver is last-insert-wins, so a
/// parseable doc for a type this playback never publishes can legitimately
/// REPLACE a definition the daemon already holds, taking whatever was decoding
/// the previous one dark. A player has no business changing how a type it is not
/// publishing gets decoded, and `--topics` is the user saying exactly which
/// types are in play.
///
/// Hand oracles on BOTH arms, so this pins the scoping and not merely "fewer
/// docs": the filtered play offers the widget and NOTHING else, while the same
/// bag played WHOLE offers all three (the second type plus its nested `deps`
/// closure — which also proves the scoping walks a closure rather than matching
/// one doc per channel).
#[test]
fn a_topic_filtered_play_offers_only_the_selected_topics_closure() {
    let _guard = env_lock();
    let vizd = StandInVizd::start();
    let _env = SocketEnv::set(&vizd.socket);

    let dir = tempfile::tempdir().expect("tempdir");
    let bag = dir.path().join("two.mcap");
    let id = unique();
    let widget_topic = format!("/w/{id}");
    let other_topic = format!("/o/{id}");
    write_two_topic_bag(&bag, &widget_topic, &other_topic);

    // ── FILTERED: only the widget topic is played.
    let (running, _played) = play_topics(&bag, "scoped", vec![widget_topic.clone()]);
    let line = vizd
        .wait_for_schema_request(Duration::from_secs(10))
        .expect("a filtered play still offers the SELECTED topic's definition");
    running.store(false, Ordering::Relaxed);
    assert_eq!(
        offered_names(&line),
        vec![WIDGET_Q.to_string(), WIDGET_PART_Q.to_string()],
        "THE PIN, both halves: the selected topic's own nested `deps` closure IS \
         offered (so the scoping still WALKS a closure), and the filtered-out \
         topic's types are NOT (so a doc for a type this playback never publishes \
         is not handed to a daemon-wide side-load): {line}"
    );

    // ── ANTI-TAUTOLOGY: the SAME bag played WHOLE offers all three, so the arm
    //    above is about the FILTER and not about the bag carrying one doc.
    let vizd_all = StandInVizd::start();
    let _env_all = SocketEnv::set(&vizd_all.socket);
    let (running, _played) = play_topics(&bag, "scopedall", Vec::new());
    let line = vizd_all
        .wait_for_schema_request(Duration::from_secs(10))
        .expect("an unfiltered play offers the whole catalog");
    running.store(false, Ordering::Relaxed);
    assert_eq!(
        offered_names(&line),
        vec![
            NESTED_Q.to_string(),
            OTHER_Q.to_string(),
            WIDGET_Q.to_string(),
            WIDGET_PART_Q.to_string()
        ],
        "an unfiltered play is unchanged — every recorded type plus the nested \
         `deps` closure: {line}"
    );
}
