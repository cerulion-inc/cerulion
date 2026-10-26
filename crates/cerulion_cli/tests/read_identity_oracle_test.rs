// SPDX-License-Identifier: AGPL-3.0-only
//! The read-identity oracle's own checks, on every platform: the judge
//! against hand-built logs (its mutants included), the record reader's
//! rules, and the harness workspace build (every generated node source
//! compiles). Nothing here loads a node or runs a graph; the live and resim
//! arms are in `read_identity_box_test`.

mod det_scaffold;
mod mp_support;
mod read_identity;

use read_identity::{
    judge_streams, payload_hash, read_from_bits, Floors, Read, ReadLog, RowVerdict,
};

/// The bit patterns a node records for the frame `(writer, seq, z)`.
fn bits(writer: u64, seq: u64, z: f64) -> [u64; 3] {
    [
        (writer as f64).to_bits(),
        (seq as f64).to_bits(),
        z.to_bits(),
    ]
}

/// A record body for `n` reads on `input` from writer `writer`, sequences
/// `1..=n`, with its closing count.
fn record(input: &str, writer: u64, n: u64) -> String {
    let mut out: String = (1..=n)
        .map(|s| {
            let b = bits(writer, s, (s * 7) as f64);
            format!("{input} {:016x} {:016x} {:016x}\n", b[0], b[1], b[2])
        })
        .collect();
    out.push_str(&format!("end {n}\n"));
    out
}

/// A log of one stream `relay.inp` with `n` reads, plus `sink.inp` with `n`.
fn chain_log(n: u64) -> ReadLog {
    let mut log = ReadLog::default();
    log.add_record("relay", &record("inp", 1, n)).unwrap();
    log.add_record("sink", &record("inp", 100, n)).unwrap();
    log
}

const CHAIN: &[(&str, &str)] = &[("relay", "inp"), ("sink", "inp")];

fn stream_mut<'a>(log: &'a mut ReadLog, node: &str) -> &'a mut Vec<Read> {
    log.streams
        .get_mut(&(node.to_string(), "inp".to_string()))
        .expect("stream")
}

#[test]
fn a_record_line_decodes_to_writer_sequence_and_hash() {
    let b = bits(3, 41, 1234.5);
    let r = read_from_bits(b);
    assert_eq!((r.writer, r.seq), (3, 41));
    assert_eq!(r.hash, payload_hash(b));
    assert_ne!(
        payload_hash(b),
        payload_hash(bits(3, 41, 1234.0)),
        "the hash covers the payload value"
    );
    let mut log = ReadLog::default();
    log.add_record("tfbuf", &record("tf", 2, 5)).unwrap();
    let s = &log.streams[&("tfbuf".to_string(), "tf".to_string())];
    assert_eq!(s.len(), 5);
    assert_eq!(
        s.iter().map(|r| r.seq).collect::<Vec<_>>(),
        vec![1, 2, 3, 4, 5]
    );
    assert!(
        log.add_record("x", "inp 1 2\nend 1\n").is_err(),
        "three fields"
    );
    assert!(
        log.add_record("x", "inp zz 0 0\nend 1\n").is_err(),
        "hex fields"
    );
    let one = record("inp", 1, 1);
    let line = one.lines().next().unwrap();
    assert!(
        log.add_record("x", &format!("{line}\n")).is_err(),
        "no closing count"
    );
    assert!(
        log.add_record("x", &format!("{line}\nend 2\n")).is_err(),
        "a short record"
    );
    assert!(log.add_record("x", "").is_err(), "an empty body");
    let z = bits(0, 1, 0.0);
    assert!(
        log.add_record(
            "x",
            &format!("inp {:016x} {:016x} {:016x}\nend 1\n", z[0], z[1], z[2])
        )
        .is_err(),
        "writer 0 is an unstamped frame"
    );
}

#[test]
fn the_reader_skips_empty_records_and_refuses_a_node_in_two_processes() {
    let dir = tempfile::tempdir().unwrap();
    std::fs::write(dir.path().join("sink_a_11_0.reads"), record("inp", 1, 3)).unwrap();
    std::fs::write(dir.path().join("sink_a_12_0.reads"), "").unwrap();
    std::fs::write(dir.path().join("unrelated.txt"), "x").unwrap();
    let log = ReadLog::collect(dir.path()).unwrap();
    assert_eq!(log.total(), 3);
    assert!(log
        .streams
        .contains_key(&("sink_a".to_string(), "inp".to_string())));
    std::fs::write(dir.path().join("sink_a_11_1.reads"), record("inp", 1, 1)).unwrap();
    let err = ReadLog::collect(dir.path()).unwrap_err();
    assert!(err.contains("two records"), "{err}");
    std::fs::remove_file(dir.path().join("sink_a_11_1.reads")).unwrap();
    std::fs::write(dir.path().join("sink_a_13_0.reads"), record("inp", 1, 1)).unwrap();
    assert!(ReadLog::collect(dir.path()).is_err(), "two processes");
    std::fs::remove_file(dir.path().join("sink_a_13_0.reads")).unwrap();
    std::fs::write(dir.path().join("bad_7.reads"), "").unwrap();
    assert!(
        ReadLog::collect(dir.path()).is_err(),
        "a name with no instance"
    );
}

/// The oracle on identical logs reads equal, on every stream, having compared
/// every read.
#[test]
fn identical_logs_judge_equal_with_every_read_compared() {
    let v = judge_streams(CHAIN, &chain_log(200), &chain_log(200), Floors::default()).unwrap();
    assert!(v.is_equal(), "{}", v.render());
    assert_eq!(v.rows.len(), 2);
    assert!(v.rows.iter().all(|r| r.compared == 200));
}

/// What a mutant's judgement must show on the stream it hit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Caught {
    DivergedAt(usize),
    Short,
    Long,
}

/// The oracle's MUTANTS: each one perturbation of the candidate a broken
/// replay could produce. Every one must be caught, on the stream it hit and
/// only there, as the verdict kind it names.
#[test]
fn every_mutant_of_the_candidate_is_caught() {
    let live = chain_log(200);
    type Mutant = (&'static str, &'static str, Caught, fn(&mut ReadLog));
    let mutants: &[Mutant] = &[
        (
            "another writer at one read",
            "relay",
            Caught::DivergedAt(37),
            |l| stream_mut(l, "relay")[37].writer = 2,
        ),
        (
            "a sequence off by one",
            "relay",
            Caught::DivergedAt(37),
            |l| stream_mut(l, "relay")[37].seq += 1,
        ),
        (
            "the same identity, another payload",
            "sink",
            Caught::DivergedAt(120),
            |l| stream_mut(l, "sink")[120].hash ^= 1,
        ),
        (
            "one read missing mid-stream",
            "sink",
            Caught::DivergedAt(90),
            |l| {
                stream_mut(l, "sink").remove(90);
            },
        ),
        ("two reads swapped", "relay", Caught::DivergedAt(10), |l| {
            stream_mut(l, "relay").swap(10, 11)
        }),
        ("a stream that stops early", "sink", Caught::Short, |l| {
            stream_mut(l, "sink").truncate(100)
        }),
        ("one read past the reference", "relay", Caught::Long, |l| {
            let s = stream_mut(l, "relay");
            let last = *s.last().unwrap();
            s.push(last);
        }),
        ("a stream missing entirely", "sink", Caught::Short, |l| {
            l.streams.remove(&("sink".to_string(), "inp".to_string()));
        }),
        (
            "a stream the reference never had",
            "ghost",
            Caught::Long,
            |l| {
                l.add_record("ghost", &record("inp", 9, 80)).unwrap();
            },
        ),
    ];
    for (name, node, caught, mutate) in mutants {
        let mut other = live.clone();
        mutate(&mut other);
        let v = judge_streams(CHAIN, &live, &other, Floors::default()).unwrap();
        let hit = v.divergent();
        assert_eq!(
            hit.iter().map(|r| r.node.as_str()).collect::<Vec<_>>(),
            vec![*node],
            "mutant `{name}`: caught on the wrong streams\n{}",
            v.render()
        );
        let got = match hit[0].verdict {
            RowVerdict::Diverged { index, .. } => Caught::DivergedAt(index),
            RowVerdict::Short => Caught::Short,
            RowVerdict::Long => Caught::Long,
            RowVerdict::Equal => unreachable!("divergent() holds no equal row"),
        };
        assert_eq!(got, *caught, "mutant `{name}`\n{}", v.render());
    }
}

#[test]
fn a_divergence_names_the_first_differing_read() {
    let live = chain_log(200);
    let mut other = live.clone();
    stream_mut(&mut other, "relay")[37].writer = 2;
    stream_mut(&mut other, "relay")[80].writer = 2;
    let v = judge_streams(CHAIN, &live, &other, Floors::default()).unwrap();
    let rows = v.divergent();
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0].node, "relay");
    match &rows[0].verdict {
        RowVerdict::Diverged { index, live, other } => {
            assert_eq!(*index, 37);
            assert_eq!((live.writer, other.writer), (1, 2));
        }
        v => panic!("expected a divergence, got {v:?}"),
    }
    let line = &v.render_lines("ranks2", "resim")[0];
    assert!(
        line.starts_with(
            "READ_IDENTITY fixture=ranks2 leg=resim node=relay input=inp verdict=diverged"
        ),
        "{line}"
    );
    assert!(line.contains("first_diff=37"), "{line}");
}

/// The tail allowance at its boundary: on 200 live reads it is 20, so a
/// candidate 20 short is equal and 21 short is not. A live run stopped by a
/// Ctrl-C may read a little past the recorded trace; the candidate never may.
#[test]
fn the_tail_allowance_holds_at_its_boundary() {
    let live = chain_log(200);
    let short_by = |n: usize| {
        let mut other = live.clone();
        stream_mut(&mut other, "sink").truncate(200 - n);
        judge_streams(CHAIN, &live, &other, Floors::default()).unwrap()
    };
    assert!(short_by(20).is_equal(), "{}", short_by(20).render());
    assert!(!short_by(21).is_equal(), "{}", short_by(21).render());
}

/// The count floors: the judge refuses to judge an expected stream the
/// reference did not exercise, and an empty expectation.
#[test]
fn the_judge_refuses_a_stream_under_its_floor() {
    let err = judge_streams(CHAIN, &chain_log(10), &chain_log(10), Floors::default()).unwrap_err();
    assert!(err.contains("under the floor"), "{err}");
    let mut live = chain_log(200);
    live.streams
        .remove(&("relay".to_string(), "inp".to_string()));
    let err = judge_streams(CHAIN, &live, &live, Floors::default()).unwrap_err();
    assert!(err.contains("relay.inp"), "{err}");
    assert!(judge_streams(&[], &live, &live, Floors::default()).is_err());
    let floors = Floors {
        min_reads: 50,
        tail_min: 200,
        tail_div: 50,
    };
    let mut other = chain_log(200);
    stream_mut(&mut other, "sink").truncate(30);
    let v = judge_streams(CHAIN, &chain_log(200), &other, floors).unwrap();
    assert!(
        !v.is_equal(),
        "a candidate that compared under the floor is never equal, whatever the allowance"
    );
    let err = v.compared_floor(floors).unwrap_err();
    assert!(err.contains("sink.inp compared 30"), "{err}");
    assert!(
        judge_streams(CHAIN, &chain_log(200), &chain_log(200), floors)
            .unwrap()
            .compared_floor(floors)
            .is_ok()
    );
}

/// Every node source the harness generates builds, and every fixture's graph
/// serializes.
#[test]
fn the_harness_workspace_builds_every_node_type() {
    let ws = read_identity::workspace();
    for (name, lib) in read_identity::built_cdylibs(ws) {
        assert!(lib.exists(), "{name}: {} built", lib.display());
    }
    for f in read_identity::Fixture::ALL {
        let (graph, prefix) = read_identity::unique_names(f.name());
        let config = f.graph_config(&graph, &prefix);
        assert!(!config.process_groups.is_empty(), "{}", f.name());
        read_identity::write_graph(ws, &config);
        assert!(ws.graphs_dir.join(format!("{graph}.yaml")).exists());
    }
}
