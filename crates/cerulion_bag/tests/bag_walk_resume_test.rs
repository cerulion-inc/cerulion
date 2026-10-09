// SPDX-License-Identifier: AGPL-3.0-only
//! A [`UserFrameWalk`] can be suspended into a [`WalkPosition`] and resumed on
//! the same bag ([`BagReader::resume_user_frames`]) — the streaming source for a
//! consumer that cannot hold the walk's borrow across yields. These pins prove
//! a walk suspended after EVERY frame yields the same spans, topics and
//! frontiers as one straight walk (through chunk boundaries and a late channel),
//! that a position from another bag is refused, and that the summary's
//! per-channel counts match the walk.

use std::collections::BTreeMap;
use std::path::PathBuf;

use cerulion_bag::{
    BagReader, BagWriter, BagWriterConfig, FrameSpan, TopicSchema, UserFrameWalk, WalkPosition,
};

fn tmp(tag: &str) -> PathBuf {
    static C: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = C.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "cerulion_walk_resume_{tag}_{}_{}_{}.mcap",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos(),
        n
    ))
}

fn topic(name: &str, hash: u64) -> TopicSchema {
    TopicSchema {
        topic: name.into(),
        schema_name: "std_msgs/UInt8".into(),
        schema_hash: hash,
        wire_fixed_size: 0,
    }
}

/// Two chunks on two topics, then a topic registered mid-run with its own
/// chunk: the walk crosses chunk boundaries and sees a Channel record between
/// chunks, so a resumed channel table matters.
fn write_bag(path: &std::path::Path, frames_per_chunk: usize) -> Vec<Vec<u8>> {
    let mut w = BagWriter::create(
        path,
        BagWriterConfig::default(),
        &[topic("/a", 0xA), topic("/b", 0xB)],
    )
    .unwrap();
    let payloads: Vec<Vec<u8>> = (0..frames_per_chunk * 3)
        .map(|i| vec![(i & 0xff) as u8; 16 + i % 7])
        .collect();
    let (first, rest) = payloads.split_at(frames_per_chunk);
    let (second, third) = rest.split_at(frames_per_chunk);
    for (chunk, frames) in [first, second].into_iter().enumerate() {
        w.write_chunk(|c| {
            for (i, p) in frames.iter().enumerate() {
                let seq = (chunk * frames_per_chunk + i) as u32;
                let t = if i % 2 == 0 { "/a" } else { "/b" };
                c.write_message(
                    t,
                    seq,
                    1000 + u64::from(seq),
                    1000 + u64::from(seq),
                    &[&p[..]],
                )?;
            }
            Ok(())
        })
        .unwrap();
    }
    w.register_topic(&topic("/late", 0xC)).unwrap();
    w.write_chunk(|c| {
        for (i, p) in third.iter().enumerate() {
            let seq = (2 * frames_per_chunk + i) as u32;
            c.write_message(
                "/late",
                seq,
                1000 + u64::from(seq),
                1000 + u64::from(seq),
                &[&p[..]],
            )?;
        }
        Ok(())
    })
    .unwrap();
    w.finalize().unwrap();
    payloads
}

/// Drain a walk, recording (topic, span, frontier after the yield) per frame.
fn drain(mut walk: UserFrameWalk<'_>) -> Vec<(String, FrameSpan, usize)> {
    let mut out = Vec::new();
    while let Some((channel_id, span)) = walk.next_user_frame().unwrap() {
        out.push((
            walk.topic(channel_id).to_string(),
            span,
            walk.file_frontier(),
        ));
    }
    out
}

#[test]
fn a_walk_resumed_after_every_frame_matches_the_straight_walk() {
    let p = tmp("resume");
    let payloads = write_bag(&p, 5);
    let r = BagReader::open(&p).unwrap();

    let straight = drain(r.user_frames().unwrap());
    assert_eq!(
        straight.len(),
        payloads.len(),
        "control: every frame is a user frame"
    );

    let mut resumed = Vec::new();
    let mut position: WalkPosition = r.user_frames().unwrap().into_position();
    let final_frontier = loop {
        let mut walk = r.resume_user_frames(position).unwrap();
        match walk.next_user_frame().unwrap() {
            Some((channel_id, span)) => {
                resumed.push((
                    walk.topic(channel_id).to_string(),
                    span,
                    walk.file_frontier(),
                ));
                position = walk.into_position();
            }
            None => break walk.file_frontier(),
        }
    };
    assert_eq!(
        resumed, straight,
        "suspending after each frame must change neither the frames, their topics, nor the \
         advise-behind frontier (mutation guard: dropping the channel table or a stack entry \
         from the position fails this)"
    );
    for ((_, span, _), payload) in resumed.iter().zip(&payloads) {
        assert_eq!(
            r.frame(span),
            &payload[..],
            "the resumed span resolves the frame"
        );
    }
    assert_eq!(
        straight.iter().filter(|(t, _, _)| t == "/late").count(),
        5,
        "the channel registered between chunks is walked under its own name"
    );

    // A walk suspended at its end is spent: resuming yields nothing more.
    let drained = r.user_frames().unwrap();
    let frontier_at_end = {
        let mut w = drained;
        while w.next_user_frame().unwrap().is_some() {}
        w.file_frontier()
    };
    assert_eq!(final_frontier, frontier_at_end);
    let mut walk = r
        .resume_user_frames(r.user_frames().unwrap().into_position())
        .unwrap();
    while walk.next_user_frame().unwrap().is_some() {}
    let mut spent = r.resume_user_frames(walk.into_position()).unwrap();
    assert!(spent.next_user_frame().unwrap().is_none());
    assert_eq!(spent.file_frontier(), frontier_at_end);

    std::fs::remove_file(&p).ok();
}

#[test]
fn a_position_saved_on_another_bag_is_refused() {
    let big = tmp("big");
    let small = tmp("small");
    write_bag(&big, 40);
    write_bag(&small, 1);
    let big_reader = BagReader::open(&big).unwrap();
    let small_reader = BagReader::open(&small).unwrap();

    // Walk deep into the big bag, then try to resume that place on the small one.
    let mut walk = big_reader.user_frames().unwrap();
    for _ in 0..100 {
        walk.next_user_frame()
            .unwrap()
            .expect("the big bag has 120 frames");
    }
    let position = walk.into_position();
    let err = small_reader
        .resume_user_frames(position.clone())
        .err()
        .expect("a position past the small bag's data section must be refused");
    assert!(
        err.to_string().contains("saved on another bag"),
        "the refusal names the cause; got: {err}"
    );
    // Control: the same position resumes on its own bag.
    let mut own = big_reader.resume_user_frames(position).unwrap();
    assert!(own.next_user_frame().unwrap().is_some());

    std::fs::remove_file(&big).ok();
    std::fs::remove_file(&small).ok();
}

#[test]
fn summary_channel_counts_match_the_walk_and_are_absent_without_statistics() {
    let p = tmp("counts");
    write_bag(&p, 4);
    let r = BagReader::open(&p).unwrap();

    let mut walked: BTreeMap<u16, u64> = BTreeMap::new();
    let mut walk = r.user_frames().unwrap();
    while let Some((channel_id, _)) = walk.next_user_frame().unwrap() {
        *walked.entry(channel_id).or_default() += 1;
    }
    let counts = r
        .channel_message_counts()
        .unwrap()
        .expect("BagWriter always emits Statistics");
    let user_channels: Vec<u16> = r
        .channels()
        .unwrap()
        .into_iter()
        .filter(|c| !c.topic.starts_with(cerulion_bag::RESERVED_PREFIX))
        .map(|c| c.id)
        .collect();
    assert_eq!(user_channels.len(), 3);
    for id in user_channels {
        assert_eq!(counts.get(&id), walked.get(&id), "channel {id}");
    }
    assert_eq!(walked.values().sum::<u64>(), 12);

    // A foreign writer may omit the optional Statistics record: None, not 0s.
    let mut buf = std::io::Cursor::new(Vec::new());
    {
        let mut w = mcap::WriteOptions::new()
            .compression(None)
            .emit_statistics(false)
            .create(&mut buf)
            .unwrap();
        let chan = w
            .add_channel(0, "/foreign", "cerulion", &Default::default())
            .unwrap();
        w.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: chan,
                sequence: 0,
                log_time: 1,
                publish_time: 1,
            },
            &[0u8; 40],
        )
        .unwrap();
        w.finish().unwrap();
    }
    let foreign = BagReader::from_bytes(buf.into_inner());
    assert!(foreign.channel_message_counts().unwrap().is_none());

    std::fs::remove_file(&p).ok();
}
