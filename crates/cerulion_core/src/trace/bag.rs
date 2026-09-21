// SPDX-License-Identifier: AGPL-3.0-only
//! Disk overflow for the publish trace — `BagWriter`.
//!
//! When attached to a `PublishTrace`, every entry evicted by the
//! ring-buffer's depth policy gets written to a rotating bag file on
//! disk via `BufWriter<File>`. Format is **JSON Lines** (one
//! `PublishTraceEntry` per line) — simple to inspect with `cat` /
//! `jq` and easy to integrate with debug tooling.
//!
//! # Throughput
//!
//! Metadata-only entries are ~80 B each as JSON Lines. Even a
//! 1000 Hz publisher writes ~80 KB/sec — well within any modern
//! disk's sustained write rate and negligible CPU overhead. No
//! zero-copy gymnastics needed (Regime A scope; full-payload
//! bagging is Regime B and lives in the `cerulion_bag` crate).
//!
//! # Rotation + retention
//!
//! - `file_size_limit`: when the current file exceeds this size,
//!   close it and open a new one with a sequential index suffix.
//! - `retention_bytes` / `retention_duration`: when total bag
//!   storage exceeds either limit, delete the oldest file(s).
//!   Optional — both can be `None` for unlimited retention.

use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::time::Duration;

use super::publish::PublishTraceEntry;

/// Schema-hash recipe identifier persisted with every JSONL record.
///
/// Recipe `3` = layout-sensitive hash over the **qualified** name
/// (`pkg/Name`) + `wire_fixed_size()` + per-field name/`canonical_str()`,
/// where a fixed-resolved nested field also folds in its target's full
/// recursive hash (see `codegen::MessageSchema::schema_hash` for the
/// canonical recipe).
/// Recipe `2` was the length-prefixed layout hash over the **bare**
/// name (no package qualification, no nested-layout fold).
/// Recipe `1` was the earlier name-only `fnv1a(schema_name)` hash;
/// recordings written before this marker existed carry no `hash_recipe`
/// key at all and should be read as recipe 1 — without the marker,
/// pre-change recordings would surface as undiagnosable hash mismatches
/// during replay against newer binaries.
///
/// Note: persisted at the `BagWriter` layer (not on the in-memory
/// `PublishTraceEntry`) because the recipe is a property of the
/// *recording*, fixed per binary — every entry a given process writes
/// shares it. `BagWriter` hand-formats JSONL (no serde), so the
/// "absent key = recipe 1" default lives in readers (the CLI's
/// `parse_jsonl_record` is key-based and tolerates both shapes) rather
/// than in a `#[serde(default)]` attribute.
pub const HASH_RECIPE: u32 = 3;

/// Retention policy: when the total bag-directory size exceeds the
/// configured limits, the oldest file(s) get deleted.
#[derive(Debug, Clone)]
pub struct BagRetention {
    /// Max total bytes across all bag files. `None` = unlimited.
    pub max_bytes: Option<u64>,
    /// Max wall-clock duration retained. `None` = unlimited.
    /// (Enforced via file modification times.)
    pub max_duration: Option<Duration>,
}

impl Default for BagRetention {
    fn default() -> Self {
        Self {
            max_bytes: Some(1024 * 1024 * 1024),           // 1 GB
            max_duration: Some(Duration::from_secs(3600)), // 1 hour
        }
    }
}

/// Rotating-file writer for evicted `PublishTraceEntry` records.
pub struct BagWriter {
    dir: PathBuf,
    file_size_limit: u64,
    retention: BagRetention,
    file_index: u64,
    /// Currently-open file. Lazy: opened on first `write_entry` call.
    current: Option<CurrentFile>,
}

struct CurrentFile {
    path: PathBuf,
    writer: BufWriter<File>,
    bytes_written: u64,
}

impl BagWriter {
    /// Create a new bag writer rooted at `dir`. The directory is
    /// created if it doesn't exist. Files are named `trace_XXXX.jsonl`
    /// with sequential indices.
    pub fn new(
        dir: impl AsRef<Path>,
        file_size_limit: u64,
        retention: BagRetention,
    ) -> std::io::Result<Self> {
        let dir = dir.as_ref().to_path_buf();
        std::fs::create_dir_all(&dir)?;
        // Find the next available file index by scanning existing
        // `trace_*.jsonl` files. Resumes numbering after restart so
        // we don't overwrite previously-written bags.
        let mut max_idx: u64 = 0;
        if let Ok(entries) = std::fs::read_dir(&dir) {
            for entry in entries.flatten() {
                if let Some(name) = entry.file_name().to_str() {
                    if let Some(idx_str) = name
                        .strip_prefix("trace_")
                        .and_then(|s| s.strip_suffix(".jsonl"))
                    {
                        if let Ok(idx) = idx_str.parse::<u64>() {
                            max_idx = max_idx.max(idx);
                        }
                    }
                }
            }
        }
        Ok(Self {
            dir,
            file_size_limit,
            retention,
            file_index: max_idx,
            current: None,
        })
    }

    /// Write a single entry as a JSON Lines record. Rotates the file
    /// when `file_size_limit` is exceeded. Enforces retention after
    /// rotation.
    ///
    /// Topic names with `"`
    /// or `\` previously produced malformed JSON. Topics are now
    /// JSON-escaped (backslashes + double-quotes) before
    /// interpolation. Other JSON special chars (`\n`, `\t`, etc.)
    /// are extremely unlikely in topic names so are not escaped;
    /// `validate_topic_name` at the publisher boundary catches the
    /// pathological case.
    pub fn write_entry(&mut self, entry: &PublishTraceEntry) -> std::io::Result<()> {
        let topic_escaped = entry.topic.replace('\\', "\\\\").replace('"', "\\\"");
        let line = format!(
            "{{\"topic\":\"{}\",\"seq\":{},\"ts_ns\":{},\"schema_hash\":\"0x{:016X}\",\"hash_recipe\":{}}}\n",
            topic_escaped, entry.sequence, entry.publish_time_ns, entry.schema_hash, HASH_RECIPE
        );
        let line_bytes = line.as_bytes();

        if self.current.is_none() {
            self.open_next_file()?;
        }

        // Rotate if appending this line would exceed the limit.
        if let Some(cur) = &self.current {
            if cur.bytes_written + line_bytes.len() as u64 > self.file_size_limit {
                self.flush()?;
                self.current = None;
                self.open_next_file()?;
                self.enforce_retention()?;
            }
        }

        let cur = self
            .current
            .as_mut()
            .expect("open_next_file just succeeded");
        cur.writer.write_all(line_bytes)?;
        cur.bytes_written += line_bytes.len() as u64;
        Ok(())
    }

    /// Flush the current file's buffer to disk. Safe to call
    /// repeatedly. Called on shutdown via `Drop`.
    pub fn flush(&mut self) -> std::io::Result<()> {
        if let Some(cur) = &mut self.current {
            cur.writer.flush()?;
        }
        Ok(())
    }

    fn open_next_file(&mut self) -> std::io::Result<()> {
        self.file_index += 1;
        let path = self.dir.join(format!("trace_{:04}.jsonl", self.file_index));
        let file = OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&path)?;
        self.current = Some(CurrentFile {
            path,
            writer: BufWriter::new(file),
            bytes_written: 0,
        });
        Ok(())
    }

    /// Walk the bag directory and delete oldest files until both
    /// `max_bytes` and `max_duration` constraints are satisfied. Best-
    /// effort: per-file errors (e.g., file disappeared) are logged via
    /// `tracing::warn!` and skipped.
    fn enforce_retention(&self) -> std::io::Result<()> {
        let entries: Vec<std::fs::DirEntry> = std::fs::read_dir(&self.dir)?
            .flatten()
            .filter(|e| {
                e.file_name()
                    .to_str()
                    .is_some_and(|n| n.starts_with("trace_") && n.ends_with(".jsonl"))
            })
            .collect();

        // Sort by modified time, oldest first.
        let mut with_meta: Vec<(std::fs::DirEntry, std::fs::Metadata)> = entries
            .into_iter()
            .filter_map(|e| e.metadata().ok().map(|m| (e, m)))
            .collect();
        with_meta.sort_by_key(|(_, m)| m.modified().ok().unwrap_or(std::time::UNIX_EPOCH));

        let now = std::time::SystemTime::now();
        // Track total bytes (after enforcing duration cutoff).
        let mut total_bytes: u64 = with_meta.iter().map(|(_, m)| m.len()).sum();

        for (entry, meta) in &with_meta {
            // Skip the currently-open file — never delete it from
            // under our own writer.
            if let Some(cur) = &self.current {
                if entry.path() == cur.path {
                    continue;
                }
            }
            let mut should_delete = false;
            // Bytes constraint.
            if let Some(max_bytes) = self.retention.max_bytes {
                if total_bytes > max_bytes {
                    should_delete = true;
                }
            }
            // Duration constraint.
            if let Some(max_dur) = self.retention.max_duration {
                if let Ok(modified) = meta.modified() {
                    if let Ok(age) = now.duration_since(modified) {
                        if age > max_dur {
                            should_delete = true;
                        }
                    }
                }
            }
            if should_delete {
                let path = entry.path();
                match std::fs::remove_file(&path) {
                    Ok(()) => {
                        tracing::debug!(?path, "deleted bag file per retention policy");
                        total_bytes = total_bytes.saturating_sub(meta.len());
                    }
                    Err(e) => {
                        tracing::warn!(?path, error = %e, "failed to delete bag file");
                    }
                }
            }
        }
        Ok(())
    }
}

impl Drop for BagWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}

// ---------------------------------------------------------------------------
// This module's half of the ABI LAYOUT PIN (see `crate::abi_layout`).
//
// `abi_pin_struct!` expands to an exhaustive destructuring pattern with no `..`
// rest pattern, so adding or removing a field of one of these structs is a
// COMPILE ERROR naming the struct; it also measures size/align/`offset_of!`,
// which `crate::abi_layout` compares against the snapshot table keyed to
// `CERULION_ABI_VERSION`. `abi_pin_enum!` does the same for a variant set.
// ---------------------------------------------------------------------------
#[cfg(test)]
pub(crate) fn abi_layout_pins() -> Vec<crate::abi_layout::MeasuredStruct> {
    use crate::abi_layout::abi_pin_struct;
    vec![
        abi_pin_struct!(BagWriter {
            dir,
            file_size_limit,
            retention,
            file_index,
            current
        }),
        abi_pin_struct!(CurrentFile {
            path,
            writer,
            bytes_written
        }),
        abi_pin_struct!(BagRetention {
            max_bytes,
            max_duration
        }),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    fn entry(seq: u32, ts_ns: u64) -> PublishTraceEntry {
        PublishTraceEntry {
            topic: Arc::from("test/topic"),
            sequence: seq,
            publish_time_ns: ts_ns,
            schema_hash: 0xDEADBEEF,
        }
    }

    #[test]
    fn topic_with_quote_or_backslash_produces_valid_jsonl() {
        // Previously
        // `entry.topic` was interpolated into JSON without escaping,
        // so a topic containing `"` or `\` would produce a malformed
        // line that breaks downstream `jq` parsing.
        let tmp = tempdir();
        let mut bag = BagWriter::new(&tmp, 100_000, BagRetention::default()).unwrap();
        let nasty = PublishTraceEntry {
            topic: Arc::from(r#"weird/topic"with\backslash"#),
            sequence: 42,
            publish_time_ns: 12_345,
            schema_hash: 0xDEADBEEF,
        };
        bag.write_entry(&nasty).unwrap();
        bag.flush().unwrap();

        let path = tmp.join("trace_0001.jsonl");
        let contents = std::fs::read_to_string(&path).unwrap();
        assert!(contents.starts_with('{'), "must start with `{{`");
        assert!(contents.ends_with("}\n"), "must end with `}}\\n`");
        // The escaped sequences MUST be present in the output line —
        // these are the JSON-correct encodings of `"` and `\`.
        assert!(
            contents.contains(r#"\""#),
            "embedded double-quote must be escaped as `\\\"`; got: {contents}"
        );
        assert!(
            contents.contains(r"\\"),
            "embedded backslash must be escaped as `\\\\`; got: {contents}"
        );
        // The unescaped raw bytes `"` or `\` MUST NOT appear inside
        // the topic value position. We verify by structurally
        // walking the line: between the opening `"topic":"` and the
        // closing `"` of the topic value, every `"` and `\` must be
        // escaped (i.e. preceded by `\`).
        let topic_key = r#""topic":""#;
        let start = contents.find(topic_key).expect("topic key present") + topic_key.len();
        // Find the closing quote — walk byte-by-byte tracking escape state.
        let bytes = contents.as_bytes();
        let mut i = start;
        let mut escaped = false;
        while i < bytes.len() {
            let c = bytes[i];
            if escaped {
                // The previous byte was `\`; this byte is the escaped char.
                escaped = false;
            } else if c == b'\\' {
                escaped = true;
            } else if c == b'"' {
                // Unescaped closing quote — end of the topic value.
                break;
            }
            i += 1;
        }
        assert!(
            i < bytes.len(),
            "topic value must terminate with an unescaped closing `\"`"
        );
    }

    #[test]
    fn writes_jsonl_to_disk() {
        let tmp = tempdir();
        let mut bag = BagWriter::new(&tmp, 100_000, BagRetention::default()).unwrap();
        bag.write_entry(&entry(1, 1_000_000)).unwrap();
        bag.write_entry(&entry(2, 2_000_000)).unwrap();
        bag.write_entry(&entry(3, 3_000_000)).unwrap();
        bag.flush().unwrap();

        let path = tmp.join("trace_0001.jsonl");
        let contents = std::fs::read_to_string(&path).unwrap();
        let lines: Vec<&str> = contents.lines().collect();
        assert_eq!(lines.len(), 3);
        assert!(lines[0].contains("\"seq\":1"));
        assert!(lines[0].contains("\"ts_ns\":1000000"));
        assert!(lines[0].contains("\"schema_hash\":\"0x00000000DEADBEEF\""));
        // Every persisted record carries the hash-recipe marker
        // (recipe 3 = layout + qualified name + nested fold) so older
        // recordings stay diagnosable (absent key = recipe 1).
        for line in &lines {
            assert!(
                line.contains("\"hash_recipe\":3"),
                "every record must carry the recipe marker; got: {line}"
            );
        }
        assert!(lines[2].contains("\"seq\":3"));
    }

    #[test]
    fn rotates_at_file_size_limit() {
        let tmp = tempdir();
        // Tiny size limit so we rotate after 2-3 entries (each line
        // is ~80 bytes).
        let mut bag = BagWriter::new(&tmp, 200, BagRetention::default()).unwrap();
        for i in 1..=10 {
            bag.write_entry(&entry(i, (i as u64) * 1_000_000)).unwrap();
        }
        bag.flush().unwrap();

        let files: Vec<_> = std::fs::read_dir(&tmp)
            .unwrap()
            .flatten()
            .filter(|e| e.file_name().to_string_lossy().ends_with(".jsonl"))
            .collect();
        assert!(
            files.len() >= 3,
            "should have rotated several times under 200-byte cap; got {} files",
            files.len()
        );
    }

    #[test]
    fn resumes_file_index_on_restart() {
        let tmp = tempdir();
        {
            let mut bag = BagWriter::new(&tmp, 100_000, BagRetention::default()).unwrap();
            bag.write_entry(&entry(1, 0)).unwrap();
        }
        // Reopen the same dir; should continue at index 2.
        let mut bag2 = BagWriter::new(&tmp, 100_000, BagRetention::default()).unwrap();
        bag2.write_entry(&entry(2, 0)).unwrap();
        bag2.flush().unwrap();

        assert!(tmp.join("trace_0001.jsonl").exists());
        assert!(tmp.join("trace_0002.jsonl").exists());
    }

    #[test]
    fn retention_bytes_evicts_oldest_files() {
        let tmp = tempdir();
        // 200-byte file limit + 600-byte retention → at most ~3 files.
        let retention = BagRetention {
            max_bytes: Some(600),
            max_duration: None,
        };
        let mut bag = BagWriter::new(&tmp, 200, retention).unwrap();
        for i in 1..=20 {
            bag.write_entry(&entry(i, (i as u64) * 1_000_000)).unwrap();
        }
        bag.flush().unwrap();

        let files: Vec<_> = std::fs::read_dir(&tmp)
            .unwrap()
            .flatten()
            .filter(|e| e.file_name().to_string_lossy().ends_with(".jsonl"))
            .collect();
        let total_size: u64 = files
            .iter()
            .filter_map(|e| e.metadata().ok())
            .map(|m| m.len())
            .sum();
        // The previous bound
        // (`< 20 * 100 = 2000`) was loose enough that the test passed
        // even with retention completely broken. Later update: the
        // `hash_recipe` marker grew each line to ~98-100 B (98 B for
        // seq 1-9, 100 B for seq 10-20). Under the 200-byte file cap and
        // the strict-`>` rotation guard, two lines fit per file
        // (98+98=196 ≤ 200; 100+100=200 ≤ 200), so each file holds TWO
        // lines (~196-200 B), not one. With 20 entries that is 10 files;
        // the retention sweep (600-byte cap + currently-open file exempt)
        // settles at 4 retained files × 200 B = 800 B. A no-eviction
        // regression is 20 lines ≈ 1982 B; the 1000 B bound keeps clear
        // detection margin while allowing the retained-files + open-file
        // slack.
        assert!(
            total_size < 1000,
            "retention should have evicted oldest files; total={total_size} \
             (intent: ~600 B retained + open-file slack; regression ≈ 2 KB)"
        );
    }

    // Test helper — minimal alternative to the `tempfile` crate dep.
    // Per-test counter + pid + nanos ensures parallel test runs do
    // not collide on directory name.
    static TEMP_COUNTER: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

    fn tempdir() -> PathBuf {
        let count = TEMP_COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let name = format!(
            "cerulion_bag_test_{}_{}_{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos(),
            count,
        );
        let path = std::env::temp_dir().join(name);
        let _ = std::fs::remove_dir_all(&path);
        std::fs::create_dir_all(&path).unwrap();
        path
    }
}
