// SPDX-License-Identifier: AGPL-3.0-only
//! The delivery log file: its records, their binary encoding, the writer and
//! the reader.
//!
//! One file per process, `<dir>/<exe>-<pid>.rdl.mcap`, written through
//! `cerulion_bag`'s MCAP writer. Four channels carry the records:
//!
//! | channel | records |
//! |---|---|
//! | [`CH_PROCESS`] | the process header (first message), node and entity declarations, a `not_replayable` mark, the trailer (last message) |
//! | [`CH_WAITS`] | every `rmw_wait` return |
//! | [`CH_DELIVERIES`] | every take (message, request, response) and every graph-query answer |
//! | [`CH_OUTPUTS`] | every output (publish, request, response) as a 64-bit FNV-1a hash of its payload |
//!
//! Every record carries the per-process order key `k`, minted under the
//! writer lock, so file order is `k` order. The MCAP `log_time` and
//! `publish_time` of each message are `k` too: the writer reads no clock and
//! the record stream is a function of the rmw call sequence alone.
//!
//! The channel names carry no `__cerulion/` prefix: `cerulion_bag` reserves
//! that namespace for the channels its own writer registers. They never start
//! with `/`, so they cannot collide with a ROS 2 topic name.
//!
//! The process header is the first message on [`CH_PROCESS`] and is flushed as
//! its own chunk, so a log torn by SIGKILL still identifies its process. The
//! same JSON is attached as [`PROCESS_ATTACHMENT`] when the log is finalized.

use std::path::{Path, PathBuf};

use cerulion_bag::{BagCompleteness, BagReader, BagWriter, BagWriterConfig, FileSink, TopicSchema};
use serde::{Deserialize, Serialize};

/// Channel of the process header, node and entity declarations and the trailer.
pub(crate) const CH_PROCESS: &str = "rdl/process";
/// Channel of the `rmw_wait` returns.
pub(crate) const CH_WAITS: &str = "rdl/waits";
/// Channel of the takes and graph-query answers.
pub(crate) const CH_DELIVERIES: &str = "rdl/deliveries";
/// Channel of the output hashes.
pub(crate) const CH_OUTPUTS: &str = "rdl/outputs";
/// The schema name every delivery-log channel is registered with.
pub(crate) const SCHEMA_NAME: &str = "cerulion_rmw/DeliveryLogRecord";
/// The finalized log's copy of the process header and trailer.
pub(crate) const PROCESS_ATTACHMENT: &str = "rdl/process.json";
/// The delivery log's file suffix.
pub(crate) const LOG_SUFFIX: &str = ".rdl.mcap";
/// The binary record encoding's version byte. A reader refuses any other.
pub(crate) const RECORD_VERSION: u8 = 1;
/// The MCAP library string stamped in the header (fixed, never build info).
const LIBRARY: &str = "rmw_cerulion delivery log";
/// Chunk size of the log. A SIGKILL loses at most the open chunk.
pub(crate) const CHUNK_BYTES: usize = 1024 * 1024;

/// A per-process entity id: the entity's index in creation order.
pub(crate) type EntityId = u32;

/// What an entity is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub(crate) enum EntityKind {
    Subscription = 1,
    Publisher = 2,
    Service = 3,
    Client = 4,
    Guard = 5,
}

impl EntityKind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::Subscription,
            2 => Self::Publisher,
            3 => Self::Service,
            4 => Self::Client,
            5 => Self::Guard,
            _ => return None,
        })
    }

    /// The stable-name prefix (`sub:/a#0`).
    pub(crate) fn prefix(self) -> &'static str {
        match self {
            Self::Subscription => "sub",
            Self::Publisher => "pub",
            Self::Service => "srv",
            Self::Client => "cli",
            Self::Guard => "gc",
        }
    }
}

/// Which take a delivery is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub(crate) enum TakeKind {
    Message = 1,
    Request = 2,
    Response = 3,
}

impl TakeKind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::Message,
            2 => Self::Request,
            3 => Self::Response,
            _ => return None,
        })
    }
}

/// Which output an output record is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum OutputKind {
    Publish = 1,
    Request = 2,
    Response = 3,
}

impl OutputKind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::Publish,
            2 => Self::Request,
            3 => Self::Response,
            _ => return None,
        })
    }
}

/// A graph query whose answer is recorded and, on replay, served.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(u8)]
pub(crate) enum GraphQuery {
    CountPublishers = 1,
    CountSubscribers = 2,
    ServerIsAvailable = 3,
    PublisherMatchedSubscriptions = 4,
    SubscriptionMatchedPublishers = 5,
}

impl GraphQuery {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Self::CountPublishers,
            2 => Self::CountSubscribers,
            3 => Self::ServerIsAvailable,
            4 => Self::PublisherMatchedSubscriptions,
            5 => Self::SubscriptionMatchedPublishers,
            _ => return None,
        })
    }
}

/// The identity of the recorded process. Replay refuses a log whose header
/// does not match the replaying process (see `check_identity`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct ProcessHeader {
    /// The record encoding version ([`RECORD_VERSION`]).
    pub format: u8,
    /// The command line, argument by argument.
    pub cmdline: Vec<String>,
    /// `ROS_DISTRO` at record time (empty when unset).
    pub ros_distro: String,
    /// The distro this `.so` was built for.
    pub built_for: String,
    /// FNV-1a 64 of the loaded rmw_cerulion library file, hex.
    pub library_digest: String,
    /// The recording process id (informational; never compared).
    pub pid: u32,
}

/// The last record of a cleanly closed log.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct Trailer {
    /// `true` when the writer saw no error and every record reached the file.
    pub complete: bool,
    /// Records the writer could not write (after a write error).
    pub dropped: u64,
    /// Total records written before the trailer.
    pub records: u64,
}

/// One `rmw_wait` return.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct WaitRec {
    pub k: u64,
    pub thread: u32,
    pub ret: i32,
    /// The timeout rcl passed, in ns; `-1` for an infinite wait.
    pub timeout_ns: i64,
    /// Every attached entity, in array order (subscriptions, guards,
    /// services, clients).
    pub attached: Vec<EntityId>,
    /// The ready entities, in the same order.
    pub ready: Vec<EntityId>,
}

/// One take: a subscription message, a service request or a client response.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct TakeRec {
    pub k: u64,
    pub thread: u32,
    pub entity: EntityId,
    pub kind: TakeKind,
    pub ret: i32,
    pub taken: bool,
    /// The wire sequence (message) or request sequence number (request,
    /// response).
    pub seq: i64,
    pub source_ts: i64,
    pub received_ts: i64,
    /// The request writer guid (requests and responses; zero for messages).
    pub guid: [u8; 16],
    /// The payload body exactly as the bridge decoded it (the wire header is
    /// not part of it).
    pub body: Vec<u8>,
}

/// One output.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct OutputRec {
    pub k: u64,
    pub thread: u32,
    pub entity: EntityId,
    pub kind: OutputKind,
    pub ret: i32,
    /// Whether a payload reached the send (false when the call failed first).
    pub has_payload: bool,
    pub len: u32,
    pub hash: u64,
}

/// One graph-query answer.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct GraphRec {
    pub k: u64,
    pub thread: u32,
    pub query: GraphQuery,
    /// The topic or service name the query named.
    pub name: String,
    pub ret: i32,
    pub answer: u64,
}

/// A declared entity.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct EntityRec {
    pub k: u64,
    pub id: EntityId,
    pub kind: EntityKind,
    /// The stable name: `sub:/a#0`, `pub:/out#0`, `srv:/x#0`, `cli:/x#0`, `gc#3`.
    pub name: String,
    /// The topic or service name (empty for a guard).
    pub topic: String,
    /// The owning node's name (empty for a guard).
    pub node: String,
}

/// Every record the log carries.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum Record {
    Header(ProcessHeader),
    Node {
        k: u64,
        name: String,
        namespace: String,
    },
    Entity(EntityRec),
    NotReplayable {
        k: u64,
        reason: String,
    },
    Trailer(Trailer),
    Wait(WaitRec),
    Take(TakeRec),
    Output(OutputRec),
    Graph(GraphRec),
}

const TAG_HEADER: u8 = 1;
const TAG_NODE: u8 = 2;
const TAG_ENTITY: u8 = 3;
const TAG_NOT_REPLAYABLE: u8 = 4;
const TAG_TRAILER: u8 = 5;
const TAG_WAIT: u8 = 10;
const TAG_TAKE: u8 = 11;
const TAG_OUTPUT: u8 = 12;
const TAG_GRAPH: u8 = 13;

impl Record {
    /// The channel this record is written on.
    pub(crate) fn channel(&self) -> usize {
        match self {
            Record::Header(_)
            | Record::Node { .. }
            | Record::Entity(_)
            | Record::NotReplayable { .. }
            | Record::Trailer(_) => 0,
            Record::Wait(_) => 1,
            Record::Take(_) | Record::Graph(_) => 2,
            Record::Output(_) => 3,
        }
    }

    /// The record's order key (`0` for the header and trailer, which carry
    /// none of their own; their MCAP log time is the writer's key).
    pub(crate) fn k(&self) -> u64 {
        match self {
            Record::Header(_) | Record::Trailer(_) => 0,
            Record::Node { k, .. } | Record::NotReplayable { k, .. } => *k,
            Record::Entity(e) => e.k,
            Record::Wait(w) => w.k,
            Record::Take(t) => t.k,
            Record::Output(o) => o.k,
            Record::Graph(g) => g.k,
        }
    }

    /// Set the order key minted by the writer.
    pub(crate) fn set_k(&mut self, key: u64) {
        match self {
            Record::Header(_) | Record::Trailer(_) => {}
            Record::Node { k, .. } | Record::NotReplayable { k, .. } => *k = key,
            Record::Entity(e) => e.k = key,
            Record::Wait(w) => w.k = key,
            Record::Take(t) => t.k = key,
            Record::Output(o) => o.k = key,
            Record::Graph(g) => g.k = key,
        }
    }

    /// Append the binary encoding to `out`.
    pub(crate) fn encode(&self, out: &mut Vec<u8>) {
        let mut w = Enc(out);
        w.u8(RECORD_VERSION);
        match self {
            Record::Header(h) => {
                w.u8(TAG_HEADER);
                w.bytes(&serde_json::to_vec(h).unwrap_or_default());
            }
            Record::Node { k, name, namespace } => {
                w.u8(TAG_NODE);
                w.u64(*k);
                w.str(name);
                w.str(namespace);
            }
            Record::Entity(e) => {
                w.u8(TAG_ENTITY);
                w.u64(e.k);
                w.u32(e.id);
                w.u8(e.kind as u8);
                w.str(&e.name);
                w.str(&e.topic);
                w.str(&e.node);
            }
            Record::NotReplayable { k, reason } => {
                w.u8(TAG_NOT_REPLAYABLE);
                w.u64(*k);
                w.str(reason);
            }
            Record::Trailer(t) => {
                w.u8(TAG_TRAILER);
                w.bytes(&serde_json::to_vec(t).unwrap_or_default());
            }
            Record::Wait(r) => {
                w.u8(TAG_WAIT);
                w.u64(r.k);
                w.u32(r.thread);
                w.i32(r.ret);
                w.i64(r.timeout_ns);
                w.ids(&r.attached);
                w.ids(&r.ready);
            }
            Record::Take(r) => {
                w.u8(TAG_TAKE);
                w.u64(r.k);
                w.u32(r.thread);
                w.u32(r.entity);
                w.u8(r.kind as u8);
                w.i32(r.ret);
                w.u8(r.taken as u8);
                w.i64(r.seq);
                w.i64(r.source_ts);
                w.i64(r.received_ts);
                w.raw(&r.guid);
                w.bytes(&r.body);
            }
            Record::Output(r) => {
                w.u8(TAG_OUTPUT);
                w.u64(r.k);
                w.u32(r.thread);
                w.u32(r.entity);
                w.u8(r.kind as u8);
                w.i32(r.ret);
                w.u8(r.has_payload as u8);
                w.u32(r.len);
                w.u64(r.hash);
            }
            Record::Graph(r) => {
                w.u8(TAG_GRAPH);
                w.u64(r.k);
                w.u32(r.thread);
                w.u8(r.query as u8);
                w.str(&r.name);
                w.i32(r.ret);
                w.u64(r.answer);
            }
        }
    }

    /// Decode one record. Every length is bounds-checked against the input;
    /// trailing bytes are refused.
    pub(crate) fn decode(buf: &[u8]) -> Result<Record, String> {
        let mut r = Dec { buf, at: 0 };
        let version = r.u8()?;
        if version != RECORD_VERSION {
            return Err(format!(
                "record version {version}, this build reads {RECORD_VERSION}"
            ));
        }
        let tag = r.u8()?;
        let rec = match tag {
            TAG_HEADER => Record::Header(
                serde_json::from_slice(r.bytes()?).map_err(|e| format!("process header: {e}"))?,
            ),
            TAG_NODE => Record::Node {
                k: r.u64()?,
                name: r.string()?,
                namespace: r.string()?,
            },
            TAG_ENTITY => {
                let k = r.u64()?;
                let id = r.u32()?;
                let kind = r.u8()?;
                let kind = EntityKind::from_u8(kind)
                    .ok_or_else(|| format!("unknown entity kind {kind}"))?;
                Record::Entity(EntityRec {
                    k,
                    id,
                    kind,
                    name: r.string()?,
                    topic: r.string()?,
                    node: r.string()?,
                })
            }
            TAG_NOT_REPLAYABLE => Record::NotReplayable {
                k: r.u64()?,
                reason: r.string()?,
            },
            TAG_TRAILER => Record::Trailer(
                serde_json::from_slice(r.bytes()?).map_err(|e| format!("trailer: {e}"))?,
            ),
            TAG_WAIT => Record::Wait(WaitRec {
                k: r.u64()?,
                thread: r.u32()?,
                ret: r.i32()?,
                timeout_ns: r.i64()?,
                attached: r.ids()?,
                ready: r.ids()?,
            }),
            TAG_TAKE => {
                let k = r.u64()?;
                let thread = r.u32()?;
                let entity = r.u32()?;
                let kind = r.u8()?;
                let kind =
                    TakeKind::from_u8(kind).ok_or_else(|| format!("unknown take kind {kind}"))?;
                let ret = r.i32()?;
                let taken = r.flag()?;
                let seq = r.i64()?;
                let source_ts = r.i64()?;
                let received_ts = r.i64()?;
                let mut guid = [0u8; 16];
                guid.copy_from_slice(r.raw(16)?);
                Record::Take(TakeRec {
                    k,
                    thread,
                    entity,
                    kind,
                    ret,
                    taken,
                    seq,
                    source_ts,
                    received_ts,
                    guid,
                    body: r.bytes()?.to_vec(),
                })
            }
            TAG_OUTPUT => {
                let k = r.u64()?;
                let thread = r.u32()?;
                let entity = r.u32()?;
                let kind = r.u8()?;
                let kind = OutputKind::from_u8(kind)
                    .ok_or_else(|| format!("unknown output kind {kind}"))?;
                Record::Output(OutputRec {
                    k,
                    thread,
                    entity,
                    kind,
                    ret: r.i32()?,
                    has_payload: r.flag()?,
                    len: r.u32()?,
                    hash: r.u64()?,
                })
            }
            TAG_GRAPH => {
                let k = r.u64()?;
                let thread = r.u32()?;
                let q = r.u8()?;
                let query =
                    GraphQuery::from_u8(q).ok_or_else(|| format!("unknown graph query {q}"))?;
                Record::Graph(GraphRec {
                    k,
                    thread,
                    query,
                    name: r.string()?,
                    ret: r.i32()?,
                    answer: r.u64()?,
                })
            }
            other => return Err(format!("unknown record tag {other}")),
        };
        if r.at != buf.len() {
            return Err(format!(
                "{} trailing bytes after a record",
                buf.len() - r.at
            ));
        }
        Ok(rec)
    }
}

struct Enc<'a>(&'a mut Vec<u8>);

impl Enc<'_> {
    fn u8(&mut self, v: u8) {
        self.0.push(v);
    }
    fn u32(&mut self, v: u32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn i64(&mut self, v: i64) {
        self.0.extend_from_slice(&v.to_le_bytes());
    }
    fn raw(&mut self, b: &[u8]) {
        self.0.extend_from_slice(b);
    }
    fn bytes(&mut self, b: &[u8]) {
        self.u32(b.len() as u32);
        self.0.extend_from_slice(b);
    }
    fn str(&mut self, s: &str) {
        self.bytes(s.as_bytes());
    }
    fn ids(&mut self, ids: &[EntityId]) {
        self.u32(ids.len() as u32);
        for id in ids {
            self.u32(*id);
        }
    }
}

struct Dec<'a> {
    buf: &'a [u8],
    at: usize,
}

impl<'a> Dec<'a> {
    fn raw(&mut self, n: usize) -> Result<&'a [u8], String> {
        let end = self
            .at
            .checked_add(n)
            .filter(|&e| e <= self.buf.len())
            .ok_or_else(|| {
                format!(
                    "record truncated: {n} bytes wanted at offset {} of {}",
                    self.at,
                    self.buf.len()
                )
            })?;
        let s = &self.buf[self.at..end];
        self.at = end;
        Ok(s)
    }
    fn u8(&mut self) -> Result<u8, String> {
        Ok(self.raw(1)?[0])
    }
    fn flag(&mut self) -> Result<bool, String> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            v => Err(format!("flag byte {v}")),
        }
    }
    fn u32(&mut self) -> Result<u32, String> {
        Ok(u32::from_le_bytes(
            self.raw(4)?.try_into().unwrap_or([0; 4]),
        ))
    }
    fn i32(&mut self) -> Result<i32, String> {
        Ok(i32::from_le_bytes(
            self.raw(4)?.try_into().unwrap_or([0; 4]),
        ))
    }
    fn u64(&mut self) -> Result<u64, String> {
        Ok(u64::from_le_bytes(
            self.raw(8)?.try_into().unwrap_or([0; 8]),
        ))
    }
    fn i64(&mut self) -> Result<i64, String> {
        Ok(i64::from_le_bytes(
            self.raw(8)?.try_into().unwrap_or([0; 8]),
        ))
    }
    fn bytes(&mut self) -> Result<&'a [u8], String> {
        let n = self.u32()? as usize;
        self.raw(n)
    }
    fn string(&mut self) -> Result<String, String> {
        String::from_utf8(self.bytes()?.to_vec()).map_err(|e| format!("string field: {e}"))
    }
    fn ids(&mut self) -> Result<Vec<EntityId>, String> {
        let n = self.u32()? as usize;
        // Bound the count by what the buffer can hold BEFORE allocating.
        if n > (self.buf.len() - self.at) / 4 {
            return Err(format!("id list of {n} exceeds the record"));
        }
        (0..n).map(|_| self.u32()).collect()
    }
}

/// FNV-1a 64 of `bytes`: the output hash (no new hash dependency).
pub(crate) fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// The writer: a `BagWriter` with the four channels registered, plus the
/// order-key counter and per-channel sequences. Not thread-safe by itself;
/// the recorder holds it under one mutex so `k` is minted in file order.
pub(crate) struct LogWriter {
    bag: Option<BagWriter>,
    path: PathBuf,
    channels: [u16; 4],
    seqs: [u32; 4],
    next_k: u64,
    records: u64,
    scratch: Vec<u8>,
    /// The first write error, if any. Once set, nothing more is written.
    failed: Option<String>,
    dropped: u64,
    header_json: Vec<u8>,
}

impl LogWriter {
    /// Create the log at `path`, which must not exist yet (an existing file is
    /// never overwritten), and write the process header as its own flushed
    /// chunk. (The recorder claims its file name itself; see
    /// `Recorder::start`.)
    #[cfg(test)]
    pub(crate) fn create(path: &Path, header: &ProcessHeader) -> Result<Self, String> {
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|e| format!("cannot create delivery log {}: {e}", path.display()))?;
        Self::with_file(path, file, header)
    }

    /// Write a new log into a file the caller created with `create_new`.
    pub(crate) fn with_file(
        path: &Path,
        file: std::fs::File,
        header: &ProcessHeader,
    ) -> Result<Self, String> {
        let topics: Vec<TopicSchema> = [CH_PROCESS, CH_WAITS, CH_DELIVERIES, CH_OUTPUTS]
            .iter()
            .map(|t| TopicSchema {
                topic: (*t).to_string(),
                schema_name: SCHEMA_NAME.to_string(),
                schema_hash: 0,
                wire_fixed_size: 0,
            })
            .collect();
        let config = BagWriterConfig {
            chunk_max_bytes: CHUNK_BYTES,
            library: LIBRARY.to_string(),
            ..BagWriterConfig::default()
        };
        let sink = FileSink::from_file(file)
            .map_err(|e| format!("cannot create delivery log {}: {e}", path.display()))?;
        let bag = BagWriter::with_sink(sink, config, &topics)
            .map_err(|e| format!("cannot create delivery log {}: {e}", path.display()))?;
        let mut channels = [0u16; 4];
        for (i, t) in [CH_PROCESS, CH_WAITS, CH_DELIVERIES, CH_OUTPUTS]
            .iter()
            .enumerate()
        {
            channels[i] = bag
                .channel_id(t)
                .ok_or_else(|| format!("delivery log channel {t} was not registered"))?;
        }
        let mut w = LogWriter {
            bag: Some(bag),
            path: path.to_path_buf(),
            channels,
            seqs: [0; 4],
            next_k: 1,
            records: 0,
            scratch: Vec::with_capacity(4096),
            failed: None,
            dropped: 0,
            header_json: serde_json::to_vec(header).unwrap_or_default(),
        };
        w.append(Record::Header(header.clone()));
        w.flush();
        match &w.failed {
            Some(e) => Err(e.clone()),
            None => Ok(w),
        }
    }

    /// The log's path.
    pub(crate) fn path(&self) -> &Path {
        &self.path
    }

    /// The first write error, if one happened.
    pub(crate) fn failed(&self) -> Option<&str> {
        self.failed.as_deref()
    }

    /// Mint the next order key, stamp it on `rec`, write it, and return the
    /// key. A failed write marks the log incomplete, logs the error ONCE and
    /// counts every later record as dropped: never a silent drop.
    pub(crate) fn append(&mut self, mut rec: Record) -> u64 {
        let k = self.next_k;
        self.next_k += 1;
        rec.set_k(k);
        let Some(bag) = self.bag.as_mut() else {
            self.dropped += 1;
            return k;
        };
        let ch = rec.channel();
        self.scratch.clear();
        rec.encode(&mut self.scratch);
        let seq = self.seqs[ch];
        self.seqs[ch] = seq.wrapping_add(1);
        if let Err(e) = bag.write_message(self.channels[ch], seq, k, k, &[&self.scratch]) {
            self.fail(format!("write of record k={k} failed: {e}"));
            self.dropped += 1;
        } else {
            self.records += 1;
        }
        k
    }

    fn fail(&mut self, what: String) {
        if self.failed.is_none() {
            tracing::error!(
                log = %self.path.display(),
                error = %what,
                "rmw_cerulion delivery log: WRITE FAILED; the log is INCOMPLETE from here on \
                 and a replay of it stops where its records stop"
            );
            self.failed = Some(what);
        }
        // A poisoned writer accepts nothing more; drop it without finalize.
        self.bag = None;
    }

    /// Flush the open chunk to the file.
    pub(crate) fn flush(&mut self) {
        if let Some(bag) = self.bag.as_mut() {
            if let Err(e) = bag.flush_chunk() {
                self.fail(format!("chunk flush failed: {e}"));
            }
        }
    }

    /// Write the trailer, the process attachment and the MCAP summary, and
    /// close the file. Idempotent: a second call does nothing.
    pub(crate) fn finish(&mut self) -> Trailer {
        let trailer = Trailer {
            complete: self.failed.is_none(),
            dropped: self.dropped,
            records: self.records,
        };
        if self.bag.is_none() {
            return trailer;
        }
        self.append(Record::Trailer(trailer.clone()));
        let mut attachment = serde_json::json!({});
        if let Ok(h) = serde_json::from_slice::<serde_json::Value>(&self.header_json) {
            attachment["header"] = h;
        }
        attachment["trailer"] = serde_json::to_value(&trailer).unwrap_or_default();
        let bytes = serde_json::to_vec_pretty(&attachment).unwrap_or_default();
        if let Some(mut bag) = self.bag.take() {
            let res = bag
                .write_attachment(PROCESS_ATTACHMENT, "application/json", 0, 0, &bytes)
                .and_then(|()| bag.finalize());
            if let Err(e) = res {
                self.bag = None;
                if self.failed.is_none() {
                    tracing::error!(
                        log = %self.path.display(),
                        error = %e,
                        "rmw_cerulion delivery log: finalize failed; the records already \
                         flushed stay readable, the summary is missing"
                    );
                    self.failed = Some(format!("finalize failed: {e}"));
                }
            }
        }
        trailer
    }
}

/// How a loaded log ended.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum LogEnding {
    /// Finalized, with a trailer saying every record was written.
    Complete,
    /// Finalized or cleanly cut, but the writer reported dropped records or
    /// no trailer was written (the process died before it closed the log).
    Incomplete(String),
}

/// A log read back: the header, every record in `k` order, and how it ended.
pub(crate) struct LoadedLog {
    pub header: ProcessHeader,
    pub records: Vec<Record>,
    pub ending: LogEnding,
}

impl LoadedLog {
    /// Read a log. A log that was never closed (SIGKILL) yields every record
    /// it flushed, one torn inside a chunk every whole record before the tear,
    /// and either ends [`LogEnding::Incomplete`]; a missing or undecodable
    /// header is an error.
    pub(crate) fn read(path: &Path) -> Result<LoadedLog, String> {
        let reader = BagReader::open(path)
            .map_err(|e| format!("cannot open delivery log {}: {e}", path.display()))?;
        let (messages, completeness) = reader
            .recover_messages()
            .map_err(|e| format!("cannot read delivery log {}: {e}", path.display()))?;
        let mut header: Option<ProcessHeader> = None;
        let mut trailer: Option<Trailer> = None;
        let mut records = Vec::with_capacity(messages.len());
        for m in &messages {
            if ![CH_PROCESS, CH_WAITS, CH_DELIVERIES, CH_OUTPUTS].contains(&m.topic.as_str()) {
                return Err(format!(
                    "{} is not a delivery log: it carries channel '{}'",
                    path.display(),
                    m.topic
                ));
            }
            let rec = Record::decode(&m.data).map_err(|e| {
                format!(
                    "delivery log {}: record at log time {} on {}: {e}",
                    path.display(),
                    m.log_time,
                    m.topic
                )
            })?;
            match rec {
                Record::Header(h) => {
                    if header.is_some() {
                        return Err(format!("delivery log {}: two headers", path.display()));
                    }
                    header = Some(h);
                }
                Record::Trailer(t) => trailer = Some(t),
                other => records.push(other),
            }
        }
        let header = header.ok_or_else(|| {
            format!(
                "{} carries no process header: not a delivery log, or torn before its first chunk",
                path.display()
            )
        })?;
        if header.format != RECORD_VERSION {
            return Err(format!(
                "delivery log {} is format {}, this build reads {RECORD_VERSION}",
                path.display(),
                header.format
            ));
        }
        records.sort_by_key(Record::k);
        let ending = match (&completeness, &trailer) {
            (_, Some(t)) if !t.complete || t.dropped > 0 => LogEnding::Incomplete(format!(
                "the recorder reported a write error ({} records dropped)",
                t.dropped
            )),
            (BagCompleteness::TornTail(e), _) => LogEnding::Incomplete(format!("torn tail: {e}")),
            (_, None) => LogEnding::Incomplete(
                "no trailer: the recorded process did not close its log (killed?)".to_string(),
            ),
            (BagCompleteness::Finalized, Some(_)) => LogEnding::Complete,
            // Cut at a chunk boundary after the trailer (and any other ending
            // a newer bag crate may name): the summary is missing.
            (_, Some(_)) => {
                LogEnding::Incomplete("the log has a trailer but no MCAP summary".to_string())
            }
        };
        Ok(LoadedLog {
            header,
            records,
            ending,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header() -> ProcessHeader {
        ProcessHeader {
            format: RECORD_VERSION,
            cmdline: vec!["python3".into(), "mixer.py".into()],
            ros_distro: "humble".into(),
            built_for: "humble".into(),
            library_digest: "00ff".into(),
            pid: 42,
        }
    }

    fn sample_records() -> Vec<Record> {
        vec![
            Record::Node {
                k: 0,
                name: "mixer".into(),
                namespace: "/".into(),
            },
            Record::Entity(EntityRec {
                k: 0,
                id: 0,
                kind: EntityKind::Subscription,
                name: "sub:/a#0".into(),
                topic: "/a".into(),
                node: "mixer".into(),
            }),
            Record::Wait(WaitRec {
                k: 0,
                thread: 0,
                ret: 0,
                timeout_ns: -1,
                attached: vec![0, 1, 2],
                ready: vec![0],
            }),
            Record::Take(TakeRec {
                k: 0,
                thread: 0,
                entity: 0,
                kind: TakeKind::Message,
                ret: 0,
                taken: true,
                seq: 7,
                source_ts: 11,
                received_ts: 12,
                guid: [3; 16],
                body: vec![1, 2, 3, 4, 5],
            }),
            Record::Output(OutputRec {
                k: 0,
                thread: 0,
                entity: 1,
                kind: OutputKind::Publish,
                ret: 0,
                has_payload: true,
                len: 5,
                hash: fnv1a(b"hello"),
            }),
            Record::Graph(GraphRec {
                k: 0,
                thread: 0,
                query: GraphQuery::CountSubscribers,
                name: "/clock".into(),
                ret: 0,
                answer: 2,
            }),
            Record::NotReplayable {
                k: 0,
                reason: "x".into(),
            },
        ]
    }

    #[test]
    fn every_record_kind_round_trips_through_the_codec() {
        for rec in sample_records() {
            let mut buf = Vec::new();
            rec.encode(&mut buf);
            assert_eq!(Record::decode(&buf).expect("decodes"), rec);
        }
        let mut buf = Vec::new();
        Record::Header(header()).encode(&mut buf);
        assert_eq!(Record::decode(&buf).unwrap(), Record::Header(header()));
    }

    #[test]
    fn the_decoder_refuses_truncation_trailing_bytes_and_other_versions() {
        let mut buf = Vec::new();
        sample_records()[3].encode(&mut buf);
        for cut in 0..buf.len() {
            assert!(Record::decode(&buf[..cut]).is_err(), "cut at {cut} decoded");
        }
        let mut long = buf.clone();
        long.push(0);
        assert!(Record::decode(&long).unwrap_err().contains("trailing"));
        let mut v2 = buf.clone();
        v2[0] = RECORD_VERSION + 1;
        assert!(Record::decode(&v2).unwrap_err().contains("record version"));
        // A hostile id count is refused before any allocation.
        let mut w = Vec::new();
        let mut e = Enc(&mut w);
        e.u8(RECORD_VERSION);
        e.u8(TAG_WAIT);
        e.u64(1);
        e.u32(0);
        e.i32(0);
        e.i64(0);
        e.u32(u32::MAX);
        assert!(Record::decode(&w).unwrap_err().contains("exceeds"));
    }

    #[test]
    fn fnv1a_matches_the_published_vectors() {
        assert_eq!(fnv1a(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(fnv1a(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(fnv1a(b"foobar"), 0x8594_4171_f739_67e8);
    }

    fn temp_path(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("rdl-unit-{}-{}", std::process::id(), name));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir.join(format!("t{LOG_SUFFIX}"))
    }

    #[test]
    fn a_finished_log_round_trips_in_key_order_and_reads_complete() {
        let path = temp_path("roundtrip");
        let mut w = LogWriter::create(&path, &header()).unwrap();
        let mut keys = Vec::new();
        for rec in sample_records() {
            keys.push(w.append(rec));
        }
        let trailer = w.finish();
        assert!(trailer.complete);
        assert_eq!(trailer.records as usize, sample_records().len() + 1);
        let log = LoadedLog::read(&path).unwrap();
        assert_eq!(log.header, header());
        assert_eq!(log.ending, LogEnding::Complete);
        assert_eq!(log.records.len(), sample_records().len());
        let got: Vec<u64> = log.records.iter().map(Record::k).collect();
        assert_eq!(got, keys, "records come back in k order");
        for (got, mut want) in log.records.iter().zip(sample_records()) {
            want.set_k(got.k());
            assert_eq!(*got, want);
        }
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    #[test]
    fn a_torn_log_reads_every_whole_record_and_says_incomplete() {
        let path = temp_path("torn");
        let mut w = LogWriter::create(&path, &header()).unwrap();
        for rec in sample_records() {
            w.append(rec);
        }
        w.flush();
        let flushed_len = std::fs::metadata(&path).unwrap().len();
        for rec in sample_records() {
            w.append(rec);
        }
        w.flush();
        // Drop without finish, then tear the second chunk in half.
        drop(w);
        let full = std::fs::read(&path).unwrap();
        let cut = flushed_len as usize + (full.len() - flushed_len as usize) / 2;
        std::fs::write(&path, &full[..cut]).unwrap();
        let log = LoadedLog::read(&path).unwrap();
        // Every record of the first chunk; of the torn one, at most the
        // records that lie wholly before the cut.
        let n = sample_records().len();
        assert!(
            log.records.len() >= n && log.records.len() < 2 * n,
            "{} records recovered",
            log.records.len()
        );
        assert_eq!(
            &log.records[..n].iter().map(Record::k).collect::<Vec<_>>(),
            &(2..2 + n as u64).collect::<Vec<_>>()
        );
        match &log.ending {
            LogEnding::Incomplete(why) => assert!(why.contains("torn tail"), "{why}"),
            other => panic!("expected a torn tail, got {other:?}"),
        }
        // A log cut before its header chunk is refused by name.
        std::fs::write(&path, &full[..20]).unwrap();
        assert!(LoadedLog::read(&path).is_err());
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    #[test]
    fn an_existing_file_is_never_overwritten() {
        let path = temp_path("exists");
        std::fs::write(&path, b"someone else's").unwrap();
        let e = LogWriter::create(&path, &header()).err().expect("refused");
        assert!(e.contains("cannot create delivery log"), "{e}");
        assert_eq!(std::fs::read(&path).unwrap(), b"someone else's");
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }

    #[test]
    fn a_log_dropped_without_finish_reads_incomplete() {
        let path = temp_path("unfinished");
        let mut w = LogWriter::create(&path, &header()).unwrap();
        for rec in sample_records() {
            w.append(rec);
        }
        w.flush();
        drop(w);
        let log = LoadedLog::read(&path).unwrap();
        assert_eq!(log.records.len(), sample_records().len());
        match log.ending {
            LogEnding::Incomplete(why) => assert!(why.contains("no trailer"), "{why}"),
            other => panic!("expected incomplete, got {other:?}"),
        }
        let _ = std::fs::remove_dir_all(path.parent().unwrap());
    }
}
