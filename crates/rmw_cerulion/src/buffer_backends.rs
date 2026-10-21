// SPDX-License-Identifier: AGPL-3.0-only
//! Hosted `rosidl::Buffer` backends (ROS 2 Lyrical and Rolling): a Buffer on an
//! accelerator backend (NVIDIA's `cuda` backend, which Isaac ROS 5.0 uses)
//! crosses processes as the backend's descriptor, so its bytes stay in device
//! memory end to end.
//!
//! The backends are the installed `rosidl_buffer_backend` plugins, loaded once
//! per process through `rosidl_buffer_backend_registry` the way rmw_zenoh_cpp
//! and rmw_fastrtps_cpp load them, and never unloaded (a Buffer the application
//! still holds keeps a deleter inside its plugin library). The C++ side is
//! `shim/buffer_shim.cpp`.
//!
//! # Who takes part
//!
//! A C++ (`introspection_cpp`) endpoint whose type has `rosidl::Buffer` fields
//! and a `rosidl_typesupport_fastrtps_cpp` type support is BUFFER-AWARE when at
//! least one backend other than `cpu` is installed. A publisher whose service
//! retains history (transient-local, or volatile on a service a transient-local
//! publisher created) is not: a retained descriptor would outlive the memory it
//! names. Everything
//! else (C type support, native Cerulion nodes, recorders, the gateway) is not
//! buffer-aware and reads the native frame.
//!
//! # Discovery
//!
//! A buffer-aware subscription creates its iceoryx2 subscriber port under a
//! name every process can read back ([`encode_port_name`]): the process id, the
//! entity number and the backends it accepts. Before the port exists it tells
//! every backend about itself (`on_creating_endpoint`) under a host-unique
//! buffer gid ([`buffer_gid`]); rmw gids are not host-unique. A buffer-aware
//! publisher reads its topic's live subscriber ports on every publish (a
//! shared-memory read), and the first time it sees a port it asks every backend
//! whether that subscription can receive descriptors (`on_discovering_endpoint`),
//! keeping the verdict for the port's life; a negative answer is asked again a
//! bounded number of times ([`PeerVerdicts`]).
//!
//! # Routing ([`route`])
//!
//! * No subscriber: nothing is sent.
//! * Any subscriber that is not buffer-aware: the native frame, as R0 sends it
//!   (a Buffer on another backend is copied to host memory and counted by
//!   [`crate::rosidl_buffer::downgrade_count`]).
//! * Every subscriber buffer-aware, at least one with a positive verdict
//!   (ACCELERATED): each accelerated subscription's message is serialized with
//!   `cdr_serialize_with_endpoint` for it, identical serializations are sent
//!   once, addressed to their subscriber ports; the other buffer-aware
//!   subscriptions (the CPU GROUP) get one plain-CDR frame, in which a Buffer
//!   on another backend is written as bytes.
//! * Every subscriber buffer-aware, none accelerated: the native frame.
//!
//! On the addressed route a message whose first serialization asked no backend
//! for a descriptor (it holds no non-CPU Buffer) goes out as the native frame,
//! and so does one whose subscriber ports changed while its frames were built
//! (they are read a second time just before the send).
//!
//! An addressed frame ([`encode_frame_prefix`]) carries its own schema hash
//! ([`buffer_frame_hash`]) and the header mark
//! [`cerulion_core::wire::RMW_ADDRESSED_FRAME_MARK`]: the rmw take paths of a
//! subscription that is not buffer-aware refuse it at their schema-hash gate,
//! and the bagd tap and the gateway refuse it by its mark and count it. The CDR inside is the CDR
//! rmw_fastrtps_cpp and rmw_zenoh_cpp put on their wire for the same message
//! and backend (the S2 golden fixtures pin it).
//!
//! # Take
//!
//! A buffer-aware subscription passes over an addressed frame that does not name
//! its port, and decodes one that does through `cdr_deserialize_with_endpoint`:
//! a descriptor becomes a Buffer through its backend. A serialized take flattens
//! it back into the native frame. A backend that returns an
//! empty Buffer for a descriptor (the cuda backend's answer to a stale one) is
//! a LOST SAMPLE, counted and never delivered.
//!
//! # Counters
//!
//! [`counters`] returns every counter under a stable name ([`COUNTER_NAMES`]);
//! the zero-copy claim for a topic is `descriptors_refused`, `cpu_group_frames`
//! and `host_copies` at 0 while `descriptor_frames` grows.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

/// The tag that opens a buffer-aware subscriber port name.
pub const PORT_NAME_TAG: &str = "rbuf1";

/// Version byte of the addressed frame prefix.
pub const FRAME_VERSION: u8 = 1;

/// Frame kind: endpoint-aware CDR (Buffers may be descriptors).
pub const FRAME_KIND_ENDPOINT_CDR: u8 = 1;
/// Frame kind: plain CDR (every Buffer written as bytes).
pub const FRAME_KIND_PLAIN_CDR: u8 = 2;

/// Fixed bytes of the addressed frame prefix before the recipient list:
/// version (1), kind (1), recipient count (u16), CDR length (u32), publisher
/// buffer gid (16).
pub const FRAME_PREFIX_FIXED: usize = 24;

static DESCRIPTOR_FRAMES: AtomicU64 = AtomicU64::new(0);
static CPU_GROUP_FRAMES: AtomicU64 = AtomicU64::new(0);
static NATIVE_FRAMES: AtomicU64 = AtomicU64::new(0);
static UNSENT_NO_SUBSCRIBER: AtomicU64 = AtomicU64::new(0);
static LOST_SAMPLES: AtomicU64 = AtomicU64::new(0);
static DESCRIPTOR_TAKES: AtomicU64 = AtomicU64::new(0);
static SKIPPED_FRAMES: AtomicU64 = AtomicU64::new(0);

/// The stable name of every counter [`counters`] returns, in that order.
pub const COUNTER_NAMES: [&str; 11] = [
    "rmw_cerulion.rosidl_buffer.descriptor_frames",
    "rmw_cerulion.rosidl_buffer.descriptors_created",
    "rmw_cerulion.rosidl_buffer.descriptors_refused",
    "rmw_cerulion.rosidl_buffer.cpu_group_frames",
    "rmw_cerulion.rosidl_buffer.native_frames",
    "rmw_cerulion.rosidl_buffer.host_copies",
    "rmw_cerulion.rosidl_buffer.unsent_no_subscriber",
    "rmw_cerulion.rosidl_buffer.descriptor_takes",
    "rmw_cerulion.rosidl_buffer.imports",
    "rmw_cerulion.rosidl_buffer.lost_samples",
    "rmw_cerulion.rosidl_buffer.skipped_frames",
];

/// Every process counter of the hosted-backend path, by stable name:
///
/// * `descriptor_frames`: addressed endpoint-CDR frames published.
/// * `descriptors_created`: descriptors a backend created for a subscriber.
/// * `descriptors_refused`: Buffers a backend declined to describe, each then
///   copied to host memory inside the CDR.
/// * `cpu_group_frames`: plain-CDR frames published to buffer-aware
///   subscriptions that accept no backend of the Buffer; each non-CPU Buffer in
///   one was copied to host memory.
/// * `native_frames`: native frames a buffer-aware publisher sent because a
///   subscriber that is not buffer-aware, or no accelerated one, was attached,
///   the message held no non-CPU Buffer, or the ports changed mid-publish.
/// * `host_copies`: every non-CPU Buffer copied to host memory for a native
///   frame ([`crate::rosidl_buffer::downgrade_count`]).
/// * `unsent_no_subscriber`: buffer-aware publishes with no subscriber attached.
/// * `descriptor_takes`: addressed frames a subscription decoded and delivered.
/// * `imports`: descriptors a backend turned back into a Buffer.
/// * `lost_samples`: addressed frames dropped because a backend returned an
///   empty Buffer for a descriptor (stale, or not importable here).
/// * `skipped_frames`: addressed frames a subscription passed over (for
///   another port).
pub fn counters() -> Vec<(&'static str, u64)> {
    let values = [
        DESCRIPTOR_FRAMES.load(Ordering::Relaxed),
        shim_counter(0),
        shim_counter(1),
        CPU_GROUP_FRAMES.load(Ordering::Relaxed),
        NATIVE_FRAMES.load(Ordering::Relaxed),
        crate::rosidl_buffer::downgrade_count(),
        UNSENT_NO_SUBSCRIBER.load(Ordering::Relaxed),
        DESCRIPTOR_TAKES.load(Ordering::Relaxed),
        shim_counter(2),
        LOST_SAMPLES.load(Ordering::Relaxed),
        SKIPPED_FRAMES.load(Ordering::Relaxed),
    ];
    COUNTER_NAMES.iter().copied().zip(values).collect()
}

/// One `name=value` pair per counter, space separated, for a log line.
pub fn counters_line() -> String {
    counters()
        .iter()
        .map(|(n, v)| format!("{}={v}", n.trim_start_matches("rmw_cerulion.rosidl_buffer.")))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(cerulion_has_buffer_backends)]
fn shim_counter(which: i32) -> u64 {
    // SAFETY: a plain read of an atomic slot; out-of-range slots read 0.
    unsafe { ffi::rmw_cerulion_bb_counter(which) }
}

#[cfg(not(cerulion_has_buffer_backends))]
fn shim_counter(_which: i32) -> u64 {
    0
}

/// The 16-byte gid a backend sees for a buffer-aware endpoint: the process's
/// pid-namespace id (u32 LE, the low half of the `/proc/self/ns/pid` inode),
/// its process id (u32 LE) and the endpoint's entity number (u64 LE). Unique on
/// the host while the process lives, across containers that share `/dev/shm`
/// with pid namespaces of their own (rmw gids are the entity number plus the
/// schema hash, the same in every process).
pub fn buffer_gid(pid_ns: u32, pid: u32, entity: u64) -> [u8; 16] {
    let mut gid = [0u8; 16];
    gid[..4].copy_from_slice(&pid_ns.to_le_bytes());
    gid[4..8].copy_from_slice(&pid.to_le_bytes());
    gid[8..].copy_from_slice(&entity.to_le_bytes());
    gid
}

/// This process's pid-namespace id: the low 32 bits of the inode
/// `/proc/self/ns/pid` names (`pid:[4026531836]`), 0 where there is none.
pub fn pid_namespace_id() -> u32 {
    static ID: std::sync::OnceLock<u32> = std::sync::OnceLock::new();
    *ID.get_or_init(|| {
        std::fs::read_link("/proc/self/ns/pid")
            .ok()
            .and_then(|p| {
                let s = p.to_string_lossy().into_owned();
                let digits: String = s.chars().filter(char::is_ascii_digit).collect();
                digits.parse::<u64>().ok()
            })
            .map_or(0, |inode| inode as u32)
    })
}

/// A buffer-aware subscription as its port name describes it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AwarePeer {
    /// Pid-namespace id of the subscription's process ([`pid_namespace_id`]).
    pub pid_ns: u32,
    /// Process id of the subscription.
    pub pid: u32,
    /// Its entity number in that process.
    pub entity: u64,
    /// Backends other than `cpu` it accepts (empty: CPU only).
    pub backends: Vec<String>,
}

impl AwarePeer {
    /// The gid its backends know it by ([`buffer_gid`]).
    pub fn gid(&self) -> [u8; 16] {
        buffer_gid(self.pid_ns, self.pid, self.entity)
    }
}

/// The port name of a buffer-aware subscription:
/// `rbuf1;<pid namespace hex>;<pid hex>;<entity hex>;<backend>,<backend>…`. `None` when it does
/// not fit `max_len` bytes or a backend name holds a character other than an
/// ASCII letter, digit, `_` or `-`.
pub fn encode_port_name(peer: &AwarePeer, max_len: usize) -> Option<String> {
    if peer.backends.iter().any(|b| {
        b.is_empty()
            || !b
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    }) {
        return None;
    }
    let name = format!(
        "{PORT_NAME_TAG};{:x};{:x};{:x};{}",
        peer.pid_ns,
        peer.pid,
        peer.entity,
        peer.backends.join(",")
    );
    (name.len() <= max_len).then_some(name)
}

/// Parse a port name written by [`encode_port_name`]. Any other name (an
/// unnamed port reads `""`) is `None`: that subscriber is not buffer-aware.
pub fn parse_port_name(name: &str) -> Option<AwarePeer> {
    let mut parts = name.split(';');
    if parts.next()? != PORT_NAME_TAG {
        return None;
    }
    let pid_ns = u32::from_str_radix(parts.next()?, 16).ok()?;
    let pid = u32::from_str_radix(parts.next()?, 16).ok()?;
    let entity = u64::from_str_radix(parts.next()?, 16).ok()?;
    let backends_field = parts.next()?;
    if parts.next().is_some() {
        return None;
    }
    let backends = backends_field
        .split(',')
        .filter(|b| !b.is_empty())
        .map(str::to_owned)
        .collect();
    Some(AwarePeer {
        pid_ns,
        pid,
        entity,
        backends,
    })
}

/// The schema hash an addressed frame of a type with native schema hash
/// `native` carries: FNV-1a 64 over a fixed tag and `native` (LE). Distinct
/// from `native`, so a reader of the native type refuses the frame.
pub fn buffer_frame_hash(native: u64) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in b"rmw_cerulion/rosidl_buffer_cdr/1"
        .iter()
        .chain(native.to_le_bytes().iter())
    {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    if h == native {
        h ^ 1
    } else {
        h
    }
}

/// Length of the prefix for `recipients` ports.
pub fn frame_prefix_len(recipients: usize) -> usize {
    FRAME_PREFIX_FIXED + 16 * recipients
}

/// Write the addressed frame prefix into `out` (exactly
/// [`frame_prefix_len`] bytes). Panics when `out` has another length or there
/// are more than `u16::MAX` recipients (callers size both).
pub fn encode_frame_prefix(
    out: &mut [u8],
    kind: u8,
    publisher_gid: &[u8; 16],
    recipients: &[u128],
    cdr_len: u32,
) {
    assert_eq!(out.len(), frame_prefix_len(recipients.len()));
    let n = u16::try_from(recipients.len()).expect("recipient count fits u16");
    out[0] = FRAME_VERSION;
    out[1] = kind;
    out[2..4].copy_from_slice(&n.to_le_bytes());
    out[4..8].copy_from_slice(&cdr_len.to_le_bytes());
    out[8..24].copy_from_slice(publisher_gid);
    for (i, r) in recipients.iter().enumerate() {
        let at = FRAME_PREFIX_FIXED + 16 * i;
        out[at..at + 16].copy_from_slice(&r.to_le_bytes());
    }
}

/// An addressed frame's payload, parsed.
#[derive(Debug, PartialEq, Eq)]
pub struct ParsedFrame<'a> {
    /// [`FRAME_KIND_ENDPOINT_CDR`] or [`FRAME_KIND_PLAIN_CDR`].
    pub kind: u8,
    /// The publisher's buffer gid.
    pub publisher_gid: [u8; 16],
    /// The recipient port ids, as LE bytes, 16 per port.
    recipients: &'a [u8],
    /// The CDR (encapsulation first).
    pub cdr: &'a [u8],
}

impl ParsedFrame<'_> {
    /// Whether the frame names the port `port`.
    pub fn addresses(&self, port: u128) -> bool {
        let want = port.to_le_bytes();
        self.recipients.chunks_exact(16).any(|c| c == want)
    }
}

/// Parse an addressed frame's payload (the bytes after the wire header). `Err`
/// names what is wrong with it.
pub fn parse_frame(payload: &[u8]) -> Result<ParsedFrame<'_>, &'static str> {
    if payload.len() < FRAME_PREFIX_FIXED {
        return Err("shorter than the addressed frame prefix");
    }
    if payload[0] != FRAME_VERSION {
        return Err("unknown addressed frame version");
    }
    let kind = payload[1];
    if kind != FRAME_KIND_ENDPOINT_CDR && kind != FRAME_KIND_PLAIN_CDR {
        return Err("unknown addressed frame kind");
    }
    let n = usize::from(u16::from_le_bytes([payload[2], payload[3]]));
    let cdr_len = u32::from_le_bytes(payload[4..8].try_into().expect("4 bytes")) as usize;
    let mut publisher_gid = [0u8; 16];
    publisher_gid.copy_from_slice(&payload[8..24]);
    let start = frame_prefix_len(n);
    let end = start
        .checked_add(cdr_len)
        .ok_or("CDR length overflows")?;
    if payload.len() < end {
        return Err("CDR runs past the frame");
    }
    Ok(ParsedFrame {
        kind,
        publisher_gid,
        recipients: &payload[FRAME_PREFIX_FIXED..start],
        cdr: &payload[start..end],
    })
}

/// What a publisher knows about one subscriber port.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PeerVerdict {
    /// Not buffer-aware: it reads native frames.
    NotAware,
    /// Buffer-aware; `accelerated` is whether a backend it accepts can hand it
    /// descriptors.
    Aware {
        /// The gid its backends know it by.
        gid: [u8; 16],
        /// At least one accepted backend other than `cpu` reported it compatible.
        accelerated: bool,
    },
}

/// The routing of one publish ([`route`]).
#[derive(Debug, PartialEq, Eq)]
pub enum Route {
    /// No subscriber is attached: send nothing.
    Nobody,
    /// Send the native frame.
    Native,
    /// Send addressed frames.
    Addressed {
        /// Accelerated subscriptions: port id and gid.
        accelerated: Vec<(u128, [u8; 16])>,
        /// Buffer-aware subscriptions that get plain CDR.
        cpu_group: Vec<u128>,
    },
}

/// Decide one publish from the verdicts of the attached subscriber ports, in
/// port-list order.
pub fn route(peers: &[(u128, PeerVerdict)]) -> Route {
    if peers.is_empty() {
        return Route::Nobody;
    }
    let mut accelerated = Vec::new();
    let mut cpu_group = Vec::new();
    for (port, verdict) in peers {
        match verdict {
            PeerVerdict::NotAware => return Route::Native,
            PeerVerdict::Aware {
                gid,
                accelerated: true,
            } => accelerated.push((*port, *gid)),
            PeerVerdict::Aware { .. } => cpu_group.push(*port),
        }
    }
    if accelerated.is_empty() {
        return Route::Native;
    }
    Route::Addressed {
        accelerated,
        cpu_group,
    }
}

/// How long a negative verdict for a buffer-aware port that accepts a
/// backend is kept before the backends are asked again: a subscription in
/// another process registers with its backends before its port exists, but a
/// backend's own host-wide registry can lag (the cuda backend refreshes its
/// view on discovery).
pub const NEGATIVE_VERDICT_RETRY: std::time::Duration = std::time::Duration::from_millis(500);

/// How many times a negative verdict is asked again before it is kept for the
/// port's life. Discovery runs inside a publish, so a peer the backends keep
/// refusing (another uid, another device) costs at most this many extra
/// discovery calls.
pub const NEGATIVE_VERDICT_RETRIES: u8 = 4;

/// A publisher's verdict cache, keyed by subscriber port id. A port's name is
/// fixed for its life and its id is never reused while it lives, so a verdict
/// stays valid until the port leaves the list ([`PeerVerdicts::retain_live`]).
#[derive(Default)]
pub struct PeerVerdicts {
    /// Verdict, when a negative one was last asked (`None`: final), and how
    /// many retries it has left.
    entries: HashMap<u128, (PeerVerdict, Option<std::time::Instant>, u8)>,
}

impl PeerVerdicts {
    /// The verdict for `port` named `name`. On first sight a buffer-aware
    /// peer that accepts at least one backend is asked about through
    /// `discover` (the backends' answer); one that accepts none is CPU-only
    /// for life without asking. A negative answer is asked again after
    /// [`NEGATIVE_VERDICT_RETRY`], at most [`NEGATIVE_VERDICT_RETRIES`] times.
    pub fn verdict(
        &mut self,
        port: u128,
        name: &str,
        now: std::time::Instant,
        discover: &mut dyn FnMut(&AwarePeer) -> bool,
    ) -> PeerVerdict {
        let retries_left = match self.entries.get(&port) {
            None => NEGATIVE_VERDICT_RETRIES,
            Some((v, None, _)) => return v.clone(),
            Some((v, Some(t), left)) => {
                if now.duration_since(*t) < NEGATIVE_VERDICT_RETRY {
                    return v.clone();
                }
                *left
            }
        };
        let (verdict, recheck) = match parse_port_name(name) {
            None => (PeerVerdict::NotAware, false),
            Some(peer) if peer.backends.is_empty() => (
                PeerVerdict::Aware {
                    gid: peer.gid(),
                    accelerated: false,
                },
                false,
            ),
            Some(peer) => {
                let accelerated = discover(&peer);
                (
                    PeerVerdict::Aware {
                        gid: peer.gid(),
                        accelerated,
                    },
                    !accelerated && retries_left > 0,
                )
            }
        };
        let (checked, left) = if recheck {
            (Some(now), retries_left - 1)
        } else {
            (None, 0)
        };
        self.entries.insert(port, (verdict.clone(), checked, left));
        verdict
    }

    /// Forget every port not in `live`.
    pub fn retain_live(&mut self, live: &[(u128, PeerVerdict)]) {
        if self.entries.len() != live.len() {
            self.entries
                .retain(|port, _| live.iter().any(|(p, _)| p == port));
        }
    }
}

/// Group identical serializations: `outputs[i]` is the CDR serialized for
/// recipient `i`; the result lists each distinct CDR once with the indices of
/// the recipients it serves, in first-seen order.
pub fn group_identical(outputs: &[Vec<u8>]) -> Vec<(usize, Vec<usize>)> {
    let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
    for (i, out) in outputs.iter().enumerate() {
        match groups.iter_mut().find(|(first, _)| outputs[*first] == *out) {
            Some((_, members)) => members.push(i),
            None => groups.push((i, vec![i])),
        }
    }
    groups
}

#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_descriptor_frame() {
    DESCRIPTOR_FRAMES.fetch_add(1, Ordering::Relaxed);
}
#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_cpu_group_frame() {
    CPU_GROUP_FRAMES.fetch_add(1, Ordering::Relaxed);
}
#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_native_frame() {
    NATIVE_FRAMES.fetch_add(1, Ordering::Relaxed);
}
#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_unsent() {
    UNSENT_NO_SUBSCRIBER.fetch_add(1, Ordering::Relaxed);
}
#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_lost_sample() {
    LOST_SAMPLES.fetch_add(1, Ordering::Relaxed);
}
#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_descriptor_take() {
    DESCRIPTOR_TAKES.fetch_add(1, Ordering::Relaxed);
}
#[cfg_attr(not(cerulion_has_buffer_backends), allow(dead_code))]
pub(crate) fn note_skipped() {
    SKIPPED_FRAMES.fetch_add(1, Ordering::Relaxed);
}

/// The C++ shim's surface.
#[cfg(cerulion_has_buffer_backends)]
pub mod ffi {
    use std::os::raw::{c_char, c_int, c_void};

    extern "C" {
        pub fn rmw_cerulion_bb_init() -> c_int;
        pub fn rmw_cerulion_bb_backend_names(out: *mut c_char, cap: usize) -> usize;
        pub fn rmw_cerulion_bb_metadata_is_empty(backend: *const c_char) -> c_int;
        pub fn rmw_cerulion_bb_last_error() -> *const c_char;
        pub fn rmw_cerulion_bb_counter(which: c_int) -> u64;
        pub fn rmw_cerulion_bb_thread_descriptor_calls() -> u64;
        pub fn rmw_cerulion_bb_has_buffer_fields(callbacks: *const c_void) -> c_int;
        pub fn rmw_cerulion_bb_on_creating(
            gid: *const u8,
            is_publisher: c_int,
            topic_type: *const c_char,
        ) -> c_int;
        pub fn rmw_cerulion_bb_on_discovering(
            gid: *const u8,
            topic_type: *const c_char,
            accepted_csv: *const c_char,
        ) -> c_int;
        pub fn rmw_cerulion_bb_serialize(
            callbacks: *const c_void,
            msg: *const c_void,
            gid: *const u8,
            topic_type: *const c_char,
            out: *mut u8,
            cap: usize,
            len: *mut usize,
        ) -> c_int;
        pub fn rmw_cerulion_bb_deserialize(
            callbacks: *const c_void,
            cdr: *const u8,
            len: usize,
            msg: *mut c_void,
            gid: *const u8,
            topic_type: *const c_char,
        ) -> c_int;
    }

    /// This thread's last shim failure reason.
    pub fn last_error() -> String {
        // SAFETY: the shim returns a NUL-terminated thread-local string.
        unsafe { std::ffi::CStr::from_ptr(rmw_cerulion_bb_last_error()) }
            .to_string_lossy()
            .into_owned()
    }
}

/// The backends other than `cpu` this process has loaded, loading the
/// installed plugins on the first call. Empty on a build without hosted
/// backends, or when the registry failed (logged once).
pub fn installed_backends() -> Vec<String> {
    static LOADED: std::sync::Once = std::sync::Once::new();
    LOADED.call_once(load_backends);
    current_backend_names()
}

#[cfg(cerulion_has_buffer_backends)]
fn load_backends() {
    // SAFETY: a plain call into the noexcept shim.
    if unsafe { ffi::rmw_cerulion_bb_init() } < 0 {
        tracing::warn!(
            reason = %ffi::last_error(),
            "rosidl::Buffer backends could not be loaded; non-CPU Buffers are carried \
             as host copies"
        );
    }
}

#[cfg(not(cerulion_has_buffer_backends))]
fn load_backends() {}

/// The loaded backend names as the shim lists them now.
#[cfg(cerulion_has_buffer_backends)]
fn current_backend_names() -> Vec<String> {
    let mut buf = vec![0u8; 256];
    loop {
        // SAFETY: the shim writes at most `buf.len()` bytes, NUL included.
        let n = unsafe { ffi::rmw_cerulion_bb_backend_names(buf.as_mut_ptr().cast(), buf.len()) };
        if n < buf.len() {
            buf.truncate(n);
            break;
        }
        buf = vec![0u8; n + 1];
    }
    let csv = String::from_utf8_lossy(&buf).into_owned();
    csv.split(',')
        .filter(|s| !s.is_empty())
        .map(str::to_owned)
        .collect()
}

#[cfg(not(cerulion_has_buffer_backends))]
fn current_backend_names() -> Vec<String> {
    Vec::new()
}

/// The backends of `installed` a subscription accepts, from its
/// `acceptable_buffer_backends` text: none for `None`, empty or `cpu` only,
/// every one for `any`, else the named ones that are installed. Malformed text
/// accepts none (the create-time validator refuses it first).
pub fn accepted_backends(requested: Option<&[u8]>, installed: &[String]) -> Vec<String> {
    let Some(text) = requested.and_then(|r| std::str::from_utf8(r).ok()) else {
        return Vec::new();
    };
    let names: Vec<&str> = text
        .split(',')
        .map(|n| n.trim_matches(|c| c == ' ' || c == '\t'))
        .filter(|n| !n.is_empty())
        .collect();
    if names.contains(&"any") {
        return installed.to_vec();
    }
    installed
        .iter()
        .filter(|b| names.contains(&b.as_str()))
        .cloned()
        .collect()
}

/// The publisher half of a buffer-aware endpoint.
#[cfg(cerulion_has_buffer_backends)]
pub struct BufferPublisher {
    callbacks: *const std::os::raw::c_void,
    gid: [u8; 16],
    topic_type: std::ffi::CString,
    frame_hash: u64,
    view: cerulion_core::transport::TopicSubscriberView,
    verdicts: PeerVerdicts,
    ports: Vec<(u128, String)>,
    live: Vec<(u128, PeerVerdict)>,
    scratch: Vec<u8>,
    outputs: Vec<Vec<u8>>,
    last_route: Option<RouteKind>,
}

/// Which kind of route a publish took, for the transition log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteKind {
    /// [`Route::Nobody`].
    Nobody,
    /// [`Route::Native`].
    Native,
    /// [`Route::Addressed`].
    Addressed,
}

// SAFETY: `callbacks` points at rosidl's immutable, process-lifetime generated
// type support data, which is only ever read; every other field is owned. The
// value is only used under the publisher's own mutex.
#[cfg(cerulion_has_buffer_backends)]
unsafe impl Send for BufferPublisher {}
#[cfg(cerulion_has_buffer_backends)]
unsafe impl Sync for BufferPublisher {}

/// Resolve the fastrtps_cpp callbacks of a C++ message type support and keep
/// them when the type has Buffer fields.
///
/// # Safety
/// `ts` is null or a valid rosidl message type support handle.
#[cfg(cerulion_has_buffer_backends)]
pub(crate) unsafe fn buffer_callbacks(
    ts: *const crate::ffi::rosidl_message_type_support_t,
) -> Option<*const std::os::raw::c_void> {
    let handle = crate::api::resolve_message_typesupport(ts, b"rosidl_typesupport_fastrtps_cpp\0");
    let Some(handle) = handle else {
        crate::ffi::rcutils_reset_error_best_effort();
        return None;
    };
    let data = (*handle).data;
    (!data.is_null() && ffi::rmw_cerulion_bb_has_buffer_fields(data) == 1).then_some(data)
}

#[cfg(cerulion_has_buffer_backends)]
impl BufferPublisher {
    /// Build the publisher half and announce it to every backend.
    pub(crate) fn new(
        callbacks: *const std::os::raw::c_void,
        entity: u64,
        topic_type: &str,
        native_hash: u64,
        view: cerulion_core::transport::TopicSubscriberView,
    ) -> Result<Self, String> {
        let gid = buffer_gid(pid_namespace_id(), std::process::id(), entity);
        let topic_type = std::ffi::CString::new(topic_type).map_err(|e| e.to_string())?;
        // SAFETY: gid is 16 bytes; topic_type is NUL-terminated.
        if unsafe { ffi::rmw_cerulion_bb_on_creating(gid.as_ptr(), 1, topic_type.as_ptr()) } != 0
        {
            return Err(ffi::last_error());
        }
        Ok(Self {
            callbacks,
            gid,
            topic_type,
            frame_hash: buffer_frame_hash(native_hash),
            view,
            verdicts: PeerVerdicts::default(),
            ports: Vec::new(),
            live: Vec::new(),
            scratch: vec![0u8; 64 * 1024],
            outputs: Vec::new(),
            last_route: None,
        })
    }

    /// This endpoint's buffer gid.
    pub fn gid(&self) -> [u8; 16] {
        self.gid
    }

    /// Read the live subscriber ports and decide this publish.
    pub(crate) fn route_now(&mut self, topic: &str) -> Route {
        self.ports.clear();
        let ports = &mut self.ports;
        self.view
            .for_each_subscriber(|id, name| ports.push((id, name.to_owned())));
        let now = std::time::Instant::now();
        let topic_type = &self.topic_type;
        self.live.clear();
        for (id, name) in &self.ports {
            let mut discover = |peer: &AwarePeer| -> bool {
                let csv = std::ffi::CString::new(peer.backends.join(",")).unwrap_or_default();
                let gid = peer.gid();
                // SAFETY: gid is 16 bytes; both strings are NUL-terminated.
                let r = unsafe {
                    ffi::rmw_cerulion_bb_on_discovering(gid.as_ptr(), topic_type.as_ptr(), csv.as_ptr())
                };
                if r == 2 {
                    tracing::warn!(
                        topic = %topic,
                        reason = %ffi::last_error(),
                        "a buffer backend failed while discovering a subscription; it receives \
                         CPU data"
                    );
                }
                r == 1
            };
            let v = self.verdicts.verdict(*id, name, now, &mut discover);
            self.live.push((*id, v));
        }
        self.verdicts.retain_live(&self.live);
        let r = route(&self.live);
        let kind = match r {
            Route::Nobody => RouteKind::Nobody,
            Route::Native => RouteKind::Native,
            Route::Addressed { .. } => RouteKind::Addressed,
        };
        if self.last_route != Some(kind) && kind != RouteKind::Nobody {
            self.log_route(topic, &r);
        }
        if kind != RouteKind::Nobody {
            self.last_route = Some(kind);
        }
        r
    }

    fn log_route(&self, topic: &str, r: &Route) {
        let not_aware = self
            .live
            .iter()
            .filter(|(_, v)| *v == PeerVerdict::NotAware)
            .count();
        match r {
            Route::Addressed {
                accelerated,
                cpu_group,
            } => tracing::info!(
                topic = %topic,
                accelerated = accelerated.len(),
                cpu_group = cpu_group.len(),
                "rosidl::Buffer frames now go to subscribers as backend descriptors (device \
                 memory stays on the device); buffer-aware subscribers that accept no backend \
                 of a Buffer get its bytes"
            ),
            _ => tracing::info!(
                topic = %topic,
                subscribers = self.live.len(),
                not_buffer_aware = not_aware,
                "rosidl::Buffer frames now go out as native frames, with every non-CPU Buffer \
                 copied to host memory: a subscriber that is not buffer-aware is attached (a \
                 native node, a recorder, the gateway, rclpy), or none can receive descriptors"
            ),
        }
    }

    /// Serialize `msg` for one recipient (`gid`, or plain CDR for `None`) into
    /// `self.scratch`, growing it on demand. Returns the CDR length.
    ///
    /// # Safety
    /// `msg` is a live C++ message of this endpoint's type.
    unsafe fn serialize_one(
        &mut self,
        msg: *const std::os::raw::c_void,
        gid: Option<&[u8; 16]>,
    ) -> Result<usize, String> {
        for _ in 0..3 {
            let mut len = 0usize;
            let r = ffi::rmw_cerulion_bb_serialize(
                self.callbacks,
                msg,
                gid.map_or(std::ptr::null(), |g| g.as_ptr()),
                self.topic_type.as_ptr(),
                self.scratch.as_mut_ptr(),
                self.scratch.len(),
                &mut len,
            );
            match r {
                0 => return Ok(len),
                1 => self.scratch = vec![0u8; len.max(self.scratch.len() * 2)],
                _ => return Err(ffi::last_error()),
            }
        }
        Err("the serialization did not fit the type support's own size estimate".to_owned())
    }

    /// Build and send the addressed frames of one publish. Every frame is
    /// stamped with `timestamp_ns` and the one sequence number
    /// `mint_sequence` returns, which is called only once every frame is
    /// built and the subscriber ports are read again and found unchanged.
    /// [`AddressedOutcome::NoDeviceBuffer`] when the first serialization asked
    /// no backend for a descriptor (the message holds no non-CPU Buffer), and
    /// [`AddressedOutcome::PortsChanged`] when a port came or went while the
    /// frames were built: nothing is sent and no number is spent in either case,
    /// and the caller sends the native frame.
    ///
    /// # Safety
    /// `msg` is a live C++ message of this endpoint's type.
    #[allow(clippy::too_many_arguments)]
    pub(crate) unsafe fn publish_addressed(
        &mut self,
        msg: *const std::os::raw::c_void,
        accelerated: &[(u128, [u8; 16])],
        cpu_group: &[u128],
        timestamp_ns: u64,
        mint_sequence: &mut dyn FnMut() -> u32,
        send: &mut dyn FnMut(&[u8]) -> Result<(), String>,
    ) -> Result<AddressedOutcome, String> {
        let mut outputs = std::mem::take(&mut self.outputs);
        outputs.clear();
        for (i, (_, gid)) in accelerated.iter().enumerate() {
            let before = ffi::rmw_cerulion_bb_thread_descriptor_calls();
            let len = self.serialize_one(msg, Some(gid))?;
            let asked = ffi::rmw_cerulion_bb_thread_descriptor_calls() != before;
            outputs.push(self.scratch[..len].to_vec());
            self.scratch[..len].fill(0);
            if i == 0 && !asked {
                self.outputs = outputs;
                return Ok(AddressedOutcome::NoDeviceBuffer);
            }
        }
        let cpu_cdr = if cpu_group.is_empty() {
            None
        } else {
            let len = self.serialize_one(msg, None)?;
            let cdr = self.scratch[..len].to_vec();
            self.scratch[..len].fill(0);
            Some(cdr)
        };
        let mut frames: Vec<(u8, Vec<u8>)> = Vec::new();
        for (first, members) in group_identical(&outputs) {
            let recipients: Vec<u128> = members.iter().map(|i| accelerated[*i].0).collect();
            frames.push((
                FRAME_KIND_ENDPOINT_CDR,
                self.build_frame(FRAME_KIND_ENDPOINT_CDR, &recipients, &outputs[first], timestamp_ns)?,
            ));
        }
        if let Some(cdr) = cpu_cdr {
            frames.push((
                FRAME_KIND_PLAIN_CDR,
                self.build_frame(FRAME_KIND_PLAIN_CDR, cpu_group, &cdr, timestamp_ns)?,
            ));
        }
        self.outputs = outputs;
        if !self.ports_unchanged() {
            return Ok(AddressedOutcome::PortsChanged);
        }
        let sequence = mint_sequence();
        let mut sent = 0;
        for (kind, mut frame) in frames {
            frame[20..24].copy_from_slice(&sequence.to_le_bytes());
            send(&frame)?;
            if kind == FRAME_KIND_ENDPOINT_CDR {
                note_descriptor_frame();
            } else {
                note_cpu_group_frame();
            }
            sent += 1;
        }
        Ok(AddressedOutcome::Sent(sent))
    }

    /// Whether the data service's subscriber ports are still exactly the ones
    /// [`Self::route_now`] read.
    fn ports_unchanged(&mut self) -> bool {
        let mut now: Vec<u128> = Vec::with_capacity(self.live.len());
        self.view.for_each_subscriber(|id, _| now.push(id));
        now.len() == self.live.len() && now.iter().zip(&self.live).all(|(a, (b, _))| a == b)
    }

    /// One addressed frame: wire header (sequence 0, stamped at send), prefix,
    /// CDR.
    fn build_frame(
        &self,
        kind: u8,
        recipients: &[u128],
        cdr: &[u8],
        timestamp_ns: u64,
    ) -> Result<Vec<u8>, String> {
        use cerulion_core::wire::WireHeader;
        let prefix = frame_prefix_len(recipients.len());
        let total = WireHeader::SIZE + prefix + cdr.len();
        let total_u32 = u32::try_from(total).map_err(|_| "frame exceeds 4 GiB".to_owned())?;
        let cdr_len = u32::try_from(cdr.len()).map_err(|_| "CDR exceeds 4 GiB".to_owned())?;
        let mut frame = vec![0u8; total];
        let header = WireHeader {
            schema_hash: self.frame_hash,
            total_size: total_u32,
            offset_table_offset: cerulion_core::wire::RMW_ADDRESSED_FRAME_MARK,
            offset_table_count: 0,
            sequence: 0,
            timestamp_ns,
        };
        header.write_to_buf(&mut frame[..WireHeader::SIZE]);
        encode_frame_prefix(
            &mut frame[WireHeader::SIZE..WireHeader::SIZE + prefix],
            kind,
            &self.gid,
            recipients,
            cdr_len,
        );
        frame[WireHeader::SIZE + prefix..].copy_from_slice(cdr);
        Ok(frame)
    }
}

/// What [`BufferPublisher::publish_addressed`] did.
#[cfg(cerulion_has_buffer_backends)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AddressedOutcome {
    /// This many frames were sent.
    Sent(usize),
    /// The message holds no non-CPU Buffer: nothing sent.
    NoDeviceBuffer,
    /// The subscriber ports changed while the frames were built: nothing sent.
    PortsChanged,
}

/// The subscription half of a buffer-aware endpoint.
#[cfg(cerulion_has_buffer_backends)]
pub struct BufferSubscription {
    callbacks: *const std::os::raw::c_void,
    topic_type: std::ffi::CString,
    /// The schema hash of this type's addressed frames.
    pub frame_hash: u64,
    /// This subscription's subscriber port id.
    pub port_id: u128,
    /// The gid its backends know it by ([`buffer_gid`]).
    pub gid: [u8; 16],
}

// SAFETY: as for BufferPublisher: `callbacks` is immutable process-lifetime
// type support data, only read.
#[cfg(cerulion_has_buffer_backends)]
unsafe impl Send for BufferSubscription {}
#[cfg(cerulion_has_buffer_backends)]
unsafe impl Sync for BufferSubscription {}

/// What a take did with an addressed frame.
#[cfg(cerulion_has_buffer_backends)]
pub(crate) enum TakeOutcome {
    /// Decoded into the message: deliver it.
    Delivered,
    /// Addressed to another port: not this subscription's.
    NotMine,
    /// A backend returned an empty Buffer for a descriptor: dropped.
    Lost,
    /// The frame could not be decoded; the reason.
    Failed(String),
}

#[cfg(cerulion_has_buffer_backends)]
impl BufferSubscription {
    /// Announce the subscription to every backend under the gid derived from
    /// `entity`, before its port exists, and return the port name to create
    /// the port with. `Err` when the backends refused or the name does not fit.
    pub(crate) fn announce(
        entity: u64,
        topic_type: &str,
        accepted: &[String],
    ) -> Result<String, String> {
        let peer = AwarePeer {
            pid_ns: pid_namespace_id(),
            pid: std::process::id(),
            entity,
            backends: accepted.to_vec(),
        };
        let name = encode_port_name(&peer, cerulion_core::transport::max_port_name_len())
            .ok_or_else(|| {
                format!(
                    "the accepted backends ({}) do not fit an iceoryx2 port name",
                    accepted.join(",")
                )
            })?;
        let ty = std::ffi::CString::new(topic_type).map_err(|e| e.to_string())?;
        let gid = peer.gid();
        // SAFETY: gid is 16 bytes; ty is NUL-terminated.
        if unsafe { ffi::rmw_cerulion_bb_on_creating(gid.as_ptr(), 0, ty.as_ptr()) } != 0 {
            return Err(ffi::last_error());
        }
        Ok(name)
    }

    pub(crate) fn new(
        callbacks: *const std::os::raw::c_void,
        entity: u64,
        topic_type: &str,
        native_hash: u64,
        port_id: u128,
    ) -> Self {
        Self {
            callbacks,
            topic_type: std::ffi::CString::new(topic_type).unwrap_or_default(),
            frame_hash: buffer_frame_hash(native_hash),
            port_id,
            gid: buffer_gid(pid_namespace_id(), std::process::id(), entity),
        }
    }

    /// Handle an addressed frame's payload.
    ///
    /// # Safety
    /// `msg` is a live, initialized C++ message of this endpoint's type.
    pub(crate) unsafe fn take(&self, payload: &[u8], msg: *mut std::os::raw::c_void) -> TakeOutcome {
        let frame = match parse_frame(payload) {
            Ok(f) => f,
            Err(e) => return TakeOutcome::Failed(e.to_owned()),
        };
        if !frame.addresses(self.port_id) {
            note_skipped();
            return TakeOutcome::NotMine;
        }
        let r = ffi::rmw_cerulion_bb_deserialize(
            self.callbacks,
            frame.cdr.as_ptr(),
            frame.cdr.len(),
            msg,
            frame.publisher_gid.as_ptr(),
            self.topic_type.as_ptr(),
        );
        match r {
            0 => {
                note_descriptor_take();
                TakeOutcome::Delivered
            }
            3 => {
                note_lost_sample();
                TakeOutcome::Lost
            }
            _ => TakeOutcome::Failed(ffi::last_error()),
        }
    }
}

/// No hosted backends on this build: the publisher half never exists.
#[cfg(not(cerulion_has_buffer_backends))]
pub enum BufferPublisher {}

/// No hosted backends on this build: the subscription half never exists.
#[cfg(not(cerulion_has_buffer_backends))]
pub enum BufferSubscription {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_port_name_round_trips_and_other_names_are_not_aware() {
        let peer = AwarePeer {
            pid_ns: 0xeffffffc,
            pid: 0x1f3a,
            entity: 7,
            backends: vec!["cuda".into()],
        };
        let name = encode_port_name(&peer, 64).unwrap();
        assert_eq!(name, "rbuf1;effffffc;1f3a;7;cuda");
        assert_eq!(parse_port_name(&name), Some(peer.clone()));
        let cpu_only = AwarePeer {
            backends: vec![],
            ..peer.clone()
        };
        let name = encode_port_name(&cpu_only, 64).unwrap();
        assert_eq!(name, "rbuf1;effffffc;1f3a;7;");
        assert_eq!(parse_port_name(&name), Some(cpu_only));
        for other in ["", "rbuf2;1;1;1;cuda", "rbuf1;1;zz;1;", "rbuf1;1;1;1", "rbuf1;1;1;1;a;b"] {
            assert_eq!(parse_port_name(other), None, "{other:?}");
        }
        let long = AwarePeer {
            backends: vec!["x".repeat(60)],
            ..peer.clone()
        };
        assert_eq!(encode_port_name(&long, 64), None);
        let bad = AwarePeer {
            backends: vec!["cu;da".into()],
            ..peer
        };
        assert_eq!(encode_port_name(&bad, 64), None);
    }

    #[test]
    fn buffer_gids_differ_by_namespace_process_and_entity() {
        assert_ne!(buffer_gid(1, 1, 1), buffer_gid(2, 1, 1));
        assert_ne!(buffer_gid(1, 1, 1), buffer_gid(1, 2, 1));
        assert_ne!(buffer_gid(1, 1, 1), buffer_gid(1, 1, 2));
    }

    #[test]
    fn the_frame_hash_differs_from_the_native_hash() {
        for native in [0u64, 1, 0xdead_beef, u64::MAX] {
            assert_ne!(buffer_frame_hash(native), native);
            assert_eq!(buffer_frame_hash(native), buffer_frame_hash(native));
        }
        assert_ne!(buffer_frame_hash(1), buffer_frame_hash(2));
    }

    #[test]
    fn a_frame_prefix_round_trips_and_names_only_its_recipients() {
        let gid = buffer_gid(3, 4, 5);
        let recipients = [7u128, u128::MAX - 3];
        let cdr = [0u8, 1, 0, 0, 9, 9];
        let mut payload = vec![0u8; frame_prefix_len(2)];
        encode_frame_prefix(&mut payload, FRAME_KIND_ENDPOINT_CDR, &gid, &recipients, 6);
        payload.extend_from_slice(&cdr);
        let f = parse_frame(&payload).unwrap();
        assert_eq!(f.kind, FRAME_KIND_ENDPOINT_CDR);
        assert_eq!(f.publisher_gid, gid);
        assert_eq!(f.cdr, &cdr);
        assert!(f.addresses(7) && f.addresses(u128::MAX - 3) && !f.addresses(8));
        assert!(parse_frame(&payload[..payload.len() - 1]).is_err());
        assert!(parse_frame(&payload[..10]).is_err());
        let mut bad = payload.clone();
        bad[0] = 2;
        assert!(parse_frame(&bad).is_err());
        bad = payload;
        bad[1] = 9;
        assert!(parse_frame(&bad).is_err());
    }

    fn aware(acc: bool) -> PeerVerdict {
        PeerVerdict::Aware {
            gid: [acc as u8; 16],
            accelerated: acc,
        }
    }

    #[test]
    fn any_subscriber_that_is_not_buffer_aware_forces_the_native_frame() {
        assert_eq!(route(&[]), Route::Nobody);
        assert_eq!(route(&[(1, aware(true)), (2, PeerVerdict::NotAware)]), Route::Native);
        assert_eq!(route(&[(1, aware(false)), (2, aware(false))]), Route::Native);
        assert_eq!(
            route(&[(1, aware(true)), (2, aware(false)), (3, aware(true))]),
            Route::Addressed {
                accelerated: vec![(1, [1; 16]), (3, [1; 16])],
                cpu_group: vec![2],
            }
        );
    }

    #[test]
    fn verdicts_are_kept_per_port_and_negative_ones_are_asked_again() {
        let mut v = PeerVerdicts::default();
        let t0 = std::time::Instant::now();
        let mut asked = 0;
        let mut yes = |_: &AwarePeer| {
            asked += 1;
            true
        };
        assert_eq!(v.verdict(1, "", t0, &mut yes), PeerVerdict::NotAware);
        assert!(matches!(
            v.verdict(2, "rbuf1;1;1;2;cuda", t0, &mut yes),
            PeerVerdict::Aware { accelerated: true, .. }
        ));
        let _ = v.verdict(2, "rbuf1;1;1;2;cuda", t0, &mut yes);
        assert_eq!(asked, 1);
        let no_count = std::cell::Cell::new(0);
        let mut no = |_: &AwarePeer| {
            no_count.set(no_count.get() + 1);
            false
        };
        let _ = v.verdict(3, "rbuf1;1;1;3;cuda", t0, &mut no);
        let _ = v.verdict(3, "rbuf1;1;1;3;cuda", t0, &mut no);
        assert_eq!(no_count.get(), 1);
        let mut t = t0;
        for _ in 0..10 {
            t += NEGATIVE_VERDICT_RETRY;
            let _ = v.verdict(3, "rbuf1;1;1;3;cuda", t, &mut no);
        }
        assert_eq!(
            no_count.get(),
            1 + usize::from(NEGATIVE_VERDICT_RETRIES),
            "a refused peer is asked again a bounded number of times"
        );
        // A CPU-only peer is never asked.
        let _ = v.verdict(4, "rbuf1;1;1;4;", t0, &mut no);
        assert_eq!(no_count.get(), 1 + usize::from(NEGATIVE_VERDICT_RETRIES));
        v.retain_live(&[(2, aware(true))]);
        assert_eq!(v.entries.len(), 1);
    }

    #[test]
    fn identical_serializations_are_grouped_in_first_seen_order() {
        let outs = vec![vec![1u8], vec![2], vec![1], vec![2], vec![3]];
        assert_eq!(
            group_identical(&outs),
            vec![(0, vec![0, 2]), (1, vec![1, 3]), (4, vec![4])]
        );
    }

    #[test]
    fn accepted_backends_follow_the_subscription_option() {
        let installed = vec!["cuda".to_owned(), "demo".to_owned()];
        assert!(accepted_backends(None, &installed).is_empty());
        assert!(accepted_backends(Some(b""), &installed).is_empty());
        assert!(accepted_backends(Some(b"cpu"), &installed).is_empty());
        assert_eq!(accepted_backends(Some(b"any"), &installed), installed);
        assert_eq!(accepted_backends(Some(b" cuda ,x"), &installed), vec!["cuda"]);
        assert!(accepted_backends(Some(b"\xff"), &installed).is_empty());
    }

    #[test]
    fn every_counter_has_a_stable_name() {
        let c = counters();
        assert_eq!(c.len(), COUNTER_NAMES.len());
        for (name, _) in &c {
            assert!(name.starts_with("rmw_cerulion.rosidl_buffer."), "{name}");
        }
    }
}
