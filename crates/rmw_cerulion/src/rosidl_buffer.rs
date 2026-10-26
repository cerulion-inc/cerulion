// SPDX-License-Identifier: AGPL-3.0-only
//! `rosidl::Buffer<uint8_t>` support (ROS 2 Lyrical and Rolling).
//!
//! On these eras an unbounded `uint8[]` member of a C++ message is a
//! `rosidl::Buffer<uint8_t>`, and a C sequence whose `is_rosidl_buffer` flag
//! is set points at one. A Buffer can live on an accelerator backend (the
//! CUDA backend that Isaac ROS 5.0 uses), and then every element accessor of
//! it throws. Where this process hosts that backend and every subscriber can
//! take its descriptor, the Buffer crosses processes as the descriptor and
//! stays in device memory ([`crate::buffer_backends`]). Everywhere else it is
//! carried as host bytes:
//!
//! * **Publish.** A non-empty Buffer on any backend other than `cpu` is
//!   copied to host memory with `to_vector()` (the DOWNGRADE) and published
//!   as plain bytes. Every downgrade adds one to [`downgrade_count`]. Per
//!   topic, the frames that carried a downgrade are counted: the first logs a
//!   warning that names the topic and the backend, and the count is logged
//!   again at every power of ten.
//! * **Take.** A Buffer the bridge fills ends up on the `cpu` backend holding
//!   the received bytes (on C, only a Buffer the sequence owns; a sequence
//!   that does not own its Buffer is detached and filled as a plain
//!   sequence).
//! * **Subscription options.** [`validate_acceptable_buffer_backends`]
//!   reads `acceptable_buffer_backends`: `cpu` is always acceptable, and a
//!   subscription naming a backend this process has not loaded starts, logs
//!   one warning naming the backends it ignores, and receives Buffers on them
//!   as CPU data; only malformed input fails the create.
//!
//! The C++ side of every call is `shim/cppstring_shim.cpp`, compiled against
//! the distro's own `rosidl_buffer` headers, or on a vendored-bindings build
//! the copy in `shim/vendored/`.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

#[cfg(cerulion_has_is_rosidl_buffer)]
use std::os::raw::{c_char, c_int, c_void};

/// The buffer backend every build delivers. The backend plugins a Lyrical
/// build loads come on top ([`crate::buffer_backends::installed_backends`]).
pub const INSTALLED_BUFFER_BACKENDS: &[&str] = &["cpu"];

/// Total number of Buffers copied from a non-CPU backend to host memory by
/// this process.
static DOWNGRADES: AtomicU64 = AtomicU64::new(0);

/// Per-topic downgrade totals, touched only when a downgrade happened.
static TOPIC_DOWNGRADES: Mutex<Option<HashMap<String, u64>>> = Mutex::new(None);

thread_local! {
    /// The backend of the first Buffer this thread downgraded since the
    /// enclosing [`PublishScope`] opened.
    static PENDING_BACKEND: std::cell::RefCell<Option<String>> =
        const { std::cell::RefCell::new(None) };
    /// Set while a size pass encodes a nested message only to measure it
    /// (see [`SizingPass`]).
    static SIZING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Marks this thread as running a size pass that encodes a nested message
/// only to learn its length, for as long as the value lives. The nested
/// encoders then stand a Buffer's bytes in with zeros of the Buffer's own
/// length ([`sizing`]), so a non-CPU Buffer is copied to host memory once
/// per publish, by the write pass, and not a second time to be measured.
pub(crate) struct SizingPass {
    previous: bool,
}

impl SizingPass {
    pub(crate) fn open() -> Self {
        Self {
            previous: SIZING.with(|s| s.replace(true)),
        }
    }
}

impl Drop for SizingPass {
    fn drop(&mut self) {
        SIZING.with(|s| s.set(self.previous));
    }
}

/// Whether a [`SizingPass`] is open on this thread.
pub(crate) fn sizing() -> bool {
    SIZING.with(|s| s.get())
}

/// How many Buffers this process has copied from a non-CPU backend to host
/// memory.
pub fn downgrade_count() -> u64 {
    DOWNGRADES.load(Ordering::Relaxed)
}

/// How many published frames on `topic` carried a downgraded Buffer.
pub fn topic_downgrade_count(topic: &str) -> u64 {
    let guard = TOPIC_DOWNGRADES.lock().unwrap_or_else(|e| e.into_inner());
    guard
        .as_ref()
        .and_then(|m| m.get(topic).copied())
        .unwrap_or(0)
}

/// Record one downgrade: the process counter, and the backend for the
/// enclosing [`PublishScope`] to report. Called only on an era with Buffers.
#[cfg_attr(not(cerulion_has_is_rosidl_buffer), allow(dead_code))]
fn note_downgrade(backend: &str) {
    DOWNGRADES.fetch_add(1, Ordering::Relaxed);
    PENDING_BACKEND.with(|p| {
        let mut p = p.borrow_mut();
        if p.is_none() {
            *p = Some(backend.to_owned());
        }
    });
}

/// Whether a per-topic total of `n` is logged: the first downgrade, then
/// every power of ten.
pub fn announces(n: u64) -> bool {
    if n == 0 {
        return false;
    }
    let mut m = n;
    while m.is_multiple_of(10) {
        m /= 10;
    }
    m == 1
}

/// Brackets one encode on behalf of `topic`: opening it forgets anything an
/// earlier encode on this thread left behind, and dropping it reports a
/// downgrade the encode made (one per frame, whatever the number of
/// downgraded members).
pub(crate) struct PublishScope<'a> {
    topic: &'a str,
}

impl<'a> PublishScope<'a> {
    pub(crate) fn open(topic: &'a str) -> Self {
        PENDING_BACKEND.with(|p| p.borrow_mut().take());
        Self { topic }
    }
}

impl Drop for PublishScope<'_> {
    fn drop(&mut self) {
        let Some(backend) = PENDING_BACKEND.with(|p| p.borrow_mut().take()) else {
            return;
        };
        let n = {
            let mut guard = TOPIC_DOWNGRADES.lock().unwrap_or_else(|e| e.into_inner());
            let map = guard.get_or_insert_with(HashMap::new);
            let entry = map.entry(self.topic.to_owned()).or_insert(0);
            *entry += 1;
            *entry
        };
        if n == 1 {
            tracing::warn!(
                topic = %self.topic,
                backend = %backend,
                topic_downgrades = n,
                total_downgrades = downgrade_count(),
                "a rosidl::Buffer on a non-CPU backend was copied to host memory and published \
                 as bytes: this process hosts no backend for it, or a subscriber that cannot \
                 take its descriptor is attached (a native node, a recorder, the gateway, \
                 rclpy), so device memory crossed processes as a CPU copy (further frames on \
                 this topic are counted, and the count is logged at every power of ten)"
            );
        } else if announces(n) {
            tracing::warn!(
                topic = %self.topic,
                backend = %backend,
                topic_downgrades = n,
                total_downgrades = downgrade_count(),
                "rosidl::Buffer CPU copies on this topic"
            );
        }
    }
}

/// Read `rmw_subscription_options_t::acceptable_buffer_backends` the way
/// rmw_zenoh_cpp and rmw_fastrtps_cpp parse it: a comma-separated list, each
/// name trimmed of spaces and tabs, empty names skipped. `None`, an empty
/// list and `cpu` mean CPU only, and `any` accepts every installed backend.
/// CPU is always acceptable ([`INSTALLED_BUFFER_BACKENDS`]), and so is every
/// backend this process has loaded
/// ([`crate::buffer_backends::installed_backends`]); a subscription naming
/// any other backend still receives Buffers on it as CPU data: `Ok` returns
/// the names it ignores (empty when there are none). `Err` is malformed input only: text that is
/// not UTF-8, or a name with a character other than an ASCII letter, digit,
/// `_` or `-`.
pub fn validate_acceptable_buffer_backends(
    requested: Option<&[u8]>,
) -> Result<Vec<String>, String> {
    let Some(requested) = requested else {
        return Ok(Vec::new());
    };
    let Ok(requested) = std::str::from_utf8(requested) else {
        return Err("acceptable_buffer_backends is not valid UTF-8".to_owned());
    };
    let names: Vec<&str> = requested
        .split(',')
        .map(|n| n.trim_matches(|c| c == ' ' || c == '\t'))
        .filter(|n| !n.is_empty())
        .collect();
    if let Some(bad) = names.iter().find(|n| {
        !n.chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    }) {
        return Err(format!(
            "acceptable_buffer_backends names '{bad}', which is not a buffer backend name \
             (ASCII letters, digits, '_' and '-', separated by commas)"
        ));
    }
    Ok(names
        .into_iter()
        .filter(|n| {
            *n != "any"
                && !INSTALLED_BUFFER_BACKENDS.contains(n)
                && !crate::buffer_backends::installed_backends()
                    .iter()
                    .any(|b| b == n)
        })
        .map(str::to_owned)
        .collect())
}

#[cfg(cerulion_has_is_rosidl_buffer)]
extern "C" {
    fn rmw_cerulion_buffer_u8_size(buf: *const c_void, out: *mut usize) -> c_int;
    fn rmw_cerulion_buffer_u8_read(
        buf: *const c_void,
        data: *mut *const u8,
        len: *mut usize,
        owned: *mut *mut c_void,
        backend: *mut c_char,
        backend_cap: usize,
    ) -> c_int;
    fn rmw_cerulion_buffer_u8_copy_free(owned: *mut c_void);
    fn rmw_cerulion_buffer_u8_assign(buf: *mut c_void, data: *const u8, len: usize) -> c_int;
}

/// Bytes read out of a Buffer: a view of a CPU Buffer's own storage, or a
/// host copy of a Buffer on another backend, freed on drop. Never built on
/// an era without Buffers.
#[cfg_attr(not(cerulion_has_is_rosidl_buffer), allow(dead_code))]
pub(crate) struct BufferBytes {
    data: *const u8,
    len: usize,
    owned: *mut std::os::raw::c_void,
}

impl BufferBytes {
    /// Whether these bytes are a host copy of a non-CPU Buffer (the
    /// downgrade), rather than a view of a CPU Buffer's storage.
    #[cfg(all(test, cerulion_has_is_rosidl_buffer))]
    pub(crate) fn is_host_copy(&self) -> bool {
        !self.owned.is_null()
    }

    /// The bytes. A view of a CPU Buffer stays valid only while that
    /// Buffer is neither mutated nor destroyed.
    pub(crate) fn as_slice(&self) -> &[u8] {
        if self.len == 0 {
            return &[];
        }
        // SAFETY: the shim returned `len` readable bytes at `data`: the
        // copy this value owns, or the storage of a Buffer the caller
        // keeps alive and unchanged while it reads.
        unsafe { std::slice::from_raw_parts(self.data, self.len) }
    }
}

impl Drop for BufferBytes {
    fn drop(&mut self) {
        #[cfg(cerulion_has_is_rosidl_buffer)]
        if !self.owned.is_null() {
            // SAFETY: `owned` came from rmw_cerulion_buffer_u8_read and is
            // freed exactly once, here.
            unsafe { rmw_cerulion_buffer_u8_copy_free(self.owned) };
        }
    }
}

/// Read the bytes of the `rosidl::Buffer<uint8_t>` at `buf`, copying them
/// to host memory (and counting the downgrade) when its backend is not
/// `cpu`. Err when the backend throws, or on an era without Buffers.
///
/// # Safety
/// `buf` points at a live `rosidl::Buffer<uint8_t>`.
pub(crate) unsafe fn read_u8(
    buf: *const std::os::raw::c_void,
) -> Result<BufferBytes, &'static str> {
    #[cfg(cerulion_has_is_rosidl_buffer)]
    {
        let mut out = BufferBytes {
            data: std::ptr::null(),
            len: 0,
            owned: std::ptr::null_mut(),
        };
        let mut name = [0 as c_char; 64];
        let status = rmw_cerulion_buffer_u8_read(
            buf,
            &mut out.data,
            &mut out.len,
            &mut out.owned,
            name.as_mut_ptr(),
            name.len(),
        );
        match status {
            0 => Ok(out),
            1 => {
                let backend = std::ffi::CStr::from_ptr(name.as_ptr()).to_string_lossy();
                note_downgrade(&backend);
                Ok(out)
            }
            _ => Err("the rosidl Buffer's backend threw while its bytes were read"),
        }
    }
    #[cfg(not(cerulion_has_is_rosidl_buffer))]
    {
        let _ = buf;
        Err("this build's ROS distro has no rosidl Buffer")
    }
}

/// Element count of the `rosidl::Buffer<uint8_t>` at `buf`, on any
/// backend, without copying anything.
///
/// # Safety
/// `buf` points at a live `rosidl::Buffer<uint8_t>`.
pub(crate) unsafe fn len_u8(buf: *const std::os::raw::c_void) -> Result<usize, &'static str> {
    #[cfg(cerulion_has_is_rosidl_buffer)]
    {
        let mut n = 0usize;
        if rmw_cerulion_buffer_u8_size(buf, &mut n) != 0 {
            return Err("the rosidl Buffer's backend threw while its size was read");
        }
        Ok(n)
    }
    #[cfg(not(cerulion_has_is_rosidl_buffer))]
    {
        let _ = buf;
        Err("this build's ROS distro has no rosidl Buffer")
    }
}

/// Make the `rosidl::Buffer<uint8_t>` at `buf` a CPU Buffer holding exactly
/// `bytes`; a Buffer on another backend gets a new CPU implementation and
/// its old one is destroyed by its own destructor. False when an
/// allocation failed (the Buffer is then unchanged), or on an era without
/// Buffers.
///
/// # Safety
/// `buf` points at a live `rosidl::Buffer<uint8_t>` the caller may write.
pub(crate) unsafe fn assign_u8(buf: *mut std::os::raw::c_void, bytes: &[u8]) -> bool {
    #[cfg(cerulion_has_is_rosidl_buffer)]
    {
        rmw_cerulion_buffer_u8_assign(buf, bytes.as_ptr(), bytes.len()) != 2
    }
    #[cfg(not(cerulion_has_is_rosidl_buffer))]
    {
        let _ = (buf, bytes);
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_uninstalled_backend_is_ignored_and_only_malformed_input_is_refused() {
        for (requested, ignored) in [
            (None, vec![]),
            (Some(&b""[..]), vec![]),
            (Some(&b"cpu"[..]), vec![]),
            (Some(&b" cpu , \tcpu"[..]), vec![]),
            (Some(&b","[..]), vec![]),
            (Some(&b"any"[..]), vec![]),
            (Some(&b"cuda, any"[..]), vec!["cuda"]),
            (Some(&b"cpu,cuda"[..]), vec!["cuda"]),
            (
                Some(&b"cuda,demo_backend"[..]),
                vec!["cuda", "demo_backend"],
            ),
            (Some(&b"Cpu"[..]), vec!["Cpu"]),
        ] {
            let ignored: Vec<String> = ignored.into_iter().map(str::to_owned).collect();
            assert_eq!(
                validate_acceptable_buffer_backends(requested),
                Ok(ignored),
                "{requested:?}"
            );
        }
        assert!(validate_acceptable_buffer_backends(Some(b"\xff")).is_err());
        let err = validate_acceptable_buffer_backends(Some(b"cpu;cuda")).unwrap_err();
        assert!(err.contains("'cpu;cuda'"), "{err}");
        assert!(validate_acceptable_buffer_backends(Some(b"cu da")).is_err());
    }

    #[test]
    fn the_first_downgrade_and_every_power_of_ten_are_announced() {
        let announced: Vec<u64> = (0..=1000).filter(|&n| announces(n)).collect();
        assert_eq!(announced, vec![1, 10, 100, 1000]);
    }

    #[test]
    fn a_scope_reports_its_own_downgrade_once_per_frame_and_forgets_stale_ones() {
        let topic = "/rosidl_buffer/unit/scope";
        // A downgrade left behind before the scope opened is not this frame's.
        note_downgrade("stale");
        {
            let _scope = PublishScope::open(topic);
        }
        assert_eq!(topic_downgrade_count(topic), 0);
        {
            let _scope = PublishScope::open(topic);
            note_downgrade("fixture_device");
            note_downgrade("fixture_device");
        }
        assert_eq!(topic_downgrade_count(topic), 1);
    }
}
