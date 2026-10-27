// SPDX-License-Identifier: AGPL-3.0-only
//! Entity gids and the publisher gid a take reports.
//!
//! Every rmw entity (publisher, subscription, client) gets a 16-byte gid when
//! it is created ([`mint_gid`]):
//!
//! | bytes  | value                                                            |
//! |--------|------------------------------------------------------------------|
//! | 0..4   | pid-namespace id, u32 LE ([`pid_namespace_id`], 0 where none)     |
//! | 4..8   | process id, u32 LE                                               |
//! | 8..16  | the process's entity counter, u64 LE                             |
//!
//! Two live processes on one host differ in the process id or, across
//! containers that share `/dev/shm` with their own pid namespaces, in the
//! namespace id. Within one process the gid is a function of creation order.
//!
//! A publisher's iceoryx2 port is created under [`publisher_port_name`], which
//! every process attached to the topic reads in the data service's dynamic
//! config. A take resolves the origin publisher id of the sample it received
//! to that name and parses the gid back out of it ([`PublisherGidResolver`]),
//! and writes it to `rmw_message_info_t::publisher_gid`. A sample from a
//! publisher whose port name is not an rmw name (a native graph node, a
//! network gateway) resolves to the all-zero gid, which no rmw entity holds.

use cerulion_core::transport::publisher_directory::PublisherDirectory;

/// Prefix of an rmw publisher's iceoryx2 port name ([`publisher_port_name`]).
pub const PUBLISHER_PORT_PREFIX: &str = "rmwpub;";

/// The gid of entity `entity` in process `pid` of pid namespace `pid_ns`.
pub fn host_gid(pid_ns: u32, pid: u32, entity: u64) -> [u8; 16] {
    let mut gid = [0u8; 16];
    gid[..4].copy_from_slice(&pid_ns.to_le_bytes());
    gid[4..8].copy_from_slice(&pid.to_le_bytes());
    gid[8..].copy_from_slice(&entity.to_le_bytes());
    gid
}

/// This process's pid-namespace id: the low 32 bits of the inode
/// `/proc/self/ns/pid` names (`pid:[4026531836]`), 0 where the link does not
/// exist (macOS). Read once per process.
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
            .map_or_else(
                || {
                    #[cfg(target_os = "linux")]
                    tracing::warn!(
                        "/proc/self/ns/pid is unreadable, so rmw gids carry pid namespace \
                         id 0; two containers that share /dev/shm without /proc can then \
                         mint equal gids for equal pids"
                    );
                    0
                },
                |inode| inode as u32,
            )
    })
}

/// A new entity gid: this process's [`host_gid`] for the next value of the
/// runtime's entity counter.
pub fn mint_gid() -> [u8; 16] {
    host_gid(
        pid_namespace_id(),
        std::process::id(),
        crate::runtime::next_entity_id(),
    )
}

/// The iceoryx2 port name of the publisher whose gid is `gid`:
/// [`PUBLISHER_PORT_PREFIX`] followed by the 16 gid bytes as 32 lowercase hex
/// digits (39 bytes).
pub fn publisher_port_name(gid: &[u8; 16]) -> String {
    use std::fmt::Write as _;
    let mut name = String::with_capacity(PUBLISHER_PORT_PREFIX.len() + 32);
    name.push_str(PUBLISHER_PORT_PREFIX);
    for b in gid {
        let _ = write!(name, "{b:02x}");
    }
    name
}

/// The gid a [`publisher_port_name`] carries, or `None` for any other name.
pub fn parse_publisher_port_name(name: &str) -> Option<[u8; 16]> {
    let hex = name.strip_prefix(PUBLISHER_PORT_PREFIX)?.as_bytes();
    if hex.len() != 32 {
        return None;
    }
    let nibble = |c: u8| match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        _ => None,
    };
    let mut gid = [0u8; 16];
    for (i, byte) in gid.iter_mut().enumerate() {
        *byte = (nibble(hex[2 * i])? << 4) | nibble(hex[2 * i + 1])?;
    }
    Some(gid)
}

/// How many origin ids a resolver remembers per publisher the topic admits.
/// Twice the maximum, so every live publisher stays cached while as many
/// detached ones age out.
pub const RESOLVER_CACHE_PER_PUBLISHER: usize = 2;

/// Resolves the origin publisher id of a received sample to that publisher's
/// rmw gid, for one subscription.
///
/// A lookup that misses the cache walks the topic's publisher list once
/// ([`PublisherDirectory::with_publisher_name`], which allocates) and caches
/// the answer, the all-zero gid included. The cache holds
/// [`RESOLVER_CACHE_PER_PUBLISHER`] entries per publisher the topic admits,
/// reserved at construction, and replaces its entries in turn once full. A
/// take allocates only on a miss: once per publisher, plus once more for a
/// live publisher evicted after as many publishers have come and gone.
pub struct PublisherGidResolver {
    directory: Option<PublisherDirectory>,
    cache: Vec<(u128, [u8; 16])>,
    capacity: usize,
    next_replace: usize,
}

impl PublisherGidResolver {
    /// A resolver over `directory`. `None` resolves every origin to the
    /// all-zero gid.
    pub fn new(directory: Option<PublisherDirectory>) -> Self {
        let publishers = directory
            .as_ref()
            .map_or(1, PublisherDirectory::max_publishers);
        let capacity = RESOLVER_CACHE_PER_PUBLISHER * publishers.max(1);
        Self {
            directory,
            // hot-path-alloc-ok: once per subscription, at create.
            cache: Vec::with_capacity(capacity),
            capacity,
            next_replace: 0,
        }
    }

    /// The rmw gid of the publisher whose iceoryx2 publisher id is `origin`;
    /// all zero when it is not an rmw publisher or is no longer attached.
    pub fn resolve(&mut self, origin: u128) -> [u8; 16] {
        if let Some((_, gid)) = self.cache.iter().find(|(o, _)| *o == origin) {
            return *gid;
        }
        let gid = self
            .directory
            .as_ref()
            .and_then(|d| d.with_publisher_name(origin, parse_publisher_port_name))
            .flatten()
            .unwrap_or([0u8; 16]);
        if self.cache.len() < self.capacity {
            self.cache.push((origin, gid));
        } else {
            self.cache[self.next_replace] = (origin, gid);
            self.next_replace = (self.next_replace + 1) % self.capacity;
        }
        gid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_gid_layout_is_namespace_pid_entity_little_endian() {
        let gid = host_gid(0x0403_0201, 0x0807_0605, 0x100f_0e0d_0c0b_0a09);
        assert_eq!(gid, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
    }

    #[test]
    fn host_gids_differ_by_namespace_process_and_entity() {
        assert_ne!(host_gid(1, 1, 1), host_gid(2, 1, 1));
        assert_ne!(host_gid(1, 1, 1), host_gid(1, 2, 1));
        assert_ne!(host_gid(1, 1, 1), host_gid(1, 1, 2));
    }

    #[test]
    fn a_publisher_port_name_round_trips_and_fits_an_iceoryx2_port_name() {
        for gid in [[0u8; 16], [0xffu8; 16], host_gid(4_026_531_836, 77, 3)] {
            let name = publisher_port_name(&gid);
            assert_eq!(name.len(), 39);
            assert!(
                name.len() <= cerulion_core::transport::publisher_directory::max_port_name_len()
            );
            assert_eq!(parse_publisher_port_name(&name), Some(gid));
        }
    }

    #[test]
    fn other_port_names_carry_no_gid() {
        let good = publisher_port_name(&host_gid(1, 2, 3));
        for other in [
            "",
            "rmwpub;",
            "rmwsub;000102030405060708090a0b0c0d0e0f",
            &good[..good.len() - 1],
            &format!("{good}0"),
            &good.to_uppercase(),
            "rmwpub;zz0102030405060708090a0b0c0d0e0f",
        ] {
            assert_eq!(parse_publisher_port_name(other), None, "{other:?}");
        }
    }

    #[test]
    fn a_resolver_without_a_directory_resolves_to_zero_and_stays_bounded() {
        let mut r = PublisherGidResolver::new(None);
        let capacity = RESOLVER_CACHE_PER_PUBLISHER;
        assert_eq!(r.capacity, capacity);
        for origin in 0..(3 * capacity as u128) {
            assert_eq!(r.resolve(origin), [0u8; 16]);
            assert!(r.cache.len() <= capacity);
        }
        assert_eq!(r.cache.capacity(), capacity);
    }

    #[test]
    fn the_pid_namespace_id_is_stable_within_the_process() {
        assert_eq!(pid_namespace_id(), pid_namespace_id());
        #[cfg(not(target_os = "linux"))]
        assert_eq!(pid_namespace_id(), 0);
    }
}
