// SPDX-License-Identifier: AGPL-3.0-only
//! Entity gids and the publisher gid a take reports.
//!
//! A publisher's gid is the 16 little-endian bytes of its iceoryx2 port id
//! ([`publisher_gid`]), the `UniquePublisherId` value that every sample the
//! port sends carries as its origin. A take writes the gid of the received
//! sample's origin to `rmw_message_info_t::publisher_gid`, so it reports the
//! same bytes `rmw_get_gid_for_publisher` returns for the sending publisher,
//! in this process or another. iceoryx2 builds the id from the creating
//! process's id, the monotonic time and a per-process counter, so two live
//! ports on one host never share it. A sample sent by a port that no rmw
//! publisher owns (a native graph node, a network gateway) reports that
//! port's id, which no rmw publisher holds.
//!
//! Subscriptions and clients get a gid when they are created ([`mint_gid`]):
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

/// The gid of the rmw publisher whose iceoryx2 port id is `port_id`
/// (`CerulionPublisher::publisher_id`, a received sample's `origin`): its 16
/// little-endian bytes.
#[inline]
pub fn publisher_gid(port_id: u128) -> [u8; 16] {
    port_id.to_le_bytes()
}

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_publisher_gid_is_the_port_id_little_endian() {
        assert_eq!(
            publisher_gid(0x100f_0e0d_0c0b_0a09_0807_0605_0403_0201),
            [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        );
        assert_eq!(publisher_gid(0), [0u8; 16]);
    }

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
    fn the_pid_namespace_id_is_stable_within_the_process() {
        assert_eq!(pid_namespace_id(), pid_namespace_id());
        #[cfg(not(target_os = "linux"))]
        assert_eq!(pid_namespace_id(), 0);
    }
}
