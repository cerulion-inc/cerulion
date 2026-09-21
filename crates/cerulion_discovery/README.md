# cerulion_discovery

LAN peer discovery types and helpers for
[Cerulion](https://github.com/cerulion-inc/cerulion).

- `ladder`: `DiscoveredPeer` and `DiscoveryRung` (a robot found on the
  network and how it was found), the bounded TCP reachability filter
  (`probe_reachable_locators`), and the connect-set planner
  (`plan_connect_set`) that decides which locators a session should dial.
- `peer_cache`: the `~/.cerulion/peers.json` format and its reader
  (`load_peers`). Entries expire after seven days.

The crate depends on `serde`, `serde_json`, `dirs` and `tracing` only. It opens
no zenoh session and runs no mDNS browse.

## Who uses it

This is an internal building block: the `cerulion` CLI's `topic list`
discovery and the `cerulion-netd` gateway daemon share it, and it is published
because both depend on it. The cache writer deliberately lives in the CLI, which
is the only side that confirms a robot is live before recording it.

See the [networking guide](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots).
Design notes for contributors live in
[docs/internals/network-daemons.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/network-daemons.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
