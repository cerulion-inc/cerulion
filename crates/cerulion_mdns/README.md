# cerulion_mdns

The `_cerulion._tcp` mDNS gateway beacon for
[Cerulion](https://github.com/cerulion-inc/cerulion): the advertise half and a
bounded browse.

A Cerulion robot announces itself on the local network with one mDNS service:
the SRV record carries the zenoh port the gateway really bound, and the TXT
record carries `robot=<identity>` plus the remote-access endpoint facts when the
remote-access daemon has published them. `advertise_gateway` registers the
service and returns a guard that withdraws it on drop. A failed advertisement is
returned as a typed `MdnsError`; callers warn and keep serving, because a
discovery beacon must never stop a run.

`browse_rung` browses the same service type for a fixed budget, maps each
answering gateway to one locator per IPv4 address (IPv6 only when it has
none), and shuts its mDNS daemon down
before it returns. `CERULION_SERVICE_TYPE` is defined here once and used by both
halves.

## Who uses it

This is an internal building block: `cerulion-netd` and the fallback gateway
process of `cerulion graph run` both advertise through it, `cerulion-netd`'s
boot-time peer fold and the `cerulion topic list` discovery ladder both browse
through it, and it is published because they depend on it.

See the [networking guide](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots).
Design notes for contributors live in
[docs/internals/network-daemons.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/network-daemons.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
