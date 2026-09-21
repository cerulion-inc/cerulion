# cerulion_mdns

The advertise half of the `_cerulion._tcp` mDNS gateway beacon for
[Cerulion](https://github.com/cerulion-inc/cerulion).

A Cerulion robot announces itself on the local network with one mDNS service:
the SRV record carries the zenoh port the gateway really bound, and the TXT
record carries `robot=<identity>` plus the remote-access endpoint facts when the
remote-access daemon has published them. `advertise_gateway` registers the
service and returns a guard that withdraws it on drop. A failed advertisement is
returned as a typed `MdnsError`; callers warn and keep serving, because a
discovery beacon must never stop a run.

`CERULION_SERVICE_TYPE` is defined here once and imported by the browsing side,
so the one string both sides must agree on cannot drift.

## Who uses it

This is an internal building block: `cerulion-netd` and the fallback gateway
process of `cerulion graph run` both advertise through it, and it is published
because they depend on it. The browse half lives in the `cerulion` CLI
(`cerulion topic list`).

See the [networking guide](https://docs.cerulion.com/cerulion/guides/network-and-remote-robots).
Design notes for contributors live in
[docs/internals/network-daemons.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/network-daemons.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
