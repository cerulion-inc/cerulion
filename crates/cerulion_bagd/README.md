# cerulion_bagd

The recorder daemon for [Cerulion](https://github.com/cerulion-inc/cerulion).

It taps live topics in shared memory, drains the scheduler trace, and writes
both into one standard MCAP bag through
[`cerulion_bag`](https://crates.io/crates/cerulion_bag). A recording that did
not finish cleanly is left unfinalized on purpose, so replay can tell a complete
bag from a truncated one.

## Who uses it

The `cerulion` CLI runs this recorder for you:

```bash
cerulion graph run my_graph --record      # record a run from its first step
cerulion bag record --all -o out.mcap     # record topics that are already live
```

`cerulion bagd --out PATH ...` is the daemon itself. Run it by hand only to
attach a recorder with settings neither verb exposes; `cerulion bagd --help`
lists its arguments. Unix only.

See the [record and replay guide](https://docs.cerulion.com/cerulion/guides/record-and-replay).
Design notes for contributors live in
[docs/internals/recording.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/recording.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
