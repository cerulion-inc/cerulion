# cerulion_bag

The recording format crate for
[Cerulion](https://github.com/cerulion-inc/cerulion): an MCAP writer and reader.

A Cerulion recording is a standard MCAP file (`.mcap`), readable by the `mcap`
crate, standard MCAP viewers and `mcap doctor`. This crate writes the bytes
itself so that:

- **the output is byte-deterministic**: the writer reads no clock, so the same
  sequence of calls produces the same file;
- **chunk boundaries belong to the caller**, which lets the recorder flush on its
  own schedule;
- **a torn tail stays readable**: a recording cut short by a crash reads back up
  to its last complete record, and the reader reports how the bag ended.

Reading goes through the `mcap` crate. Unix only.

## Who uses it

You normally reach this crate through the CLI: `cerulion graph run --record`
and `cerulion bag record` write bags, and `cerulion bag play`, `cerulion bag
info` and `cerulion bag play --resim all --verify` read them. Depend on it
directly to read Cerulion bags from your own Rust tools (`BagReader`).

See the [record and replay guide](https://docs.cerulion.com/cerulion/guides/record-and-replay).
Design notes for contributors live in
[docs/internals/recording.md](https://github.com/cerulion-inc/cerulion/blob/main/docs/internals/recording.md).

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
See the [LICENSE](https://github.com/cerulion-inc/cerulion/blob/main/LICENSE)
file. A separate commercial license is available; see the
[project README](https://github.com/cerulion-inc/cerulion/blob/main/README.md#license).
