# native_ros2_messages - agent notes

Vendored ROS 2 `.msg` corpus + build-time codegen. `src/lib.rs` is only
`include!(OUT_DIR)` of `build.rs` output - to change a type, edit the `.msg` under
`msg/<pkg>/` and `cargo build`. A green crate suite does NOT prove the corpus matches
upstream ROS 2: only `upstream_drift_test` compares against an external truth
(`upstream_msg_manifest.txt`); the sibling tests diff the tree against itself.

## Invariants

- `resolve_fixed_nested` runs over the WHOLE corpus before `generate_schema`; any
  resolver warning panics the build - an unresolved nested ref silently changes the
  parent's wire LAYOUT while the schema hash stays equal (wire skew, not cosmetics).
- Every `build.rs` collection is `BTreeMap`/sorted so generated code is byte-identical
  across builds (replay determinism). Never swap in a `HashMap`.
- A `build.rs` panic (parse failure, unreadable file, resolver warning) is a bug in
  the `.msg` input - fix the file; silently skipping would let references rebind.
- Nested refs in `.msg` files are QUALIFIED (`pkg/Type`). A bare ref hashes
  differently from what an rmw publisher emits (silent no-render); zero tolerance,
  no waiver - pinned by `the_vendored_corpus_declares_no_bare_nested_refs`.
- Hash-affecting corpus changes deploy to desk AND robot in lockstep - nothing on
  the wire signals the skew; affected topics render nothing, silently.

## Testing

```bash
cargo test -p native_ros2_messages   # whole crate; parallel-safe (isolated SHM roots)
```

- Adding/removing a `.msg` extends, in the same PR: `layout_equivalence_test` (hard
  corpus-count assert + one `assert_layout_matches!` line per schema),
  `roundtrip_test` (a `roundtrip_fixed!`/`roundtrip_var!` case), and the manifest
  (the drift gate fails closed on a missing entry).
- Run the FULL crate suite after touching the corpus - a curated local run masks
  the exhaustive tests.
- Never re-bless `schema_hash_pin_test`'s wire-capture hashes from the actual
  value: a failure there is a wire-compatibility break, not a stale pin.

## Gotchas

- Stale `target/debug/build/native_ros2_messages-*/out/` dirs with DIFFERENT
  generated code coexist after schema changes. The only truth for a generated
  constant is a probe against the linked crate (`<T as ShmMessage>::CONST`) -
  never a file found under `target/`.
- Manifest `!accept` waives CONSTANTS only - field drift under it still fails;
  `!accept-fields` is the only field-drift silencer and the last resort.
- `tools/scripts/refresh_upstream_msg_manifest.sh` rewrites signature blocks only and
  REFUSES to run without an existing manifest - the `!source`/waiver lines are
  hand-authored judgements it must not delete.

Deep reference: docs/internals/native-ros2-messages.md - read before modifying
`build.rs`, any `.msg`, or the manifest/drift gate.
