# cerulion_android

The embedded runtime of [Cerulion](https://github.com/cerulion-inc/cerulion)
for Android apps.

An app loads one native library that holds Cerulion and the app's graph
nodes. Its Java code drives the runtime through the
`com.cerulion.android.CerulionRuntime` facade in `java/`: load the library,
initialize the runtime with the app's files directory, then start a graph,
publish app topics, record them to an MCAP bag, read the status and stop. The
runtime serves the app's topics to other machines through its gateway.

The facade is Java source: the app adds this crate's `java/` directory to its
Gradle sources, from the same release or commit as the crate, because its
native method declarations must match the library. A GitHub release that
carries this crate also attaches that directory as
`cerulion-android-java-<version>.tar.gz`. An app copies one more thing into
its own build: the iceoryx2 `[patch.crates-io]` block of Cerulion's root
`Cargo.toml`, since a patch applies only in the workspace that declares it.

## Graph nodes in an app

The app's runtime crate is a `cdylib` that depends on this crate and on each
node crate with `default-features = false`, and lists the nodes once:

```rust
cerulion_android::embed_nodes! {
    "timer" => timer::TimerNodeEntry,
    "printer" => printer::PrinterNodeEntry,
}
```

Java then calls `CerulionRuntime.loadLibrary("<runtime library name>")`
before any other method. A graph node whose `type:` is not in that list fails
the start with error code 5.

See the [Android guide](https://github.com/cerulion-inc/cerulion/blob/main/docs/android.md)
for the build, packaging and configuration steps.

## License

Licensed under the GNU Affero General Public License v3.0 only (AGPL-3.0-only).
