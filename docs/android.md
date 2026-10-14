# Android

The following runtime crates build for Android:

`cerulion_core`, `native_ros2_messages`, `cerulion-wire`, `cerulion_pairing`,
`cerulion_hygiene`, `cerulion_discovery`, `cerulion_mdns`, `cerud`,
`cerulion_bag`, `cerulion_netd`, `cerulion_bagd`, `cerulion_cli_engine`,
`cerulion_cli`, `cerulion_wsd`, `cerulion_wireclient`,
`cerulion_link`, `cerulion_connectd`, `cerulion_remoted`, `rmw_cerulion`,
and `cerulion_dds`.

The supported build targets are `arm64-v8a` and `x86_64`, with Android API 30 or
newer and NDK 27.2. Use cargo-ndk 4.1.2:

```sh
cargo ndk -t arm64-v8a --platform 30 check -p cerulion_core
cargo ndk -t x86_64 --platform 30 check -p cerulion_core
cargo ndk -t arm64-v8a --platform 30 build -p cerulion_core
```

Replace `cerulion_core` with any crate in the list to check that crate.

Android does not provide `/dev/shm` in every environment. Named shared-memory
objects used by Cerulion and iceoryx2 are regular files on Android. Publish and
subscribe work within one process and between Android processes. The iceoryx2
root path is `/data/local/tmp/iceoryx2`. A non-empty absolute `CERULION_SHM_DIR`
selects where Cerulion and transport shared-memory objects are stored. If it is
unset, empty, or relative, the runtime uses
`$TMPDIR/cerulion-shm` when `TMPDIR` is absolute; otherwise it uses
`/data/local/tmp/cerulion-shm`. The selected directory must be writable by the
process, owned by the process user, and not writable by group or others; a
symlinked directory is refused. Objects remain until unlinked, so a process
that exits without cleanup leaves files in the directory, and on Android they
survive a reboot. Remove the directory's contents to clear them. Processes
must be able to write to `/data/local/tmp` and the selected shared-memory
directory.

Android event-trigger datagram sockets use the abstract namespace and do not
create socket files. SELinux governs their access rather than file permissions.

Android does not provide robust mutexes. If a process exits while holding a
shared lock, other processes cannot recover that lock. There is no app embedding
API.

## Single-process apps

An Android app that owns all of its local publishers, subscribers, and gateway
in one process can opt into `local-transport`. It uses an in-process service,
so it does not require cross-process shared memory. The existing zero-copy
publish and subscribe path remains in use across threads in that process. A
gateway can still forward topics over TCP.

Do not enable this feature for the CLI or daemons, or for a graph that uses
`process_groups`. Explicit cross-process initialization, detached pre-creation
for other processes, and non-empty process groups are refused. Build node
cdylibs with the same transport kind as their host; when this feature is enabled,
a cross-process or legacy cdylib is refused when loaded.

Enable the feature on the dependency used by the app:

```toml
[dependencies]
cerulion_core = { version = "1", features = ["local-transport"] }
```

## CLI and node workspaces

Build the CLI and network daemon on a host with the Android NDK:

```sh
cargo ndk -t arm64-v8a --platform 30 build -p cerulion_cli -p cerulion_netd
```

Use `-t x86_64` for an x86_64 device. Build node cdylibs from their workspace
on the same host, with the same Rust compiler used for the CLI:

```sh
cd path/to/node-workspace
cargo ndk -t arm64-v8a --platform 30 build
```

Run node builds on the host. When `cerulion node build` cannot find Cargo on
Android, it directs you to cross-build the workspace with `cargo ndk`.

`cargo-ndk` places libraries under `target/<target-triple>/<profile>`. The CLI
looks there when loading Android nodes, so keep the target-triple directory when
copying the workspace to the device. Do not flatten its libraries into
`target/<profile>`.

An independent node workspace must include the complete iceoryx2 family
`[patch.crates-io]` block from Cerulion's root `Cargo.toml`, with every entry
pinned to the same fork revision. Without it, the node cdylib can link an
iceoryx2 build whose Android shared-memory calls return `ENOSYS`. Node
initialization refuses that library with:

```text
node library <name>: the binary links an iceoryx2 build without Android shared-memory support; build the workspace with the iceoryx2 fork [patch.crates-io] block (see docs/android.md)
```

For an adb shell session, provide writable home and temporary directories:

```sh
export HOME=/data/local/tmp/cer/home
export TMPDIR=/data/local/tmp/cer/tmp
export CERULION_LOGIN_GATE=off
mkdir -p "$HOME" "$TMPDIR"
```

When configured, netd and wsd use abstract-namespace control sockets on
Android. Their configured socket paths supply the abstract names and do not
create filesystem socket entries. Each daemon accepts only peers whose
`SO_PEERCRED` UID matches its own effective UID; peers with a different UID or
unverifiable credentials are dropped. The pidfile and singleton lock remain
filesystem-backed.

`cerulion viz` is not available on Android because its visualization daemon
requires a desktop. Run it on a desktop host and connect that host to device
topics over the network. For a forwarded TCP listener, create the host-side
forward with `adb forward tcp:7683 tcp:7683`; configure the device's netd to
listen on `tcp/0.0.0.0:7683`, then connect the host CLI to
`tcp/127.0.0.1:7683`.

## Run two processes

After placing the `cross_process_pubsub_iox2_test` binary at
`/data/local/tmp/cross_process_pubsub_iox2_test`, run the subscriber in one
shell:

```sh
adb shell 'cd /data/local/tmp && CERULION_XPROC_TOPIC=/xproc/example ./cross_process_pubsub_iox2_test --exact subscriber_role --include-ignored --nocapture --test-threads=1'
```

After `READY` appears, run the publisher in another shell:

```sh
adb shell 'cd /data/local/tmp && CERULION_XPROC_TOPIC=/xproc/example ./cross_process_pubsub_iox2_test --exact publisher_role --include-ignored --nocapture --test-threads=1'
```

The subscriber reports the number of messages received, the first and last
sequence numbers, and any gaps.
