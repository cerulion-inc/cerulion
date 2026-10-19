# Android

The following runtime crates build for Android:

`cerulion_core`, `native_ros2_messages`, `cerulion-wire`, `cerulion_pairing`,
`cerulion_hygiene`, `cerulion_discovery`, `cerulion_mdns`, `cerud`,
`cerulion_bag`, `cerulion_netd`, `cerulion_bagd`, `cerulion_wireclient`,
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

The Android transport described below comes from a pinned iceoryx2 fork. The
workspace applies that fork through `[patch.crates-io]` in the root
`Cargo.toml`, and Cargo applies such a patch only to builds of this workspace.
A project that depends on a released `cerulion_core` from crates.io resolves
upstream iceoryx2 instead, so it does not get the file-backed shared memory,
the `CERULION_SHM_DIR` placement, or the abstract-namespace sockets described
here. Such a project must add the same `[patch.crates-io]` entries for the
whole iceoryx2 family, at the same revision, to its own root `Cargo.toml`.

Android does not provide `/dev/shm` in every environment. Named shared-memory
objects used by Cerulion and iceoryx2 are regular files on Android. Publish and
subscribe work within one process and between Android processes. The iceoryx2
root path is `/data/local/tmp/iceoryx2`. The runtime chooses the directory for
Cerulion named shared-memory objects in this order: an absolute directory
passed to `cerulion_core::posix_shm::set_shm_dir`, then a non-empty absolute
`CERULION_SHM_DIR`, then `$TMPDIR/cerulion-shm` when `TMPDIR` is absolute.
iceoryx2 places its own shared-memory segments through the pinned fork,
which reads the environment on its own: a non-empty absolute
`CERULION_SHM_DIR`, then `$TMPDIR/cerulion-shm` when `TMPDIR` is absolute,
then `/data/local/tmp/cerulion-shm`. It never sees `set_shm_dir` and has no
app-sandbox step, so an app process that calls `set_shm_dir` must also export
`CERULION_SHM_DIR` (or an absolute `TMPDIR`) with the same app-writable
directory before iceoryx2 starts; otherwise iceoryx2 fails with a permission
error even though Cerulion's own objects work.
Neither `set_shm_dir` nor `CERULION_SHM_DIR` moves the iceoryx2 root path,
where iceoryx2 keeps its other files (node and service state, not the named
shared-memory objects); the root path moves through the `[global] root-path`
key in `config/iceoryx2.toml` under the process's working directory.
`set_shm_dir` must run before the first
named object is opened or unlinked, because either call resolves and latches
the directory; calling it again with the resolved directory is a
no-op, and calling it with a different directory afterwards returns an error.
When nothing is configured, shell and root processes fall back to
`/data/local/tmp/cerulion-shm`. An app process cannot write there,
so the runtime next tries the app's own sandbox: the package name is read
from `/proc/self/cmdline` (any `:process` suffix stripped), and when
`/data/user/<user>/<package>` exists and belongs to the app, the directory
becomes `/data/user/<user>/<package>/cache/cerulion-shm`. An app whose
sandbox cannot be resolved gets an `EACCES` failure on the first
named-object operation and a single error log naming `set_shm_dir` and
`CERULION_SHM_DIR`; the embedder must export `CERULION_SHM_DIR` with an
app-writable directory such as one under its files directory, since that is
the one setting both Cerulion and iceoryx2 read.
The selected directory must be writable by the process, owned by the process
user, and not writable by group or others. The directory itself must not be a
symbolic link, even when the path ends in `/` or `/.`, and a path containing
`..` is refused. Symbolic links in parent directories are followed; the
ownership and permission checks apply to the directory that is finally opened.
Objects remain until unlinked, so a process that exits without cleanup leaves
files in the directory, and on Android they survive a reboot. Point
`CERULION_SHM_DIR` at a directory used only by Cerulion, as the default
`cerulion-shm` directories are. To clear leftover objects, stop every Cerulion
process that uses the directory, then remove the files in it. Processes must be
able to write to `/data/local/tmp` and the selected shared-memory directory.

Android event-trigger datagram sockets use the abstract namespace and do not
create socket files. SELinux governs their access rather than file permissions.

Android does not provide robust mutexes. If a process exits while holding a
shared lock, other processes cannot recover that lock. The CLI,
`cerulion_cli_engine`, and `cerulion_wsd` do not build for Android. There is no
app embedding API.

## Run two processes

After placing the `cross_process_pubsub_iox2_test` binary at
`/data/local/tmp/cross_process_pubsub_iox2_test`, run the subscriber in one
shell:

```sh
adb shell 'cd /data/local/tmp && CERULION_XPROC_TOPIC=/xproc/example CERULION_XPROC_IDLE_FILE=/data/local/tmp/xproc.idle ./cross_process_pubsub_iox2_test --exact subscriber_role --include-ignored --nocapture --test-threads=1'
```

After `READY` appears, run the publisher in another shell:

```sh
adb shell 'cd /data/local/tmp && CERULION_XPROC_TOPIC=/xproc/example CERULION_XPROC_IDLE_FILE=/data/local/tmp/xproc.idle ./cross_process_pubsub_iox2_test --exact publisher_role --include-ignored --nocapture --test-threads=1'
```

The publisher sends all but the last message and creates `xproc.idle.sent`.
After receiving those messages and seeing that file, the subscriber drains the
notifications and creates `xproc.idle`. The publisher then sends the last
message and waits for `xproc.idle.done` before exiting. The subscriber reports
received, first, last, gaps, `wakes`, `queued_reads`, `timeout_drains` (which
must be 0), and `final_read`. `final_read` must be `Woken`, which proves that a
publisher event woke an idle subscriber across processes.
