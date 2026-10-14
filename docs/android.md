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
```

Replace `cerulion_core` with any crate in the list to check that crate.

Android named shared-memory objects are regular files. Publish and subscribe
work within one process and between Android processes. Iceoryx2 transport files
are under `/data/local/tmp/iceoryx2`. Cerulion and transport named shared memory
use `CERULION_SHM_DIR`: a non-empty absolute value selects the directory;
otherwise an absolute `TMPDIR` selects `$TMPDIR/cerulion-shm`; otherwise the
directory is `/data/local/tmp/cerulion-shm`. Processes must be able to write to
`/data/local/tmp` and the selected shared-memory directory.
Android event sockets use the abstract namespace, so SELinux governs access
rather than file permissions.

Android does not provide robust mutexes. If a process exits while holding a
shared lock, other processes cannot recover that lock. The CLI,
`cerulion_cli_engine`, and `cerulion_wsd` do not build for Android. There is no
app embedding API.

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
