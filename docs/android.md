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

Android does not provide `/dev/shm` in every environment. Named shared-memory
objects used by Cerulion are regular files on Android. Set `CERULION_SHM_DIR` to
an absolute directory to choose their location. If it is unset, empty, or
relative, the runtime uses `$TMPDIR/cerulion-shm` when `TMPDIR` is absolute;
otherwise it uses `/data/local/tmp/cerulion-shm`.
The selected directory must be writable by the process, owned by the process
user, and not writable by group or others. The directory itself must not be a
symbolic link, even when the path ends in `/` or `/.`, and a path containing
`..` is refused. Symbolic links in parent directories, such as the system links
in an Android app's data path, are followed; the ownership and permission checks
apply to the directory that is finally opened. As with POSIX shared memory, an
object remains until it is unlinked, so a process that exits without cleanup
leaves its files in the directory, and on Android they also survive a reboot.
Point `CERULION_SHM_DIR` at a directory used only by Cerulion, as the default
`cerulion-shm` directories are. To clear leftover objects, stop every Cerulion
process that uses the directory, then remove the files in it.

Creating a transport node fails on Android, because the iceoryx2 Android
platform layer does not implement named shared memory, so publish and subscribe
are not available on Android, within one process or between processes. The CLI,
`cerulion_cli_engine`, and `cerulion_wsd` do not build for Android. There is no
app embedding API.
