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
objects used by Cerulion are regular files on Android. The runtime chooses
their directory in this order: an absolute directory passed to
`cerulion_core::posix_shm::set_shm_dir`, then an absolute `CERULION_SHM_DIR`,
then `$TMPDIR/cerulion-shm` when `TMPDIR` is absolute. `set_shm_dir` must run
before the first named object is opened; calling it again with the resolved
directory is a no-op, and calling it with a different directory afterwards
returns an error. When nothing is configured, shell and root processes fall
back to `/data/local/tmp/cerulion-shm`. An app process cannot write there, so
an app with no configured directory gets an `EACCES` failure on the first
named-object operation and a single error log naming `set_shm_dir` and
`CERULION_SHM_DIR`; the embedder must call `set_shm_dir` (or export
`CERULION_SHM_DIR`) with an app-writable directory such as one under its
files directory.
The selected directory must be writable by the process, owned by the process
user, and not writable by group or others. The directory itself must not be a
symbolic link, even when the path ends in `/` or `/.`, and a path containing
`..` is refused. Symbolic links in parent directories are followed; the
ownership and permission checks apply to the directory that is finally opened. As with POSIX shared memory, an
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
