# Android internals research: native graph nodes in an app

Research note for issue #301. It asks how user-authored native nodes can run inside an
Android app with the same behavior as a desktop graph run. Companions: `core-transport.md`
(transport model, `local-transport`), `core-scheduler-graph.md` (graph runtime),
`../android.md` (Android build and shm). Code on `main` beats this note; when they disagree,
fix the note.

Labels used below:

- **VERIFIED**: a research lane read the code or the primary source at the cited line, or
  ran the experiment and saw the result.
- **INFERRED**: follows from verified facts, but nobody checked it directly.

Paths: `W/` is this repo. `W/` line numbers were refreshed against `main` at `4b08fc18`.
`I/` is the iceoryx2 fork with Android shm support, at rev `56ddaedad`, the rev the research
ran against; `main` now pins `8657bc53` (`W/Cargo.toml:654`), so `I/` line numbers can
drift. AOSP line numbers are from `refs/heads/main`, fetched 2026-10-15, unless a line names
a release branch. Experiments ran on an Android 11 arm64 emulator (API 30), one run each,
from a research worktree below `main`. Line numbers inside experiment output (test panics,
log lines) are from that worktree. The cited logs (`research/e1ctl.log`, `e1c.log`,
`e4b_logcat.txt`) were not kept; the lines quoted here are the record.

## Question

With `local-transport` on (`iceoryx2::service::local_threadsafe::Service`, state in
statics), a graph node loaded from a cdylib delivers no frames to host subscribers. Stopping
a graph whose node library was loaded also crashed with SIGSEGV in `pthread_key_clean_all`.
So today the app refuses a `graph_yaml` that has native nodes (error code 5). What must change
so native nodes run inside an Android app with desktop parity?

## Short answer

Option (f): build the Android app **without** `local-transport`, use the same cross-process
(`ipc_threadsafe`) transport and `dlopen` loader as desktop, keep the iceoryx2 root and the
Cerulion shm directory in app-private storage (`main` already does this for the host copy;
see PR 2), and never unload a node library on Android.
One run on an Android 11 emulator delivered 158 native-node frames and stopped cleanly.

What (f) does **not** give, and must not be claimed:

- **Multi-process parity.** The app builds one in-process graph, like desktop
  `cerulion graph run --single-process`. No fault isolation; `process_groups` do not start
  processes. Only (e) gives multi-process parity.
- **Run-time node loading on Play.** Play forbids downloaded `.so` files, so a Play build
  ships every node in the APK/AAB, as (a) does. Loading from `files/` is for dev or sideload
  builds only.

(f) is still the best choice: same transport, loader and serial cdylib scheduling as desktop,
smallest change. It is **not ready to ship** until four gates pass: a second start in one
process, hot-path latency with shm on `/data`, a run on API 36 (Play's current targetSdk),
and a node-side iceoryx2 config check (see "Gates before the parity claim").

## Why #301 happens

### Two registries in one process (VERIFIED)

- A node is a `cdylib` only: `crate-type = ["cdylib"]`, `default = ["cdylib"]`
  (`W/crates/cerulion_cli_engine/src/templates.rs:22-41`). It depends on `cerulion_core`
  statically, and `cerulion_core` brings in iceoryx2.
- The repo says each node has its own copy: "This cdylib statically links its OWN copy of
  `iceoryx2` + `iceoryx2-log`" and "its OWN tracing static"
  (`W/crates/cerulion_macros/src/codegen.rs:2091-2120`); "the cdylib links its OWN copy of
  cerulion_core, whose `INSTANCE` static the host never initialized"
  (`W/crates/cerulion_core/src/transport/mod.rs:2340-2348`).
- `CerService` is `local_threadsafe::Service` under `local-transport`
  (`W/crates/cerulion_core/src/transport/mod.rs:85-88`). That service maps every storage type
  to `process_local` (`I/iceoryx2/src/service/local_threadsafe.rs:47-67`).
- The registry is a static, one per linked copy:
  `static PROCESS_LOCAL_STORAGE: LazyLock<Mutex<BTreeMap<FilePath, StorageEntry>>>`
  (`I/iceoryx2-cal/src/dynamic_storage/process_local.rs:198-208`). Zero-copy connections
  (`I/iceoryx2-cal/src/zero_copy_connection/process_local.rs:15-16`) and the local event
  (`SocketPairCountingBitSet`, `I/iceoryx2-cal/src/event/recommended.rs:19`, state in
  `process_local::Storage` at `I/iceoryx2-cal/src/event/trigger/socket_pair.rs:329-331`)
  use it.
- The host passes iceoryx2 port objects it created into the node:
  `cerulion_node_init(*mut NodeContext)` hands over a `Box<NodeContext>`
  (`W/crates/cerulion_macros/src/codegen.rs:2185-2188`). `NodeContext` is `repr(Rust)` and
  holds `IndexMap<String, AnyPublisher>` / `IndexMap<String, AnySubscriber>`
  (`W/crates/cerulion_core/src/graph/node.rs:1605-1691`). The code that runs on those ports
  (loan, send, receive, connection update) is the node's own copy of iceoryx2.
- Connections are made lazily with `open_or_create`
  (`I/iceoryx2/src/port/publisher.rs:318-333`;
  `I/iceoryx2-cal/src/zero_copy_connection/common.rs:392-429`, `583-605`). The node copy finds
  nothing in its own registry and **creates** a new connection there; the host does the same
  in its registry. Both succeed, so the loss is silent. Inputs fail the same way in reverse.
- Under `ipc_threadsafe` the same lookup goes through named files, so separate copies meet.
  That is why desktop graph runs work.

**Symbol evidence (VERIFIED, `nm` / `llvm-nm` / `llvm-readelf`):**

- macOS host test: the node `.dylib` has 8931 defined iceoryx2 symbols, exports only 20
  symbols (all `cerulion_*` C entry points), and has 0 undefined iceoryx2 or cerulion_core
  references. It holds a local (`d`) `PROCESS_LOCAL_STORAGE` in crate instance
  `Cs6koCar3BFsJ`; the host test binary holds its own (`D`) in `CsgtQfdpr7ErN`. Two
  registries, one process.
- Android arm64 node `.so`: 20 dynamic symbols, 0 iceoryx2, 0 cerulion_core, 2910 local
  iceoryx2 symbols; DT_NEEDED is only `libdl`, `libm`, `libc`.

**Reproduction (VERIFIED, macOS, rustc 1.99.0):** with the refusal skipped by a research-only
env var and the fixture built with the `local-transport` feature of `cerulion_core`:

```
WARN cerulion_core::transport::publisher: SentSample notify reached fewer listeners than this topic has ... undelivered=2 triggered=0 listeners=2
panicked at crates/cerulion_android/tests/embedded_runtime_test.rs:120:5: assertion failed: subscriber.wait_for_message(...) > 0
```

With the default transport the delivery assertions at lines 120 and about 150 pass. Note:
`embedded_runtime_test.rs:82` looks for a `.so`, so on macOS the fixture needs a `.so`
symlink to the `.dylib` first. The same control run has two other failures that matter for
(f); see "Restart in one process" below.

**Why the symbols cannot merge (VERIFIED, AOSP and Rust Reference):**

- `dlopen` from native code uses the caller's namespace (`bionic/linker/linker.cpp:2056-2057`,
  `:2123`). One soname loads once per namespace (`linker.cpp:1371-1416`, `:1456-1468`).
- RTLD_LOCAL is the default since API 23, and lookup searches the global group then the
  library's own DT_NEEDED closure (`bionic/android-changes-for-ndk-developers.md:69-77`,
  `:101-108`; `linker.cpp:803`). RTLD_GLOBAL only changes imports; it cannot remove a copy
  that was linked statically into a cdylib.
- Rust Reference, Linkage: "if dynamic libraries B and C were each statically linked to
  library A, then a crate could not link to B and C together because there would be two
  copies of A."
- INFERRED: rustc exports only the `#[no_mangle]` / `extern` API from a cdylib, so the
  internal symbols cannot be interposed. The symbol counts above agree.

### The stop crash (VERIFIED)

- `rustc --print cfg --target aarch64-linux-android` has no `target_thread_local`, so std uses
  the `os` thread-local backend (`library/std/src/sys/thread_local/mod.rs:36-57`). Each
  thread-local with a destructor gets a pthread key whose destructor `destroy_value::<T>`
  (`os.rs:157`, `225`; `key/unix.rs:26-28`) is code inside the node `.so`.
- Bionic `pthread_key_create` stores only a function pointer and keeps no reference to its
  library (`bionic/libc/bionic/pthread_key.cpp:130-135`); `pthread_key_clean_all` calls it at
  thread exit (`:78-117`). Only `__cxa_thread_atexit_impl` pins the library
  (`bionic/libc/bionic/__cxa_thread_atexit_impl.cpp:45-46`), and Rust does not use it on
  Android. On glibc Rust uses that path (`destructors/linux_like.rs:40-53`).
- The Android node `.so` imports `pthread_key_create` / `pthread_key_delete` and contains
  `std::sys::thread_local::os::Storage` symbols. Node thread-locals include `LAST_ERROR`
  (`codegen.rs:1883-1886`) and the node's own tracing copy (`codegen.rs:2091-2105`,
  `node.rs:2946`).
- `DylibNodeEntry::drop` drops `_library` (dlclose) unless `leak_library_on_drop` is set
  (`W/crates/cerulion_core/src/graph/node.rs:6677-6686`; field docs 4041-4055). It is
  `false` at construction (`node.rs:4820`) and set only in the doorbell-thread case
  (`node.rs:6604-6605`).
- Reproduced in a real app process (E4 run 1 below):
  `Fatal signal 11 (SIGSEGV) ... in tid 5052 (cerulion-graph)`,
  `#01 libc.so (pthread_key_clean_all()+136)`, `#02 libc.so (pthread_exit+72)`.
- The NDK doc names the two fixes that work at every API level: `-Wl,-z,nodelete` (or
  RTLD_NODELETE), or never call `dlclose` (`bionic/android-changes-for-ndk-developers.md:443-461`).
- INFERRED: the node thread exits after dlclose and bionic calls a destructor in unmapped
  memory. With the leak on (E4 run 2) the crash is gone.
- Leaking matches desktop: on glibc "the cdylib's TLS forces `DF_1_NODELETE`", so the library
  is never unmapped there either (`node.rs:4048-4051`, VERIFIED).

### Where the code refuses today (VERIFIED)

- Android-only dependency `cerulion_core = { ..., features = ["local-transport"] }`
  (`W/crates/cerulion_android/Cargo.toml:26-27`).
- `refuse_dlopen_nodes` (`W/crates/cerulion_android/src/runtime.rs:824`, `1617-1624`), error
  code `GRAPH = 5` (`W/crates/cerulion_android/src/error.rs:18`), documented in
  `W/docs/android.md:29` and `198-200`.
- The ABI value carries the transport kind (`W/crates/cerulion_core/src/transport/kind.rs:13-32`,
  `60-83`). The core loader refuses a node when the kinds **differ** (`node.rs:4279-4290`).
  At research time that was its only transport check: a host and node both built with
  `local-transport` loaded and lost frames silently (E1, `e1c.log:17`, log not kept), and
  `refuse_dlopen_nodes` was the only guard against #301.
- `main` now has a second guard: `DylibNodeEntry::load` refuses every cdylib node in a
  `local-transport` host, matching kinds included (`node.rs:4246-4262`;
  `W/crates/cerulion_core/tests/local_transport_test.rs:475-512`). The matching-kind test is
  `#[ignore]`, because it needs a fixture built with `local-transport`.
- No crate other than `cerulion_android` turns on `local-transport`
  (`W/crates/cerulion_core/Cargo.toml:265`, `W/crates/cerulion_android/Cargo.toml:27`). CI has
  a `local-transport` job and an Android check of it (`.github/workflows/ci.yml:1436-1437`,
  `1466-1551`).

## Android platform facts that decide the options

| Fact | Status | Evidence |
|---|---|---|
| App libs load in classloader namespace `clns-N`, search path = app JNI lib dir | VERIFIED | `art/libnativeloader/library_namespaces.cpp:53-61`; `art/libnativeloader/README.md` |
| `/data` (and `/mnt/expand`) is always a permitted path; `LoadedApk` adds the data dir | VERIFIED | `library_namespaces.cpp:78`, `:242-248`; `frameworks/base/core/java/android/app/LoadedApk.java:927` |
| `dlopen` (mmap with execute) of a `.so` in the app data dir is allowed for untrusted apps, with `auditallow`; same rule on android11-15 and main | VERIFIED | `system/sepolicy/private/untrusted_app_all.te:26-28` (main; line 26 on android11, 27 on 12-15) |
| W^X blocks only `execute_no_trans` (execve) on `app_data_file`; `execute` (mmap) is kept for compatibility | VERIFIED | `private/app_neverallows.te:64-73` (rule), `:77-79` (comment citing b/237289679) |
| `execve()` of files in the app home dir is denied for targetSdk 29 and up | VERIFIED (policy + E3) | `private/app_neverallows.te:64-73`; `private/untrusted_app_27.te:22-25`; developer.android.com Android 10 behavior changes |
| `execve()` of a file in `nativeLibraryDir` (`apk_data_file`) is allowed if libs are extracted | VERIFIED (policy + E3) | `private/file_contexts:611`; `private/app.te:442-443`; `public/global_macros:24`; our app sets `useLegacyPackaging = true` (`W/crates/cerulion_android/android/app/build.gradle:23`) |
| targetSdk 34 and up runs as `untrusted_app`; 30-31 as `untrusted_app_30`; 32-33 as `untrusted_app_32`. On android11, `minTargetSdkVersion=30` maps to `untrusted_app` | VERIFIED | `private/seapp_contexts:205-211`; android11 `seapp_contexts:169`; our app is targetSdk 34 (`build.gradle:13`) |
| App gets `create_file_perms` (including `map`) on `app_data_file`; same on android11-15 and main | VERIFIED | `private/app.te:249-258` (main); android11 `public/app.te:70`, android12 `:73`, android13 `private/app.te:253`, android14 `:260`, android15 `:254`; `public/global_macros:25` |
| App may create unix datagram sockets and `sendto` itself | VERIFIED | `public/domain.te:29` (android11-14), `private/domain.te:29` (15, main) |
| Cross-app datagram injection is blocked by MLS (`sendto` needs `l1 eq l2`); same on android11-15 and main | VERIFIED | `private/mls:32-33` |
| No `shm_open` in bionic; no `/dev/shm` label | VERIFIED | `bionic/docs/status.md:25-30`; `file_contexts` |
| `/dev/ashmem` direct use only for targetSdk below 29; use `ASharedMemory` | VERIFIED | `private/untrusted_app_27.te:32-35`; developer.android.com/ndk/reference/group/memory |
| `memfd_create` from API 30; app tmpfs is `appdomain_tmpfs` with map/read/write | VERIFIED | `bionic/libc/include/sys/mman.h:186-197`; `public/te_macros:242-248` |
| `/data/local/tmp` is `shell_data_file`; apps get read only, so cannot create files there | VERIFIED | `private/file_contexts:626`; `private/untrusted_app_all.te:55-56` |
| iceoryx2 fork default root is `/data/local/tmp/iceoryx2` | VERIFIED | `I/iceoryx2-pal/configuration/src/lib.rs:59-65`; `W/docs/android.md:59-60` |
| Forked children count as phantom processes, default limit 32 | VERIFIED | `frameworks/base/services/core/java/com/android/server/am/ActivityManagerConstants.java:229`, `:328`, `:2195-2198`; `PhantomProcessList.java:55-80` |
| `isolatedProcess=true` runs as `isolated_app` with no `app_data_file` access | VERIFIED | `private/seapp_contexts:182`; `private/app.te:247` |
| Play: an app "may not download executable code (such as dex, JAR, .so files) from a source other than Google Play" | VERIFIED | support.google.com/googleplay/android-developer/answer/9888379 |
| Android 14 "safer dynamic code loading" covers DEX/JAR/APK via class loaders; native `dlopen` is not checked. `Runtime.load0` (Java `System.load`) logs a warning for a writable `.so`, behind a flag, saying it "will throw on a future Android version" | VERIFIED | developer.android.com/about/versions/14/behavior-changes-14; `libcore/ojluni/src/main/java/java/lang/Runtime.java:930-935` |
| New apps and updates must target API 36 since 2026-08-31; apps targeting 35+ must support 16 KB pages, and Play blocks non-compliant updates from 2027-02-01 | VERIFIED | developer.android.com/google/play/requirements/target-sdk; developer.android.com/guide/practices/page-sizes |
| Our built `.so` files are 16 KB ready: every `PT_LOAD` aligned to `0x4000`; the fork reads page size at run time | VERIFIED | `llvm-readelf -l` on `libcerulion_android.so` and `libtest_node_macro_period_cdylib.so`; `I/iceoryx2-bb/posix/src/memory_mapping.rs:600-608` |
| Credential encrypted storage "is the default storage location and only available after the user has unlocked the device"; device-protected storage via `createDeviceProtectedStorageContext()` | VERIFIED (doc); `filesDir` is CE storage is INFERRED | developer.android.com/privacy-and-security/direct-boot |

## Experiments

All on an Android 11 arm64 emulator, one run each, from a research worktree. No change was
committed.

| # | What | Result | Status |
|---|---|---|---|
| E1 | #301 on macOS host with local transport and refusal skipped | Frames lost (`triggered=0`), test fails at line 120; default transport passes the same asserts | VERIFIED |
| E1-ctl | Same test, default transport, serial (`--test-threads=1`), macOS | Fails at line 161 of the research worktree (second `start`: `EventOpenOrCreateError::EventOpenError(InternalFailure)` on `/embedded_<pid>/ticker/cmd`) and line 56 (`restart_after_component_failure_finalizes_both_bags`). Same test passes on Linux CI | VERIFIED (`research/e1ctl.log:35-41`, log not kept; Linux CI run 38679568296, job 116241170660) |
| E2 | `cross_process_pubsub_iox2_test` via `run-as`, iceoryx2 root and `CERULION_SHM_DIR` under `files/` | Subscriber `received=1000 first=0 last=999 gaps=0`; no denial from the test's domain. Domain was `runas_app`, not `untrusted_app` (`seapp_contexts` line 174), and exec worked only because of `private/runas_app.te:11` | VERIFIED |
| E3 | Scratch app (targetSdk 34, `useLegacyPackaging = true`) exec of a binary | `lib/arm64/libexecprobe.so` exits 0 in `u:r:untrusted_app`; same binary in `files/` fails with `error=13, Permission denied` | VERIFIED |
| E4 run 1 | Real app process, no `local-transport`, iceoryx2 `root_path=<files>/iox2/`, `CERULION_SHM_DIR=<files>/shm` (both 0700), node `.so` in jniLibs | Graph ran (`"nodes":1,"state":"running"`); on stop, SIGSEGV in `pthread_key_clean_all` | VERIFIED |
| E4 run 2 | Same, plus `leak_library_on_drop: cfg!(target_os = "android")` at the construction site (now `node.rs:4820`) | Status after stop: `"state":"stopped"`, recorder `/exp/ticker/cmd: 158`, `/app/heartbeat: 15`, `last_error: null` (`e4b_logcat.txt:270`, log not kept); process alive 5 s later; no denial from the app's domain (the 4 `avc` lines at `e4b_logcat.txt:22,24,29,108` are `system_server` against a `shell` socket); shm and storage files under `files/`, but event sockets are abstract (see below) | VERIFIED |

E4 scope: one start and one stop, a source-only ticker node (no inputs), node packaged in
jniLibs, API 30 emulator, clean start state. Not tested: a second start, start after a kill,
a node with inputs, loading from `files/`, the TCP gateway, latency, API 34+, a physical
device.

Side findings (VERIFIED):

- The `*_node.0_10_0.global_mgmt` file is created mode 0666, inside a 0700 directory (E2, E4).
- On Android the fork uses **abstract** unix datagram sockets for events (commit `ab7fc172e`;
  `I/iceoryx2-bb/posix/src/unix_datagram_socket.rs:315-334`); only marker files sit under the
  root (`I/iceoryx2-cal/src/communication_channel/unix_datagram.rs:281-282`). The abstract
  namespace is shared by all apps, so the 0700 directory does not protect them. Injection is
  blocked by `private/mls:32-33`. INFERRED: another app can bind the same name first (the name
  comes from a public path), a denial-of-service risk, not a functional blocker.

## Options

| Option | Works on Android? | Desktop parity | Effort | Risk | Evidence |
|---|---|---|---|---|---|
| **(f) Drop `local-transport` in the app; cross-process transport with root and shm in app storage; keep `dlopen`; never unload on Android** | Yes, on Android 11 emulator, one start/stop (E4 run 2) | **Single-process parity** (like `--single-process`): same transport, loader and serial cdylib scheduling as desktop. No process isolation; `process_groups` run as one process. Run-time loading only outside Play; on Play, nodes ship in the APK. No reload of a rebuilt node without a process restart | Small, about 3-5 files | Medium: second start fails on macOS under this transport; shm moves to flash-backed `/data`; node-side iceoryx2 config differs from host; one API level, one run | E4; `untrusted_app_all.te:26-28`; `app.te:249-258`; `runtime.rs:1596-1615` |
| (a) Link node crates into the app `.so` at build time (generated registry of `{Name}Entry`) | Yes by construction (INFERRED); not run | Same node code and `NodeEntry`; rebuild per node change (as (f) on Play); macro nodes run on the rayon parallel path, cdylib nodes serially (`codegen.rs:872-877`, `node.rs:3820-3826`) | Medium, about 4-6 files plus a CLI or Gradle step; `cerulion_android` is already `["rlib","cdylib"]` (`Cargo.toml:8-9`) | Low at run time; `#[no_mangle]` symbol clashes unless node crates use `default-features = false` (INFERRED); no stop crash; no second registry | `codegen.rs:1-8`, `119`, `661`, `1762`; `runtime.rs:4087-4089` (core); `crates/cerulion_android/src/runtime.rs:840-851`; `tests/node_state_test.rs:388` |
| (b) One shared Rust `dylib` for cerulion_core + iceoryx2, DT_NEEDED from host and nodes | Platform allows one soinfo per soname per namespace (VERIFIED); not run | Single-process, run-time loading | Small in manifests, plus shipping `libstd-*.so` | High: unstable Rust dylib ABI across builds; `cerulion_android` depends on `iceoryx2 = "=0.10.0"` directly (`Cargo.toml:17`), likely conflict (INFERRED); weak cargo-ndk support; TLS crash NOT fixed | `linker.cpp:1371-1416`; Rust Reference Linkage |
| (c) Host-owned transport behind a stable C ABI vtable; node SDK crate without iceoryx2 | Yes (INFERRED); not run | Single-process, and a better desktop design (removes `repr(Rust)` layout and fingerprint check) | Large: `codegen.rs` `gen_cdylib`, `node.rs`, `output_proxy.rs`, subscriber views, ABI bump | High, touches desktop; TLS crash NOT fixed | `node.rs:1605-1691`, `1641-1647`, `4332-4353` |
| (d) Keep `local-transport`; share one process-local registry across copies | Not run | Single-process | Large: fork-wide changes to statics | High: the repo already documents host/node static traps (`transport/mod.rs:2351-2373`) | Skeptic review; not cheaper than (f) |
| (e) Nodes in child processes on the cross-process transport | Exec only from `nativeLibraryDir` (E3); phantom-process limit 32 | **Only option with multi-process parity** (fault isolation, `process_groups`) | Medium-large: packaged worker, supervisor in `cerulion_android` | Medium-high: no robust mutexes on Android, a child that dies holding a lock is unrecoverable (`W/docs/android.md:106-107`); system may kill children in background (INFERRED) | `partition.rs:169`; `kind.rs:85-109`; `graph_cmd.rs:4587`, `11618`; `ActivityManagerConstants.java:229` |

Every option that keeps `dlopen` in one process ((f), (b), (c)) also needs the Android leak
fix; only (a) and (e) remove the stop crash by construction. (e) can be built later on top of
(f), because both use the same cross-process transport.

Process-model evidence (VERIFIED): the app builds with
`build_live_with_schema_hashes_policy_and_recorded` (`W/crates/cerulion_android/src/runtime.rs:1182`),
which passes `CrossProcessWiring::none()` with the comment "Single-process live build"
(`W/crates/cerulion_core/src/graph/runtime.rs:4162-4183`). Desktop `cerulion graph run` on
Unix derives a partition and "runs multi-process" (`W/crates/cerulion_core/src/graph/config.rs:85-96`).
In the core runtime, `process_groups` is a colocation hint only (`runtime.rs:5183-5191`);
`cerulion_android` has no `process_groups` handling (grep, no match).

## Recommendation: option (f), scoped

(f) uses the same transport and loader as desktop, is the smallest change, and is the only
option shown working in a real app process. It gives single-process parity, not
multi-process parity. On Play it means "nodes bundled in the APK/AAB"; run-time loading from
`files/` is for dev or sideload builds only, and the file must be read-only first. If
multi-process parity becomes a requirement, add (e) on top of (f). (a) stays the fallback if
`dlopen` itself is blocked.

### Gates before the parity claim

These must pass on a device before anyone claims (f) works:

1. **Restart in one process.** Start, stop, start again in one process, and start after the
   process was killed (leftover files: `W/docs/android.md:95-107`). The only data point today is
   a failure on macOS (E1-ctl). INFERRED: on macOS the node copy is really unloaded and
   reloaded; on Linux glibc keeps it mapped, which is close to Android with the leak on. So
   Android may pass, but nobody has tried.
2. **Latency and flash wear.** On Android, shm objects are ordinary files made with `openat`
   in `CERULION_SHM_DIR` (`I/iceoryx2-pal/posix/src/android/mman.rs:160-180`). Moving that
   directory into app storage puts every sample pool on `/data`. The pool notes assume "LAZY
   demand-paged tmpfs … oversizing is latency-free" (`W/docs/internals/core-transport.md:513-514`),
   which does not hold on `/data` (INFERRED cost: block allocation on first write, dirty-page
   writeback, flash wear). Benchmark (f) on `/data` against `local-transport` and desktop
   tmpfs. This also affects apps **without** `graph_yaml`, which today publish in process.
3. **Node-side iceoryx2 config.** `install_iceoryx_config` sets only the host copy's global
   config (`W/crates/cerulion_android/src/runtime.rs:1596-1615`). The node copy, if it ever calls
   `Config::global_config()`, searches a relative `config/`, the user config dir and `/etc`
   (`I/iceoryx2/src/config.rs:633-660`), then falls back to `/data/local/tmp/iceoryx2/`
   (`I/iceoryx2-pal/configuration/src/lib.rs:59-65`), which apps cannot write. iceoryx2 uses
   that fallback when it creates a Node without explicit config (`I/iceoryx2/src/node/mod.rs:625`,
   `:1594`). Host-side callers today: `run_registry.rs:1415`, `:1455`;
   `mirror_registry.rs:1694-1702`; `flashback/channel.rs:900`. On desktop both copies default
   to the same `/tmp/iceoryx2/`, so a break would show **only on Android**. E4 passed because
   the node only used ports the host made (the rule in `transport/mod.rs:2173-2178`). Test a
   node with inputs and a subscriber.
4. **API levels and pages.** Run on API 30, 34, 35 and 36, on a 16 KB page-size emulator, and
   on one physical device, with targetSdk 36. Also run the TCP gateway.

### Implementation PRs (smallest first)

1. **Never unload node libraries on Android.** In `W/crates/cerulion_core/src/graph/node.rs`,
   construct `DylibNodeEntry` with `leak_library_on_drop: cfg!(target_os = "android")`
   (line 4820), and update the field docs (4041-4055). Leaking at construction does **not**
   cover the early returns in `load` (`node.rs:4245-4824`): they drop the local `Library`
   before `DylibNodeEntry` exists. One of them runs node code that touches `LAST_ERROR`: a
   failed `cerulion_node_platform_check` (`node.rs:4495-4502`; `codegen.rs:2027-2050`,
   `1883-1903`). So on Android open with
   `libloading::os::unix::Library::open(path, RTLD_NOW | RTLD_NODELETE)` in place of the
   `Library::new` call at `node.rs:4265` (NDK doc `:443-461`), which covers failed loads too,
   or forget the `Library` on every error path after the first node call. Side
   effect: a rebuilt node at the same path does not reload until the process restarts
   (`linker.cpp:1371-1416`, INFERRED). No Android unit test: CI only runs `cargo ndk check` and
   `cargo ndk build` (`.github/workflows/ci.yml:1415-1440`); PR 4 is the real gate.
2. **App-private iceoryx2 root and shm dir.** `main` already does the host half:
   `install_iceoryx_config` (`runtime.rs:1596-1615`, called at `:503`) writes
   `files/iceoryx2.toml` with `global.root_path = <files>/iceoryx2` and loads it as the host's
   global config, tested by `install_iceoryx_config_roots_the_state_inside_the_app_sandbox`
   (`runtime.rs:2176`); `init` creates `files/cerulion-shm` 0700 and passes it to
   `set_shm_dir` (`W/crates/cerulion_android/src/host.rs:404-425`). Left to do: the fork places
   its own shm segments from `CERULION_SHM_DIR` and never sees `set_shm_dir`
   (`W/docs/android.md:64-71`). Set `CERULION_SHM_DIR` safely: from Java with `Os.setenv` before
   `System.loadLibrary`, or as a config field. Not with `std::env::set_var` in Rust (the
   research diff did this; ART threads already exist, and each copy latches the value in its
   own static: `W/crates/cerulion_core/src/posix_shm.rs:246-247`, `275`, and the fork's
   `mman.rs`).
   Make the node copy resolve the same root (gate 3): put the config where every copy searches,
   or add a rule plus a test that node code never calls `Config::global_config()`. Choose CE
   (`filesDir`) or device-protected storage: a robot app that starts at boot cannot use CE
   storage until the user unlocks. This PR prepares PR 3; it fixes nothing alone, because under
   `local-transport` nothing uses the root.
3. **Switch the Android build to the cross-process transport.** Remove the Android-only
   `local-transport` line (`W/crates/cerulion_android/Cargo.toml:26-27`), remove
   `refuse_dlopen_nodes` and its tests (`runtime.rs:824`, `1617-1624`, `2200-2219`), and update
   `W/docs/android.md:29`, `198-200` and `308`. Then either delete the `local-transport` feature
   and its CI jobs (`ci.yml:1436-1437`, `1466-1551`), or, if gate 2 says apps without native
   nodes need it, keep it: the refusal in `DylibNodeEntry::load` under
   `cfg(feature = "local-transport")` is already on `main` (`node.rs:4246-4262`), so #301 can
   never be silent. Bump the app to targetSdk 36.
4. **Ship a node in the sample app; required device test.** Node `.so` in jniLibs,
   `node_lib_dir = nativeLibraryDir` (`MainActivity.java:274`). An instrumented test asserts
   frames on a node output, a node with inputs, start/stop/start, start after kill, and no
   denial from the app's domain. Run the gate 4 matrix. Add a check that node `.so` files have
   `PT_LOAD` alignment of at least `0x4000`.
5. **Tighten the mgmt file mode.** Create `*_node.*.global_mgmt` 0600 in the fork instead of
   0666, or document why the 0700 parent is enough. Note the abstract event sockets are
   outside that directory.

## Challenges considered

Three adversarial reviews checked this note. What changed:

| Challenge | Result |
|---|---|
| "Desktop parity, loadable at run time" | **Accepted.** Parity is single-process only; on Play nodes ship in the APK |
| Second-start failure dismissed as "not #301" | **Accepted.** It happens under the transport (f) adopts; now gate 1 |
| shm on `/data` changes hot-path latency | **Accepted** as unmeasured risk; now gate 2 |
| Node copy keeps its own iceoryx2 config | **Accepted** (INFERRED break, Android only); now gate 3 and part of PR 2 |
| "All iceoryx2 objects under `files/`" | **Corrected.** Event sockets are abstract; injection blocked by MLS |
| "No `avc: denied`" | **Narrowed** to the app's domain |
| targetSdk 34, "Android 11-15" | **Updated** to targetSdk 36; SELinux rules checked on android11-15 and main |
| `std::env::set_var` for `CERULION_SHM_DIR` | **Accepted.** PR 2 sets it from Java or config |
| Direct Boot and CE storage | **Added** as a PR 2 choice |
| "PR 2 safe on its own" / "unit test on Android" / "keep `local-transport` for other users" | **Corrected** in PRs 1-3 |
| `RTLD_NODELETE` instead of leaking | Preferred in PR 1: a leak at construction misses failed loads |
| Cheaper options: shared registry (d), build-time (a) | Neither beats (f) for parity; (a) stays the fallback |
| Recommendation (f) refuted? | **No.** All three reviews kept (f); its claim is now scoped |

## Open questions

- Do we need a Play build? If yes, (f) with bundled nodes is the default; downloaded nodes
  are not allowed (answer/9888379).
- Is multi-process parity (fault isolation, `process_groups`) a requirement? If yes, plan (e)
  on top of (f).
- Does `-Wl,-z,nodelete` on node links read better than host-side `RTLD_NODELETE`? Both
  cover failed loads; a leak at construction does not.
- E3: the denied exec in `files/` logged no `avc` line. Is that denial silent? Not checked.
- No repo test shows the frame loss itself. `loader_refuses_a_matching_local_transport_cdylib`
  (`local_transport_test.rs:508-512`) covers the refusal, but it is `#[ignore]`, because no
  fixture is built with `local-transport`. Needed only if the feature stays (PR 3).

## Sources

Repo (`W/`, `main` at `4b08fc18`):

- `crates/cerulion_cli_engine/src/templates.rs:22-41`, `43-60`
- `crates/cerulion_macros/src/codegen.rs:1-8`, `119`, `661`, `872-877`, `1762`,
  `1883-1903`, `2027-2050`, `2091-2120`, `2185-2188`
- `crates/cerulion_core/src/graph/node.rs:1605-1691`, `1641-1647`, `2946`, `3820-3826`,
  `3782-4062`, `4041-4055`, `4245-4824`, `4246-4262`, `4265`, `4279-4290`, `4332-4353`,
  `4495-4502`, `4820`, `6604-6605`, `6677-6686`
- `crates/cerulion_core/src/graph/runtime.rs:4087-4089`, `4162-4183`, `5183-5191`
- `crates/cerulion_core/src/graph/config.rs:85-96`
- `crates/cerulion_core/src/graph/partition.rs:169`
- `crates/cerulion_core/src/transport/mod.rs:85-88`, `2340-2348`, `2351-2373`
- `crates/cerulion_core/src/transport/kind.rs:13-32`, `60-83`, `85-109`
- `crates/cerulion_core/src/posix_shm.rs:115-128`, `246-247`, `275`
- `crates/cerulion_core/Cargo.toml:265`
- `crates/cerulion_core/tests/local_transport_test.rs:475-512`
- `crates/cerulion_core/tests/node_state_test.rs:388`
- `crates/cerulion_android/Cargo.toml:8-9`, `17`, `26-27`
- `crates/cerulion_android/src/runtime.rs:503`, `824`, `840-851`, `1182`, `1596-1615`, `1617-1624`, `2176`, `2200-2219`
- `crates/cerulion_android/src/host.rs:404-425`
- `crates/cerulion_android/src/error.rs:18`
- `crates/cerulion_android/tests/embedded_runtime_test.rs:82` (the experiment lines 56, 120,
  161 are from the research worktree)
- `crates/cerulion_android/android/app/build.gradle:12`, `13`, `23`; `MainActivity.java:274`
- `crates/cerulion_cli_engine/src/graph_cmd.rs:4587`, `11618`
- `docs/android.md:29`, `59-60`, `64-71`, `95-107`, `198-200`, `308`
- `docs/internals/core-transport.md:513-514`
- `.github/workflows/ci.yml:1415-1440`, `1466-1551`
- `Cargo.toml:654`

iceoryx2 fork (`I/`, rev `56ddaedad`):

- `iceoryx2/src/service/local_threadsafe.rs:47-67`
- `iceoryx2/src/port/publisher.rs:318-333`
- `iceoryx2/src/config.rs:233-283`, `633-660`, `735-760`
- `iceoryx2/src/node/mod.rs:625`, `1594`
- `iceoryx2-cal/src/dynamic_storage/process_local.rs:198-208`
- `iceoryx2-cal/src/zero_copy_connection/process_local.rs:15-16`
- `iceoryx2-cal/src/zero_copy_connection/common.rs:392-429`, `583-605`
- `iceoryx2-cal/src/event/recommended.rs:19`
- `iceoryx2-cal/src/event/trigger/socket_pair.rs:329-331`
- `iceoryx2-cal/src/communication_channel/unix_datagram.rs:281-282`
- `iceoryx2-bb/posix/src/unix_datagram_socket.rs:315-334`
- `iceoryx2-bb/posix/src/memory_mapping.rs:600-608`
- `iceoryx2-pal/configuration/src/lib.rs:59-65`
- `iceoryx2-pal/posix/src/android/mman.rs:35`, `160-180`

Rust std (rustc 1.99.0 source):

- `library/std/src/sys/thread_local/mod.rs:36-57`; `os.rs:157`, `225`; `key/unix.rs:26-28`;
  `destructors/linux_like.rs:40-53`
- https://doc.rust-lang.org/reference/linkage.html

AOSP (`refs/heads/main` unless a release branch is named, https://cs.android.com):

- `bionic/linker/linker.cpp:803`, `1215-1217`, `1371-1416`, `1456-1468`, `2056-2057`, `2123`
- `bionic/android-changes-for-ndk-developers.md:69-77`, `101-108`, `294-355`, `443-461`
- `bionic/libc/bionic/pthread_key.cpp:78-117`, `130-135`
- `bionic/libc/bionic/__cxa_thread_atexit_impl.cpp:45-46`
- `bionic/libc/include/sys/mman.h:186-197`; `bionic/docs/status.md:25-30`, `125`
- `art/libnativeloader/library_namespaces.cpp:53-61`, `78`, `242-248`; `art/libnativeloader/README.md`
- `frameworks/base/core/java/android/app/LoadedApk.java:927`
- `frameworks/base/services/core/java/com/android/server/am/ActivityManagerConstants.java:229`, `328`, `2195-2198`
- `frameworks/base/services/core/java/com/android/server/am/PhantomProcessList.java:55-80`
- `libcore/ojluni/src/main/java/java/lang/Runtime.java:930-935`
- `system/sepolicy/private/app_neverallows.te:64-73`, `77-79`
- `system/sepolicy/private/untrusted_app_all.te:26-28`, `37-38`, `55-56` (and android11-15)
- `system/sepolicy/private/untrusted_app_27.te:22-25`, `32-35`
- `system/sepolicy/private/app.te:247`, `249-258`, `442-443`; android11 `public/app.te:70`,
  android12 `:73`, android13-15 `private/app.te:253`, `260`, `254`
- `system/sepolicy/private/domain.te:7-8`, `29-30`; `public/domain.te:29` (android11-14)
- `system/sepolicy/private/mls:32-33`
- `system/sepolicy/private/file_contexts:611`, `626`
- `system/sepolicy/private/seapp_contexts:174`, `182`, `205-211`; android11 `:169`
- `system/sepolicy/private/runas_app.te:11` (android11)
- `system/sepolicy/public/global_macros:24`, `25`, `27`; `public/te_macros:242-248`

developer.android.com and Google Play:

- https://developer.android.com/about/versions/10/behavior-changes-10
- https://developer.android.com/about/versions/14/behavior-changes-14
- https://developer.android.com/google/play/requirements/target-sdk
- https://developer.android.com/guide/practices/page-sizes
- https://developer.android.com/privacy-and-security/direct-boot
- https://developer.android.com/guide/topics/manifest/application-element
- https://developer.android.com/guide/topics/manifest/service-element
- https://developer.android.com/ndk/reference/group/memory
- https://support.google.com/googleplay/android-developer/answer/9888379
