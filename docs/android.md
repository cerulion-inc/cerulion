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
root path is `/data/local/tmp/iceoryx2`. The runtime chooses the directory for
Cerulion named shared-memory objects in this order: an absolute directory
passed to `cerulion_core::posix_shm::set_shm_dir`, then a non-empty absolute
`CERULION_SHM_DIR`, then `$TMPDIR/cerulion-shm` when `TMPDIR` is absolute.
Neither `set_shm_dir` nor `CERULION_SHM_DIR` moves the iceoryx2 root path,
where iceoryx2 keeps its other files (node and service state, not the named
shared-memory objects); the root path moves through the `[global] root-path`
key in `config/iceoryx2.toml` under the process's working directory.
`set_shm_dir` must run before the first
named object is opened; calling it again with the resolved directory is a
no-op, and calling it with a different directory afterwards returns an error.
When nothing is configured, shell and root processes fall back to
`/data/local/tmp/cerulion-shm`. An app process cannot write there,
so the runtime next tries the app's own sandbox: the package name is read
from `/proc/self/cmdline` (any `:process` suffix stripped), and when
`/data/user/<user>/<package>` exists and belongs to the app, the directory
becomes `/data/user/<user>/<package>/cache/cerulion-shm`. An app whose
sandbox cannot be resolved gets an `EACCES` failure on the first
named-object operation and a single error log naming `set_shm_dir` and
`CERULION_SHM_DIR`; the embedder must call `set_shm_dir` (or export
`CERULION_SHM_DIR`) with an app-writable directory such as one under its
files directory.
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
shared lock, other processes cannot recover that lock.

## App integration

`cerulion_android` provides a Java facade backed by JNI. Build its native
library for each device ABI and place it in the app's `jniLibs` directory:

```sh
cargo ndk -t arm64-v8a --platform 30 \
  -o crates/cerulion_android/android/app/src/main/jniLibs \
  build --release -p cerulion_android
```

Use `-t x86_64` for an x86_64 device. The Android runtime writes
`files/iceoryx2.toml` with the default transport configuration before startup.

Android apps use the in-process transport. A `graph_yaml` containing native
nodes is refused with error code 5 because node libraries loaded at runtime
cannot share the in-process transport. Run the app without `graph_yaml`.
The app runtime runs the gateway and recorder alongside app-side publishing.

`init` creates `files/cerulion-shm` with mode 0700 and passes it to
`set_shm_dir`, so named shared-memory objects land in a directory the app
owns rather than the unusable `/data/local/tmp` fallback. Calling `init`
again with the same files directory is a no-op; a call that resolves a
different shared-memory directory after the first fails.

Initialize once per process, then start with JSON configuration. `start` and
`stop` return immediately; read `status` to observe their result. `publish`
returns an error code if the runtime is not ready or the message cannot be
encoded. When `start` returns `OK`, status has already transitioned to `starting`
and may have advanced to `running` before the next read. `publish` can return `BUSY`
while start or stop is in progress, including while the start worker is finishing
its lifecycle after status first reports `running`; retry a `BUSY` publish rather
than dropping it. A `stop` issued while startup is still in flight is accepted
and runs once startup finishes. Starting again after a failure or a finished
graph first stops what is left of the previous run and closes LAN access, even
when the new configuration is then rejected.

```java
int init = CerulionRuntime.init(getFilesDir().getAbsolutePath(), "android-app");
if (init != CerulionRuntime.OK) {
    Log.e(TAG, "init=" + init);
    return;
}
// The method wrapping this call is `startRuntime`; `startRetry` re-runs it,
// mirroring the sample activity's `Runnable startRetry = this::startRuntime`.
final Handler handler = new Handler(Looper.getMainLooper());
final Runnable startRetry = this::startRuntime;
int start = CerulionRuntime.start(configJson);
if (start == CerulionRuntime.BUSY) {
    // Another start or stop is in flight, so schedule the retry now: this
    // block runs again until start answers OK or the activity ends. A retry
    // answered BAD_STATE means a runtime is already running, owned by whoever
    // started it, and this caller must not stop it.
    handler.postDelayed(startRetry, 100);
} else if (start == CerulionRuntime.OK) {
    // Publishes during startup return an error, so poll status until the
    // state is "running" first (off the UI thread, or via a Handler).
    handler.post(new Runnable() {
        @Override
        public void run() {
            try {
                String status = CerulionRuntime.status();
                String state = new JSONObject(status).getString("state");
                if ("running".equals(state)) {
                    int publish = CerulionRuntime.publish(
                            "/app/heartbeat", "{\"data\":\"tick\"}");
                    if (publish == CerulionRuntime.BUSY) {
                        handler.postDelayed(this, 50);
                        return;
                    }
                    if (publish != CerulionRuntime.OK) {
                        Log.e(TAG, "publish=" + publish + " status=" + status);
                    }
                    handler.postDelayed(this, 500);
                } else if ("starting".equals(state)) {
                    handler.postDelayed(this, 50);
                } else if ("finished".equals(state) || "failed".equals(state)) {
                    // Stop to finalize the recording; retry until the stop is accepted.
                    Log.w(TAG, "runtime state=" + state);
                    int stop = CerulionRuntime.stop();
                    if (stop != CerulionRuntime.OK) {
                        Log.w(TAG, "stop=" + stop + ", retrying");
                        handler.postDelayed(this, 100);
                    }
                } else {
                    // stopped, idle or unknown: end the poll; restart it after a new start.
                    Log.w(TAG, "runtime state=" + state);
                }
            } catch (JSONException error) {
                Log.e(TAG, "heartbeat status parse failed", error);
            }
        }
    });
}
// In onDestroy, remove the pending start retry and the polls, then stop:
// handler.removeCallbacks(startRetry);
// handler.removeCallbacksAndMessages(null);
// CerulionRuntime.stop();
```

The JSON passed to `start` can look like this:

```json
{
  "listen": ["tcp/0.0.0.0:7683"],
  "record": {"file": "app.mcap"},
  "publish": [
    {
      "topic": "/app/heartbeat",
      "schema": "std_msgs/String",
      "max_payload_bytes": 4096
    }
  ]
}
```

The configuration keys are:

| Key | Meaning |
|---|---|
| `graph_yaml` | Optional graph YAML text. |
| `node_lib_dir` | Optional absolute path to packaged node libraries. Android refuses `graph_yaml` containing native nodes with error code 5. |
| `listen` | Gateway listen endpoints. Defaults to `[]`, which disables the gateway. |
| `record` | Optional recording object with `file` and optional `topics`. The file must be a plain `.mcap` name. An existing file is refused with error code 7, so each start needs a new name. |
| `publish` | Optional array of `{ "topic", "schema", "max_payload_bytes" }` entries. Defaults to `[]`. |
| `lan_access` | `"paired"` (the default) refuses every LAN peer; `"open"` serves any LAN peer, for local development. Any other value is rejected with error code 3. |
| `mdns` | Raise the `_cerulion._tcp` mDNS beacon while the gateway has a LAN-dialable nonzero fixed TCP listen port. Port-zero and loopback endpoints do not raise a beacon. Defaults to `true`. |

Omitting `record.topics` records all graph output and app-published topics.
Topics use canonical names with a leading `/`. Publish schemas must be known
built-in schemas or available under `files/schemas/`; duplicate publish topics
are rejected. `max_payload_bytes` defaults to 4096 and must be from 1 to
1048576. JSON publish values support `Bool`, `I8` through `U64`, `F32`, `F64`,
`String`, `Bytes`, and `StringFixed(n)`. `Bytes` uses an array of integers from
0 to 255. The top-level publish value must be an object; unknown fields and
unsupported field types are rejected. Missing fields use zero or empty values.
The JSON text is limited to `max_payload_bytes * 8 + 65536` bytes before it is
parsed. This allows for escaped strings, byte arrays, field names, and
whitespace while bounding the input size.

The stable return codes are `0 OK`, `1 NOT_INITIALIZED`,
`2 ALREADY_INITIALIZED`, `3 INVALID_ARGUMENT`, `4 BAD_STATE`, `5 GRAPH`,
`6 NETWORK`, `7 RECORDER`, `8 UNKNOWN_TOPIC`, `9 ENCODE`, `10 PUBLISH`,
`11 BUSY`, `12 PANIC`, and `13 SPAWN`.

`status` returns JSON with `state`, `robot`, `graph`, `nodes`, `listen`,
`lan_access`, `mdns_advertised`, `record_path`, `published`, `recorder`, and
`last_error`. State is `idle`, `starting`, `running`, `stopping`, `finished`,
`stopped`, or `failed`; `last_error` is null or an object with a stable code and
message. `finished` means a graph ended on its own after a node requested
shutdown. Call `stop` to finalize its recording, then start again. Before
initialization, the state is `uninitialized`. With recording disabled, a
snapshot can look like this:

```json
{
  "state": "running",
  "robot": "android-app",
  "graph": null,
  "nodes": 0,
  "listen": ["tcp/0.0.0.0:7683"],
  "lan_access": "paired",
  "mdns_advertised": true,
  "record_path": null,
  "published": {"/app/heartbeat": 2},
  "recorder": null,
  "last_error": null
}
```

The `recorder` summary is available after stop and includes message, channel,
byte, per-topic, and output-path totals. App files are private to the app:
recordings are stored in `files/bags/`, and custom schemas are read from
`files/schemas/`.

### Discovery and access

With a TCP listen endpoint and `mdns` enabled, the app registers a
`_cerulion._tcp` mDNS beacon for the device, so `cerulion topic list` on a desk on the same Wi-Fi network
lists it under `ROBOTS` without a typed address. A failed registration is
logged and leaves the device reachable by address; `mdns_advertised` in
`status` reports whether the beacon is up. The beacon is withdrawn when the
gateway stops or its thread fails. Port `0` and loopback hosts are not
LAN-dialable and do not raise a beacon. Android drops inbound multicast
unless an app holds a `WifiManager.MulticastLock`, which needs the
`android.permission.CHANGE_WIFI_MULTICAST_STATE` permission. The sample app
acquires the lock before `start` and releases it after `stop`:

```java
WifiManager wifi = (WifiManager) getApplicationContext()
        .getSystemService(Context.WIFI_SERVICE);
WifiManager.MulticastLock lock = wifi.createMulticastLock("cerulion-discovery");
lock.setReferenceCounted(false);
lock.acquire();
```

The gateway is not open by default. Under `"lan_access": "paired"` every LAN
request (topic demand, catalog and schema) is refused. The robot identity can
still appear in discovery, but topic announce tokens are withheld so the LAN
does not learn topic names; `topic list` warns that the robot refused to serve
its catalog. The posture stays in front of every installed demand authorizer.
Choose the posture at runtime startup with the app's `lan_access` key; opening
access after a paired start does not replay withheld announcements. A LAN peer
carries no paired identity, and an app does not host the paired remote plane (see
[Remote plane](remote_plane.md)), so under `paired` the app's topic data stays
on the device. For local development on
a trusted network, set `"lan_access": "open"`; the sample app honors the
`lan_access` launch intent extra only in debuggable builds:

```sh
adb shell am start -n com.cerulion.sample/.MainActivity --es lan_access open
adb forward tcp:7683 tcp:7683
cerulion topic list --connect tcp/127.0.0.1:7683
```

After stopping the runtime, launching the sample again with `am start` starts a
new runtime and reads the latest intent extras.

Release builds ignore the `lan_access` extra, log a warning, and keep the
paired default. A non-Unicode `CERULION_LAN_ACCESS` value also fails closed to
`paired`.
Stopping the runtime closes LAN access and revokes every registered bridge
grant, so demand workers cannot restore grants from an earlier open run.

The Android emulator's network sits behind NAT and does not pass multicast to
the host, so test discovery from the device itself or from a physical device
on the desk's network.

For a debuggable app, copy its recording to the host. `status` names the
written file in `record_path`; substitute its basename:

```sh
adb exec-out run-as com.cerulion.sample cat files/bags/<record_path basename> > recording.mcap
```

Listen endpoints and the robot name are fixed for the process lifetime by the
first start. A later start with a different robot name returns `BAD_STATE`.
A first start with an empty `listen` fixes the gateway as disabled for the
process; restart the app process to change either setting. App publish topics
must not collide with graph output topics because graph topics are single-writer;
a conflicting start returns `INVALID_ARGUMENT`. The gateway does not forward
incoming topics to the app. JSON publishing supports only the field types listed
above.

## Single-process apps

An Android app that owns all of its local publishers, subscribers, and gateway
in one process can opt into `local-transport`. It uses an in-process service,
so it does not require cross-process shared memory. The existing zero-copy
publish and subscribe path remains in use across threads in that process. A
gateway can still forward topics over TCP.

Do not enable this feature for the CLI or daemons, or for a graph that uses
`process_groups`. Explicit cross-process initialization, detached pre-creation
for other processes, and non-empty process groups are refused. A
`local-transport` host refuses every native cdylib node at load: each node
library statically links its own copy of `cerulion_core`, so its process-local
service registry is a different registry than the host's and its frames cannot
be delivered. Declare the node in-process or run a cross-process
host.

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
prefers those paths when the requested node's cdylib is present there; otherwise
it uses that node's library under `target/<profile>` when present. Missing
libraries are reported under cargo-ndk's target directory. Keep the target-triple
directory when copying the workspace to the device. Do not flatten its libraries
into `target/<profile>`.

Workspaces created by `cerulion workspace create` or `cerulion workspace init`
already carry the complete iceoryx2 family `[patch.crates-io]` block. A
workspace written by hand must add the complete block from Cerulion's root
`Cargo.toml`, with every entry pinned to the same fork revision. Without it,
the node cdylib can link an iceoryx2 build whose Android shared-memory calls
return `ENOSYS`. Node initialization refuses that library with:

```text
node library <name>: the binary links an iceoryx2 build without Android shared-memory support; build the workspace with the iceoryx2 fork [patch.crates-io] block (see docs/android.md)
```

The check runs in two places: the host-side loader probes the host's iceoryx2
before it calls the library's init symbol, and the generated `--raw-ffi`
template calls `cerulion_core::transport::refuse_unsupported_android_shm`
inside the library itself, because a cdylib statically links its own copy of
iceoryx2 and the host's probe proves nothing about it. A hand-written raw-FFI
library that does not use the template must make the same call in its init.

For an adb shell session, provide writable home and temporary directories:

```sh
export HOME=/data/local/tmp/cer/home
export TMPDIR=/data/local/tmp/cer/tmp
export CERULION_LOGIN_GATE=off
mkdir -p "$HOME" "$TMPDIR"
```

On Android, netd and a `graph run` gateway refuse every LAN peer unless
`CERULION_LAN_ACCESS=open` is set, the same posture as the app's
`lan_access`; they raise the same mDNS beacon. Set it in the shell that
starts them for local development:

```sh
export CERULION_LAN_ACCESS=open
```

When configured, netd and wsd use abstract-namespace control sockets on
Android. Their configured socket paths supply the abstract names and do not
create filesystem socket entries. Both ends check that the peer's `SO_PEERCRED`
UID matches their own effective UID; a different UID or unverifiable credentials
are refused. The pidfile and singleton lock remain filesystem-backed.

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
