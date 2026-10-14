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

Initialize once per process, then start with JSON configuration. `start` and
`stop` return immediately; read `status` to observe their result. `publish`
returns an error code if the runtime is not ready or the message cannot be
encoded. A `stop` issued while startup is still in flight is accepted and
runs once startup finishes. Starting again after a failure first stops what
is left of the failed run.

```java
CerulionRuntime.init(getFilesDir().getAbsolutePath(), "android-app");
int start = CerulionRuntime.start(configJson);
if (start == CerulionRuntime.BUSY) {
    // Another start or stop is in flight, so retry (the sample activity
    // re-posts this block after 100 ms until it is OK or the activity ends).
    // A retry answered BAD_STATE means a runtime is already running, owned
    // by whoever started it, and this caller must not stop it.
} else if (start == CerulionRuntime.OK) {
    // Publishes during startup return an error, so poll status until the
    // state is "running" first (off the UI thread, or via a Handler).
    Handler handler = new Handler(Looper.getMainLooper());
    handler.post(new Runnable() {
        @Override
        public void run() {
            try {
                String state = new JSONObject(CerulionRuntime.status()).getString("state");
                if ("running".equals(state)) {
                    CerulionRuntime.publish("/app/heartbeat", "{\"data\":\"tick\"}");
                } else if ("failed".equals(state) || "stopped".equals(state)) {
                    Log.e(TAG, "runtime reached " + state + " before publish");
                } else {
                    handler.postDelayed(this, 50);
                }
            } catch (JSONException error) {
                Log.e(TAG, "status parse failed", error);
            }
        }
    });
}
// Later, for example in onDestroy: CerulionRuntime.stop();
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

The stable return codes are `0 OK`, `1 NOT_INITIALIZED`,
`2 ALREADY_INITIALIZED`, `3 INVALID_ARGUMENT`, `4 BAD_STATE`, `5 GRAPH`,
`6 NETWORK`, `7 RECORDER`, `8 UNKNOWN_TOPIC`, `9 ENCODE`, `10 PUBLISH`,
`11 BUSY`, and `12 PANIC`.

`status` returns JSON with `state`, `robot`, `graph`, `nodes`, `listen`,
`lan_access`, `mdns_advertised`, `record_path`, `published`, `recorder`, and
`last_error`. State is `idle`,
`starting`, `running`, `stopping`, `stopped`, or `failed`; `last_error` is null
or an object with a stable code and message. Before initialization, the state is
`uninitialized`. With recording disabled, a snapshot can look like this:

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

With a TCP listen endpoint, the app registers a `_cerulion._tcp` mDNS beacon
for the device, so `cerulion topic list` on a desk on the same Wi-Fi network
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
Use `NetworkManager::set_lan_access` before gateway startup; opening access later
does not replay withheld announcements. A LAN peer carries no paired
identity, and an app does not host the paired remote plane (see
[Remote plane](remote_plane.md)), so under `paired` the app's topic data stays
on the device. For local development on
a trusted network, set `"lan_access": "open"`; the sample app honors the
`lan_access` launch intent extra only in debuggable builds:

```sh
adb shell am start -n com.cerulion.sample/.MainActivity --es lan_access open
adb forward tcp:7683 tcp:7683
cerulion topic list --connect tcp/127.0.0.1:7683
```

Release builds ignore the `lan_access` extra, log a warning, and keep the
paired default. A non-Unicode `CERULION_LAN_ACCESS` value also fails closed to
`paired`.
Stopping the runtime disables every registered network bridge, so a later
paired restart cannot retain grants from an earlier open run.

The Android emulator's network sits behind NAT and does not pass multicast to
the host, so test discovery from the device itself or from a physical device
on the desk's network.

For a debuggable app, copy its recording to the host. `status` names the
written file in `record_path`; substitute its basename:

```sh
adb exec-out run-as com.cerulion.sample cat files/bags/<record_path basename> > recording.mcap
```

Listen endpoints are fixed for the process lifetime by the first start, and
a first start with an empty `listen` fixes the gateway as disabled for the
process; restart the app process to change them. The gateway does not
forward incoming topics to the app. JSON publishing supports only the field
types listed above.

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

On Android, netd and a `graph run` gateway refuse every LAN peer unless
`CERULION_LAN_ACCESS=open` is set, the same posture as the app's
`lan_access`; they raise the same mDNS beacon. Set it in the shell that
starts them for local development:

```sh
export CERULION_LAN_ACCESS=open
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
