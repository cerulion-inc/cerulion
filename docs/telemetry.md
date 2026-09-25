# Usage telemetry

The `cerulion` CLI and the `cerulion-vizd` visualization daemon can send
coarse usage events, so we can see which parts of Cerulion are used and where
they fail. This page lists everything that is sent, when, and how to turn it
off.

## When anything is sent

Only a release build sends events. The release artifacts (the install
script, the Debian package and the Homebrew formula) carry a telemetry key, baked in when
they are built. A build from source (`cargo install`, `cargo build`) has no
key and sends nothing, whatever the settings below say, unless you supply a
key yourself through `POSTHOG_API_KEY`, which also takes precedence over a
baked key.

Robot and runtime code never sends anything. The graph runtime, the
transport, the recorder, the network daemons and every node run without this
code linked in, and a CI check fails if one of them ever depends on it.

## What is sent

Every event carries:

| Property | Example | Meaning |
|---|---|---|
| `surface` | `cli` | `cli` or `vizd` |
| `env` | `prod` | `prod` for a release build, `dev` for a debug build |
| `app_version` | `0.4.0` | the binary's version |
| an id | see below | who the event belongs to |

The events:

| Event | Sent by | Properties |
|---|---|---|
| `cli_command_run` | every CLI command | `verb` and `subverb` (the command's name, such as `graph` and `run`), `exit_code`, and `duration_bucket` (`lt_1s`, `1s_10s`, `10s_1m`, `1m_10m`, `gte_10m`), and `install_method` (`install.sh`, `deb` or `brew`, read from the marker file the installer left beside the binary, where a package's marker wins over one left by `install.sh`; absent for a source build) |
| `cli_login_completed` | `cerulion login`, and the login a command starts on a machine that never signed in | `is_account_switch` (whether a different account was signed in before) |
| `graph_run_started` | `cerulion graph run` and `cerulion ros2 attach`, when a run is requested inside a workspace (before the graph is loaded and checked, so a run rejected there records one too; `graph_run_completed` then has `is_success` false) | `is_single_process` |
| `graph_run_completed` | `cerulion graph run` and `cerulion ros2 attach`, when the run ends | `duration_bucket`, `is_success` |
| `node_build_completed` | `cerulion node build` | `duration_bucket`, `is_success`, `is_release` |
| `bag_record_completed` | `cerulion bag record`, on success | `duration_bucket`, `size_bucket` (the bag files' size on disk: `lt_1mb`, `1mb_10mb`, `10mb_100mb`, `100mb_1gb`, `gte_1gb`), `topic_count` (topics that recorded a message) |
| `bag_record_failed` | `cerulion bag record`, on failure | `duration_bucket` |
| `bag_replay_completed` | `cerulion bag play` | `duration_bucket`, `is_success` |
| `resim_completed` | `cerulion bag play --resim` | `duration_bucket`, `exit_code`, `is_divergent` |
| `ros2_bridge_started` | `cerulion ros2 attach`, when it hands its bridge graph to `graph run` (before that graph is checked; the `graph_run_started` and `graph_run_completed` pair that follows reports whether it ran) | none |
| `connect_session_completed` | `cerulion connect` | `duration_bucket`, `exit_code` |
| `pair_completed` | `cerulion pair` | `is_success` |
| `vizd_started` | the vizd daemon, once at start | `os`, `arch` |
| `vizd_heartbeat` | the vizd daemon, every 15 minutes | `uptime_minutes` |

Never sent: command arguments, file or directory names, paths, topic, node,
graph or robot names, URLs, email addresses, message contents, or anything
you typed. Every property is checked before it is queued: a value that looks
like a URL, an email address or a path, or is longer than 128 characters, is
dropped instead of sent.

The id is your Cerulion account id once this machine has signed in, and a
random `anon:<uuid>` before that. The random id lives in the consent file.

When a machine that has never signed in runs its first login, the login
request carries the random id so the events from before the login are
joined to the account. Nothing else is added to the login. If that first
login happens in the run that printed the notice, which sends nothing, an
empty `telemetry_alias_pending` file next to the consent file marks the join
as owed, and the next run that sends makes it and deletes the file. Only a
hosted account id is joined this way; the events of any other account stay
under the random id. When a different
account signs in on the same machine, the random id is replaced, so later
anonymous events are never joined to the previous account.

These commands record no event at all: `cerulion telemetry`, `cerulion
completions`, and the internal subprocesses a command starts for itself.

## The first-run notice

The first command that could send prints a short notice on stderr and sends
nothing. Later commands send. The notice is shown once per machine.

## Turning it off

Any one of these turns telemetry off:

```bash
cerulion telemetry off     # persists for this machine
export DO_NOT_TRACK=1      # any tool that honours the convention
export CERULION_TELEMETRY=0
```

`cerulion telemetry status` prints the current decision and which setting
made it; `cerulion telemetry on` turns it back on. The settings are checked
in this order, and the first one that decides wins:

1. `DO_NOT_TRACK=1` (or `true`) turns telemetry off.
2. `CERULION_TELEMETRY=0` turns it off, `CERULION_TELEMETRY=1` turns it on.
3. The consent file, written by `cerulion telemetry on|off`.
4. Otherwise telemetry is on (in a release build).

A running vizd daemon checks again before each heartbeat, so turning
telemetry off stops its heartbeats without a restart.

## The consent file

`${CERULION_HOME:-~/.cerulion}/telemetry.json`, readable only by you:

```json
{"enabled": true, "anon_id": "anon:<uuid>", "notice_shown": true,
 "updated_at": "2026-09-09T21:00:00.000Z"}
```

Deleting it resets the notice and the random id. A consent file that exists
but cannot be read counts as off.

## Where events go

Events are sent to PostHog over HTTPS in one small batch when a command
exits. Sending never delays a command by more than 300 ms: if the network is
slow or down, the events are dropped, never written to disk and retried.
