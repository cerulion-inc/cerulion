# Usage telemetry

The `cerulion` CLI and the `cerulion-vizd` visualization daemon can send
coarse usage events, so we can see which parts of Cerulion are used and where
they fail. This page lists everything that is sent, when, and how to turn it
off.

## When anything is sent

Events are sent only by a build that has a telemetry key. The key is baked
into the release artifacts (the install script, the Debian package and the
Homebrew formula) when they are built, if the release pipeline provides one.
A build from source (`cargo install`, `cargo build`) has no baked key unless
`CERULION_POSTHOG_KEY` is set in its environment while it compiles. A build
with neither a baked key nor `POSTHOG_API_KEY` sends nothing whatever the
settings below say.
Any build also uses a key you supply yourself through `POSTHOG_API_KEY`,
which takes precedence over a baked key.
`cerulion telemetry status` says when the build has no key.

Robot and runtime code never sends anything. The graph runtime, the
transport, the recorder, the network daemons and every node run without this
code linked in, and a CI check fails if one of them ever depends on it.

## What is sent

Every event carries:

| Property | Example | Meaning |
|---|---|---|
| `surface` | `cli` | `cli` or `vizd` |
| `env` | `prod` | `prod` for a release build, `dev` for a debug build |
| `app_version` | `1.0.0` | the binary's version |
| `ci` | `false` | `true` when the run is under CI (see below) |
| `$lib` | `cerulion_telemetry` | the sending library, a PostHog field |
| `$lib_version` | `1.0.0` | that library's version |
| an id | see below | who the event belongs to (`distinct_id`) |

Each event also has a `uuid` (a time-ordered UUID v7) and the `timestamp`
it was queued at.
Every event sent under the random id, which includes every event from a CI
run, carries `$process_person_profile: false`, so PostHog creates no person
profile for it. Requests carry the header
`User-Agent: cerulion_telemetry/<version>`.

The events:

| Event | Sent by | Properties |
|---|---|---|
| `cli_command_run` | every CLI command | `verb` and `subverb` (the command's name, such as `graph` and `run`), `exit_code`, and `duration_bucket` (`lt_1s`, `1s_10s`, `10s_1m`, `1m_10m`, `gte_10m`), and `install_method` (`install.sh`, `deb` or `brew`, read from the marker file the installer left beside the binary, where the marker the last install wrote wins when both a package's and the script's are present, and the package's when both were written at the same instant; absent when no marker is beside the binary, as for a build from source, since the method is read from the directory the binary runs from, not from the binary) |
| `cli_login_completed` | `cerulion login`, and the login a command starts on a machine that never signed in | `is_account_switch` (whether a different account was signed in before) |
| `graph_run_started` | `cerulion graph run`, `cerulion node run` (which runs a one-node graph) and `cerulion ros2 attach`, when a run is requested inside a workspace (before the graph or node is loaded and checked, so a run rejected there records one too; `graph_run_completed` then has `is_success` false) | `is_single_process` (false for `node run`) |
| `graph_run_completed` | `cerulion graph run`, `cerulion node run` and `cerulion ros2 attach`, when the run ends | `duration_bucket`, `is_success` |
| `node_build_completed` | `cerulion node build`, when a build runs inside a workspace (a build requested outside one is refused before cargo starts and records only its `cli_command_run`) | `duration_bucket`, `is_success`, `is_release` |
| `bag_record_completed` | `cerulion bag record`, on success | `duration_bucket`, `size_bucket` (the size on disk of the finalized bag files whose size could be read: `lt_1mb`, `1mb_10mb`, `10mb_100mb`, `100mb_1gb`, `gte_1gb`), `topic_count` (topics that recorded a message) |
| `bag_record_failed` | `cerulion bag record`, on failure | `duration_bucket` |
| `bag_replay_completed` | `cerulion bag play` (plain playback, not `--resim`) | `duration_bucket`, `is_success` |
| `resim_completed` | `cerulion bag play --resim` | `duration_bucket`, `exit_code`, `is_divergent` |
| `ros2_bridge_started` | `cerulion ros2 attach`, when it hands its bridge graph to `graph run` (before that graph is checked; the `graph_run_started` and `graph_run_completed` pair that follows reports whether it ran) | none |
| `connect_session_completed` | `cerulion connect` | `duration_bucket`, `exit_code` |
| `pair_completed` | `cerulion pair` | `is_success` |
| `vizd_started` | the vizd daemon, once at start | `os`, `arch` |
| `vizd_heartbeat` | the vizd daemon, every 15 minutes | `uptime_minutes` |
| `$create_alias` | a first login's own run, or, when that run sent nothing, the next run outside CI that sends (see below) | `alias`: the random id being joined to the account |

The command's name (`verb` and `subverb` above) is the only part of a command
line that is sent. Never sent: command arguments, file or directory names,
paths, topic, node, graph or robot names, URLs, email addresses, or message
contents. Every property is checked before it is queued: a value that looks
like a URL, an email address or a path, or is longer than 128 characters, is
dropped instead of sent.

The id is your Cerulion account id once this machine has signed in, and a
random `anon:<uuid>` before that; a CI run always uses the random id. The
random id lives in the consent file.

When a machine that has never signed in runs its first login, the login
request carries the random id so the events from before the login are joined
to the account; the CLI also sends a `$create_alias` event for the join. A
CI run never joins: its login carries no id and sends no alias. Nothing else
is added to the login. A machine that signed out keeps its account on record
and carries nothing. If that first login happens in the run that printed the
notice, which sends nothing, an empty `telemetry_alias_pending` file next to
the consent file marks the join as owed, and the next run outside CI that
sends makes it and deletes the file. Only a hosted account id is joined this
way; the events of any other account stay under the random id. A
`telemetry_anon_account` file next to the consent file records the account
the random id was used for. It is created, empty, before a login carries the
id, and the account is written into it once the login completes. A login
that fails after that request keeps it: the account service may have joined
the id anyway, so the id is never carried again and is replaced when an
account next signs in. A run that sends while signed in records the account
too, CI included, and a run that sends nothing records nothing. A random id
with a record is never carried into a later login, even after the sign-in
state is removed. When a different account signs in on the same machine, the
random id is replaced, so later anonymous events are never joined to the
previous account.

These commands record no event at all: `cerulion telemetry`, `cerulion
completions`, `cerulion bagd` (the recorder daemon), and the internal
subprocesses a command starts for itself.

## The first-run notice

The first command that could send, run with stderr on a terminal, prints one
line on stderr, and the CLI sends nothing in that run:

```text
Cerulion sends usage telemetry; turn it off with `cerulion telemetry off` (details: `cerulion telemetry status`).
```

Later commands send. The notice is shown once per machine: it is saved as
shown only once it has been written, so a run that is killed while printing
it shows it again next time, and two first commands started at the same
moment may both show it (neither sends). A command whose stderr is not a
terminal prints no notice and sends as usual; the notice is then printed by
the first command that could send, run on a terminal. The vizd daemon prints
no notice of its own and sends nothing until the CLI has shown it on the
machine (or under CI), so a `cerulion viz` that prints the notice may start
a daemon that sends. `cerulion telemetry status` prints the link to this
page.

## CI runs

A run is under CI when one of `CI`, `GITHUB_ACTIONS`, `GITLAB_CI`,
`BUILDKITE`, `CIRCLECI`, `TRAVIS`, `JENKINS_URL`, `TEAMCITY_VERSION`,
`TF_BUILD`, `BITBUCKET_BUILD_NUMBER`, `CODEBUILD_BUILD_ID`, `DRONE` or
`APPVEYOR` is set and not empty; `CI` set to `false`, `0`, `no` or `off`
does not count. A CI run prints no notice, on a terminal or not, and still
sends. Its events carry `ci: true`, go out under the random id with no
person profile, and are never joined to an account, so CI traffic can be
filtered out of user counts. It leaves the notice to the first run on a
terminal outside CI. The settings below turn telemetry off in CI as anywhere else.

## Turning it off

Any one of these turns telemetry off:

```bash
cerulion telemetry off     # persists for this machine
export DO_NOT_TRACK=1      # any tool that honours the convention
export CERULION_TELEMETRY=0   # or false, off, no
```

`cerulion telemetry status` prints the current decision and which setting
made it; `cerulion telemetry on` turns it back on. The settings are checked
in this order, and the first one that decides wins:

1. `DO_NOT_TRACK=1` (or `true`) turns telemetry off.
2. `CERULION_TELEMETRY` set to `0`, `false`, `off` or `no` (either case)
   turns it off, and exactly `1` turns it on. Any other value is ignored.
3. The consent file, written by `cerulion telemetry on|off`.
4. Otherwise telemetry is on (a build without a key still sends nothing).

A CLI command that is already running checks again when it exits, so turning
telemetry off from another terminal stops its event too. A running vizd
daemon checks again before each heartbeat, so turning telemetry off stops its
heartbeats without a restart.

## The consent file

`${CERULION_HOME:-~/.cerulion}/telemetry.json`, readable only by you:

```json
{"enabled": true, "anon_id": "anon:<uuid>", "notice_shown": true,
 "updated_at": "2026-09-09T21:00:00.000Z"}
```

Deleting it resets the notice and the random id. A consent file that exists
but cannot be read counts as off.

## Where events go

Events are sent to PostHog's US cloud (`https://us.i.posthog.com`; any run
uses `POSTHOG_HOST` instead when it is set) over HTTPS. Each event is sent
as soon as it is queued, so a long command such as `cerulion graph run`
sends `graph_run_started` while it runs. Sending never delays a command's
exit by more than 300 ms: if the network is slow or down, the events are
dropped, never written to disk and retried.

PostHog records the IP address each request comes from (`$ip`) and, from
it, an approximate location stored with the event as `$geoip_*` properties:
continent, country, region, city, postal code, time zone, and a latitude and
longitude near the city. Events
are kept for PostHog's event retention period, from one to seven years
depending on the project's plan
(<https://posthog.com/docs/data/events-retention>).
