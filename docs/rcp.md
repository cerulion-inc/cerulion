# Robot Context Protocol (`cerulion-rcpd`)

`cerulion-rcpd` is an MCP server that lets an agent read the live state of a
robot running Cerulion and call the tools its manifest declares. It identifies
itself as `cerulion-rcp`. Every tool call is written to a hash-chained receipt
log, a mutating call before it reaches the robot. Every call that passes the
checks is published into the graph as a frame on a topic, so a recorded run
holds every call that reached the robot; refused calls are in the receipt log
only.
[`examples/aspire_arm`](../examples/aspire_arm/README.md) is a simulated arm an
agent drives this way, with a recording that replays with no agent. Its demo
script expects an [ASPIRE](https://github.com/NVlabs/ASPIRE) embodiment that is
not yet published; the plant and the replay run from this repository alone.

Runs started with `--local`, `--network off` or `CERULION_NETWORK=off` keep their graph, state and tool journal local. Remote requests are refused before reaching those topics; configured external eye requests also respect this scope. Cached private state stays local after its producer exits. Local observation and tool calls remain available. Emergency-stop operations remain available independently of network scope.

## Run it

```bash
cerulion-rcpd --manifest rcp.yaml            # stdio (the default)
cerulion-rcpd --manifest rcp.yaml --stdio
CERULION_RCP_TOKEN=... cerulion-rcpd --manifest rcp.yaml --http   # 127.0.0.1:7801
CERULION_RCP_TOKEN=... cerulion-rcpd --manifest rcp.yaml --http 127.0.0.1:7900
cerulion-rcpd --manifest rcp.yaml --receipts /var/log/rcp/receipts.jsonl
cerulion-rcpd --verify-receipts /var/log/rcp/receipts.jsonl   # 0: chain intact, 1: missing, empty or broken
cerulion-rcpd --verify-bag run.mcap --manifest rcp.yaml       # 0: every Jev answer has its receipt
cerulion-rcpd jev status --manifest rcp.yaml                  # also: jev on, jev off
```

`--receipts` defaults to `.cerulion/rcp/receipts.jsonl` beside the manifest.
One daemon owns a receipt log at a time: a second daemon pointed at the same
log refuses to start.

In stdio mode stdout carries only the MCP stream; every log line goes to
stderr (`RUST_LOG` sets the level, default `info`).

Streamable HTTP is served at `/mcp`. Every request must carry
`Authorization: Bearer <token>` matching `CERULION_RCP_TOKEN`, or it is
answered `401`. Without the variable `--http` refuses to start. The default
address is loopback; binding any other address logs a warning, and only
`localhost`, `127.0.0.1` and `::1` are accepted as the `Host` header. A
request may carry `x-cerulion-caller: <caller>` (1 to 200 characters from
`[A-Za-z0-9:._@/-]`); a tool call it carries is receipted under that caller
instead of `rcp-agent`, and a repeated or malformed value fails the call.

## The manifest

```yaml
robot: go2              # optional
graph: graphs/go2.yaml  # optional, relative to this file
journal:                # optional; these are the defaults
  calls_topic: /rcp/calls
  results_topic: /rcp/results
state:                  # optional
  - name: arm
    topic: /arm/state
limits:                 # optional; these are the defaults
  estop_clear: agent    # agent | operator
  irreversible: allow   # allow | refuse
tools:                  # optional
  - name: arm.step
    description: Advance the arm one control period toward a joint target.
    safety_class: reversible    # read | reversible | irreversible
    timeout_ms: 2000            # optional, 1..=60000, default 2000
    input_schema:
      type: object
      properties:
        q_target: { type: array, items: { type: number }, minItems: 6, maxItems: 6 }
        gripper: { type: number, minimum: 0, maximum: 1 }
      required: [q_target, gripper]
```

`robot` names the robot in every document. Without it the graph's `prefix`
is used, and without a graph the host name. `graph` is the graph YAML whose
nodes `robot://graph` lists. Unknown keys are refused.

`input_schema` takes a subset of JSON Schema: an object root with
`properties`, `required` and `additionalProperties` (default `false`), and
properties of type `number` or `integer` (`minimum`, `maximum`), `string`
(`maxLength`), `boolean`, or an `array` of one of those (`minItems`,
`maxItems`); any of them may carry a `description`. A keyword outside that
list, or one used on a type it does not apply to, is refused when the manifest
loads, so a schema never states a constraint the daemon skips. An `integer` is
any number without a fractional part. Tool and state names are 1 to 128
characters from `[A-Za-z0-9_.-]`, and the calls, results and state topics must
all differ.

`limits` holds two switches:

| Key | Values | Effect |
|---|---|---|
| `estop_clear` | `agent` (default), `operator` | `agent`: the built-in `estop_clear` tool is listed and clears the e-stop. `operator`: `estop_clear` is not listed, a call to it is refused (`estop_clear_disabled`), and only deleting the latch file and restarting the daemon clears the e-stop. |
| `irreversible` | `allow` (default), `refuse` | `allow`: a tool whose `safety_class` is `irreversible` is served. `refuse`: the manifest fails to load, naming the first such tool, and the daemon exits with status 1. |

## Safety classes

| `safety_class` | Intent receipt | While e-stopped | `tools/list` annotations |
|---|---|---|---|
| `read` | no | runs | `readOnlyHint: true`, `destructiveHint: false` |
| `reversible` | yes | refused | `readOnlyHint: false`, `destructiveHint: false` |
| `irreversible` | yes | refused | `readOnlyHint: false`, `destructiveHint: true` |

Every manifest tool also carries `openWorldHint: false`, and its listed
description ends with `(safety class: <class>)`. Every result of a manifest
tool call after the name lookup, success or refusal, carries
`"safety_class"`. `estop` and `estop_clear` are listed with
`idempotentHint: true` and `openWorldHint: false`; `estop_clear` also
carries `destructiveHint: true`. Receipt lines
record the tool name as `verb`; the class of a verb is the one its manifest
declares.

## Tools

`tools/list` returns the manifest's tools, the built-in `estop`, and, under
`limits.estop_clear: agent`, the built-in `estop_clear`. Nothing else can be
called. A call runs these steps in order:

1. A name the manifest does not declare is refused (`not_in_manifest`).
2. The arguments are checked against `input_schema` (`invalid_args`).
3. A call whose frame would exceed 63 KiB is refused (`args_too_large`).
4. While the e-stop is engaged a `reversible` or `irreversible` tool is
   refused (`estopped`). `read` tools skip this step.
5. A `reversible` or `irreversible` tool appends an `intent` receipt and
   syncs it to disk.
6. The call is published on `journal.calls_topic` as one `std_msgs/String`
   frame, `{"call_id": N, "tool": "...", "args": {...}}`, and the daemon waits
   up to `timeout_ms` for a frame on `journal.results_topic` whose JSON carries
   the same `call_id`. Every frame on the results topic is read in order;
   frames that answer no pending call are dropped. On `timeout` the error
   carries `"may_have_executed": true`: the call was published and the graph
   may still act on it.
7. An `ok` or `error:<class>` receipt closes the call.

A successful call returns one text block,
`{"call_id": N, "receipt": {"seq", "hash", "prev"}, "result": <the graph's answer>, "safety_class": "..."}`.
A refused call returns `isError: true` with
`{"error": {"code", "message"}, "receipt": ...}`. Calls other than `estop`
are handled one at a time, in the order they arrive.

## E-stop

`estop` takes an optional `reason` (cut to 200 characters). It closes the
latch before anything else, without waiting for a call in flight; publishes
its own frame ahead of anything queued; and returns `estopped: true` with
`graph_acknowledged` saying whether the graph answered. While the latch is
closed, `reversible` and `irreversible` calls are refused, including one
already accepted but not yet published. A `reversible` or `irreversible`
call whose MCP request is cancelled before it is published is never
published. The latch is also written to `<receipts>.estop`, and a daemon
that starts and finds that file starts with the latch closed (so does one
that cannot tell whether the file exists).

`estop_clear` takes the same optional `reason` (at most 200 characters; any
other argument is refused as `invalid_args`). It waits behind a call in
flight, then:

1. refuses with `estop_overtaken`, receipted `error:estop_overtaken` with
   nothing changed, when an `estop` was called after the clear was
   requested; otherwise appends an `intent` receipt and syncs it to disk;
2. deletes `<receipts>.estop` and opens the latch (a file that cannot be
   deleted leaves the latch closed and returns `clear_failed`, receipted
   `error:clear_failed`);
3. when the latch was closed, publishes
   `{"call_id": N, "tool": "estop_clear", "args": {"reason": ...}}` on the
   calls topic, latch-gated like a motion call, and waits up to 2 s for the
   graph's answer; when it was open, publishes nothing;
4. appends `ok`; `error:graph_unacknowledged` when no answer came; or
   `error:estop_overtaken` when an `estop` closed the latch after step 2 and
   before the frame was published, which returns the `estop_overtaken` error and leaves the
   `estop` frame as the graph's last.

It returns `estopped` (the latch after the clear), `was_estopped`, and
`graph_acknowledged`; the latch stays open whether or not the graph
answered. A later `estop` closes the latch and writes the file again.
`estop` closes the latch, then, inside a lock, closes it again, writes the
file and its `intent` receipt, and queues its frame ahead of everything else,
so a clear that follows it publishes after that frame; `estop_clear` writes
its `intent`, deletes the
file and opens the latch inside the same lock. The receipt log therefore
orders engages and clears as they took effect. An `estop` that lands while
a clear holds that lock closes the latch at once and publishes its frame
after the clear's receipt write. When `estop` cannot write the latch file or
its `intent` receipt, it logs the error after its frame is queued and the
lock is released, so writing that error line does not hold the frame. A
receipt `estop_clear` cannot write is logged after it releases the lock,
so that line does not hold an `estop` waiting for the lock. The error line
for a receipt that could not be written, from any tool call or eye answer,
is also written after the receipt log's lock is released; an `estop` takes
that lock before it queues its frame.

A latch-gated call (a motion call or `estop_clear`) whose MCP request was
already cancelled when the journal takes it from its queue is dropped, not
published. A
cancelled `estop_clear` that already opened the latch leaves it open without
an `estop_clear` frame on the calls topic.
Deleting the file and restarting the daemon clears the e-stop under either
`limits.estop_clear` value.

## Receipts

Each receipt line records `seq`, `timestamp_ns`, `caller`, `verb`, the
SHA-256 of the canonical arguments, the `outcome`, `prev_hash` and
`entry_hash`. Changing, inserting, removing or reordering a line breaks the
chain, and so does cutting lines from the head or deleting a rotated file
between others; `--verify-receipts` reports each, reading the rotated files
(`<receipts>.1`, `.2`, ...) too. Lines cut from the END of the newest
file leave a shorter valid chain; compare the last hash it prints with a copy
kept elsewhere to detect that. The log rotates at 8 MiB and keeps four
rotated files.

The graph consumes the calls topic as an external input, so
`cerulion graph run --record` records every call, and
`cerulion bag play --resim all --verify` re-injects them and compares the
graph's answers with the recorded ones byte for byte.

## Resources

### `robot://graph`

One JSON document (`application/json`), refreshed at most once per second
and cached between refreshes:

| Field | Meaning |
|---|---|
| `version` | The document version, `1`. |
| `robot` | The robot identity. |
| `refreshed_at_ns` | The transport clock reading at the last refresh. |
| `runs.completeness` | `{"kind": "settled"}` when every live run answered, `incomplete` (with `live_writers` and `writers_heard`) when some did not, `unavailable` (with `error`) when the lookup failed. Only `settled` means the list is the whole answer. |
| `runs.records[]` | `run_id`, `graph_name`, `state` (`live` or `ending`), `supervisor_pid`, `run_started_at_ns`. |
| `nodes[]` | `node_id` and `node_type` from the graph YAML. `counters` is always `null` in this version. |
| `catalog` | The live topic catalog, in the same shape as the robot's LAN topic catalog: per topic its `producer_count` and `liveness`, whose `rate_estimate` carries an integral `millihertz` and `is_floor`. |
| `topics_error` | `null`, or the reason the topic listing failed; `catalog` is then empty and says nothing about which topics exist. |
| `monitors[]` | The topic monitor's row per topic: `state`, raised `conditions` (`stalled`, `silent`, `rate_deviation`), `baseline_mhz`, `observed_mhz` and `samples`. A topic is not judged during its first 5 seconds. |

Clients may subscribe to `robot://graph` (with `resources/subscribe` or
`subscriptions/listen`) and receive `notifications/resources/updated` after
each refresh.

### `robot://state/{name}`

One per `state` entry: the newest frame seen on that `std_msgs/String` topic,
as text. Reading one before any frame has arrived is an error.

## Jev, the gate's second eye

With an `eyes.jev` section, the daemon answers the graph's requests for a
second look at a wrist-camera frame with Jev, TypeSafe's System One model,
through OpenRouter by default. Jev reads; the graph's gate decides. Jev is a
cross-check that can only veto: a reading that disagrees with the gate's own
measurement refuses, a reading that agrees never passes a gate by itself,
and an absent Jev is never agreement.

When Jev is on, each look sends one still frame from the wrist camera, the
question set's text and the ask's numbers to the route: by default
OpenRouter, which passes them to TypeSafe in the US. No robot or user id is
in the request; the route sees the robot's network address and its key.
Nothing else from any camera leaves the robot for Jev, and a recording of
the frames and answers stays on the robot. TypeSafe lists where it
processes data at https://trust.typesafe.ai/subprocessors.

```yaml
eyes:
  jev:
    base_url: https://openrouter.ai/api/v1  # default; requests go to {base_url}/systemone
    model: typesafe/jev-1.14                # default; a pinned name, never an alias
    credential_file: keys/jev.key           # optional, relative to this file
    region: US                              # optional; picks the data notice
    frame_topic: /wrist/image               # sensor_msgs/CompressedImage, JPEG
    ask_topic: /rcp/eye/ask                 # default
    answer_topic: /rcp/eye/answer           # default
    timeout_ms: 1500                        # default and maximum
    question_sets:
      - name: press_gate
        state: Wrist camera frame from an arm in front of a coffee machine.
        numbers: [jaw_gap_mm]               # optional: the numbers an ask may carry
        questions:
          edge_visible: { type: noul, instructions: The rim's lower edge is visible in `frame`. }
          rim: { type: box, instructions: The chrome rim band in `frame`. }
        visible: { question: edge_visible, threshold: 0.7 }   # optional
```

A node asks with one `std_msgs/String` frame on the ask topic:
`{"ask_id": 7, "question_set": "press_gate", "frame": {"topic": "/wrist/image", "seq": 412}, "deadline_ms": 1500, "numbers": {"jaw_gap_mm": 3.5}}`.
`seq` is the wire sequence number of a frame on the frame topic; the daemon
keeps the newest 8 and waits up to 100 ms for one not yet seen. Each
publisher numbers its own frames, so an ask whose `seq` held frames from two
publishers carry (two cameras on the topic, or one that restarted) is
refused (`frame_ambiguous`) and neither frame is sent. The deadline
runs from the ask frame's wire timestamp when that stamp is at most 250 ms
behind the host's monotonic clock, and from when the daemon reads the ask
otherwise, so a run resumed after a pause, a graph on a virtual or external
time source and an ask mirrored from another machine keep their whole
deadline. It sends one
request carrying that frame unchanged, the set's text and the numbers, and
publishes one answer frame per ask on the answer topic with an `outcome`:
`ok`, or one of `disabled`, `no_credential`, `unavailable`, `timeout`,
`rate_limited`, `quota`, `throttled`, `refused`, `wrong_model`, `malformed`,
`not_visible` and `not_recorded`, each meaning Jev is absent. A reply the
daemon takes up at or after its deadline is receipted `late` and never
published, and the ask is answered `timeout`. The frame records
the questions, the state, the model asked and the model that answered, and
the SHA-256 of the image and of the request body; never the image bytes or
the key. Every answer frame carries `"provenance": "third_party"`, so a
training or labelling export can leave Jev's output out. A receipt
(`eye.jev`, caller `rcp-eye`) is written before each answer is published,
and an answer whose receipt cannot be written is not published.

A gate picks the answer frames for an ask by its `ask_id`, reads each with
`EyeAnswer::parse(answer, ask)`, passing the exact text of the ask frame it
published, and compares Jev's reading with its own through `Look::compare`,
which gives `Agree`, `Disagree` or `Absent(outcome)`. An answer frame whose
`ask_sha256` is not the SHA-256 of that text was given for another ask, such
as another frame under a reused `ask_id`, and reads as `not_recorded`.
A gate that holds the frame passes its JPEG bytes to
`EyeAnswer::for_image`: an `ok` answer whose `image_sha256` is not their
SHA-256 reads as `not_recorded`. `bag play` re-publishes a recording's
answer frames byte for byte, and ask ids and frame sequence numbers restart
with each run, so a recorded answer can carry a live gate's exact ask text
while naming another frame.

The daemon answers at most 20 asks at once and 10 a second after that,
whatever the outcome; an ask over that bound gets no answer and no receipt,
so the asking node's own deadline makes Jev absent; the first drop is
logged, and later ones are counted, in the log and in `jev status`.
The eye's receipts share the receipt log, and its lock, with the tools and
the e-stop.

Before any request the daemon refuses an ask that is not one of these shapes
(`malformed`), an image that is not a JPEG of at most 640 x 480 pixels and
200 KB with sides of at least 280 pixels (`refused`), a request over 4,000
estimated input tokens (`refused`), more than 2 requests in flight or more
than 5 looks a second after a burst of 10 (`throttled`), every request for
the `Retry-After` of a 429, at most 30 s, or 1 s when the header is missing
or gives a date (`rate_limited`), and, once the route reports a
spend limit reached, every request until 00:00 UTC (`quota`, logged once).

The key is read before every look from `CERULION_JEV_KEY_FILE`, else
`credential_file`, else `jev.key` beside the receipt log. The file holds one
per-robot key and must not be readable by group or others. Jev is on while a
key is on file and the switch is not off:

- `cerulion-rcpd jev off` stops requests with the next ask (`disabled`);
  `jev on` turns it back on; `jev status` prints on or off, the route, the
  model, today's asks, looks and asks dropped unanswered (UTC) and the last
  outcome. A restarted daemon goes on counting the day the status file
  records.
- The first time Jev is on, at start or before the first request of a run
  (a key placed while the daemon runs), the daemon logs one line saying that
  wrist frames leave the robot and how to turn Jev off. A region whose law
  requires more gets the line at every run (`clear`), or Jev off until
  `jev on` (`opt_in`).
- The ask and frame topics belong to the graph, which sets their depth when
  it creates them. The daemon opens each once it exists, trying every
  10 ms, and never creates it, so it can start before or after the graph.
  It creates the answer topic.
- An eye that cannot start (its answer topic cannot be created, or a topic
  name is not valid) is logged and the daemon serves its tools without it;
  an HTTPS client that cannot be built (no system root certificates)
  answers every look `unavailable`.

The answer topic has no producer inside the graph, so
`cerulion bag play --resim all` re-injects the recorded answer frames at
their recorded steps, whatever the re-executed graph asks, and makes no Jev
request itself. A gate that reads them with its own ask text sees an answer
given for another ask text as `not_recorded`, and `--resim all --verify`
reports a changed ask as a divergence on the ask topic. A running daemon's
eye reads the same topics, and tells a played recording by where its frames
come from: `bag play` publishes from the transport node `cerulion_bag_play`
and `bag play --resim` from `cerulion_replay_` and the recording's identity.
An ask from either gets no request, no answer and no receipt, with Jev on
or off, and a wrist frame from either is never held, whatever their wire
timestamps say, so an unpaced resim whose replay clock passes the host's
time is dropped as surely as a recording from another boot. So are an ask
and a frame whose publisher's node cannot be named. The first such ask is
logged, later ones are counted, and `jev status` reports them. A live graph
whose own node name takes one of those forms (a graph named `bag_play`, one
whose name begins `replay_`, or a multi-process graph named `replay`) reads
as a replay, and the log names the node.
`--verify-bag` checks that every answer in a recording has its receipt,
names an ask the recording holds and carries the SHA-256 of that ask's
text, and, where the recording holds the frame, that the image and request
digests recompute from it and the recorded model, state and questions; of
several recorded frames with that number, it uses the one whose SHA-256 the
answer records. Every `late` receipt for a request an answer in the
recording names must follow that request's `timeout` answer. The receipt
log is shared by every run, so a `late` receipt for a request no answer in
the recording names is another recording's and is left out.

## Reaching rcpd from a desk

`cerulion-remoted` serves the robot's rcpd to desks on its access list on the
`cerulion/rcp/1` plane. Run rcpd with `--http` on its default loopback address
and give it the token `cerulion-remoted` uses. At startup `cerulion-remoted`
creates `<state root>/remoted/rcp.token` when it is absent: one
`CERULION_RCP_TOKEN=<random token>` line, mode `0600`, owned by the user
`cerulion-remoted` runs as. When the file exists but is not a regular file,
or gives its group or others any permission (read, write or execute),
`cerulion-remoted` logs that at
error and serves on: its ops and wire planes work, and every rcp request is
answered with an error until the file is private (`chmod 600`). The file's
format is a systemd environment file, so an rcpd systemd unit can read it
with `EnvironmentFile=`, for example:

```ini
[Service]
EnvironmentFile=/var/lib/cerulion/remoted/rcp.token
ExecStart=/usr/bin/cerulion-rcpd --manifest /etc/cerulion/rcp.yaml --http
```

On the desk, `cerulion rcp <robot>` serves the robot's rcpd as a local stdio
MCP server. While the desk's network kill-switch is engaged
(`CERULION_NETWORK=off` or `--network off`) it is refused with exit 2 before
anything reaches the robot, and without a sign-in. The robot admits only a
desk on its access list and checks every message: `resources/*`,
`subscriptions/listen` and `tools/list` need the observe capability;
`tools/call` needs the teleop capability and the operator role, except that
any desk on the access list may call `estop`. The robot checks an Observe or
Act message's access again just before it sends the message to rcpd; an
`estop` is not checked again. It sends a `notifications/cancelled` to rcpd
after rcpd has taken the tool call it names, or once `--rcp-head-timeout-ms`
passes. Tool calls made this way are receipted under the desk's identity,
`iroh:<device key hex>/account:<account hex>`, and the journaled call frame
carries the same value as `"caller"`. `cerulion-remoted --rcp-addr` (a
loopback address) and `--rcp-token-file` change the address and the file.
The plane's bounds and timers are flags too: `--rcp-sessions-per-connection`,
`--rcp-connections-per-key`, `--rcp-in-flight`, `--rcp-estops-in-flight`,
`--rcp-session-buffer-bytes`, `--rcp-revoke-check-ms`, `--rcp-head-timeout-ms`
and `--rcp-drain-ms` (`cerulion-remoted --help` gives each default).

### One e-stop for the ops and rcp planes

The ops e-stop that `cerulion-remoted` holds is one latch for the ops act
verbs, the ops verb `restart` and the rcp plane. An `estop` call from a desk engages it before the
call reaches rcpd, then rcpd engages its own latch as for any `estop`. The
ops plane's `engage-estop` engages the same latch, then sends an `estop` to
rcpd until rcpd answers, so rcpd engages its own latch too. While the ops latch is engaged:

- every other tool call from a desk except `estop_clear` is refused before it
  reaches rcpd, with error `-32001` and the message `refused: the robot's
  e-stop is engaged`;
- `estop_clear` is refused too, with `refused: the robot's ops e-stop is
  engaged; it clears when cerulion-remoted restarts`;
- the ops act verbs `topic-publish` and `service-call` are refused;
- the ops verb `restart` is refused (`unavailable: the robot's e-stop is
  engaged, so restart is refused`) before it stops anything. It reads the
  latch again after its stop command and before its start command, and an
  engage in between means the start command does not run (`unavailable:
  the robot's e-stop was engaged during the restart, so the graph was not
  started again after its stop`). An engage after that second read does
  not stop the graph the start command starts;
- `estop`, `tools/list`, the resources and the session messages still work.

LAN ingress does not read the latch.

The ops latch is read when a desk's message arrives and again on the open
connection to rcpd just before the plane sends it. A tool call already sent
to rcpd is stopped by rcpd's own latch, which closes on an `estop` (see
[E-stop](#e-stop)): rcpd refuses each mutating call it holds when the `estop`
reaches it and has not yet published, and the call it is running has already
reached the graph, which receives the e-stop frame next.

The `estop` that an ops `engage-estop` sends goes over the same loopback link
and token file, on an MCP session of its own, with the `reason` `the robot's
ops e-stop was engaged (engage-estop)`; rcpd receipts and journals it under
the engager's identity, `iroh:<device key hex>/account:<account hex>`, or
`iroh:<device key hex>` for a device bound to no account. `engage-estop`
answers without waiting for it, and the ops latch stays engaged whatever
happens to it. Each attempt waits for each response head for
`--rcp-head-timeout-ms` or 1 s, whichever is longer, and for each answer for
10 s after its head. The `estop` reaches rcpd once rcpd answers that its
latch closed: while rcpd is not reachable, the token file fails its check, or
rcpd answers with an error, `cerulion-remoted` sends it again, waiting 100 ms
after the first failure, twice as long after each further one and at most
2 s, on a new MCP session each time. It logs the first failure at warn with
its reason, and the 10th, 100th, 1000th failure and so on at warn with the
count, the others at debug, which a release build compiles out, and logs at
warn once rcpd's latch closes. One such delivery runs at a time, under the
identity of the first engager since rcpd last answered one: an `engage-estop`
while one runs starts none, since the one running keeps sending, and when that
`engage-estop` came after the attempt rcpd answers began, the delivery sends
the `estop` once more. A delivery that ends before rcpd answers logs at error;
one that ends after rcpd answered that its latch closed, while it deletes
its MCP session, logs nothing. These lines go through the daemon's log queue,
so a stderr that takes no more lines delays no attempt (see
[`docs/remote_plane.md`](remote_plane.md#e-stop-does-not-wait-behind-another-session)).
When `cerulion-remoted` stops gracefully, on Ctrl-C or SIGTERM, a delivery in
the middle of an attempt first deletes the MCP session rcpd issued it, within
2 s, inside the 5 s the stop waits for the delivery's task. A stop the
hard-exit deadline or a second signal ends, a SIGKILL (as a `cerulion-netd`
stop sends) or a crash may end it with no line and no `DELETE`.

The latch clears only when `cerulion-remoted` restarts. To recover, restart
`cerulion-remoted`, then clear rcpd's own latch: `estop_clear` under
`limits.estop_clear: agent`, or deleting `<receipts>.estop` and restarting
rcpd. While the ops latch is clear, `estop_clear` reaches rcpd like any other
tool call. Clients that talk to rcpd directly, over stdio or its own HTTP
listener, see only rcpd's latch.

When rcpd restarts, it forgets every MCP session. The next request on a desk's
session is answered with error `-32603` (`rcpd lost this MCP session (it
restarted); restart this MCP server`). The robot then sends nothing more to
rcpd on that session and reads it on until the MCP client ends it, or for
`--rcp-drain-ms`: an `estop` still engages the ops e-stop and is answered
`-32603` with a message that says it did not reach rcpd and the ops e-stop is
engaged, every other request gets the same session-lost error, and
notifications are dropped. Restarting the MCP server in the MCP client opens a
new session; when the robot ends the session first, `cerulion rcp` exits with
status 1.
