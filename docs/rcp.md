# Robot Context Protocol (`cerulion-rcpd`)

`cerulion-rcpd` is an MCP server that lets an agent read the live state of a
robot running Cerulion and call the tools its manifest declares. It identifies
itself as `cerulion-rcp`. Every tool call is written to a hash-chained receipt
log, a mutating call before it reaches the robot. Every call that passes the
checks is published into the graph as a frame on a topic, so a recorded run
holds every call that reached the robot; refused calls are in the receipt log
only.
[`examples/aspire_arm`](../examples/aspire_arm/README.md) runs an
[ASPIRE](https://github.com/NVlabs/ASPIRE) skill program against a simulated arm
this way and replays the recording with no agent.

## Run it

```bash
cerulion-rcpd --manifest rcp.yaml            # stdio (the default)
cerulion-rcpd --manifest rcp.yaml --stdio
CERULION_RCP_TOKEN=... cerulion-rcpd --manifest rcp.yaml --http   # 127.0.0.1:7801
CERULION_RCP_TOKEN=... cerulion-rcpd --manifest rcp.yaml --http 127.0.0.1:7900
cerulion-rcpd --manifest rcp.yaml --receipts /var/log/rcp/receipts.jsonl
cerulion-rcpd --verify-receipts /var/log/rcp/receipts.jsonl   # 0: chain intact, 1: missing, empty or broken
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
`localhost`, `127.0.0.1` and `::1` are accepted as the `Host` header.

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
tools:                  # optional
  - name: arm.step
    description: Advance the arm one control period toward a joint target.
    safety_class: reversible    # read | reversible
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
all differ. A `safety_class` of
`irreversible` is refused too: such a tool needs a human's confirmation, which
this version does not implement.

## Tools

`tools/list` returns the manifest's tools and the built-in `estop`. Nothing
else can be called. A call runs these steps in order:

1. A name the manifest does not declare is refused (`not_in_manifest`).
2. The arguments are checked against `input_schema` (`invalid_args`).
3. A call whose frame would exceed 63 KiB is refused (`args_too_large`).
4. While the e-stop is engaged a `reversible` tool is refused (`estopped`).
   `read` tools skip this step.
5. A `reversible` tool appends an `intent` receipt and syncs it to disk.
6. The call is published on `journal.calls_topic` as one `std_msgs/String`
   frame, `{"call_id": N, "tool": "...", "args": {...}}`, and the daemon waits
   up to `timeout_ms` for a frame on `journal.results_topic` whose JSON carries
   the same `call_id`. Every frame on the results topic is read in order;
   frames that answer no pending call are dropped. On `timeout` the error
   carries `"may_have_executed": true`: the call was published and the graph
   may still act on it.
7. An `ok` or `error:<class>` receipt closes the call.

A successful call returns one text block,
`{"call_id": N, "receipt": {"seq", "hash", "prev"}, "result": <the graph's answer>}`.
A refused call returns `isError: true` with
`{"error": {"code", "message"}, "receipt": ...}`. Calls other than `estop`
are handled one at a time, in the order they arrive.

`estop` takes an optional `reason` (cut to 200 characters). It closes the
latch before anything else, without waiting for a call in flight; publishes
its own frame ahead of anything queued; and returns `estopped: true` with
`graph_acknowledged` saying whether the graph answered. While the latch is
closed, `reversible` calls are refused, including one already accepted but
not yet published. The latch is also written to `<receipts>.estop`, and a
daemon that starts and finds that file starts with the latch closed: an
operator clears the e-stop by deleting the file and restarting the daemon.

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
