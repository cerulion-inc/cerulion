# Robot Context Protocol (`cerulion-rcpd`)

`cerulion-rcpd` is an MCP server that lets an agent read the live state of a
robot running Cerulion. It identifies itself as `cerulion-rcp` and serves MCP
resources; it exposes no tools.

## Run it

```bash
cerulion-rcpd --manifest rcp.yaml            # stdio (the default)
cerulion-rcpd --manifest rcp.yaml --stdio
CERULION_RCP_TOKEN=... cerulion-rcpd --manifest rcp.yaml --http   # 127.0.0.1:7801
CERULION_RCP_TOKEN=... cerulion-rcpd --manifest rcp.yaml --http 127.0.0.1:7900
```

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
```

`robot` names the robot in every document. Without it the graph's `prefix`
is used, and without a graph the host name. `graph` is the graph YAML whose
nodes `robot://graph` lists. Unknown keys are refused.

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
