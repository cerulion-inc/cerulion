# Cerulion User API

**Ground-truth reference for the surface a Cerulion application author touches.** Anything not in this document is internal: the framework reserves the right to change it.

The user surface is intentionally narrow:

1. **The `cerulion` CLI**: workspace, node, graph, topic, schema, recording, visualization, remote access.
2. **Three macros**: `#[cerulion_node]` on the struct, `#[cerulion_node_impl]` on the impl block, and `#[derive(CerulionState)]` on a type your node holds.
3. **Field attributes**: `#[input(...)]`, `#[output(...)]` and `#[cerulion(...)]`.
4. **Graph YAML**: declarative wiring. Topology only; trigger policy lives on the node's macro.
5. **Schema YAML** (or ROS2 `.msg`): message type definitions.
6. **A handful of conventions inside `tick()`**: direct field access, `?` error propagation, `self.now_ns()`, `self.request_shutdown()`.
7. **Environment variables**: see [Environment variables](#environment-variables) for the complete set.

You do **not** import `cerulion_core::graph::*`, `cerulion_core::transport::*`, or instantiate `ClosureNodeEntry` / `GraphRuntime` / `TransportManager` directly. Those are framework internals; they can and do change between releases.

---

## Quick start

```bash
# 0. Sign in once per machine; later commands read the result locally.
cerulion login

# 1. Create a workspace
cerulion workspace create my_robot
cd my_robot

# 2. Create a node: produces nodes/sensor/ (Cargo.toml + src/lib.rs).
#    A source-only node (no -i / -T) MUST declare a non-data trigger policy.
#    Built-in ROS 2 types work with no setup; a workspace schema still needs
#    its Rust type wired by hand, so start with a built-in.
cerulion node create sensor --policy period_ms=100 -o geometry_msgs/Vector3 reading

# 3. Edit nodes/sensor/src/lib.rs to fill in the tick body.

# 4. Build the node
cerulion node build sensor

# 5. Create a graph and stage the node
cerulion graph create perception
cerulion node stage sensor -g perception

# 6. Run
cerulion graph run perception
```

### Automation and CI

Signing in is a once-per-machine step, not a per-command one: it writes state
under the Cerulion config directory, and every later command reads it locally,
with no network, offline, and after the session behind it has expired. A build
machine or a service that cannot open a browser therefore does not need its own
sign-in: provision it with the config directory of a machine that did sign in,
and point `CERULION_HOME` at that directory. The identity is the one that
directory already holds, so treat it as a credential; setting the variable does
not create one.

---

## CLI reference

`cerulion --help` prints the full tree. Every subcommand also responds to `--help`.

### `cerulion workspace`

| Command | Effect |
|---|---|
| `workspace create <NAME>` | New workspace under `./<NAME>/` (Cargo workspace + `graphs/`, `nodes/`, `schemas/`, `.cargo/config.toml`). |
| `workspace init [LOCATION]` | Initialize the current (or given) directory as a workspace in place. |

For a CLI built with a stable Rust release, both commands write
`rust-toolchain.toml` naming that release and the minimal profile, but only when
a rustup toolchain is already installed whose release and full commit hash match
the CLI's own. The check is offline and never installs a toolchain. A missing,
mismatching or hash-less compiler produces a warning instead, and the workspace
keeps whatever compiler your environment selects. A file that is written makes
rustup use that compiler for node builds inside this workspace without changing
the machine's default. An existing `rust-toolchain` or `rust-toolchain.toml`,
symlinks included, is preserved with a warning. An explicit `RUSTUP_TOOLCHAIN`
and a rustup directory override set at the workspace both take precedence over
the file, while an override set on a more distant ancestor does not; whatever
wins must still select a compatible compiler. A nightly or beta build warns
rather than inferring a toolchain from an undated release string. Existing
workspaces are not migrated.

Workspace scaffolding chooses dependencies based on where the `cerulion`
binary itself lives, not on the directory you run it from. A CLI installed
from a release archive, crates.io, Homebrew or apt scaffolds published crates
with exact pins matching its own version (`cerulion_core = "=X.Y.Z"` and
`native_ros2_messages = "=X.Y.Z"`). A CLI built inside a `cerulion`
checkout (`cargo install --path crates/cerulion_cli`, or run from its `target/`)
scaffolds absolute path dependencies into that checkout instead. The command
prints which one it chose on a `dependencies:` line under `Created workspace`.

To override the default, edit the two entries in the generated root
`Cargo.toml` under `[workspace.dependencies]`; member manifests inherit them
with `{ workspace = true }`. Use another published version or a local `path`
dependency as appropriate. Recreating the workspace rewrites this file; there is
no command that rewrites an existing workspace's manifest in place.

### `cerulion node`

| Command | Effect |
|---|---|
| `node create <TYPE> [-i SCHEMA NAME] [-T SCHEMA NAME] [-o SCHEMA NAME] [--policy SPEC] [--raw-ffi]` | New node crate at `nodes/<TYPE>/`, written from the `#[cerulion_node]` macro template. (`--raw-ffi` selects the legacy hand-written FFI template instead. It carries no panic safety or error propagation of its own, and gets none of the per-set Sync, held-input or node-logging behaviour this document describes. Use the macro template unless you need the raw FFI.) `-i` declares a regular input; `-T` declares a TRIGGER input (the new field becomes `#[input(trigger)]` and the node's policy becomes `data_trigger=<NAME>`). At most ONE `-i`, ONE `-o` and ONE `-T` per call (repeat the verb, or use `node modify`, to add more ports); both values of each flag are REQUIRED. Bare SCHEMA names resolve against workspace schemas and built-in ROS 2 messages; see `Port SCHEMA resolution` below. See `--policy` below for the SPEC grammar and the defaulting rules. |
| `node delete <TYPE>` | Remove `nodes/<TYPE>/` and its workspace member entry. |
| `node modify <TYPE> [-i SCHEMA NAME] [-T SCHEMA NAME] [-o SCHEMA NAME] [--policy SPEC]` | Add ports to an existing node and/or change its trigger policy. For macro-form nodes, splices a new `#[input]` / `#[output]` field into the struct directly; the surrounding tick body and any custom helper fields are preserved. For a legacy `--raw-ffi` node it regenerates the `INFO_BYTES` JSON instead. `-T SCHEMA NAME` adds a TRIGGER input (equivalent to `-i SCHEMA NAME --policy data_trigger=NAME`). Bare SCHEMA names resolve against workspace schemas and built-in ROS 2 messages; see `Port SCHEMA resolution` below. `--policy SPEC` rewrites the macro's trigger policy atomically; see `--policy` below. |
| `node build <TYPE> [--release]` | `cargo build` the node crate as a cdylib. Also probes the node's **optional SYSTEM dependencies** and enables each feature whose system library is actually present; see `Optional SYSTEM dependencies` below. The probe report is printed to stderr BEFORE cargo runs (so a build that fails because of an enabled feature still says which feature that was), and a malformed declaration REFUSES the build rather than silently producing a capability-less artifact. Cargo's own output is captured and shown only when the build fails, so one progress line (`Building '<TYPE>'. …`) goes to stderr just before cargo starts; stdout carries only the result line, `Built '<TYPE>'`. The first build of a workspace also compiles the Cerulion runtime and can take a few minutes. |
| `node stage <TYPE> [-i ID] [-g GRAPH] [-I NAME SOURCE]...` | Add the node as an instance in a graph YAML. Repeated `-I name source` wires inputs (`-I image camera/image`). A staged instance carries no prefix of its own: `prefix:` is a GRAPH-level key (set by `graph create -n PREFIX`), so topic resolution for every instance in the file comes from there. |
| `node run <TYPE> [-p PREFIX] [-i ID] [--release] [--network off] [--no-cpu-dma-lock] [--no-monitor-wait]` | Run a single node on its own, which is useful for development without a full graph. Runs through `graph run` (a one-node graph, report skipped), so an output whose `schema:` the workspace defines more than once is refused the same way. `-p` sets the topic prefix (default `standalone`) and `-i` the instance id (default: the type name), so an output lands on `/standalone/<TYPE>/<port>` unless you say otherwise. `--release`, `--network off`, `--no-cpu-dma-lock` and `--no-monitor-wait` mean what they mean on `graph run`. |
| `node list` | List all node types in the workspace. |
| `node info <TYPE>` | Print the node's metadata (ports, schemas, trigger policy) by parsing `nodes/<TYPE>/src/lib.rs`. |

### `cerulion graph`

| Command | Effect |
|---|---|
| `graph create <NAME> [-n PREFIX]` | New `graphs/<NAME>.yaml`. Default prefix is the hostname (with `.local` stripped). |
| `graph run <NAME> [--time-source real\|external\|virtual] [--no-validate] [--release] [--peer-loss continue\|fail] [--single-process] [--auto-partition] [--yes] [--trace-limit N] [--record[=DIR]] [--record-env allowlist\|all\|none] [--record-cpu CORE\|none] [--no-rings] [--network off] [--no-cpu-dma-lock] [--no-monitor-wait]` | Load + validate + execute the graph. `--release` loads each node's release-profile cdylib; without it the freshest built library wins (debug or release, whichever profile you built most recently), and the other profile stays a fallback when only one is built. `--network off`, `--no-cpu-dma-lock` and `--no-monitor-wait` are documented with their environment counterparts under [Environment variables](#environment-variables); `--record-env` and `--record-cpu` are described at the end of this row. **A FAILING validation check refuses the run**; warnings stay advisory. `--no-validate` skips the pre-flight validation REPORT, not validation as such. The graph topology, the macro trigger wiring and the node libraries are re-checked when the graph is built, so the flag cannot get a run past those refusals; it moves you from one refusal to an identical one. What it does still skip is the `schema:` family: a NON-EMPTY value naming no resolvable schema, one contradicting the producing node's own declaration, or one disagreeing with the consumer's. And any `schema:` spelling that is AMBIGUOUS in the workspace is REFUSED outright, by the workspace LOOKUP, before the graph is built and whether or not the report ran: `--no-validate` skips the report, not the resolution, and a spelling with more than one definition has no meaning for a run to proceed on, so the run (and `graph profile`, which never runs the report) refuses it with the same refusal reason the report carries, in a refusal of its own (`graph 'perception': output 'cam'.out declares schema: Foo, which this workspace refuses: 'Foo' is ambiguous …`, with every refused spelling named at once, each by the first port that declares it; a bare name the `.msg` store defines in two packages is refused the same way; a schema file the lookup cannot read or parse stops the run too, framed as `could not check` and naming that file), naming every source: an entry name declared by several `schemas/*.yaml` files, a file stem beside an entry of the same name in another file, or a YAML definition the `.msg` store also spells (a bare entry or stem beside any `schemas/<pkg>/msg/<Name>.msg`; a nested `schemas/<pkg>/<Type>.yaml` beside `schemas/<pkg>/msg/<Type>.msg`, for the qualified spelling; a bare name one nested sole-entry `schemas/<pkg>/<Name>.yaml` and any `schemas/<any>/msg/<Name>.msg` (or a second such nested file) also spell, refused before the store may answer; a nested file beside a TOP-LEVEL definition is not a twin: the bare spelling names the top-level one and the nested file is `<pkg>/<Name>`) (`'Foo' is ambiguous in this workspace`, then `defined by: schemas/a.yaml (entry Foo), schemas/b.yaml (entry Foo). …`); `cerulion schema info` refuses the same spelling the same way (a port whose Rust type a `#[cfg]` decides (a gated module beside a same-named crate, an ancestor's module reached through a gated glob (or a gated ancestor module reached through a glob), a gated `use` of the module a type's path starts with, cfg-exclusive imports of one leaf (or of that module), or a conditional import shadowing an outer scope's) has two or more valid names and accepts any of them, as a pass with a warning; a `#[cfg]`-gated glob that is the ONLY path to a port's type is not one of these: with the cfg off the type is unbound and that configuration does not build, so the port carries one name and no alternative, `graph validate` describes that configuration only, and the parser says so for the port; the node parser's own warning, said when the node is read, names the shape and, for a gated module, glob or module import, the `use ::X::T` / `use self::X::T` (or `use super::X::T`) spelling that ends it). An ABSENT or empty `schema:` is NOT in that set: it is refused by `validate_graph`, i.e. as a topology failure, which this flag cannot skip either: such a graph is REFUSED whether or not `--no-validate` is passed. A graph whose `schema:` is NON-EMPTY but wrong (the family the flag DOES skip) RUNS instead: the wire layout comes from the node's Rust type, not the label, so the consequence appears later, as a bag channel whose label resolves to nothing or to the wrong definition, a Studio decoder that renders nothing, or a replay that refuses a healthy bag. How much of the channel is wrong depends on the producing node: one built with `#[cerulion_node]` supplies both the wire hash and the fixed SIZE, so the frames are described correctly and only the NAME is wrong; a node whose port metadata carries no size (a `--raw-ffi` node whose info block omits the key, or a node library built before that key existed) falls back to the workspace schema's size, and records `0` when neither source can size the channel, so a wrong label can be joined by a wrong or unknown size. **`--no-validate` CONFLICTS with `--record` at parse time:** a recording makes those later consequences permanent: a bag written with the schema checks off carries channel labels nothing ever checked, so it cannot be reliably replay-verified (`cerulion bag play --resim` can refuse a healthy bag, or replay one under the wrong label). Refused before the graph is read, so it costs no shared memory, no workers and no bag. Drop one: record with validation on, or run without `--record`. **Default (`--time-source real`): live event-driven loop** (RealClock on the iceoryx2 WaitSet), which wakes within microseconds of a publish. `virtual` selects the deterministic VirtualClock + 1 ms poll path (replay / benchmarking). `external` is accepted but **inert + warns**: an external time master (a sim's `/clock`) is not supported. **Multi-process:** a graph declaring `process_groups:` runs one worker PROCESS per group on Unix (Linux AND macOS), in barrier lockstep by default, or free-run under the `CERULION_EXECUTION_MODE=free_run` opt-in (see [Environment variables](#environment-variables)); non-Unix falls back to the monolith with a loud notice (`--single-process` forces the monolith anywhere). **Multi-process by DEFAULT:** an UNPARTITIONED graph (no `process_groups:`) on Unix under the real clock DERIVES a partition (a cost-aware one, fused greedily, when `graphs/<NAME>.costs.yaml` exists, written by `cerulion graph profile`, else the process-per-node baseline) and runs it multi-process. Persisting the derivation into the YAML needs consent: `--yes` writes (surgical splice + `.bak`); a TTY run previews the bands + diff and asks y/N (`N` still runs multi-process with the groups held IN-MEMORY, file untouched); a non-TTY run NEVER mutates the file (in-memory + a loud notice naming `--yes` and `--single-process`). `--single-process` opts out entirely (no derivation, no prompt). **A `block` edge is split only when it can be credited:** a topic with an in-graph producer and a `block` consumer has its WHOLE FLOW (producers, `block` consumers, and any non-`block` siblings) placed in ONE group by the derivation (shown in `graph partition`'s consent preview; logged at `info` by `graph run`), because crossing a process boundary costs a real hop and a worker that cannot see a mixed topic's sibling installs a defer the monolith degrades. A HAND-WRITTEN partition may still split the edge, and is accepted when the supervisor can mint it a **cross-process credit word** (exactly one in-graph producer, no non-`block` consumers), which carries the defer through shared memory losslessly. Every other split is refused before any worker spawns, naming which bar it hit: a multi-producer topic (the word counts ONE producer's outstanding frames) or a MIXED topic (its `block` consumers degrade to `drop_oldest`, so there is no lossless defer to credit). `--auto-partition` re-derives over an EXISTING `process_groups:` block (diff vs your block; TTY `N` keeps yours; conflicts with `--single-process`). Virtual/external time sources keep their earlier monolith routing on unpartitioned graphs. See `docs/multi_process.md` ("When does a run go multi-process?"). `--peer-loss` sets the worker-death policy. `continue` (default): the dead group is dropped from the barrier and the survivors keep running DEGRADED (loud; exit 0 unless every worker crashed); `fail`: any worker death stops the whole deployment (fail-loud, for CI/replay). `--trace-limit N` caps the in-memory fire-trace ring (default 100 000; `0` rejected, because the trace is a bounded observability window, not the replay record). `--time-source external` is REJECTED for multi-process (every worker advances its OWN gating clock, the barrier-gated lockstep quantum by default or its wall-faithful clock under free-run, never an external time master). **Scheduler-trace rings:** every MULTI-PROCESS run provisions one shared-memory scheduler-trace ring per rank plus a supervisor departure ring, **recording or not**. That is what makes a Flashback capture of ANY serving run re-executable with `cerulion bag play --resim`, instead of only a run somebody decided in advance to record. Cost: ~40.06 MiB APPARENT per rank (`65_600 + 2^20 × 40` = 42,008,640 B) plus one 106,560 B departure ring; the segment is `ftruncate`d rather than written, so it costs a page at first and converges on the full figure only as the ring fills. `--no-rings` declines them for a memory-tight robot, and ALSO stops the Flashback window recorder being started for that run: with no trace rings nothing captured could be re-executed, so the run takes NO captures rather than frames-only ones (`CERULION_FLASHBACK=off` is the SEPARATE, orthogonal switch for the state plane + anchors). `--no-rings` CONFLICTS with `--record` at parse time (a recording without a scheduler trace is not a recording). It is an option of `graph run` ALONE: `cerulion ros2 attach` and `cerulion node run` do not define it, so passing it there is an unexpected-argument parse error, not an inert flag (those verbs reach a graph run internally and mint no ring on the monolith shapes below, but there is no flag on them to say so). On the `graph run` shapes that mint no ring anyway it is NOT a no-op: a `--single-process` or `--time-source external` run still has its window recorder stopped by it, so the run takes NO captures, and each says at launch which it was. Two shapes are true no-ops: a `--time-source virtual` MONOLITH (it starts no recorder either way, though a `virtual` run of a `process_groups:` graph still routes to the supervisor, which does start one), and any NON-UNIX build, where the recorder, the run directory and the `cerulion flashback` verb are all `#[cfg(unix)]` and there is nothing for the flag to stop. Those monolith shapes mint no ring DELIBERATELY: their gating clock is wall-driven, so a trace taken there would carry boundaries a resim cannot re-advance to (a `resimmable: true` that would not hold); routing them onto the recording clock's discipline is not implemented. Not to be confused with `--trace-limit`, which caps the IN-MEMORY fire-trace deque `cerulion graph` introspection reads. **Recording:** `--record[=DIR]` records the run to a replay-grade MCAP bag (Unix-only; requires the live clock, so `virtual`/`external` are rejected). A single-process run records wall-faithful `fire_time_ns` on the wall-following gating clock; a `process_groups:` graph records the MULTI-PROCESS run into ONE bag: the run's per-rank trace rings + the supervisor's departure ring (the ones above; `--record` drains them, it does not create them) drained by one `bagd`, each record's `reserved` stamped with its ring's rank (per-rank manifests + the `u32::MAX` departure sentinel). Under the default barrier lockstep, mp recordings are QUANTUM-timed (lockstep gating clocks), not wall-faithful; under the `CERULION_EXECUTION_MODE=free_run` opt-in each rank records its OWN wall-faithful timeline from a shared epoch (the bag's `coordination` stamp says which); `--single-process --record` forces the single-process wall-faithful path. See `docs/multi_process.md` ("Recording a multi-process run"). **`--record-env`** (requires `--record`) chooses how the environment is captured into the bag's `env.json`: `allowlist` (default) records every variable NAME but only records VALUES for `CERULION_*`, `RUST_LOG` and `IOX2_*`, hashing every other value so a re-execution can DETECT divergence without the bag carrying your secrets; `all` records values verbatim (full fidelity, and the bag embeds whatever was in your environment, which is warned loudly at record time); `none` records names only. **`--record-cpu`** (requires `--record`) says where the spawned recorder runs: a core id to pin it to, or `none` to let it float. Absent means AUTO: on Linux with at least 4 **permitted** cores the recorder is pinned to the highest-numbered permitted one, which removes the x86 recording tail. Permitted, not logical: the count comes from this process's CPU affinity mask, so a container allowed 2 cores of a 64-core host is capacity-bound and skips the pin. An explicit core must be in that permitted set, or it is a loud error naming the set. Pinning is Linux-only; elsewhere an explicit core warns loudly and the recorder floats. |
| `graph validate <NAME> [--release]` | Parse + validate without running. The report names the node library it found for each node and its profile; `--release` makes it look for release-profile libraries only, matching `node build --release` and `graph run --release`. |
| `graph list` | List all graphs in the workspace. |
| `graph levels <NAME>` | Read-only view of the derived DAG levelization: one row per level (nodes + trigger policies), the triggering edges leaving each level, and, when the graph declares `process_groups:`, the group → owned-level band mapping with the spawner-consumability verdict (an invalid partition still prints in full, then exits nonzero: CI-gateable). That verdict also NAMES any split `block` edge it found CREDITABLE, so this is the verb that answers "will my hand-written split work?". It says which edges depend on a supervisor-minted credit word. Judged on SOURCE metadata; `graph run` re-checks the built cdylibs and refuses on drift. |
| `graph profile <NAME> [--duration SECS] [--fires N] [-o/--out PATH]` | Profile the graph LIVE and write its cost snapshot to `graphs/<NAME>.costs.yaml` (`-o/--out` overrides). An output whose `schema:` the workspace defines more than once is refused before profiling, naming every source (the resolver's refusal, the same as `graph run`'s: a profile executes the graph and never runs the validation report). Runs on the real clock; measures per-node p50 tick durations + per-edge fire rates, and FREEZES the core-count default budget into the v2 artifact (`derived_budget_ns` = `ceil(Σ p50 / profile_cores)`, from the profiling machine's permitted cores), the input of the cost-aware auto-partitioner (see `docs/auto_partitioning.md`). **Default: per-node fire targets are AUTO-DERIVED**: a warm-up of `clamp(cap/10, 1s, 3s)` observes each node's rate, projects its fires over the cap, halves that (rate-droop tolerance) and clamps into [20, 1000]; the run stops when every warm-up-active node meets its OWN target, at the `--duration` cap (default 30 s), or on Ctrl+C. Harvest isolation is judged against the same observation re-projected to the ACTUAL observed window: an early Ctrl+C isolates only genuinely under-sampled nodes, never everything against a cap-length projection. `--fires N` overrides with ONE uniform target for every node (the escape hatch for nodes whose first fire lands after the warm-up, e.g. a period > 3 s). A node that falls short of its target is **ISOLATED**: no cost is recorded (never fabricated) and it stays in its own process group, warned loudly per node (a node silent through warm-up carries the "no target derived" marker, and a never-fired node whose triggering inputs saw a zero-rate topic gets the starved-trigger hint: profile under representative load, running the driver graph alongside), but exit 0 (isolation is a valid outcome; raise `--duration`, or force uniform mode with `--fires N`). The artifact is user-editable; its `hop:` block (per-platform defaults) can be overwritten with hop costs measured on your machine. See [Splitting a `block` edge across processes](#splitting-a-block-edge-across-processes-the-credit-word). |
| `graph partition <NAME> [--costs PATH] [--budget-ns N] [--dry-run] [--yes]` | Derive and WRITE the graph's `process_groups:` partition: a cost-aware greedy fusion when a cost snapshot exists (the default `graphs/<NAME>.costs.yaml`, or an explicit `--costs`, which must exist and parse; a present-but-malformed artifact is a hard error, never a silent baseline fallback), else the process-per-node baseline. With a cost snapshot the DAG levels are refined first, and a `level_assignments:` block is written beside the partition only when that refinement moves at least one node (no snapshot, or a refinement that changes nothing, writes none and removes a stale one); see [Deployment keys](#deployment-keys-process_groups-process_group_order-level_assignments). `--budget-ns` caps each group's summed p50. **Default: the artifact's FROZEN `derived_budget_ns`** (`ceil(Σ p50 / profile_cores)`, computed at profile time on the profiling machine and never re-derived by a reader; re-profile on the TARGET machine to re-derive); an explicit value always overrides, and an earlier (v1) artifact or absent frozen value falls back to unbounded fusion with a loud re-profile info. The `graph run` default resolves the budget through the SAME point (the two surfaces cannot diverge). The rewrite is SURGICAL: only the `process_groups:` and `level_assignments:` blocks change (a stale `process_group_order:` block is also removed, because the emitted listing order IS the rank order); comments and formatting are preserved byte-for-byte and the prior file is backed up to `<file>.bak`. NEVER writes without consent: a TTY run shows the proposed bands + a block-scoped diff and asks y/N; non-TTY requires `--yes` (else a loud refusal naming `--yes` and `--dry-run`); `--dry-run` previews only (wins over `--yes`). Uses replace-scoped validation (every check EXCEPT the partition blocks being replaced), so it is also the recovery tool for a stale/broken `process_groups:` block. |

### `cerulion topic`

| Command | Effect |
|---|---|
| `topic list [--all] [--no-network] [--connect LOCATOR]... [--listen LOCATOR]... [--scan]` | List LOCAL topics FIRST/instantly, then REMOTE topics discovered over the LAN. The framework's own channels (the recorder's `/bagd/status`, anything under `/__cerulion/`) are HIDDEN from the LOCAL section of the default listing (REMOTE rows are not filtered: a robot's own channel that reaches the announce or demand space, or a local mirror of one, prints as a plain REMOTE row with or without `--all`); when any were, ONE count line says so (`1 internal topic hidden (--all shows it)`), and `--all` lists them with an `internal` marker after the path (the path stays the row's first token; `topic echo`/`info`/`hz` read an internal topic by name either way). Automagic: remote discovery runs BY DEFAULT (multicast + gossip scouting ON, a bounded sub-second query across the demand + announce key-spaces), rendered as a `REMOTE TOPICS` section, so an unpaired robot shows up with no flags; when the bounded gather finds nothing the section collapses to ONE line, `remote: none discovered in 500 ms (a robot off the LAN needs --connect tcp/<host>:7683)` (a peer that IS reachable, a given locator or a discovered robot, but advertised no topic in time gets a `retry` hint instead of the `--connect` one). `--no-network` skips the remote query (scripts / CI / offline). `--connect tcp/192.168.123.99:7683` (repeatable; `--listen` mirrors it) ADDS locators to reach a peer scouting can't find (7683 is the well-known permissive-gateway port; a graph with an explicit `network:` block listens wherever its `listen:` says); no gating flag required. The remote half is best-effort: a session/query failure is a LOUD note + exit 0 (the local list already printed), never a silently-empty section, never a fake-success. **Discovery ladder:** the LADDER (mDNS `_cerulion._tcp` browse, the PRIMARY rung, plus verified cached peers `~/.cerulion/peers.json` plus the `CERULION_PEERS`/config/`<name>.local` hostname convention, in parallel under a ~1.5 s ceiling) finds robot gateways with zero typed addresses, folds their locators into the query session, and renders a `ROBOTS` section (robot, gateway port, locator, rung) above `REMOTE TOPICS`; nothing discovered = no `ROBOTS` section at all. **`--scan` (rung 4, OPT-IN):** also unicast-sweeps the local `/24`(s) on the well-known gateway port for robots that BOTH multicast and mDNS reflection hide: verify-before-trust (only a gateway that answers a beacon probe is shown), the subnet clamped to the host's `/24`. OFF by default and STRUCTURALLY unreachable without the flag (a horizontal connect sweep reads as port-scan recon to corporate IDS; `--scan` is the sole producer, no env/config path). See `docs/networking.md` "Finding robots". |
| `topic info <TOPIC>` | Schema name and schema hash, plus the last message's sequence number and wire timestamp. |
| `topic echo <TOPIC> [--truncate-length N]` | Stream messages with schema-aware decoding for stock types (`std_msgs/String`, `sensor_msgs/Image`, …); hex fallback otherwise. `--truncate-length` caps how many elements of a long array or string each line prints (default 128, minimum 1); raise it to see a whole payload, lower it to keep a fast topic readable. |
| `topic hz <TOPIC>` | Publish-rate measurement. |

### `cerulion schema`

| Command | Effect |
|---|---|
| `schema create <NAME>` | New `schemas/<NAME>.yaml`. PascalCase'd from the name. |
| `schema delete <NAME>` | Remove `schemas/<NAME>.yaml`. Workspace schemas only; built-in ROS 2 messages cannot be deleted. |
| `schema info <NAME>` | Display fields (fixed vs variable), wire fixed size + schema hash, as ONE unified recursive tree: every non-primitive field's schema expands inline, indented directly beneath it (built-in nested types expand exactly like custom ones; arrays expand their element type once; a repeated type reads `(…, expanded above)`). Resolves workspace schemas (`schemas/*.yaml`, by file stem or schema name) AND built-in ROS 2 types (`pkg/Type` or `pkg::Type`, e.g. `sensor_msgs/Image`), and a robot's custom types over the network when unresolved locally. One consistent `source:` provenance line names the origin (built-in / workspace file / `.msg` store file / robot over the network). A workspace schema shadowing a built-in name wins, with a loud stderr warning. |
| `schema list` | List workspace schemas plus all built-in ROS 2 types grouped by package; shadowed built-ins are marked inline. |

**A port's `schema:` is CHECKED, and an un-checkable one is refused.**
Where a port names a workspace schema, `graph validate` (and `graph run`'s pre-flight gate, and the authoring
verbs `node create -o/-i` and `node modify`) size the definition the workspace binds
before accepting the name: a schema the wire cannot carry is refused with the
same text `cerulion schema info` gives it. Three conditions are REFUSALS rather
than warnings, because accepting them would let the name pass validation while
the recording fold warned and skipped the file, so the name would get no
hash-map entry and the downstream checks would be silently skipped:

Each row is the payload; `graph validate` wraps it as shown below the table.

| Condition | What the message says |
|---|---|
| the `schemas/*.yaml` the spelling binds cannot be re-read | `could not verify this schema's size — schemas/<file> (bound by the port spelling): the file the workspace lookup bound could not be re-read (<io error>)` |
| that file no longer parses | `… : the file no longer parses (<YAML error>)` |
| that file no longer declares the entry | `… : it no longer declares '<entry>'` |

Each names the FILE to fix, and the three are kept apart because their remedies
differ: a syntax error is not a missing declaration. Where one file declares the
same schema twice (`pkg/Type` and `pkg::Type` are one identity), the name is
refused if EITHER declaration is un-carryable, matching `cerulion schema info`,
which judges both declarations and refuses on the un-carryable one. (A duplicated
identity whose declarations are ALL carryable is accepted here and by `schema info`,
while the recording fold binds no hash for it, so a bag records that channel with no resolved schema. Give the identity one definition to have it recorded with a hash.)

These are "could not CHECK" answers, not "does not EXIST" ones, and
`graph validate` says so: it reports `could not be checked against this
workspace … what failed is the CHECK, not the lookup`, so you look at the file
named rather than hunting for a missing definition.

### `cerulion ros2`

The ROS 2 interop family. `run` / `launch` run stock ROS 2 entry points on
Cerulion transport: swap the command, not the stack. Both are **verbatim
pass-through wrappers** (Unix only: they stage the environment and `exec()`
the native ros2, which is what carries the platform restriction): everything
after them is forwarded to the native `ros2 run` / `ros2 launch` untouched
(hyphenated flags, the package form, every future native flag), so their
argument surface IS ros2's; see `ros2 run --help` / `ros2 launch --help`.
`attach` bridges a live ROS 2 / DDS robot onto Cerulion (there is no
`cerulion ros attach`: that spelling fails with an error naming this one). `migrate` is a normal verb with its own
surface (nothing is forwarded, nothing is exec()'d) and carries no platform
restriction of its own; it runs wherever the clang engine binary does.

| Command | Effect |
|---|---|
| `ros2 run [ARGS...]` | `ros2 run demo_nodes_cpp talker` becomes `cerulion ros2 run demo_nodes_cpp talker`; nothing else changes. Stages the child environment: `RMW_IMPLEMENTATION=rmw_cerulion`, the Cerulion lib dir prepended to `LD_LIBRARY_PATH`, a minimal ament prefix (whose `lib/librmw_cerulion.so` links the built cdylib) prepended to `AMENT_PREFIX_PATH`, and an automatic `LD_PRELOAD` of `libcerulion_heaphook.so`, which the Linux packages install beside the binary (one `info` line names the injected path; `CERULION_ROS2_PRELOAD=off` disables). It then **`exec`s** `ros2 run [ARGS...]`. The `cerulion` process BECOMES `ros2`: stdio, Ctrl+C and the exit code all flow through the kernel untouched. Nodes started this way find each other by topic name on the shared-memory plane, but the ros2 CLI graph tools (`ros2 topic list`, `ros2 topic echo`, `ros2 node list`, rqt) do not see Cerulion topics (the rmw's discovery is process-local); inspect them with `cerulion topic list` / `cerulion topic echo`. The rmw cdylib is looked for next to the `cerulion` binary (the Linux packages install it there; in a checkout `cargo build -p rmw_cerulion` puts it there); `CERULION_LIB_DIR` points elsewhere. **Exit codes** are the verb's own only when it never got to `exec`: **69** `librmw_cerulion.so` missing (on Linux the message names the install routes and the override; on macOS it says ROS 2 Jazzy has no macOS binaries and the verbs need a Linux machine), or `--adopt-take` given at all (see below), **127** `ros2` not on `PATH`, **1** any other pre-exec failure, **2** clap's own usage error (bare `cerulion ros2`) or `CERULION_RMW_ADOPT_TAKE` set in the environment WITHOUT `--adopt-take`; a successful `exec` inherits `ros2`'s exit code. A bad package name, a missing launch file or a wrong flag are `ros2`'s own errors, reported by `ros2` itself. **`--adopt-take` is REFUSED by this verb: exit 69.** It is Cerulion's ONE own flag here and must come FIRST, right after `run` (everything else is forwarded verbatim, so the position is what tells it apart from a `ros2` flag of the same name). It asks for zero-copy plain takes: a take of a forgeable type serves the shared-memory bytes IN PLACE instead of copying them, and the application's own `free` of each sequence releases the sample. That needs `libcerulion_heaphook.so` loaded IN THE NODE, and the launcher hands the hook over as an inherited descriptor (`LD_PRELOAD` names `/proc/self/fd/<N>`, never the hook's path, so no name is resolved a second time between validation and `exec`), but `ros2 run`/`ros2 launch` are a Python CLI that spawns the node as a FURTHER subprocess with `close_fds=True`, so that descriptor is already closed when the node's loader reads `LD_PRELOAD`. The hook would not load and every take would be served by copy, after this verb had already reported success, so the verb refuses at launch instead, naming the cause and the alternative; no two-hop-safe binding scheme exists. **To get zero-copy plain takes, launch the node executable DIRECTLY**: one hop keeps the preload. Run this in a shell that has already sourced your ROS 2 setup; each path variable is PREPENDED to the sourced value, exactly as this verb stages it, because replacing them would drop the distro's own ament index and libraries: `LD_PRELOAD=<lib dir>/libcerulion_heaphook.so${LD_PRELOAD:+:$LD_PRELOAD} LD_LIBRARY_PATH=<lib dir>${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH} AMENT_PREFIX_PATH=<ament prefix>${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH} RMW_IMPLEMENTATION=rmw_cerulion CERULION_RMW_ADOPT_TAKE=1 <install>/lib/<pkg>/<executable>`. The refusal message prints that line with your actual paths filled in **when those paths can carry it**. When they cannot, it prints no command at all and says why instead: `LD_PRELOAD` is split by the loader on spaces AND colons (`LD_LIBRARY_PATH` and `AMENT_PREFIX_PATH` on colons), and a path whose bytes are not valid UTF-8 cannot be rendered as text without silently rewriting it, so a pasted command would start the node WITHOUT the hook, or point at a location that does not exist. Quoting does not help: the shell hands the variable through intact and the loader splits its value. Relocate the library to a path with no space, no colon and valid UTF-8 (for example beside the `cerulion` binary), then run the verb again to get the command. Adoption is **Linux/GNU only** either way: the hook interposes glibc's `malloc`/`free`. Two environment variables belong to the child, not to this verb: **`CERULION_RMW_ADOPT_TAKE`** is read by `rmw_cerulion`; this verb does not set it (the flag refuses), and setting it yourself here is refused (exit 2) rather than honoured as a request nothing validated; set it on a node you launch DIRECTLY, with the hook preloaded; and **`CERULION_RMW_ADOPT_TAKE_BUDGET`** (a positive integer) caps how many samples one subscription may hold adopted at once, defaulting to the built-in borrow budget; an unusable value warns and uses the default (once per subscription that reads it, not once per process: creates are rare and each is its own report). |
| `ros2 launch [ARGS...]` | The same staged environment and `exec()`, forwarding to `ros2 launch [ARGS...]`: `ros2 launch demo.launch.py use_rviz:=false`, the `pkg file.launch.py` package form, `--show-args`, all verbatim. Same exit contract, and the same leading-position-only `--adopt-take` (given right after `launch`), refused for the same reason. |
| `ros2 attach --iface IP [--domain N] [--timeout SECS] [--graph-name NAME] [--topic-prefix PREFIX] [--robot-name NAME] [--dry-run] [--yes]` | Discover a running ROS 2 system's DDS topics, resolve each type, and generate a bridge graph (`graphs/<NAME>.yaml` + `graphs/<NAME>.bridge.yaml`) that carries those topics onto Cerulion. `--iface` is REQUIRED and is the robot-LAN interface IP to run discovery on; it restricts discovery to that interface so a multi-homed host does not drop fragmented discovery data. `--domain` must match the robot's `ROS_DOMAIN_ID` (default 0). `--timeout` is the discovery collection window in seconds (default 5; must be finite and within `(0, 3600]`, rejected at parse time). Types that resolve neither in the workspace `.msg` store nor in the built-in corpus are acquired over the wire from the robot's own `~/get_type_description` service and materialized under `schemas/<pkg>/msg/`; see `docs/schema_resolution.md`. Nothing is written or run until you consent: `--dry-run` prints the discovery report and stops (it wins over `--yes`), `--yes` writes and runs non-interactively, and a TTY run asks. `--robot-name` sets the generated graph's `prefix:` namespace; it is NOT the announced network identity, which comes from this machine's hostname or `CERULION_ROBOT_IDENTITY`. **Every run's report ends with an automatic MIGRATION section** (no flag; `--dry-run` prints it too): the discovered processes grouped by whether they could run under `rmw_cerulion` instead of being bridged: restartable today (every message type resolves locally), restartable once this attach's acquired schemas are written on consent, or stays bridged (unresolvable types named; endpoints with no `ros_discovery_info` record are worded as absence of evidence, a vendor/raw-DDS process or a node table the window did not observe; and a robot where that is everything gets the zero-restartable line), plus the equivalent `cerulion ros2 launch` line, a paste-ready `ros2:` graph-entry block (`<package>`/`<executable>` placeholders: DDS discovery sees endpoints, not launch metadata), the bridged-vs-native cost facts (a bridged topic pays a per-message CDR decode in the bridge; a native topic has no decode hop and is zero-copy eligible), and a closing pointer at `cerulion ros2 migrate`. Pure rendering over the discovery data already in hand: it never blocks, opens no network, and changes no outcome or exit code. **No visualization node is staged on the robot**: to see the data, run `cerulion viz --robot <name>` on your own machine. |
| `ros2 migrate [--workspace DIR] [--write] [--yes]` | Migrate the colcon workspace's C++ publish call sites to the ROS 2 **loaned-message API** (`borrow_loaned_message()` → fill → `publish(std::move(loaned))`) wherever a clang AST prover shows the rewrite is behavior-preserving: an AST transform over the workspace's `compile_commands.json` (build with `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` first; the verb names that command when the database is absent), never a regex. The safe pattern is a locally-built message (`std::make_unique`/`std::make_shared`/a stack local) filled and published once, to a provably-fixed `rclcpp::Publisher`, with no other use of the pointer; everything unprovable lands in a **manual-candidates report** with a fixed reason (pointer escapes, built in another function, retained member, conditional publish, publisher not provable, constructor arguments, …). rclpy nodes are **report-only**: rclpy has no loaned-message API upstream, so those keep one copy per side (two copies become one under the launcher's heap hook). Migrating does not couple the workspace to Cerulion; rclcpp falls back to allocate+copy when the rmw cannot loan. **INITIALIZATION, HOWEVER, IS NOT IDENTICAL ON EVERY RMW, and this decides whether a site is safe to migrate.** `make_unique<T>()` VALUE-INITIALIZES: every field the code does not write holds its declared default (a Quaternion's `w=1`, zeros elsewhere). A LOANED message carries that guarantee only where the rmw provides it, and `rclcpp::LoanedMessage` does NOT construct on the can-loan path; it takes the pointer `rcl_borrow_loaned_message` returns and casts it (`rclcpp` jazzy, `loaned_message.hpp`), placement-new'ing `MessageT()` only in the CANNOT-loan branch. So: on `rmw_cerulion` initialization matches `make_unique` on both paths; a loanable (recursively-fixed) type's SHM slot is constructed by running the typesupport's own `init_function` with `MessageInitialization::ALL` (`rmw_borrow_loaned_message` → `init_loaned_payload`, so declared defaults hold), and a type it cannot loan returns `RMW_RET_UNSUPPORTED` so rclcpp heap-constructs it. On ANY rmw that cannot loan the type, rclcpp constructs it too, so that case matches everywhere. But on an rmw that CAN loan and does not initialize, the loaned buffer is **uninitialized, possibly recycled memory**: `rmw_fastrtps`, the ROS 2 default, calls Fast DDS `loan_sample()` with no `LoanInitializationKind` and constructs nothing afterwards, and that parameter's default is `NO_LOAN_INITIALIZATION` ("Do not perform initialization of sample … the user should take care of writing every field on the data type before calling write"). So **the rewrite value-initializes the loaned message itself**: it emits `<loaned>.get() = <MessageT>();` immediately after the borrow, restoring exactly what `make_unique<T>()` gave on every rmw, at the cost of one value-init store on a path that has just taken a loan. It is an ASSIGNMENT rather than a placement-new deliberately: on the cannot-loan path rclcpp has already constructed a real message that may own heap members, and a placement-new over it would leak them, while on the loaning path the type is `is_plain()` and the implicit copy-assignment is trivial (it writes the destination without reading it). Without that store a field the site does not write would read as whatever the loan pool last held, with no compile error and no crash. (Fast DDS loans only "plain" fixed types, so a message with a string or a vector, `std_msgs/String` among them, takes the constructing fallback; a plain one such as `geometry_msgs/Twist` does not, which is why the matrix carries a partially-written `Twist` fixture.) **Dry-run by default**: prints the full unified diff + the candidates report and refreshes the machine-readable manifest at `.cerulion/ros2-migrate-manifest.json` (its only write; atomic). `--write` applies behind consent (a TTY asks; non-TTY requires `--yes`): it REFUSES a dirty git tree, writes **one commit** plus `cerulion-ros2-migration.patch` (undo = `git revert <sha>` or `git apply -R cerulion-ros2-migration.patch`). **One case puts other paths in that commit:** a `pre-commit` hook stages INSIDE `git commit`, after this verb's own index check, so a hook that stages (`git add -A` is the common shape) adds its own paths to the migration commit. Hooks are the user's responsibility; the run does not refuse or unwind, it COMMITS AND WARNS, naming the unplanned paths, and when that warning appears the safe undo is **`git apply -R cerulion-ros2-migration.patch`**, which reverses only the migration's own edits. `git revert <sha>` is NOT an equivalent there: it would undo the hook's paths too. After the commit, `--write` runs `colcon build --packages-select <affected>`: a build failure exits 1 and names the revert path; the commit stays. A Ctrl-C during the write window does not silently strand a partial migration: the interrupt is observed at the write safepoints, the run rolls back what it wrote and refuses, and anything the rollback cannot restore (a write failure) or deliberately preserves (another actor changed the file mid-run) is REPORTED for hand repair, never dropped silently (after the commit exists, an interrupt changes nothing: the commit is the durable state; a hard kill such as SIGKILL can still leave partial state, which is what the patch file and the clean-tree gate's committed baseline are for). While `--write` is applying, it holds the workspace lock (`<workspace>/.cerulion/workspace.lock`), **on Unix** (see the platform note below), so a second Cerulion writer on the same directory (another `ros2 migrate`, or a `ros2 attach` / `node` / `graph` / `schema` command when your Cerulion workspace root IS this colcon workspace) WAITS (with one loud line naming the lock file) instead of interleaving its writes with the migration's. The lock is per-directory: if `--workspace` is not also your Cerulion workspace root, those other verbs lock a different file and do not wait. It is taken after you consent and released before the build, so reading the diff blocks nobody and the build does not hold it; Ctrl-C ends the wait (the run refuses without rewriting a source file or committing anything; the wait may have created the untracked `.cerulion/`, and the message says so) rather than leaving you with a process no ordinary signal can stop. One consequence of the commit happening under the lock: a `pre-commit` hook that itself runs a `cerulion` command which writes this workspace will deadlock; hooks are your responsibility. The dry-run takes no lock and leaves no lock file (so a dry-run run alongside someone else's `--write` can report a half-migrated tree; re-run it if the report looks inconsistent). **Platform note:** the lock is a kernel `flock(2)`, so every guarantee in this paragraph is a UNIX guarantee. On a non-Unix build the guard is a declared-weaker no-op that locks nothing and waits for nobody; two concurrent writers there can interleave. Every `cerulion` workspace writer behaves that way on a non-Unix build. The dry-run diff and the applied bytes come from the same code path (byte-identical by construction); a second run proposes nothing (idempotent). Requires the migration engine binary `cerulion-ros2-migrate-clang` (beside the `cerulion` binary, on `PATH`, or via `CERULION_ROS2_MIGRATE_TOOL`): **exit 69** with container build instructions when absent, never a silently-inert analysis. |

### `cerulion viz`

| Command | Effect |
|---|---|
| `viz [TOPIC...] [--robot NAME] [--connect LOCATOR]... [--listen LOCATOR]... [--detach]` | Visualize live topics in Cerulion Studio. Zero YAML, zero hand-written nodes. The verb is a thin CLIENT of the long-lived `cerulion-vizd` daemon: it starts the daemon if absent and sends one attach per topic. It starts no viewer: Studio connects to the same daemon and renders whatever is attached, so open Studio to see the topics (before or after the run). The daemon owns the taps, the schema-generic decode and the render. Each `TOPIC` is absolute (a missing leading `/` is added); pin a type with `TOPIC=SCHEMA` (`/go2/cloud=sensor_msgs/PointCloud2`) when it cannot be resolved locally; otherwise a local topic's schema back-fills from its first frame. **With NO arguments** it attaches every decodable local topic the daemon finds (silent or undecodable topics are skipped). `--robot NAME` visualizes a REMOTE robot: the daemon declares gateway ingress, re-injects the robot's frames into desk-local shared memory and taps the mirror; the robot serves raw bytes and nothing else, and each topic's type is resolved from the robot's served catalog, so no schema pin is needed. A topic the robot does not serve is a hard error, not a silent skip. `--connect`/`--listen` (repeatable, each requires `--robot`) add zenoh locators for a robot scouting cannot reach; they are threaded into the daemon's config only when THIS command starts it; a daemon already running keeps its boot-time locators. `--detach` attaches and returns immediately, leaving the daemon up. Without `--detach` the verb stays attached until Ctrl+C, then detaches only the topics it added; the shared daemon keeps running. |

### `cerulion connect` / `pair` / `login` / `account`

Reaching a robot that is not on your LAN. See `docs/remote_plane.md`.

| Command | Effect |
|---|---|
| `cerulion pair [ROBOT] [--eid HEX] [--addr IP:PORT]... [--code CODE] [--account HEX] [--label NAME] [--key-file PATH] [--relay-url URL] [--relay-disabled]` | Enroll this desk with a robot, once. `ROBOT` is a 64-char-hex endpoint id or a name resolved from the robot's mDNS record (else `~/.cerulion/robots.toml`); `--eid` gives the id directly. `--code` is the short pairing code the robot owner shared; **prefer omitting it** and typing it at the prompt, because a code passed on the command line is visible in process listings for the ceremony window and it authorizes durable enrollment. `--label` is the name the robot stores on its access-list row so its owner recognizes this desk (default: your hostname). `--key-file` is the desk device key (default `~/.cerulion/desk.key`, created `0600` on first pair and reused after). Pairing writes the robot's name pin, so `cerulion connect <name>` works afterwards. |
| `cerulion connect [ROBOT] [--eid HEX] [--addr IP:PORT]... [--topic TOPIC]... [--all] [--key-file PATH] [--schemas-dir DIR] [--relay-url URL] [--relay-disabled] [--network off]` | Dial a paired robot's wire plane and re-inject its topics into desk-local shared memory, so `topic echo` and `viz` see them as LOCAL topics. `--topic` (repeatable) selects what to bring across; `--all` brings everything. Runs until Ctrl+C. It spawns the `cerulion-connectd` sibling binary (a separate binary because it links iroh), which it looks for beside `cerulion`. The release archive, which the installer and the apt package unpack, puts it there; it is not published to crates.io, so a CLI installed with `cargo install` has none. In a source checkout `cargo build -p cerulion_connectd` builds it, and `CERULION_CONNECTD_BIN` points at one that lives elsewhere. |
| `cerulion login` | Sign in to a Cerulion account. Runs the RFC 8628 device-code flow: prints a short code and a verification URL to open on any device, then waits for you to authorize. It is headless-friendly, so it works over SSH. Use it any time to sign in, re-authenticate or switch accounts. Ordinary commands require a signed-in machine: on one that has never signed in, the first command starts this flow at a terminal and refuses with this command's name anywhere else. Exempt: `login` itself, `completions`, `--help`, `--version`, and the internal `graph run-worker` and `graph run-gateway` subprocess verbs a gated parent spawns. |
| `cerulion account devices list` / `devices revoke <DEVICE_ID>` | List and revoke the desks enrolled on your account: self-service management of your own machines. Robot access management lives in Studio and the web account page, not the CLI. |

### Other

| Command | Effect |
|---|---|
| `cerulion bag play <BAG> [-r\|--rate N] [--loop] [--topics TOPIC]... [-s\|--start-offset S] [-u\|--duration D]` | **Playback.** Republish a bag's recorded frames onto local shared memory, wall-paced from the bag's own log times. Every frame goes back out BYTE-VERBATIM, its wire header's `sequence` and `timestamp_ns` intact, under its recorded topic name, in the order the recorder saw them. Nothing is re-executed and no node is loaded: to anything attached to local shared memory this is indistinguishable from a robot publishing, which is what makes it the way to develop a consumer against yesterday's data. Local only. `--rate` scales the pacing (default 1.0), `--loop` restarts at the end, `--topics` (repeatable) plays a subset, and `--start-offset` / `--duration` bound the span: skip the first S seconds of BAG TIME, and stop after D seconds of it (fractional accepted for both). A topic whose publisher slot is already held by a live producer is refused BY NAME and skipped; the rest still play. Add `--resim` to re-execute instead of republish; see the next row. |
| `cerulion bag record [TOPIC...] [-a\|--all] [-e\|--regex PATTERN] [-x\|--exclude PATTERN]... [-o\|--out PATH] [--duration SECS] [--schema-wait-ms MS] [--run[=RUN]]` | **Record.** Tap live topics into a standard MCAP bag, the `ros2 bag record` shape, driving Cerulion's production recorder. Name topics positionally, take everything with `--all`, or select with `--regex` and prune with `--exclude` (repeatable). `-o` defaults to `recording.mcap`; `--duration` stops after N seconds (omit to record until Ctrl+C). **Local payload capture:** this taps THIS machine's shared memory, recording a robot's topics means running this verb ON the robot and copying the file afterwards. Schema resolution may query network peers within its configured budget; `CERULION_RECORD_SCHEMA_DEMAND_MS=0` disables those networked schema lookups. A named topic that is not live here is refused by name. `--schema-wait-ms` (default 2000) is how long each topic's first frame is waited for before the bag is created, so its channel can carry the real wire schema hash; frames seen during the wait are recorded, not dropped, and a topic still silent when it elapses gets a placeholder hash for the life of that bag. `--run` ATTACHES to a live `cerulion graph run`, so the bag describes a RUN, carrying its effective graph, env snapshot, host and run identity, and recording the topics the run declares rather than everything live on the machine. Pass it bare when one run is live; **`--run=<RUN>`, with the `=`, names one** by run id or graph name when several are (several live runs and no id is a refusal, never a guess). Note the `=` is required: `--run foo` attaches to the sole run and records the TOPIC `foo`. Recording begins where it attaches, nothing before that point is recoverable, and the bag says so. `--run` is also the one `bag record` shape whose topic set is INFERRED rather than named, so live-topic discovery is ON for it (a run's declared outputs are not everything it puts on the wire); `CERULION_RECORD_DISCOVERY=off` turns that off. Channels carry the learned wire schema hash; schema resolution adds a qualified name and embeds the required custom definitions when available. Unresolved types remain explicitly unknown. A bare topic recording carries no graph execution context. With `--run`, the recorder also copies the run's graph, environment and identity, and attaches its available scheduler-trace rings. A mid-run resim additionally needs compatible checkpoints: if the run reports a standing Flashback recorder, this recorder leaves its single-consumer state rings alone, so use that recorder's captures for anchored replay. Otherwise it can discover available state rings, without arming checkpoints itself. `bag info` reports the retained context and state coverage; `bag play --resim all --verify` checks the actual recording's prerequisites before comparing outputs. See `docs/flashback.md` for the attach and resume conditions. |
| `cerulion bag info <BAG>` | Show what a bag holds (topics, frame counts, schemas, time span) without publishing anything. The same pre-scan `bag play` prints. Also the way to read a torn bag: it reports what is readable. |
| `cerulion bag play <BAG> --resim all [--verify] [-u\|--duration D] [--strict-state] [--report FILE] [--tolerance FILE]` | RE-EXECUTE a bag carrying supported execution context, such as a `cerulion graph run --record` recording or a replayable Flashback capture: the bag supplies the graph, the env, the recorded external inputs and the scheduler trace, and your workspace's CURRENT node builds are re-run against them. **Bare `--resim all` is NEUTRAL**, it re-executes and reports what happened, makes no claim about whether the result matches the recording, and exits 0 on any completed run; divergence is the output, which is what an edit → build → re-run loop wants. **`--verify` asks for the verdict**: every produced frame is diffed byte-for-byte against the recording, so a code change that alters a node's output is caught as a regression. With `--verify` the exit code is 0 = pass, 1 = data violation, 2 = not replay-grade / corrupt bag, 3 = node failure (build load error, or a node PANICKED, a crash is reported as the root cause, never disguised as a data diff; a node that merely returns `Err` is normal behavior and re-runs to 0), 4 = tolerance-config error, 5 = internal, 6 = the re-executed schedule diverged. When several apply, 3 beats 6 beats 1, the report still shows everything. Neutral mode declines exactly the two COMPARISON codes (1 and 6); 2, 3 and 5 mean the re-execution could not be performed and stay loud in both modes. A node SUBSET (`--resim planner,…`) is not supported and is refused by name. `--duration D` covers only the first D SECONDS of BAG TIME; there is no `--max-ticks` step cap, because a per-rank resim has k step axes and no shared step number while every rank's boundary stream shares the GO epoch, so a bound in TIME is rank-uniform where a step count is not (`--max-ticks` is not accepted: clap's unknown-argument error, exit 2, no alias, no shim); `--strict-state` refuses the run unless every executed node's state was restored (inert on a bag that begins at step 0), both legal in EITHER mode, because they shape the RUN rather than the comparison; `--report FILE` writes the machine-readable JSON verdict for CI and `--tolerance FILE` relaxes the diff per field, both require `--verify`, since a neutral run has no verdict to persist or relax. The playback flags (`--rate` / `--loop` / `--topics` / `--start-offset`) are refused under `--resim`, each with its reason, `--start-offset` because it names an arbitrary point in playback time, while a re-execution can resume only from a recorded checkpoint that carries the per-rank anchors it needs. Re-executing a bag recorded on a different architecture or OS WARNS (float results may differ in last-bit ULPs across arches, a byte-mismatch may be arch skew, not a regression) but never refuses. If a produced topic's **output schema changed** since recording, it refuses up front with exit 2 (naming the drifted topics + both layout hashes) rather than an unexplained per-frame byte mismatch, re-record or check out the recording-era schemas. For why a re-execution diverges (wall clock, unseeded RNG, hash order, cross-host float skew, …) and the sanctioned alternatives, see `docs/replay_determinism_footguns.md`. **There is no separate replay verb**: typing `replay` after `cerulion` is an unrecognized subcommand (exit 2), with no migration message. |
| `cerulion bag migrate <BAG> [-o PATH] [--dry-run] [--yes]` | Rewrite a bag whose embedded graph carries keys the graph format does not define, into a **NEW** bag that `bag play --resim` accepts. An older bag may embed your on-disk `graphs/<name>.yaml` rather than the config the run executed, so an undefined setting (a legacy `policy:` block is the one that really happens) travels into the bag and refuses to resim (exit 2). An MCAP attachment is sealed, so there is no way to edit it in place; this writes a corrected copy instead. It lists every key it would remove, with the PATH and the LINE it sits on in the embedded document, and asks before writing: `--dry-run` shows what would go and writes nothing (it wins over `--yes`), `--yes` writes without asking (required when stdin is not a terminal). **Your original bag is never modified**: the migrated bag is a second file, `<name>.migrated.mcap` beside it unless `-o` says otherwise, and an output that already exists is refused rather than overwritten. Every recorded frame (user topics and Cerulion's internal streams alike, with its topic, sequence, timestamps and payload bytes, in the order the recorder wrote them), the scheduler trace, and every other attachment are copied through unchanged; what changes is `graph.yaml` plus one new `__cerulion/migration.json` attachment recording the date, the source bag's SHA-256, and exactly which keys were removed. The migrated graph is re-rendered from the parsed config, so YAML comments and key order do not survive it: the same normalization `graph run --record` applies; nothing but Cerulion reads that document, and your own `graphs/<name>.yaml` is not touched by any of this. It REFUSES, rather than guessing, when: the bag's graph already parses (nothing to migrate); the bag carries no embedded graph; the bag is not finalized (a torn bag has no attachment index; `cerulion bag info` reports what is readable in it); or the graph fails to parse for a reason other than an undefined key, which is real YAML damage a rewrite cannot repair. |
| `cerulion flashback [--note TEXT] [--pin] [--no-wait]` | **Capture the moment** (Unix only): write a bag covering the last ~30 seconds plus the next ~15, from the rolling window every serving `graph run` already holds. Nothing has to be armed in advance; the window is always on. The capture lands in `recordings/flashbacks/` and the verb WAITS until the bag is written (`--no-wait` returns as soon as the capture is accepted; the accepted line already carries the path it will have). `--note` records a sentence into the capture, so a bag found three weeks later says what it was about. `--pin` exempts it from retention rotation. Turn the whole plane off with `CERULION_FLASHBACK=off`; every size and retention knob is under [Environment variables](#environment-variables), and `docs/flashback.md` has the full model. |
| `cerulion bagd --out PATH ...` | The recorder daemon itself (Unix only): the process `graph run --record` and `cerulion bag record` spawn for you. Run it by hand only when you want to attach a recorder to something already running with settings neither of those verbs exposes; `cerulion bagd --help` lists its full argument surface. Attach-mode channels learn their schema hash from the first frame; configured schema resolution can supply names and definitions. Whole-graph resim additionally requires the graph, environment, scheduler trace and any needed resume checkpoints. `graph run --record` supplies the run context to this same recorder; invoking `bagd` on topics alone does not create it. |
| `cerulion completions <SHELL>` | Print the shell code that enables tab-completion (`zsh` / `bash` / `fish` / `elvish` / `powershell`). Completes subcommands, flags and enum values from the command tree, **plus live names**: topics on `topic echo/info/hz` and `viz`, node types on `node build/run/info/...`, graph names on `graph run/levels/partition/...`, workspace + built-in ROS 2 schemas on `schema info`, robot names on `viz --robot` / `connect` / `pair`, and `.mcap` bags on `bag play` / `bag info`. Install once, per shell (see `docs/cli_completions.md` for the full table); zsh needs TWO lines because the generated script ends in `compdef`, a function `compinit` defines, and macOS's system zshrc never calls it: `echo 'autoload -Uz compinit && (( $+functions[compdef] )) || compinit' >> ~/.zshrc` then `echo 'source <(COMPLETE=zsh cerulion)' >> ~/.zshrc`. These forms regenerate the shell code on demand, so they self-correct across upgrades (redirecting this command into a file also works, but hard-codes the binary's path, so re-run it after an upgrade). **A TAB press never opens the network, never starts a process (`cerulion-netd` least of all), never prints to stderr, and is bounded at 150 ms**: every source is a local directory / `~/.cerulion` file / compile-time static read. Consequence: a REMOTE robot's topics complete only while something HOLDS a mirror of them, and demands are process-scoped; a running `cerulion viz --robot NAME` leaves one standing (vizd holds the demand), `topic echo/info/hz` holds one only for as long as that command runs, and `topic list` demands NOTHING at all (it is a service-directory scan plus a liveliness gather), so with no vizd attach remote topics do not complete. Topic candidates also carry no origin label, because distinguishing a mirror from a local producer costs 620 ms. See `docs/cli_completions.md`. |
| `cerulion tui` | Interactive ratatui dashboard (Nodes / Topics / Echo tabs). |
| `cerulion clean [--report-only]` | Sweep iceoryx2 bookkeeping for **dead nodes only**; live nodes' state is preserved. On macOS it also reports the `/tmp/*.shm_state` population (the per-segment files `iceoryx2` leaves behind when a process is killed, which every dead-node sweep must then read past) and reclaims the ones whose creating process is **provably gone** (`kill(pid, 0)` says no such process; anything less certain is left in place and reported) **and** whose iceoryx2 namespace has no node registered in any registry the sweep covers: a state file is the whole namespace's name mapping, not one process's, so a namespace that is still in use keeps its file however long ago its first process died. The registries it covers are the one under your configured root path, the one under the compiled-in default root, and any registry one directory below either of them (which is where an isolated test root's registry lives). A process running under a hand-written `root_path` somewhere else entirely is invisible to that search, and its mappings are not protected. One refusal the sweep can never clear on its own is healed by this verb: a dead node's directory holding nothing but the `.port_tag` files of ports it had already deregistered (a publisher destroyed while one of its loaned samples had been leaked: the tag outlives the port, and a dead process removes nothing), which fails iceoryx2's final `rmdir` on every sweep forever and blocks the state-file reclamation with it. After the first sweep, `clean` removes those tags (only when the owning process is provably gone and the directory, re-listed at that instant, holds nothing else; anything else is refused and named) and then sweeps ONCE MORE, so the registry converges in the same run (a directory something else already emptied is reported as converged pending that sweep, not as a refusal). `--report-only` is look-first for BOTH reclaims: it lists the orphan port tags it would remove and the `.shm_state` files it would reclaim, removes neither, and skips the second sweep; it does **not** suppress the first dead-node sweep, which is the verb's main job (iceoryx2's own reclaim of a dead node's resources) and runs either way. **You should rarely need this**: `cerulion graph run` sweeps dead nodes at startup under a 2 s budget, and reclaims what it can at graceful EXIT under another 2 s, where the graph is already over, so nothing you are waiting on is delayed. A run killed with `SIGKILL` skips its own exit pass; the next run that ends gracefully picks that residue up, so a desk that runs graphs heals progressively without you doing anything. Reach for `clean` when you want it all cleared at once, or when a run reported that its budget ran out. |
| `cerulion trace inspect <dir>` | Read legacy publish-trace files (`trace_*.jsonl`, a JSON-Lines record of which topic published which sequence number when, with no payloads) and print a human-readable timeline. `--filter TOPIC`, `--limit N`, `--reverse`. No `cerulion` verb writes these files, so this verb only matters if you already hold some; see [Publish trace (legacy)](#publish-trace-legacy). **Not the recording bag**: `graph run --record` bags are standard **MCAP** (`.mcap`) and are read by `cerulion bag play` (with or without `--resim`), not this command. |
| `cerulion --verbose <SUBCMD>` | Bump logging to debug. |

### Port `SCHEMA` resolution (`node create` / `node modify`)

The `SCHEMA` argument of `-o` / `-i` / `-T` accepts qualified names
(`geometry_msgs/Vector3` or `geometry_msgs::Vector3`) and BARE names
(`Vector3`), resolved at the command boundary BEFORE anything is created
or edited (so a bare built-in name never scaffolds a broken
`use Vector3;` import):

- A bare name matching a **workspace schema** (`schemas/*.yaml`, by file
  stem or schema name) stays bare: workspace wins over built-ins, with a
  loud stderr `WARNING:` when it shadows a built-in name. (Making the
  workspace schema's generated type resolvable in the node crate is
  the workspace author's responsibility.)
- A name matching a **`.msg` store schema**
  (`schemas/<pkg>/msg/<Type>.msg`, e.g. one acquired by `ros2 attach`) is
  RECOGNIZED, bare or qualified, and **REFUSED for node ports**, with an
  error naming the store type and the remedy: a store type has no
  generated Rust type, so the scaffolded `use native_ros2_messages::…;`
  import could never compile. The store tier sits BETWEEN workspace YAML
  and built-ins (a same-named workspace YAML schema wins and scaffolds; a
  same-named built-in is outranked, so the refusal, not the built-in,
  answers). A bare name defined in **several store packages** is an
  ambiguity error listing every candidate. A bare name whose ONLY store
  package's qualified spelling a workspace YAML file ALSO declares
  (`pkg/Type` in both) is refused too, and so is that QUALIFIED spelling,
  which is the same collision named from the other side: no tier outranks
  another for a spelling you name. **The only remedy is to remove one
  of the two definitions** (naming one of them without removing the other is
  not supported). Until then neither spelling can type a port, be introspected,
  or be recorded under that name.
- **An ambiguous workspace spelling is REFUSED, naming every source.** The
  SPELLING surfaces (port resolution, `graph run`'s pre-build lookup and
  `schema info`) take that verdict from the one workspace lookup described
  under `graph run --no-validate` above (an entry name declared by several
  `schemas/*.yaml` files; a `<name>.yaml` beside another file's entry
  `<name>`; a YAML definition the `.msg` store also spells). Two further
  shapes bind nothing in the ADVISORY maps (serving, the recording
  wire-size map, the `graph run` hash map and the replay registry), which
  cannot error mid-run and so degrade loudly instead (`schema list` marks
  every row of such an entry name with the refusal in place of a hash): a
  `<name>.yaml` declaring several entries and none named `<name>`, and a
  `<name>.yaml` whose sole entry's own name is ambiguous. Those two still
  name exactly ONE file, so `schema info <name>` renders that file and a
  port may carry the stem; what no map can do is pick WHICH entry a channel
  labelled `<name>` carries, and each says so. No precedence picks a winner:
  declare each name once. `.msg` store types remain first-class
  everywhere a name/hash suffices: `graph validate`, recording, replay and
  `schema info`.
- A bare name matching exactly **one built-in** ROS 2 message resolves to
  its qualified form (`Vector3` → `geometry_msgs/Vector3`), printing a
  stderr note on every invocation; qualify the name to silence it.
- A bare name defined in **several built-in packages** (e.g. `Pose2D`) is
  an error listing every candidate; qualify to disambiguate.
- An **unknown** name is an error pointing at `cerulion schema list`.
- A **qualified** name (`pkg/Type` or `pkg::Type`) that names a `.msg`
  store schema is validated against the store (and refused for node ports,
  as above); any other qualified name passes through unchanged (an unknown
  one fails later, at compile time; `cerulion graph validate` is the
  existence gate).

### `--policy SPEC` (`node create` / `node modify`)

Sets the node's trigger policy in one place. Accepted SPECs:

| SPEC | Macro shape | Meaning |
|---|---|---|
| `period_ms=N` | `#[cerulion_node(period_ms = N)]` | Fire every N ms. |
| `sync_window_ms=N` | `#[cerulion_node(sync_window_ms = N)]` | Bounded sync: fire when all of the node's `#[input(trigger)]`-marked inputs receive within an N-ms window (Sync aligns ONLY trigger-marked inputs; a plain `#[input]` is a latest-value read that never gates the fire). |
| `external` | `#[cerulion_node(external)]` | Self-triggering **ingress / driver** node: watches a non-Cerulion signal (a device fd, a blocking SDK) and fires itself. Requires an `external_source()` method; see [External nodes (ingress / drivers)](#external-nodes-ingress-and-driver-policy). |
| `data_trigger=NAME` (or `trigger=NAME`) | `#[input(trigger)] NAME: T` | Fire on each message arriving on the input named `NAME`. The matching input must be declared in the same call (via `-i` or `-T`) or already exist on the node. |

`N` must be **> 0** for the time-based forms (`period_ms`,
`sync_window_ms`). Bare forms (e.g. `--policy period_ms` with no `=N`)
are rejected.

`unbounded_sync` has no `--policy` SPEC: it is **macro-source-only**. Write
`#[cerulion_node(unbounded_sync)]` on the struct by hand; the CLI rejects
`--policy unbounded_sync` as an unknown spec.

`-T SCHEMA NAME` is shorthand for `-i SCHEMA NAME --policy
data_trigger=NAME`; supplying both is accepted only when they name the SAME input (they are rejected when they disagree, and `-T` with a non-data `--policy` is always rejected).

> **Deadline watchdog:** there is no node-level `deadline_ms`
> trigger. "Fire on data, miss if no data within N ms" is expressed as a
> data trigger carrying a per-input watchdog,
> `#[input(trigger, expect_within_ms = N)] NAME: T`, which fires the node
> on arrival and counts a miss (via `expect_within_missed_count`) when no
> fresh data lands within N ms. See **QoS deadlines and miss counters** below.

Setting `--policy` on `node modify` strips any conflicting node-level args
(`period_ms` / `sync_window_ms` / `external`) before
writing the new one; the macro rejects mutually-exclusive combos at
compile time. `data_trigger=NAME` is carried by the `#[input(trigger)]`
field-attribute on `NAME`, not by a node-level macro arg; if `NAME`
already exists on the node, the engine promotes its existing
`#[input]` to `#[input(trigger)]` and clears the conflicting macro
args.

#### Defaulting rules for `node create` (when `--policy` is absent)

There is no "workspace default" policy keyword: the rules below
*are* the default behavior. The CLI applies them based on the input
count declared in the same `node create` invocation.

| Inputs declared | Default behavior | Why |
|---|---|---|
| 0 (no `-i`, no `-T`) | **error** | A source-only node can't be data-triggered; pick `--policy period_ms=N` or `--policy external` explicitly. |
| 1+ (one or more `-i`, no `-T`) | no policy attr is written, and **the scaffold does not build as created**: the macro refuses a node with no trigger (`no trigger policy: add #[input(trigger)] to a field, or specify period_ms or external on the node`) | The CLI will not guess which trigger you meant (a data trigger on one input, a sync window, a period). Declare one before `node build`: pass `-T` instead of `-i`, pass `--policy`, or run `cerulion node modify <TYPE> --policy data_trigger=<NAME>`. `node list` shows such a node with POLICY `-`. |
| Any input count with `-T NAME` | `data_trigger=<NAME>` | `-T` is itself an explicit policy declaration. |

### Optional SYSTEM dependencies (`node build`)

Before any of this, `cerulion node build` needs **your own** Rust toolchain: it
compiles the node with `cargo`, which the CLI does not ship, so `cargo` must be
on `PATH` (Rust **1.93+**, the MSRV, via [rustup](https://rustup.rs)) alongside a
**C linker** (`build-essential` on Ubuntu/Debian, the Xcode Command Line Tools on
macOS). If `cargo` is missing the build fails with a message naming rustup and
the linker, rather than a raw "No such file or directory" that reads like a lost
build artifact.

The node must also be built by the **same compiler that built your `cerulion`
binary**: the loader compares the two fingerprints and refuses a skewed node
(see the `cdylib` feature row under [Cargo features](#cargo-features)). The
pre-build PATH `rustc` probe only warns when its release differs from the
CLI's compiler, because Cargo may select another compiler through `RUSTC`, its
configuration, or a wrapper. The build continues; loading the resulting node
requires its full compiler fingerprint to match the host.

Some nodes need a **system** library: GStreamer, librealsense, CUDA. Making
that a hard dependency punishes everyone: `cargo build` then fails for every
contributor on every machine without it, including the ones building the other
nodes in your workspace.

Cargo cannot solve this itself. A build script **cannot enable a feature of its
own crate**: the feature graph is resolved before any `build.rs` runs, and
`cargo:rustc-cfg` sets a `cfg`, not a feature, so it cannot turn on an optional
*dependency*. There is no pure-Cargo way to say "link GStreamer if this machine
has it". So the decision lives one level up, in `cerulion node build`.

Declare it in the node crate's own `Cargo.toml`:

```toml
[features]
cdylib = []
gstreamer = ["dep:gstreamer", "dep:gstreamer-app"]
default = ["cdylib"]          # NOT gstreamer; see below

[package.metadata.cerulion.optional-system-deps.gstreamer]
pkg-config  = ["gstreamer-1.0", "gstreamer-app-1.0"]
summary     = "live camera capture (H.264 decode -> JPEG)"
without-it  = "the node has no capture source, so `cerulion graph run` refuses it at launch"
install.macos  = "brew install gstreamer"
install.debian = "sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
```

The table name is the **cargo feature** to enable, and it must be a feature the
crate actually has: an explicit `[features]` key, or an optional dependency
(cargo mints an implicit feature per optional dep). Declare as many as the node
needs.

Every value below is required and required means **non-blank**: an empty string
passes serde but renders a notice naming no capability, no consequence and no
command, which is the unactionable warning the requirement exists to prevent.

| Key | Required | Meaning |
|---|---|---|
| `pkg-config` | yes | Module names. **ALL** must resolve, or the feature stays off; linking half a library family is the breakage the probe exists to avoid. |
| `summary` | yes | What the feature provides, in user words. Appears in both the found and the missing notice. |
| `without-it` | yes | What happens at RUN time without it. Your node's own statement. |
| `install.<platform>` | yes, ≥1 | Platform key → install command. Keys are free-form; `macos`, `brew`, `debian`, `ubuntu`, `linux` are the ones the CLI prefers on the matching machine. |

`cerulion node build <TYPE>` then:

1. probes every listed module with `pkg-config`,
2. passes `--features <feature>` when **all** of them resolve,
3. otherwise builds WITHOUT it and says so LOUDLY, naming what is
   unavailable, what that costs at run time, and the install command.

The build still SUCCEEDS in case 3. A missing system library is not a build
failure; a **silent** capability loss would be.

**The feature must not be in `default`.** A gated
feature left in `default` makes the system library a hard prerequisite for the
plain `cargo build` again, and the probe buys nothing. `node build` warns
loudly when it finds one.

What the notice will and will not claim:

- A **malformed block** (a misspelled key inside it, a missing required
  field, an empty `pkg-config` list) is a hard build failure naming the
  manifest. It is never ignored: a typo that silently disabled the probe would
  ship the node without its capability while the build stayed green.
- A **blank required value** (`summary = ""`, a whitespace-only `without-it`,
  an empty `install.<key>` command, an empty module name) is a hard build
  failure naming the field. Presence is not content.
- A **table name that is not a cargo feature of the crate** is a hard build
  failure naming the manifest and listing the features the crate does have.
  Left unchecked this drift fails ONLY on a machine that HAS the library
  (there the probe resolves, `--features <name>` is passed, and cargo refuses),
  while every machine without it stays green. Green in CI, broken on the robot.
- A misspelled **table name** (`ceruleon`, `cerulion-deps`) cannot be a hard
  error (`package.metadata` is a namespace shared with every other tool), so
  it is a loud warning naming both spellings.
- A resolved module is a **build-time** fact. It proves the headers are there
  and the crate will link; it does not prove the feature's run-time components
  are installed (GStreamer's dev headers and its plugins are separate
  packages). The notice says so, and the node checks its own run-time
  requirements when it runs.
- If `pkg-config` **itself** is not installed, nothing was probed at all. The
  notice says that, and gives you pkg-config's install command; it does not
  claim your library is missing.
- On a platform whose exact key is not declared, the closest match is shown
  **labelled** (`[debian] sudo apt install …`) rather than presented bare as
  your machine's command. `target_os` cannot tell Debian from Fedora, and an
  unlabelled `apt` line on Fedora would be wrong.

---

## Workspace layout

```
my_robot/
├── Cargo.toml              # [workspace] members = ["nodes/*"]
├── rust-toolchain.toml     # the CLI's compiler, when a match is installed
├── .cargo/
│   └── config.toml         # IOX2_LOG_LEVEL=error, RUST_LOG defaults
├── .gitignore              # ignores .cerulion/ (the CLI's per-workspace state)
├── graphs/
│   └── perception.yaml     # one file per graph
├── nodes/
│   └── sensor/
│       ├── Cargo.toml
│       └── src/lib.rs      # #[cerulion_node] struct + #[cerulion_node_impl] impl (single source of truth for ports + trigger policy)
├── schemas/
│   └── Reading.yaml        # one file per schema
└── recordings/             # NOT scaffolded: created by the first `graph run`
    └── flashbacks/         #   (the Flashback window); `--record` bags land beside it
```

You shouldn't need to hand-edit `Cargo.toml` or `.cargo/config.toml`; the CLI keeps them in sync. Hand-edit `nodes/<type>/src/lib.rs`, `graphs/*.yaml`, and `schemas/*.yaml`. Port and trigger-policy metadata lives entirely in `src/lib.rs` (there is no `.cerulion.yaml` sidecar, and graph YAML carries no `policy:` block; the macro on the node is the single source of truth, and `cerulion node info` / `cerulion node list` parse the source directly).

---

## Node author API (the macros)

A node lives in `nodes/<type>/src/lib.rs` and looks like this:

```rust
use cerulion_core::prelude::*;
use native_ros2_messages::geometry_msgs::Vector3;
use native_ros2_messages::sensor_msgs::LaserScan;

#[cerulion_node(period_ms = 10)]                      // node-level trigger: 100 Hz period
struct SafetyController {
    #[input(depth = 1)]                               // field-level port; `trigger` here would
    scan: LaserScan,                                  // data-trigger the node (conflicts with `period_ms`)

    #[output]                                         // Vector3 = fixed-only schema (pub x/y/z: f64)
    linear_velocity: Vector3,

    #[output]                                         // also fixed-only, used for debugging
    debug: Vector3,

    // Non-port fields are normal user state.
    last_min_range: f32,
}

#[cerulion_node_impl]
impl SafetyController {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Read variable-length input fields via typed accessors (the read path):
        // `self.scan.ranges()` returns `&[f32]` over the loaned SHM payload: no copy.
        let min_range = self
            .scan
            .ranges()
            .iter()
            .copied()
            .filter(|r| r.is_finite() && *r > 0.0)
            .fold(f32::INFINITY, f32::min);
        self.last_min_range = min_range;

        // Direct field writes for fixed primitives (the write path via Deref):
        // `Vector3` is a fixed-only schema, so its Shm type Derefs to a struct with
        // `pub x/y/z: f64`. Each assignment lands directly in the loaned SHM slot.
        // A reading that is not finite or not positive is invalid and never counts;
        // with no valid reading at all the controller stops rather than cruises.
        self.linear_velocity.x = if min_range.is_finite() && min_range >= 0.5 { 0.3 } else { 0.0 };
        self.debug.x = min_range as f64;

        Ok(())
    }

    // Optional: init runs once before the first tick.
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        Ok(())
    }

    // Optional: shutdown runs once after the last tick.
    fn shutdown(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}
```

### `#[cerulion_node(...)]`: node-level attributes

Two categories of attributes: **trigger policy** (exactly one must apply, or infer from `#[input(trigger)]`) and **QoS** (orthogonal annotations, can stack).

**Trigger policy:**

| Attribute | Meaning |
|---|---|
| `period_ms = N` | Fire every `N` milliseconds. |
| `sync_window_ms = N` | Bounded sync: fire ONCE PER COMPLETE ALIGNED SET, in set order, when all of the node's `#[input(trigger)]`-marked inputs have a message within an `N`-ms window (Sync aligns ONLY trigger-marked inputs; a plain `#[input]` is a latest-value read that never gates the fire and HOLDS its last value across steps, exactly like a data-trigger node's non-trigger inputs). **Each trigger message is consumed by at most one set** (per-message FIFO consumption. This is the semantic, not a knob: the `latest` and `batched` alternatives are NOT expressible, so there is no `consume = ...` attribute to write), so a burst holding k complete sets yields k fires, each tick reading exactly ITS set's members. A sync attribute only means something with two or more `#[input(trigger)]` fields: with none it is a compile error, and with exactly ONE it compiles but is ignored (the node fires as a plain data trigger on that input, and the graph build says so with one `warn!` naming the node). Every trigger-marked port must be wired in the graph YAML: an unwired trigger port fails the graph build loudly (it would silently shrink the alignment set), and a sync node whose trigger set resolves empty is rejected by the scheduler with the cause + fix named. |
| `unbounded_sync` | Unbounded sync (loose AND): fire ONCE PER COMPLETE ALIGNED SET the moment every `#[input(trigger)]`-marked input has an unconsumed message, with no timing bound (plain `#[input]`s are latest-value reads that never gate the fire). **Each trigger message is consumed by at most one set**, and the set is the closest-in-time combination among the frames that have ARRIVED: it fires IMMEDIATELY rather than waiting for a possibly-nearer frame (the chosen trade: latency over retrospective optimality). Mutually exclusive with `sync_window_ms`. Needs two or more `#[input(trigger)]` fields, under the same rule as `sync_window_ms` (none is a compile error; exactly one compiles, is ignored, and warns at graph build). Every trigger-marked port must be wired in the graph YAML: an unwired trigger port fails the graph build loudly (same guard as `sync_window_ms`), and an empty trigger set is rejected by the scheduler with the cause + fix named. **Not recommended for control loops**: worst-case fire latency is the slowest publisher's inter-arrival interval, unbounded if it stops. |
| `external` | Self-triggering **ingress / driver** node: watches a non-Cerulion signal and fires itself on the live path. Requires an `external_source()` method returning an `ExternalSource` (`Fd` / `Blocking` / `HostDriven`); see [External nodes (ingress / drivers)](#external-nodes-ingress-and-driver-policy). |

(If none of these are set, you must mark exactly one input as `#[input(trigger)]`. With 2+ trigger inputs, you must pick `sync_window_ms` or `unbounded_sync` explicitly; the validator rejects the ambiguous combination.)

#### Per-set Sync delivery

A Sync node fires **once per complete aligned set, in set order**, and each
trigger message is consumed by **at most one set**, the same contract ROS 2's
`message_filters` gives you, and the same `fifo` semantic a data trigger already
has, applied to aligned sets. A burst that arrives between two fires does not
collapse: k complete sets yield k fires, and the k-th tick reads the k-th set's
members, not the freshest frame on each topic.

**Which frames make a set.** Sets are formed in arrival order per input (no
message is ever consumed out of order), and among the orderings that preserve
that, the members chosen are the ones with the smallest **spread**, the
`max − min` of the tuple's timestamps, which is the same quantity
`sync_window_ms` bounds. Two consequences worth knowing before you tune
anything:

* **Closest among ARRIVED, fired immediately.** A set is served as soon as one
  can be formed from frames that are already here. A frame that would have made
  a tighter set but has not arrived yet does not delay the fire. The trade is
  decided: latency over retrospective optimality.
* **Serving a whole backlog beats tightening one set.** While every trigger
  input still holds a second arrived frame, a complete LATER set exists too, so
  the alignment never spends a frame to tighten the current one. Tightening
  happens only where a partner has genuinely run out.

**Per-input counters. The two mean opposite things**, so read them together (how
any counter reaches you is under
[Reading the runtime's counters](#reading-the-runtimes-counters)):

| Counter | Meaning | Healthy value |
|---|---|---|
| `sync_closer_skip_count` | A nearer arrived member of the same stream was chosen. **The feature working.** Counted, and logged at `debug!` only. | Large and growing on any mismatched-rate node (~180/s on a 200/20 Hz pair) |
| `sync_unmatched_discard_count` | The frame was provably in NO set: a partner ran more than `sync_window_ms` past it. **Something is wrong.** Logged loudly: a flood-latched `warn!` (`sync discarded an UNMATCHABLE frame`) naming the node, the topic, the stamps and the window, then `sync is still discarding UNMATCHABLE frames` with the running `total_failures`. | 0 |

How a set's member is held: on a per-set node a head is a **member of the set being
formed**. It is written once, and it is released when the alignment pass FIRES that
set or, under `sync_window_ms` only, when that pass rejects it as unmatchable.
(`unbounded_sync` declares no window, so it has no unmatchable path at all; there,
only a fire releases a head.) Window tuning belongs to the UNMATCHABLE counter above,
which is the one the window governs. (The runtime keeps a third counter,
`sync_head_refusals`, for framework-internal callers that signal a per-set node
directly; a graph run by `cerulion graph run` never moves it, so it is not a
diagnostic for your node.)

A climbing UNMATCHABLE count has three fixes, in the order worth trying: widen
`sync_window_ms`; fix a stalled or skewed producer; or **raise the fast input's
`depth`**. That last one is not obvious and applies to graphs that look
perfectly healthy: a trigger input's queue is bounded by its declared `depth`
(default 10) with `drop_oldest` reclaim UPSTREAM of the alignment, so once the
fast:slow rate ratio exceeds that depth the intervening frames are evicted
before the matcher ever sees them, the fast head lands NEWER than the slow head,
and the slow frame is never matched. At the headline 200/20 Hz that ratio is exactly 10,
the default, with zero jitter margin, so declare `#[input(depth = 16)]` on the
fast port.

Under `#[input(backpressure = block)]` that failure mode does not exist, because
there is no eviction to cause it; the fast **producer** is deferred instead. An
under-provisioned depth then costs the producer's rate (loud, counted, lossless)
rather than an unmatched slow frame, so the remedy inverts: raise `depth` to
restore the producer's rate, not to stop the evictions.

A complete set that sits unfired (a burst that stopped at its per-step cap, or a
set held at a pre-fire defer) is a carried backlog: the live loop sizes its next
wake on it, so the set is served at the next boundary rather than waiting for a
new arrival.

**Not every Sync node gets this.** Per-set delivery needs the node's trigger
reads to go through the frozen-slot path, so it is what `#[cerulion_node]`
nodes get, in-process and cdylib alike. A node that cannot provide it keeps
the **legacy latest-per-set** semantic (it still fires once per
complete alignment, but its tick reads the FRESHEST frame on each trigger
rather than the members the alignment chose, and both skip counters stay 0
forever, so the depth-tuning advice above does not apply to it). Two ways in:
a legacy `--raw-ffi` node, or running with
`CERULION_DRAIN_DISCIPLINE=separate`. It is never silent: the graph build
emits one `warn!` naming the node, the reason and the fix.

**`expect_within_ms` on a Sync trigger reports the STARVED input, not the
flowing one.** A per-set trigger's arrival deadline is reset by every frame that
arrives on it, `sample(N)`-decimated frames included; the watchdog is a
PRODUCER-liveness surface and `sample(N)` is a consumer read policy, so a
healthy producer feeding a decimating input never trips it. And once an input's
frame has been chosen as a set's member it sits there until the set fires, so a
window that lapses while that member is HELD (because a partner starved, or
because the node's own `throttle_ms` / `block` gate is deferring a complete set)
is reported through `expect_within_backlogged_count()` (a `debug!` per window,
one `info!` per regime) and NOT counted as a miss. The starved partner has no
held member, so it is not suppressed and still trips: on a stalled pair, the
input that is late is the one the log names.

**Backpressure on a trigger input works, and the two policies do different
things to a set.** `#[input(backpressure = ...)]` is per-input and is honoured on
a Sync trigger exactly as it is on a data trigger; there is no degrade rung for
it, and declaring one on one trigger input says nothing about the others.

* **`sample(N)` decimates BEFORE matching.** The gate and the alignment window
  compose as a *pipeline*, never as a joint condition: the gate admits a
  subsequence of that input's arrivals spaced at least `N` ms apart by wire
  stamp, and only admitted frames are ever eligible to join a set. So a frame the
  alignment would have preferred can be gone before the alignment sees it:
  that is the policy working, not loss. Two consequences worth knowing. A gate
  drop counts in `backpressure_sampled_count(input)` and never in
  `sync_closer_skip_count(topic)`; the two accountings are disjoint. And a
  decimated read caps the per-step burst at one set: the backlog is still served
  in full and in order, one set per level boundary, rather than k-in-one-step.
  A RESUMED run re-opens the gate on whatever frame it sees first, so the
  admitted subsequence can shift for the rest of the run: on a dense stream it
  may share no frame with the recorded one, and a resumed `--verify` resim then
  reports a byte mismatch on every fire with nothing naming the cause. Whole-run replay is unaffected:
  the gate re-executes identically from the recorded frames. The gate's state
  is not part of the resume anchor, so a resumed resim cannot restore it.
  If `N` exceeds `sync_window_ms` the graph build emits one `warn!` naming both
  windows; it is not refused (two gated partners whose admitted frames land in
  phase align perfectly well at any `N`), but a stable offset between the
  admitted streams larger than the window will report every frame UNMATCHABLE on
  a graph whose producers are healthy.
* **`block` stays lossless end to end, and a HELD member counts as occupancy.**
  A frame the alignment has chosen sits in the input until its set fires, so it
  is unserved and it counts against the declared `depth`: the producer runs
  exactly `depth` frames ahead of the consumer, held members included, not
  `depth` plus whatever the matcher happens to be holding. The cost is
  the *starved-partner hold*: a `block` trigger input whose partner stops
  arriving holds its producer at `depth` unserved frames indefinitely, and the
  producer is a whole node, so its other outputs stall too. That IS the lossless
  contract: `block` promises the producer never outruns the consumer, and a Sync
  node that cannot complete a set is not consuming. The surfaces are
  `backpressure_block_fires_deferred_count` on the producer side and
  `expect_within_ms` on the starved partner. (While a member is held the input is
  re-offered rather than re-drained, so the CONSUMER-side `BackpressureEvent`
  goes quiet in that regime; the producer's counter is the live one.) Mixed
  topics degrade exactly as they do for a data trigger (see
  [Multi-consumer fanout](#multi-consumer-fanout-block-degrades-gracefully)),
  and a degraded input is back on `drop_oldest`, so the `depth` precondition
  above applies to it verbatim.

**A publisher clock RESTART re-bases the node.** Within one run a single
writer's wire stamps only move forward, so a stamp that jumps BACKWARD by more
than `sync_window_ms` in an input's own stream is evidence its publisher
restarted (a rebooted robot behind a mirror, a restarted worker, a re-attached
bridge). On that evidence the node discards the other inputs' held frames
(counted per input as `sync_epoch_reset_discards`, announced once) and re-bases
onto the new clock. Without it the node would never fire again: a held frame from
the epoch that ended is the tuple's MAXIMUM, and alignment can only ever evict
from the bottom. Nothing is wrong with your producers when you see this line and
there is nothing to fix. A `multi_publisher_topics` trigger input mixes clocks
and can trigger it spuriously; the cost is bounded at counted discards.

**Validation limits of per-set backpressure.** It is
validated in-process, not across the cdylib FFI: `sample(N)` and `block` on a Sync
trigger input of a DYLIB-loaded node ride the same `sync_head_op` seam the
in-process path drives and the declaration is carried across the FFI (ABI v8), but
that combination is not validated end to end. And a `block`+per-set run is validated
byte-identical across two live runs, not across a RECORD→REPLAY cycle: the
producer's defer decisions re-derive from occupancy, which re-derives from the
recorded frames, so replay should reproduce them, but that is not
validated.

**Non-trigger inputs are unaffected.** Non-trigger `#[input]`s are
latest-value, held across steps, and frozen once per step, so
every fire of one step's burst reads the same frozen context bytes while each
fire reads its own set's trigger members.

#### External nodes: ingress and driver policy

`#[cerulion_node(external)]` is for a **driver**: a node with no upstream Cerulion topic that watches a **non-Cerulion signal** (a device fd, a blocking SDK) and **self-triggers** the graph. It MUST define one method (a compile error otherwise):

```rust
fn external_source(&mut self) -> ExternalSource { ... }
```

`external_source()` is queried **once** at live-loop startup (after `init()`), on the **LIVE path only**, never during a replay (`cerulion bag play --resim`). The wake is record-only: it changes *when* a step runs, never *what* fires, so replay stays byte-identical to live (Principle #7). `ExternalSource` (re-exported in the prelude) has three variants:

| Variant | Meaning |
|---|---|
| `Fd(RawFd)` | A pollable device fd (v4l2, socket, evdev, serial). Attached **non-owning** to the live WaitSet: the runtime never reads or closes it; the node owns the device and drains it in `tick()`. **Level-triggered:** an fd left readable re-fires the node every step, so drain to `EAGAIN` (open `O_NONBLOCK`; a blocking read in `tick()` stalls the whole graph). Close it only in teardown; do **not** close a handed-over fd out of band (`EBADF` hazard). |
| `Blocking(Box<dyn FnMut() -> bool + Send>)` | For fd-less SDKs whose only wait is a blocking call. Driven on a helper thread + a doorbell; each `true` return rings the node (N rings before a step coalesce to one fire). Use a **bounded** internal timeout so the thread can observe shutdown between calls; a closure panic is caught → the source is poisoned **loudly** and stops waking the node. |
| `HostDriven` | "No self-source." A placeholder: nothing in a `cerulion graph run` can fire such a node, so the run refuses it (below). It is what the `--policy external` scaffold returns so the crate compiles before you write the real source. |

**`cerulion graph run` REFUSES any external node that is provably inert at launch** with a sticky, aggregated error naming every offender **with its reason** and the fixes. The reasons: `host-driven` / `no source` (the node declares no source of its own, so nothing in the run can fire it), `invalid fd` / `duplicate fd` (the declared `Fd` cannot be watched by the live loop), `poisoned node`, `doorbell failed` (the `Blocking` doorbell could not be established), and `fd above select limit` (a live `Fd` `>= FD_SETSIZE` with the monitor-wait park OFF, where the select-backed WaitSet cannot watch it; fix: enable the monitor-wait park, which polls via `poll(2)` with no fd-number ceiling, OR shrink the graph so the fd is minted below `FD_SETSIZE`). All are discovered at collect time, before the loop. Fix each named node: give it a working `Fd` / `Blocking` source and resolve any listed fd/doorbell problem. Under `--time-source virtual` (the deterministic poll loop) **every** external node is inert, because nothing watches their fd/doorbell, so the run is **refused at startup** with reason `virtual time` (fix: re-run with `--time-source real`), not merely warned. The refusal text also mentions driving the graph from an embedding host; that is a framework-internal seam (see the top of this document), not a user path.

```rust
#[cerulion_node(external)]
struct Camera {
    #[output] image: Image,
    // ... device state ...
}

#[cerulion_node_impl]
impl Camera {
    fn external_source(&mut self) -> ExternalSource {
        let fd = open_device();          // v4l2 / socket / evdev / serial
        ExternalSource::Fd(fd)           // runtime watches it, non-owning
    }
    fn tick(&mut self) -> Result<(), NodeError> {
        // drain the device to EAGAIN, fill `self.image`; returning Ok publishes it
        Ok(())
    }
}
```

Two runnable driver workspaces show the two real sources: `examples/v4l2_camera` is a complete V4L2 camera driver (hand-rolled MMAP-streaming FFI, `ExternalSource::Fd`, drain-and-publish in `tick()`), and `examples/realsense` drives an Intel RealSense through librealsense2, which has no fd to watch, so it hands the runtime an `ExternalSource::Blocking` wait on a helper thread and copies each frameset into the loaned slots in `tick()`.

`cerulion node create <type> --policy external` scaffolds an external node **with a `HostDriven` `external_source()` stub** so it compiles as scaffolded; replace the body with a real `Fd` / `Blocking` source. Toggling `external` ON via `node modify` deliberately does **not** inject the method: the macro's compile error tells you exactly what to add.

**Non-trigger (latest-value context) inputs.** A plain `#[input]` that is *not* the node's trigger (every input on a `period_ms` / `external` node, a non-trigger input alongside a `data_trigger`, or a plain `#[input]` on a `sync_window_ms` / `unbounded_sync` node (sync aligns only `#[input(trigger)]`-marked ports)) is a **latest-value context** read: in the tick body it yields the most recent message delivered on that topic, **held across steps**. Until that input has delivered at least once, the tick is a **no-op**: it produces no output (Cerulion never fabricates a default on genuinely-absent data). After the first delivery, the held value is replayed on every later step until a newer message arrives, so a slow / latched context source (e.g. a 1 Hz map) does **not** starve a faster consumer. The held value is a pure function of past deliveries, so replay is bit-identical to live. *Staleness:* the hold serves the last value indefinitely, so a producer that **stalls without disconnecting** gives no signal on its own; add `#[input(expect_within_ms = N)]` to count/warn on a missed inter-arrival (the watchdog is NOT reset by a held replay, so it still trips while the stale value is being served). *Limitations:* the cross-step hold is provided by **macro nodes**, in-process AND cdylib (a macro cdylib exports the `cerulion_node_{set_,}snapshot_inputs` FFI pair, so its non-trigger inputs hold across steps too; pinned for Period nodes by `cdylib_non_trigger_hold_test` and for Sync nodes by `cdylib_sync_nontrigger_test`); raw-FFI / earlier cdylibs lacking the symbols read live (no hold); and a `block`-backpressure non-trigger input also reads live (it is producer-paced and excluded from the step-boundary snapshot). *External sources:* a macro node holding an **external / absolute-source** input as latest-value context provisions that topic's service with `subscriber_max_borrowed_samples = 3`; if a foreign publisher already created the service at the iceoryx2 default (2), the graph build fails loudly (start the holding graph first, or have the foreign side provision 3).

**QoS (orthogonal to trigger policy):**

| Attribute | Meaning |
|---|---|
| `tick_within_ms = N` | Per-node tick-execution deadline. The scheduler wraps `tick()` with `Instant::now()` timing; if the callback's wall-clock duration exceeds `N` ms, the node's `tick_within_missed_count` increments and a `tracing::warn!` fires with structured `node_id`, `tick_within_ms`, `elapsed_ms` fields. Stacks with any trigger policy. |
| `throttle_ms = N` | **Producer rate cap**. The scheduler defers the node's tick while `now - last_fire < N` ms, capping how often the node fires regardless of its trigger. Stacks with every trigger **except** `period_ms` (period already pins the rate; combining the two is rejected at compile time). Composes with input-side `block` backpressure: the tick is deferred if **either** gate fires. Distinct from `#[input(backpressure = sample(N))]`, which decimates *reads* on the subscriber side. `N` must be > 0. **The live loop waits TO the throttle deadline**: a throttled node with data waiting is due-NOW by every other signal, so a loop that did not know the window would wake at its 1 ms floor for the whole window and decide to defer again each time: on a 100 ms cap, ~100 wakeups per window. The scheduler is told the window and reports the remaining time instead. This affects only WHEN the loop wakes, never what fires. |

**Determinism opt-outs (bare flags):**

| Attribute | Meaning |
|---|---|
| `allow_non_deterministic` | Suppress the determinism lint below **entirely** for this node. Declares that this node is allowed to be non-deterministic, so a recording of it cannot be re-executed byte-for-byte. |
| `uses_live_io` | Suppress only the lint's **IO-class** rows. The narrower opt-out: a node that genuinely reads the filesystem still gets the clock and thread rows enforced. |

That is the COMPLETE `#[cerulion_node(...)]` surface: `period_ms`,
`sync_window_ms`, `unbounded_sync`, `external`, `tick_within_ms`,
`throttle_ms`, `allow_non_deterministic`, `uses_live_io`. Anything else is a
compile error listing these.

#### The determinism lint

Principle #7 says a replay is byte-identical to the live run, and the commonest
way to break it is one line in a tick body. So `#[cerulion_node_impl]` walks
your impl block for calls that cannot replay and **refuses to compile** three
of them:

| Call | Write instead |
|---|---|
| `Instant::now()` | `self.now_ns()`, the clock the scheduler is running on, so it replays identically. (`tokio::time::Instant::now()` is caught by the same rule.) |
| `SystemTime::now()` | `self.now_ns()`, or `self.real_ns()` for a duration measured against this machine's hardware clock. Note neither is a CALENDAR clock; both count from an arbitrary origin, so neither converts to a date. If you genuinely need wall-clock date-and-time there is no replay-safe way to read it: keep the call and declare `allow_non_deterministic`. |
| `thread::spawn(...)` | Split the work across nodes; the runtime already runs them in parallel where the graph allows it. An unmanaged thread's interleaving is not reproducible. |

A further set is declared warn-class in the same table: `std::fs::read_dir`
(IO-class), `std::env::var`, `rand::thread_rng`, `rand::random`,
`std::thread::sleep`, `std::process::id` and `std::thread::current().id`.
**Nothing diagnoses these**: stable Rust gives a proc macro no way to
warn, and no CLI-side check reports them either, so
writing one of them compiles silently. Each costs replay determinism just
as surely as the three above, so treat the list as a rule to follow rather than
one the compiler enforces. `std::env::var` has a direct replacement:
`ctx.env` / `ctx.env_str` read the frozen build-time snapshot instead of live
`std::env`, so they replay identically; see
[reading configuration off the `NodeContext`](#inside-init-reading-configuration-off-the-nodecontext).

The match is on the **last two path segments** of a called path, so both
`std::time::Instant::now()` and a bare `Instant::now()` under a `use` are
caught. It deliberately accepts one false positive: your own type named
`Instant` with a `now()` method matches too; the opt-outs above are the escape
hatch. Method calls on a value (`self.timer.now()`) are never matched.

### `#[input(...)]` field attribute

Declares an input port. Field type is the schema marker (e.g. `LaserScan`).

| Inner attribute | Meaning |
|---|---|
| `trigger` | This input is the trigger source for Data-trigger nodes. |
| `depth = N` | Queue depth in messages. Default: 10 (`DEFAULT_CONSUMER_DEPTH`). **Maximum: 64** (`MAX_CONSUMER_DEPTH`): every unit of depth commits a full `max_slice_len`-sized SHM slot, so deeper queues reserve pathological amounts of shared memory (64 × 128 MiB = 8 GiB apparent for an `Image`-class topic; apparent, not resident: the pool is lazy/demand-paged, but the reservation still counts against `/dev/shm` `size=` and address space). Rejected at compile time (macro) and at graph-load (topology validation). Applies identically to dylib-loaded nodes; the declaration crosses the cdylib FFI (ABI v8). 64 in-flight messages is the cap; a consumer that needs more, whether from a burst, a transient stall or a sustained rate it cannot keep up with, wants a backpressure policy rather than a deeper queue. |
| `backpressure = drop_oldest \| sample(N) \| block` | What to do when the subscriber's queue would overflow (or, for `sample(N)`, when reads arrive faster than one per `N` ms). Default: `drop_oldest`. Zero-copy: no Cerulion-side buffer, no copy. Applies identically to dylib-loaded nodes, because the declaration crosses the cdylib FFI (ABI v8). See **Backpressure policies** section below for runtime semantics + counter surface + `#[on_event]` callbacks. On a **per-set-capable** Sync node's `#[input(trigger)]` port both `sample(N)` and `block` are honoured with no degrade; see [Per-set Sync delivery](#per-set-sync-delivery) for what each does to a SET. A Sync node that is NOT per-set-capable (a legacy `--raw-ffi` node, or any node under `CERULION_DRAIN_DISCIPLINE=separate`) keeps the legacy latest-per-set path, and there these policies have a different shape: the gate sees only the survivor of a drain-to-latest rather than every frame, and the block mirror decrements at tick-body-read time rather than at slot exit. The build says which path a node is on, loudly, once. |
| `expect_within_ms = N` | **QoS subscriber-side deadline.** Expected inter-arrival interval on this input. If `N` ms elapse without new data, the node's `expect_within_missed_count` increments and a `tracing::warn!` fires with structured `node_id`, `input`, `expect_within_ms`, `elapsed_ms` fields. Independent of `trigger`: input deadlines are orthogonal to firing policy. `N` must be > 0. **A window that elapses while the input's data is sitting unserved is BACKLOG, not silence**: on a Data node's `trigger` input with signalled arrivals unserved, and on a per-set Sync node's trigger input whose frame is already the aligned set's member, the node is behind (its own `throttle_ms` cap or a `block` gate deferred the fire), so the window is reported (`expect_within_backlogged_count`, plus a `debug!` per window and one `info!` per regime) and *not* counted as a miss; the cost is that a producer which dies mid-backlog is detected only once the backlog drains, plus at most one more window. **That bound assumes the held head is eventually consumed, and the exception below is specific to the Unified drain discipline** (the default for macro data-trigger nodes; forced-Separate, `block`/`sample` and drain-ineligible cdylib paths use independent trigger/body subscribers, never re-offer a held body head, and so keep the one-window bound): under Unified, a COLLAPSED READ CHAIN, a latest-value context input declared before this trigger whose first message never arrives (the pre-first-delivery wait), re-offers the held head at every level boundary, so the backlog never drains and the watchdog stays in BACKLOG indefinitely, never reporting the dead producer; the diagnostic for that state is the held-head `warn!` (fired after a bounded streak of consecutive re-offers naming both causes and how to tell them apart), plus a climbing `expect_within_backlogged_count`. |

That is the COMPLETE `#[input(...)]` surface: `trigger`, `depth`, `backpressure`, `expect_within_ms`.

> **Not accepted:** `fifo` / `lifo`, `max_age_ms = N`, and `filter = "fn_name"` are not input attributes. Writing one is a compile error: the generic unknown-attribute message, which names the SUPPORTED set (`trigger`, `depth`, `backpressure`, `expect_within_ms`). None of them (`filter`, `max_age_ms`, and queue policy) is supported.

### `#[output(...)]` field attribute

Declares an output port. Field type is the schema marker.

| Form | Meaning |
|---|---|
| `#[output]` | The canonical form, for EVERY schema, fixed-only or variable. Nothing to declare per field: assignment resolves fixed vs variable at compile time (see below). |
| `#[output(promise_within_ms = N)]` | **QoS publisher-side commitment.** Promise to publish on this output every `N` ms. If `N` ms elapse without a publish, the node's `promise_within_missed_count` increments and a `tracing::warn!` fires. `N` must be > 0. |

That is the COMPLETE `#[output(...)]` surface: bare, or `promise_within_ms = N`.

> **Not accepted:** the `#[output(data, encoding)]` variable-field lists and `#[output(complex(header))]` are not output attributes. Variable-field assignment needs NO declaration. Every `self.<port>.<field> = expr` is rewritten to a uniform fallible write (under the hood: a codegen-emitted `__cer_assign_<field>(…)?` shim on the generated schema type resolves fixed-vs-variable at compile time). Writing one of the removed forms is a compile error that says `#[output]` takes no field list and tells you to assign the field in the node body instead.
>
> **Reserved prefix:** schema field names beginning with `__cer` are reserved for this generated plumbing and rejected at codegen time.

### Inside `tick()`

| Pattern | Effect |
|---|---|
| `self.<port>.<fixed_field>` (read) | Reads directly from shared memory via Deref. Zero copy. |
| `self.<port>.<var_field>()` (read) | Variable-field reads are ACCESSOR METHODS: `self.scan.ranges()` returns `&[f32]` borrowed from SHM; string fields return `Result<&str, WireError>`. The un-parenthesized `self.<port>.<var_field>` is a write-only marker: reading it compiles but yields nothing useful and fires a deprecation warning pointing at the accessor. |
| `self.<port>.<field> = expr` (output) | ONE rule for fixed AND variable fields: macro-rewritten to the fallible `__cer_assign_<field>(&expr)?` shim, which writes directly into shared memory (zero copy; variable fields do one boundary memcpy from `expr`). The `?` propagates `TransportError` as `NodeError::Transport`. |
| `self.<port>.<var_field>.fill_from(producer)?` (output) | **Zero-copy producer-callback write.** Macro-rewritten to `__cer_fill_from_<var_field>(producer)?` on the SHM type. Producer (closure, `SliceSource`, or `impl FillFrom`) receives `&mut [T]` directly into the iceoryx2-loaned region; no intermediate `Vec`. See [`FillFrom`](#fill_from-zero-copy-producer-writes). |
| `self.<non_port_field>` | Plain Rust state. |
| `self.now_ns()` | Active-source time in ns; **determinism-safe** (reads the graph's active clock: Real live / Virtual replay / External). The read node logic should use. |
| `self.real_ns()` | Raw hardware monotonic ns; always-on, **NON-deterministic** (benchmarks / explicit real-time only; not replay-safe). |
| `self.virt_ns()` | `Some(ns)` only under `VirtualClock` (replay/test); `None` otherwise. |
| `self.ext_ns()` | `Some(ns)` only under `ExternalClock` (external time master); `None` otherwise. |
| `self.request_shutdown()` | Signal the runtime to drain & exit gracefully. |
| `?` | Standard `Result<>` propagation. `TransportError` and `std::io::Error` convert to `NodeError` automatically. |

Misuse shapes are diagnosed at compile time:

| Shape | Diagnostic |
|---|---|
| `self.<port>.<var>[i] = x;` / `self.<port>.<var> += expr;` | Targeted error: "…unsupported on variable-length fields. Replace the whole value with `self.<port>.<var> = expr`, or write incrementally with `self.<port>.<var>.fill_from(\|buf\| …)`." |
| `let x = self.<port>.<var>;` / `&self.<port>.<var>` | Compiles, but the value is a useless write-only marker; a deprecation warning points at the `<var>()` reader accessor. |
| `self.<port>.<var>.push(x);` and other non-`fill_from` methods | Rustc "no method" error naming the generated marker type; use `=` or `fill_from`. |
| `self.<port>.<field>` inside a macro argument (`tracing::debug!`, `format!`, …) | Targeted error: the rewriter does not descend into macro token streams; hoist it: `let v = …; tracing::debug!(…, v);`. |

**Helper methods that write port fields must return `Result`.** Every rewritten assignment carries `?`, and a `()` method can never use `?`; the macro rejects it with a targeted error instead of rustc's generic E0277. Before/after:

```rust
// REJECTED: `()` helper writing a port field
fn set_speed(&mut self) { self.cmd_vel.x = 0.3; }

// OK: return Result and propagate at the call site (tick unchanged otherwise)
fn set_speed(&mut self) -> Result<(), NodeError> { self.cmd_vel.x = 0.3; Ok(()) }
// in tick(): self.set_speed()?;
```

**The same rule applies inside closures.** A port write inside a closure
that cannot use `?` (e.g. `for_each`, `map`) fails with rustc's E0277:
the injected `?` needs a `Result`-returning context. Two remedies:

```rust
// REJECTED: `()`-returning closure writing a port field
detections.iter().for_each(|d| { self.out.count = d.id; });   // E0277

// Remedy 1 (hoist): compute in the closure, write outside it.
let last = detections.iter().map(|d| d.id).last().unwrap_or(0);
self.out.count = last;

// Remedy 2: use a fallible combinator; the closure returns Result.
detections.iter().try_for_each(|d| -> Result<(), NodeError> {
    self.out.count = d.id;
    Ok(())
})?;
```

### Inside `init()`: reading configuration off the `NodeContext`

`init(&mut self, ctx: &mut NodeContext)` hands you the node's context. The part
of it you reach for routinely is configuration:

| Pattern | Effect |
|---|---|
| `ctx.env_str("KEY", "fallback")` | Read a string setting, returning the fallback when the key is absent. |
| `ctx.env("KEY", default)` | Read and parse a setting as any `FromStr` type. Returns `default` when the key is absent **or** the value doesn't parse, and an unparseable value fires a `tracing::warn!` carrying the key, the raw text and the target type, so a typo'd number shows up in the log instead of silently becoming the default. |

**Both read a snapshot of the environment taken when the graph was built, never
live `std::env`**, which is what makes them replay-safe. Three consequences,
each a contract the runtime holds you to:

- The value a node reads on its first tick is the value it reads on every later
  tick. Nothing done to the process environment after the graph was built
  reaches it; setting the variable and removing it both leave the node reading
  what was there at build time.
- A key unset at BUILD time returns the default forever; exporting it afterwards
  does not reach the node.
- There is no live fallback anywhere. A context built outside a graph runtime
  carries an empty snapshot, so every key returns its default rather than
  quietly reading the real environment.

Nodes therefore read the configuration captured for their own run, and a resim
restores the recording's captured configuration, so a recording re-executes
against the same values. Calling `std::env::var` in node code bypasses
all of it; see [the determinism lint](#the-determinism-lint).

### Only outputs you write are published (lazy-loan)

**An `#[output]` port your tick never writes is never loaned, never
published, and never discarded: it's a zero-traffic non-event.** The macro
loans an output's shared-memory slot ON THE FIRST WRITE to it (via any
write form: `self.out.f = …`, `self.out.set_…(…)`, `self.out.…fill_from(…)`,
a nested-field write, or a helper that writes it), not at tick start. So a
SPARSE writer, a node that leaves some outputs untouched on a given tick
(a bridge whose empty-queue tick writes nothing; a multi-output node writing
a different subset each tick), pays nothing for the outputs it skips:

- A skipped FIXED-schema output ships no frame at all (a zero-default frame every
  tick would be fabricated data).
- A skipped VARIABLE-schema output raises no loud "dropped without
  writing all declared variable fields" discard error on an empty tick
  (without lazy loan a sparse writer would log one such error per empty tick).

A per-skipped-port `tracing::trace!` (off by default) surfaces which outputs
a tick skipped for on-demand debugging; there is no counter (a healthy sparse
writer would grow it unboundedly).

> **A bare READ of an output field also counts as a touch**: `let cur =
> self.cmd.x;` on an `#[output]` port routes through the same get-or-loan
> receiver, so it LOANS the slot, just as a write would. This is intentional
> (a rare bare use loans on demand). What
> happens next depends on the schema:
>
> - **Fixed-only output** (the `cmd.x` example): on an `Ok` tick the loaned
>   slot PUBLISHES: a zero-default frame you did not mean to send.
> - **Variable-schema output**: the loan wrote no variable fields, so it hits
>   the all-variables gate and is DISCARDED with the loud once-per-regime
>   "dropped without writing all declared variable fields" error: no frame at
>   all, and an error every regime.
>
> Either way it is a bug. Keep per-tick state in your struct fields and read
> your INPUTS; do not read your OUTPUT fields for state.

**A tick that returns `Err` publishes NOTHING.** Every output the
tick DID loan (i.e. wrote to) before the error is released back to shared
memory without sending, whether it wrote some fields or every field (staged
nested writes included; staging is dropped without flushing). This covers
"complete-write-then-Err" for variable schemas. Framework-side failures inside
the tick get the same treatment: an input frame that fails schema validation
(or any other transport error surfaced through the tick) discards the loaned
outputs identically. Under the hood a loaned output is publish-DEFERRED from
the moment the write loans it, and only a fully successful tick arms it, so
ANY early exit (including a panic) publishes nothing, by construction.
**"Fully successful" means the tick BODY genuinely ran and returned
`Ok(())`**, not merely that the node's `tick()` entry point returned without
error. A no-op tick (the "Non-trigger (latest-value context) inputs" case
above: an undelivered non-trigger input collapses the WHOLE tick before the
body ever runs) never writes anything either, so, like a skipped output,
nothing loans and nothing publishes.

- A LOANED-but-INCOMPLETE output (you wrote SOME, not all, of a variable
  schema's fields, then the tick succeeded) still discards LOUDLY: the
  "dropped without writing all declared variable fields" `error!` fires
  (flood-latched once per regime). Lazy-loan changes when the loan
  happens, not this partial-write safety net.
- The tick error itself is the loud signal: it is returned to the runtime,
  logged, and the fire is still recorded (`fire_count` counts failed ticks).
  Each discarded loaned output additionally emits a `tracing::debug!`
  breadcrumb naming the topic.
- No discard-noise on the error path: a failed tick does not ALSO fire the
  unwritten-variable-field error or a staged-flush error; the discard
  preempts both.
- Publishing happens when the tick's loaned output proxies finalize at tick
  end; if a frame was already sent before the error, it stays sent; errors
  never un-send.
- The wire `timestamp_ns` is stamped at loan time, which under lazy-loan is
  the port's first-write time (not tick-start). This is replay-safe: the
  gating clock does not advance mid-step in deterministic runs, so every write
  in a tick reads the same time.

#### Fieldless outputs (heartbeats/pulses): `emit()`

A **zero-field** output schema (`std_msgs/Empty` and any other message with
no fields) has no field to write, so lazy-loan gives it nothing to trigger
the loan. Publish one with the explicit `emit()` gesture:

```rust
use native_ros2_messages::std_msgs::Empty;

#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct Heartbeat {
    #[output]
    pulse: Empty,
}

#[cerulion_node_impl]
impl Heartbeat {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.pulse.emit()?;   // publish one Empty frame this tick
        Ok(())
    }
}
```

`emit()` follows the exact same rules as any other write: calling it loans the
port (the publish intent), and the frame ships when the tick returns `Ok(())`.
A fieldless output you DON'T `emit()` on a tick is a zero-traffic non-event,
exactly like an unwritten fielded output, so a conditional `if beat {
self.pulse.emit()?; }` publishes only on the ticks you choose.

`emit()` exists ONLY on zero-field schemas. Calling it on an output that has
fields is a compile error (no such method); those ports publish by writing a
field.

### `fill_from`: zero-copy producer writes

For sources that produce bytes (or typed elements) into a destination buffer (cameras, codecs, file ingest, network reads), `fill_from` hands the producer the SHM-loaned `&mut [T]` directly. The framework handles loan + commit; the producer just writes.

```rust
use cerulion_core::prelude::*;          // FillFrom + SliceSource
use native_ros2_messages::sensor_msgs::Image;

#[cerulion_node(period_ms = 33)]
#[derive(Default)]
struct CameraNode {
    #[output]
    image: Image,
    // Your device handle (any type with a `Default`). A handle is rebuilt, not
    // captured, so say so: without this attribute the state derive refuses the
    // field at compile time. See `#[derive(CerulionState)]` below.
    #[cerulion(reconstruct)]
    driver: MyCameraDriver,
}

#[cerulion_node_impl]
impl CameraNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        const WIDTH: u32 = 1920;
        const HEIGHT: u32 = 1080;
        const BYTES_PER_PIXEL: u32 = 3;              // rgb8

        self.image.width = WIDTH;
        self.image.height = HEIGHT;
        self.image.encoding = "rgb8";
        self.image.step = WIDTH * BYTES_PER_PIXEL;   // bytes per row
        self.image.header.frame_id = "cam0";         // nested write, see below
        self.image.header.stamp.sec = 0;

        // Closure form: producer writes directly into SHM. A packed frame is
        // `height * step` bytes; a shorter fill describes fewer rows than
        // `height` claims.
        self.image.data.fill_from(|buf: &mut [u8]| {
            let n = self.driver.read_frame_into(buf)
                .map_err(|e| TransportError::NodeError {
                    node_id: "camera".into(),
                    reason: e.to_string(),
                })?;
            Ok(n)  // bytes written; framework truncates the loan to N
        })?;
        Ok(())
    }
}
```

Three ways to satisfy `fill_from`:

| Producer form | Use when |
|---|---|
| Closure `FnMut(&mut [T]) -> Result<usize, TransportError>` | Inline write logic, ad-hoc adapters |
| `SliceSource::new(slice)` | You already have a `&[T]` (drains progressively on repeated calls) |
| Custom `impl FillFrom<T> for MyStruct` | Reusable driver shims (`V4l2Camera`, `MyRealsenseDriver`, …) |

Key semantics:

- **Reserves remaining capacity**, calls producer, then truncates the cursor + offset-table entry to the actual elements written. The producer can write fewer elements than the loaned region; the framework finalizes the size.
- **Producer errors propagate** via `?` as `TransportError`. The cursor is rewound and the field is NOT marked written, so `OutputProxy::Drop` refuses to publish a partial frame.
- **Producer panics** propagate via Rust unwind; the publish gate stays closed (same end state as `Err`).
- **Defensive clamp**: if a buggy producer returns `written > dst.len()`, the framework clamps to `dst.len()` before committing.
- **`T` defaults to `u8`**. For typed-array fields (`DynamicArray<f32>` etc.), the producer closure takes `&mut [f32]` directly: `self.imu.samples.fill_from(|buf: &mut [f32]| { ... Ok(n) })?`.
- **String fields**: producer hands raw `&mut [u8]` and is responsible for valid UTF-8. The reader emits `WireError::InvalidUtf8` on misuse.

### Nested field writes (leaf-assignment sugar)

Write into a nested sub-message by leaf assignment, at any depth, the same `self.<port>.<field> = expr` sugar, extended through the dots:

```rust
// std_msgs/Header is Image's `header` field; Header has `stamp` (a
// builtin_interfaces/Time) and `frame_id` (a string).
self.image.header.frame_id = "cam0";   // variable leaf, 1 level of nesting
self.image.header.stamp.sec = 5;       // fixed leaf, 2 levels of nesting
self.imu.orientation.x = 1.0;          // fixed nested (Quaternion), 1 level
```

Any nested write is macro-rewritten to a fallible closure chain
(`__cer_<port>.__cer_with_nested_<f1>(|v| … v.__cer_assign_<leaf>(…))?`), so
the **same fallibility rule applies**: each nested write carries `?`, and a
helper that writes nested port fields must return `Result` (a `()` helper is
rejected with the same targeted error as flat writes). `fill_from` composes
through the nesting too: `self.image.header.frame_id.fill_from(producer)?`.

**`with_<field>`: the closure alternative.** For several writes into one
nested field, or to keep a nested writer in scope, use the generated
`with_<field>` closure API:

```rust
self.image.with_header(|h| {
    // Inside the closure `h` is the nested writer directly (NOT `self.<port>`),
    // so use its setter methods for variable leaves and direct assignment for
    // fixed leaves (the `self.<port>.<f>.<leaf> = …` sugar is only rewritten
    // outside the closure).
    h.set_frame_id("cam0")?;
    h.stamp.sec = 5;
    Ok(())
})?;
```

Repeated `with_<field>` / sugar calls on the SAME field **accumulate** (they
resume the staged state: the second call sees the first's writes). This
holds at every depth: a nested field's own staged sub-fields (e.g. the
`header` inside a nested `joint_trajectory`) persist across separate chains
within one tick and flush together, bottom-up, at publish.

**Reads are NOT sugared.** `let s = self.image.header.stamp.sec;` reads
through Deref like any fixed read; a bare variable-field READ
(`self.image.header.frame_id` without `()`) stays the write-only marker with
the deprecation-warning-points-at-the-accessor behavior. Only `=` and
`.fill_from(...)` on a nested path are rewritten.

**Schema-blind cost (E0599).** The rewriter cannot know a segment's field
kind, so a nested path routed through a *non-nested* field
(`self.image.data.len = 3`, where `data` is a plain `uint8[]`) compiles to a
call to a `__cer_with_nested_data` method that does not exist: a rustc E0599
naming a `__cer_*` method. That is the accepted cost of the declaration-free
design; the diagnostic points at generated plumbing, never silently compiles.

**One write mechanism per nested field per message.** Mixing the staged leaf
sugar (`self.image.header.<leaf> = …` / `with_header(…)`) with a whole-field
write (`self.image.header = bytes` / `set_header_bytes` / `loan_header_bytes`
/ `fill_from_header_bytes`) on the SAME field within one tick is a loud
`TransportError::NestedWriteConflict` (rejected in BOTH orders: the staged
flush runs at publish time, so either order would silently clobber). Write
each nested field by exactly ONE mechanism. This guard is for **complex
(variable) nested fields only** (e.g. `Image.header`): a **fixed** nested path
(`self.imu.orientation.x`) lands immediately in disjoint shared memory and is
plain last-write-wins, no guard.

**The every-variable-field rule extends to nested schemas.** Just as an output
schema discards its frame if any of ITS variable-length fields is unwritten, a
staged nested field must write EVERY variable-length field of its OWN schema
each tick, or the frame is discarded with a `NestedWriteConflict`-sibling
`TransportError::NestedChildIncomplete` naming the dotted `<parent>.<child>`
location, at ANY depth: a depth-2 miss names the full user-writable path
(e.g. `joint_trajectory.header.frame_id`), so the message's suggested
assignment always compiles from the port.
Example: writing only `self.image.header.stamp.sec` (a fixed leaf)
but never `self.image.header.frame_id` (Header's lone variable field) discards
the frame; write `frame_id` too, or write the whole `header` as bytes. Fixed
child leaves stay ungated (they default to zero), the same fixed/variable
asymmetry the top-level schema has.

### `#[derive(CerulionState)]`: state a capture can restore

A Flashback capture and a `bag play --resim` restart both need to put a node
back the way it was, which means capturing the node's own fields. **Your node
struct does not need this derive**: `#[cerulion_node]` emits the same
machinery itself, so an ordinary node costs zero extra lines. You reach for it
on the types your node HOLDS: the `Pose`, the `PadState`, the `SampleQueue`.
It is also the fix the compiler already names when a field cannot be captured.

```rust
use cerulion_core::state::CerulionState;

#[derive(CerulionState)]
struct Pose { x: f64, y: f64 }
```

Most fields need nothing. The per-field escapes, for the exceptional field:

| Attribute | Meaning |
|---|---|
| `#[cerulion(reconstruct)]` | A HANDLE: not captured, and left untouched on restore. Use it for something that must be rebuilt rather than restored (a connection, a device). |
| `#[cerulion(serde)]` | Capture through the field's own `Serialize` / `Deserialize` instead of Cerulion's walk. |
| `#[cerulion(unordered)]` | A hash-like container whose key has no total order; captured in iteration order. |

A resource type the framework already recognises (`File`, `TcpStream`,
`JoinHandle`, a `Box<dyn Trait>`, `Arc<TransportManager>`) is treated as
`reconstruct` **without** the attribute, so you only write one where the
framework cannot tell. An unknown key inside `#[cerulion(...)]` is a compile
error, never a silently ignored one.

### `NodeError` variants

You return `Result<(), NodeError>` from `tick`/`init`/`shutdown`. The variants you'll typically construct:

| Variant | When |
|---|---|
| `NodeError::Logic(String)` | Recoverable logic error. |
| `NodeError::InvalidInput { input, reason }` | An input message failed validation. |
| `NodeError::Fatal(String)` | Marks the error unrecoverable in intent. The runtime logs and counts it like any other tick error and keeps scheduling the node: there is no hard-stop lifecycle. |
| `NodeError::Custom(Box<dyn Error + Send + Sync>)` | Wrap any other error type. The `Send + Sync` bound is real: an error type that is not both cannot be boxed into it. |
| `NodeError::Transport(TransportError)` | Auto-converted from `?` on transport calls. |
| `NodeError::Io(std::io::Error)` | Auto-converted from `?` on `std::fs` / `std::io`. |

You typically do **not** match on `TransportError` variants; they're framework-level. Just `?` them.

---

## Graph YAML reference

Graph YAML carries **topology only**: `id`, `type`, `inputs`,
`outputs` (or, for a stock ROS 2 process launched beside the native nodes, `id`
plus a `ros2:` block; see [`ros2:` entries](#ros2-entries-stock-ros-2-nodes-in-the-same-graph)).
Trigger policy lives on the node's macro
(`#[cerulion_node(period_ms = N)]`, `#[input(trigger)]`, etc.) and
is the single source of truth. Graph YAML once accepted a
parallel `policy:` block that overrode the macro; that path is
gone; a node that needs different policies in different graphs
should be different node types.

A graph is named by its FILE: `graphs/perception.yaml` is the graph
`perception`, and that stem is what every surface uses: the run directory,
the bag filename stem, the trace/state ring tags, Flashback capture names and
the `graph=` field on every log line. There is no `name:` key to keep in sync
with it.

**Every key is checked, with exactly one exception.** A key the format does
not define (a misspelling like `dpeth: 32`, or a leftover legacy
`policy:` block) is a loud parse error naming the offending key and listing
the accepted ones, not a setting silently dropped on the floor.

The exception is `name:`. It is read and ignored by design: the
file stem is the graph name, so a legacy `name:` still parses, and one that
DISAGREES with the stem gets a single deprecation `warn!` at load naming both
values. Delete the line to silence it. Nothing else is grandfathered: "an old
graph file still loads" is true of `name:` and of nothing else.

The identity itself has a representability bound: it names an iceoryx2 node,
which admits only ASCII below U+0080 and at most 128 bytes, and the cap
applies to the COMPLETE node name, prefix included, so the identity's own
budget is 119 bytes on a `graph run` (`cerulion_{identity}`) and 112 on a
resim (`cerulion_replay_{identity}`). An identity that breaks either rule
(an accented or emoji name is refused even though it is valid UTF-8) gets a
typed refusal naming the identity, the constraint it broke (both numbers on
the length arm), and the remedy: rename the graph, or the bag, and re-run.
Never a panic. The refusal fires on the paths that mint the identity
verbatim: a single-process `graph run`, `graph profile`, and a resim (the
exit-2 not-replay-grade arm). A MULTI-PROCESS run's worker and supervisor
node names sanitize the graph's bytes (non-ASCII becomes `_`) and so proceed
without it; the sanitized name is a process-naming convenience there, not
the recording identity.

That boundary reaches your BAGS. An older bag may embed the on-disk `graphs/<name>.yaml` rather than the effective config the
run executed, so if that file carried a `policy:` block the key travels into
the bag and `bag play --resim` refuses it (exit 2) naming the key and the
attachment. The recording itself is fine; its embedded graph is stale.

**To fix it: `cerulion bag migrate <bag>`** (see the CLI reference above). It
writes a NEW bag (the same recording, with the undefined keys taken out of its
embedded graph), lists every key it removes before writing anything, and never
touches the original.

If the graph has simply moved on since, re-recording from the current graph is
the better answer; migration is for a recording of something you cannot do
again.

```yaml
prefix: my_robot                # optional; defaults to hostname (.local stripped). Topic resolution is "/<prefix>/<node_id>/<port>".
nodes:
  - id: camera                  # required; unique within the graph
    type: camera                # required; must match a nodes/<type>/ folder
                                # (policy comes from the macro: `#[cerulion_node(period_ms = 33)]`)
    outputs:
      - name: image             # required; matches an #[output] field name
                                # optional `topic: /abs/name` publishes to that ABSOLUTE topic instead (see Topic naming)
        schema: sensor_msgs/Image # required (`validate_graph` REFUSES an absent or
                                #   malformed one, so `cerulion graph run` and `graph validate`
                                #   both fail rather than defaulting it silently); matches the
                                #   field's Rust type. `pkg/Type`, `pkg::Type`, or a bare name
                                #   for a schema in this workspace's `schemas/`
        max_slice_len: 134217728 # optional; falls back to <T>::MAX_SLICE_LEN, then 128 MiB + warn
        history_size: 8         # optional; iceoryx2-native per-publisher history for late joiners (default 0; max 64 = MAX_CONSUMER_DEPTH: replays through consumer queues capped at that depth, so a larger history can't be delivered in full). Late joiners receive history even from a QUIESCENT producer that has stopped publishing (the runtime services them each live-loop iteration).

  - id: detector
    type: detector
                                # `#[input(trigger)] image: Image` on the macro side
                                # synthesizes the data trigger
    inputs:
      - name: image
        source: camera/image    # <node_id>/<port> reference

  - id: sync_consumer
    type: sync_consumer
                                # `#[cerulion_node(sync_window_ms = 50)]` on the macro
                                # fuses the `#[input(trigger)]`-marked ports within the
                                # window (a plain `#[input]` rides along as
                                # latest-value context and never gates the fire)
    inputs:
      - name: image
        source: camera/image
      - name: scan
        source: lidar/scan

  - id: camera_driver
    type: camera
                                # `#[cerulion_node(external)]` ingress/driver on the macro side:
                                # self-triggers off a device fd/SDK via its external_source() method
    outputs:
      - name: image
        schema: sensor_msgs/Image
```

### Trigger policy variants

Set on the macro (in `nodes/<type>/src/lib.rs`), not in graph YAML:

| Macro form | Effect |
|---|---|
| `#[cerulion_node(period_ms = N)]` | Fire every N ms. |
| `#[input(trigger)] field: T` | Fire on each message arriving on `field`, with **per-message FIFO delivery**: every message gets its own fire, in arrival order, and the tick reads exactly that message. A burst of N messages between fires yields N fires (one per scheduler step; the backlog carries across steps until drained), so no message is silently skipped; messages are lost only to the input's declared backpressure policy (eviction/decimation, both counted). (Uses field-level attribute; no node-level macro arg needed.) |
| `#[input(trigger, expect_within_ms = N)] field: T` | Fire on each message arriving on `field` (per-message FIFO, as above), AND count a miss if no fresh data lands within `N` ms (the deadline-watchdog decomposition, in place of a `deadline_ms` trigger, which does not exist; the timeout records a miss, it does **not** fire the node). |
| `#[cerulion_node(sync_window_ms = N)]` + `#[input(trigger)]` on each port | Bounded sync: fire when all `#[input(trigger)]` ports receive a message within an N-ms window. Needs two or more trigger inputs to align anything (none is a compile error; exactly one compiles, is ignored, and warns at graph build). A plain `#[input]` alongside them is a latest-value context read (held across steps; see the non-trigger inputs section) that never gates the fire. |
| `#[cerulion_node(unbounded_sync)]` + `#[input(trigger)]` on each port | Unbounded sync (loose AND): fire when every trigger port has ≥1 unconsumed message; no timing bound. Needs two or more trigger inputs, under the same rule as `sync_window_ms`. Mutually exclusive with `sync_window_ms`. Plain `#[input]`s never gate the fire. |
| `#[cerulion_node(external)]` | Self-triggering **ingress / driver**: watches a non-Cerulion signal (device fd / blocking SDK) and fires itself; requires an `external_source()` method. See [External nodes (ingress / drivers)](#external-nodes-ingress-and-driver-policy). |

A **macro** node with ports, no node-level policy attribute and no `#[input(trigger)]` field is a **compile error**: the macro tells you to add `#[input(trigger)]` to a field or declare `period_ms` / `external` on the node. The fire-on-any-input default, with a `tracing::warn!` at graph-build time so the surprise is visible, applies only to a legacy `--raw-ffi` node whose `INFO` block declares no policy. The macro also rejects illegal combos (e.g. `period_ms = 100, external`) at compile time.

### Reading the runtime's counters

Every counter this document names lives on the runtime's per-node handle, a framework
object: node code is never handed one, and no `cerulion` verb prints one. What reaches
you is what the runtime says about them:

- **Log lines.** Each miss, eviction, decimation, defer or discard logs with the node
  and the port as structured fields; the flood-latched ones log the first event of a
  regime loudly (`warn!`) and the repeats at `debug!`, each carrying the running
  `total`. The sections below quote the lines.
- **`#[on_event]` handlers** in your node (see **Backpressure**), for the events that
  have one.
- **Shutdown telemetry.** A run prints `live loop delivery telemetry` lines as it stops:
  `fires` per node, plus `drop_oldest` for every data-trigger input.

Counter names are given so a log line can be matched to the counter it reports.

### QoS deadlines and miss counters

Three independent observability layers, each with its own miss counter. Counters increment atomically + `tracing::warn!` fires on each miss with structured fields.

| Attribute | Owner | Counter | What it observes |
|---|---|---|---|
| `#[input(expect_within_ms = N)]` | subscriber | `expect_within_missed_count` | This subscriber didn't see data on this input within N ms. |
| `#[input(expect_within_ms = N)]` | subscriber | `expect_within_backlogged_count` | The same window elapsed while this input's data was sitting unserved: the node was BEHIND, not starved. Two shapes: a Data node's *trigger* input with signalled arrivals unserved, and a per-set Sync node's trigger input whose frame is already the aligned set's member while the set waits on a sibling or on the node's own rate cap. (Signalled ARRIVALS, not queue depth: on the unified drain discipline the boundary signals one arrival per frame it serves, so this says the node owes a fire, never how many frames are waiting.) Disjoint from the miss counter (a lapsed window lands in exactly one), reported at `debug!` per window plus one `info!` when the regime opens and closes, and it emits no `ExpectWithinEvent`. |
| `#[output(promise_within_ms = N)]` | publisher | `promise_within_missed_count` | This publisher didn't publish on this output within N ms (commitment to downstream subscribers). |
| `#[cerulion_node(tick_within_ms = N)]` | node | `tick_within_missed_count` | `tick()` callback's wall-clock duration exceeded N ms. |

Mirrors ROS2's QoS-event design: `promise_within` and `expect_within` are **independent observations on opposite endpoints**, not propagation from publisher to subscriber. A subscriber with no `#[input(expect_within_ms = N)]` is unaware of its publisher's promise misses; read the publisher's warn line, or declare your own input deadline.

Each miss path emits `tracing::warn!` with structured fields you can filter via `RUST_LOG`:

```
WARN cerulion_core::scheduler: input `expect_within_ms` exceeded (no fresh data within the expected window) — `expect_within_missed` counter incremented node_id=consumer input=image expect_within_ms=100 elapsed_ms=140
```

`N` must be > 0 for all three deadlines; the macro validator rejects zero values at compile time.

### Other runtime counters

Beyond the QoS deadline counters above and the backpressure counters in the **Backpressure** section, the runtime keeps two more per-node diagnostics (see [Reading the runtime's counters](#reading-the-runtimes-counters) for how any of them reach you):

| Counter | What it observes |
|---|---|
| `signal_failed_count` | Number of trigger signals the scheduler **rejected** for this node because they did not match its trigger policy (a wiring-desync diagnostic). RAW per-signal count: a persistent desync climbs it at the input data rate (it is not latched). `0` means no desync was ever observed. |
| `publisher_disconnects_observed_count` | Number of publisher-loss (`Lost`) transitions the runtime liveliness sweep observed on this node's inputs (a disconnect and a crash are indistinguishable via the live publisher count). The `Alive` edge does not bump it. **Live-only**: the drop is not bit-reproducible from a free re-run (replay reproduces it via the recorded trace). The always-on counterpart to the `LivelinessEvent` `#[on_event]` handler (see **Backpressure**, `#[on_event]` callbacks). |

### Publish trace (legacy)

The publish trace is an older, metadata-only observability record: one entry per
publish (`topic`, `sequence`, `publish_time_ns`, `schema_hash`), no payloads, spilled
to disk as JSON-Lines files named `trace_*.jsonl`. It is a framework-internal
facility. No `cerulion` verb attaches one to a run, and nothing in a node or a graph
file turns it on, so a `graph run` writes no such files.

The one user-facing piece is the reader, for a directory of those files you
already hold:

```bash
cerulion trace inspect ./trace-dir
cerulion trace inspect ./trace-dir --filter perception/image --limit 100 --reverse
```

To record a run, use one of `cerulion graph run <NAME> --record`,
`cerulion bag record`, or `cerulion flashback`. Those write standard MCAP bags with
full payloads (see `docs/bag.md`), which `cerulion bag play` reads.

### `max_slice_len` resolution

Three tiers, top-down:

1. Explicit YAML `max_slice_len: N` wins.
2. Else `<T as ShmMessage>::MAX_SLICE_LEN` (codegen-emitted per-schema budget) is used silently.
3. Else `DEFAULT_MAX_SLICE_LEN = 128 MiB` with a `tracing::warn!` per topic.

Stock ROS2 schemas all have explicit budgets (16 KiB to 128 MiB depending on shape); a user-defined variable schema gets the codegen catch-all budget (128 MiB) **silently**; it does not hit the warn path. The tier-3 `DEFAULT_MAX_SLICE_LEN` warn fires only when no codegen-emitted budget exists at all (raw-FFI shapes). These per-tier defaults are generous: iceoryx2 `Static` pools are demand-paged, so resident memory tracks the working set rather than the reservation, while the reservation still consumes virtual address space and shared-memory capacity.

---


### Topic naming

Canonical topic names carry a **leading slash** (ROS2 convention): an output `image` on node `camera` in a graph with `prefix: my_robot` publishes `/my_robot/camera/image`. The slash is part of the iceoryx2 service name and of what `cerulion topic list`/`echo`/`hz` show.

Two absolute (leading-`/`) escapes:

| Surface | Meaning |
|---|---|
| `source: /some/topic` on an input | ABSOLUTE reference: bypasses the prefix. If an in-graph output publishes that exact topic (derived or via `topic:`), it wires normally. If nothing in-graph produces it, it is an **external topic**: the graph attaches with its buffer requirement only (no port provisioning; out-of-graph publishers attach freely), and `block` backpressure is rejected on it at build (the scheduler cannot defer an external publisher). An external reference that falls INSIDE the graph's own prefix namespace additionally gets a loud build warn (with a did-you-mean when an overridden output's derived spelling matches); it's usually a typo'd in-graph reference, but two same-host graphs legitimately share the hostname-default prefix, so it validates rather than rejects. (A multi-process worker validates only its own process group's slice of the graph, where an input fed by another group appears as exactly such a reference; the supervisor tells each worker which topics its sibling groups produce, so a correct cross-group edge does not draw this warn. Any other in-prefix miss still does.) At run time, any external topic with NO publisher attached after a 5s grace (graph clock) warns once: the silent-forever typo backstop. |
| `topic: /some/topic` on an output | ABSOLUTE override: the output publishes to that name verbatim instead of the derived one (for externally-fixed global names like `/tf`). The value must be absolute (`topic: tf` is rejected with a did-you-mean). The topic stays graph-OWNED: single-writer provisioning holds, and two nodes (or graphs) publishing the same override collide at build / publisher creation, UNLESS the topic is listed in `multi_publisher_topics:`. Relative references to an overridden output are rejected with the actionable fix (`reference it as `source: /tf``). |
| `multi_publisher_topics: [/tf,...]` (top-level) | Opt a topic into MULTIPLE publishers. Entries must be absolute, well-formed, and unique (derived names embed the node id and can't be shared by construction). A listed topic with in-graph producer(s) relaxes the double-producer rule and provisions the three iceoryx2 service knobs as **shared graph-independent loose constants** (publishers 16, subscribers 16, buffer ceiling 16). There is deliberately NO per-topic count knob: you can't always know how many publishers a `/tf`-class topic will carry, and graph-dependent values would lock a second graph out of the creator's service (iceoryx2 open requirements are at-least; equal constants pass everywhere). `block` consumers defer EVERY in-graph producer, and the gate is a single conservative total across all of the topic's publishers, so one producer's burst alone can defer a sibling producer that published nothing (lossless by design; per-connection queues never overflow). `block` is still rejected when the topic has no in-graph producer (deferring out-of-graph publishers needs a host-level scheduler, which does not exist). A graph that ALONE exceeds a shared cap (consumers or depth) is rejected at build with the arithmetic named. A listed topic nothing references warns; listed + consumed-only warns (External topics already admit multiple publishers). **Scheduling:** among a DAG level's in-graph producers of a listed topic that SHARE A PROCESS and reach the within-level parallel REST, publishes happen in graph (declaration) order: that level's REST walks serially instead of under the rayon pool, so the shared FIFO's interleave is the same on every run and in replay. A serial-gated producer on that level (a node library with non-trigger inputs and a no-op snapshot, or any `block`-involved node) publishes FIRST, in the serial pass ahead of the REST, so the level's order is fixed but is not declaration order across that boundary. Producers SPLIT across processes (the default multi-process partition puts them in different workers) have NO cross-process order, and this rule creates none: which worker's connection is drained first is an attach-order race nothing here pins. What IS guaranteed is that a consumer never sees one writer's frames interleaved with another's (the drain is per publisher CONNECTION), and that the recorder labels every frame of such a topic with its writer, so replay is per-writer. This changes WHEN those nodes fire, never WHAT: the fire set, trace and data are unchanged; the level forgoes within-level parallelism. **Under multi-process, a listed topic with exactly ONE in-graph producer is still CREDITABLE**, so a hand-written `process_groups:` may split its `block` edge and the supervisor mints it a cross-process credit word: the listing is a permission, not a producer count, and the word describes the in-graph producer it can actually defer. The residual is the same one the paragraph above states for the co-located case, and the split does not change it: an OUT-OF-GRAPH writer on a listed topic fills the consumer's queue without raising `outstanding`, so the defer arrives late and the queue can evict. There is no detector for that; it is a property of `block` on a listed topic, co-located or split. |

```yaml
nodes:
  - id: tf_broadcaster
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: tf2_msgs/TFMessage
        topic: /tf            # absolute override; single-writer enforced
  - id: localizer
    type: localizer
    inputs:
      - name: tf
        source: /tf           # absolute: matches the in-graph producer above
      - name: scan
        source: lidar/scan    # relative: → /{prefix}/lidar/scan
      - name: ext
        source: /vendor/cam   # absolute, no in-graph producer: external (block forbidden)
```

Malformed shapes are rejected at graph load: bare `/`, trailing `/`, empty `//` segments, and prefixes that start/end with `/`.

### `ros2:` entries: stock ROS 2 nodes in the same graph

One graph file can describe the WHOLE robot: native Cerulion nodes (`type:`)
and stock ROS 2 nodes (`ros2:`), brought up together by `cerulion graph run` on
one transport. A `ros2:` entry names a ROS 2 process to launch; the run spawns
it as a supervised child on the same staged environment `cerulion ros2 run`
uses (`RMW_IMPLEMENTATION=rmw_cerulion`), so its topics land on the shared
transport beside the native nodes'.

```yaml
nodes:
  - id: detector              # native, unchanged
    type: yolo_node
    outputs:
      - name: boxes
        schema: vision_msgs/Detection2DArray

  - id: move_group            # a ROS 2 node: `ros2 run <package> <executable>`
    ros2:
      package: moveit_ros_move_group
      executable: move_group
      args: ["--log-level", "warn"]      # optional; the executable's own args
      params_file: config/move_group.yaml # optional; --ros-args --params-file
      params:                             # optional; --ros-args -p key:=value
        planning_time: 5.0
        use_sim_time: false

  - id: bringup               # OR a launch file: `ros2 launch <file> [args]`
    ros2:
      launch: launch/robot.launch.py
      args: ["use_rviz:=false"]           # launch arguments
```

| Key | Meaning |
|---|---|
| `package:` + `executable:` | The `ros2 run` form: `ros2 run <package> <executable> [args...]`, plus `--ros-args --params-file <params_file> -p <k>:=<v>...` when either params key is present. Both keys are required together. |
| `launch:` | The `ros2 launch` form: `ros2 launch <file> [args...]`. The ALTERNATIVE to `package`/`executable`: an entry declares one shape or the other, never both. Relative paths resolve against the workspace root and must exist (a missing file is refused by name before anything spawns, because `ros2 launch` would otherwise reinterpret the name as a package). |
| `args:` | Forwarded verbatim: the executable's arguments in the `run` form, launch arguments (`name:=value`) in the `launch` form. |
| `params_file:` / `params:` | ROS parameters for the `run` form only (a launch file carries its own; pass launch arguments through `args:`). `params:` values must be scalars: a string, number or bool, one `-p key:=value` each, in declaration order; put nested parameter trees in a `params_file:`. |

**What a `ros2:` entry is NOT.** It is an opaque process the run spawns and
supervises; spawning and supervising is ALL it does:

- **Not scheduled.** No trigger policy, no DAG level, no barrier seat, no
  determinism claim. `graph levels` lists these entries apart from the DAG as
  "spawned, not scheduled"; `level_assignments:` and `process_groups:` may not
  name them (each is refused with the remedy named).
- **No ports.** `inputs:` / `outputs:` are refused on a `ros2:` entry. Its topics
  meet native nodes on the shared transport BY NAME (a native `#[input]` whose
  `source:` is the absolute `/tf` reads a ROS 2 `/tf` publisher directly), so
  the wiring is simply not modelled in the graph (remaps are not rewritten).
- **Recorded, never re-executed.** `--record` records its frames like any
  other live topic (live-topic discovery covers them; the entry declares no
  outputs to name), and the bag's embedded `graph.yaml` carries the `ros2:`
  entries VERBATIM, in authored node order, interleaving included (so does the
  run directory's; both embed the effective config cloned BEFORE the split). `bag play --resim`
  re-executes the NATIVE nodes against the recorded frames and
  classifies-and-skips the entries: one warn names them, the `--report`
  JSON carries them under `ros2_entries_skipped`, and ROS 2 is never
  respawned; `bag info` renders them as their own section.
- **Not the whole graph.** A graph of only `ros2:` entries is refused: the
  runtime has nothing to schedule; `cerulion ros2 launch <launch-file>` is the
  tool for a bare launch file.

**Lifecycle.** Every `ros2:` entry spawns before the graph starts, on every run
shape (`--single-process`, the multi-process default, `--record`, a virtual
clock), in its own process group; the graph process drives its teardown. On
shutdown each child is frozen and PEEKED first (an exit already in flight is
judged as the child's own death by the run's `--peer-loss` policy, never as
the run's stop), then gets a graceful SIGINT, a 10 s grace (long enough for `ros2
launch` to wind its own nodes down), then a SIGKILL backstop to the entry's
WHOLE process group (each entry runs as a group leader on Unix, so a hung `ros2
launch` takes the nodes it started with it); a leader that dies on its own is
given the same group kill before it is reaped, so a crashed `ros2 launch` cannot
strand the nodes it started. Every exit path takes the entry's group with it,
with two exceptions: a group kill the kernel refuses is logged loudly and the leader
alone is reaped, and a leader whose exit the kernel will not let the run look at
(a refused `waitid` peek, said once, then at debug) is judged from the plain
wait alone with its group untouched. An error path that skips the graceful
teardown (a graph build or run that fails) still takes every entry's group with
it: at once, with no SIGINT and no grace. A child that **exits 0** mid-run is a warning and
the graph keeps running (a sidecar finishing must not stop the robot); a child
that **dies** (non-zero, or lost) follows `--peer-loss`: the default `continue`
warns and keeps the graph running degraded (it is not restarted), `fail` stops
the run and exits non-zero naming the entry. A death is judged when it happens
by the death-watch AND at teardown: before the run signals a child it FREEZES it
and looks (a frozen child cannot exit, so there is no instant between the look
and the signal for a death to slip through), and a child found already dead is
judged by the policy, while a child the run itself stops is never counted as
one. The look itself happens on every platform; the freeze and the graceful
signal are the Unix half. Should the kernel refuse the freeze (no live child of
the run produces this), the run looks without freezing, judges what the look can
see and signals; only a death landing in that one instant is then taken for a
response to the signal. On a platform without job control there is no graceful
signal at all: the look still judges a child already dead, and a running child
is stopped by the backstop at the grace deadline (said loudly). If the
death-watch thread cannot start at all, a `fail` run is REFUSED before the graph
starts (the promise would be carried by nothing); a `continue` run warns and
judges deaths at teardown only. `librmw_cerulion.so` is a deployment
precondition checked before anything spawns (the same 69-class message as
`cerulion ros2 run`, with its remediation), and so is `ros2` on `PATH`.

### Deployment keys: `process_groups`, `process_group_order`, `level_assignments`

Three optional top-level keys shape how the graph is DEPLOYED across processes
and levels. All three are written for you by `cerulion graph partition` and are
hand-editable; all three are validated loudly at load, because a hand-edited
file is untrusted input.

```yaml
process_groups:                 # optional; group name → the node ids in it
  perception: [camera, detector]
  control:    [planner, actuator]

process_group_order:            # optional; explicit rank order (first = rank 0)
  - control
  - perception

level_assignments:              # optional; node id → DAG level index
  camera: 0
  detector: 1
  planner: 1
  actuator: 2
```

| Key | Meaning |
|---|---|
| `process_groups:` | One worker PROCESS per group (Unix), running in barrier lockstep by default or free-run under `CERULION_EXECUTION_MODE=free_run`. Declaring it pins the partition; omitting it lets `graph run` derive one (see the `graph run` row for the full default). Every node must appear in exactly one group. A topic with an in-graph producer and a `block` consumer may be SPLIT across groups only when the edge is **creditable** (exactly one in-graph producer and no non-`block` consumers), in which case the supervisor mints it a cross-process credit word and the defer crosses the boundary losslessly. Splitting any other `block` topic's flow is refused before any worker spawns, naming which bar it hit (two or more in-graph producers, or a MIXED topic whose `block` consumers are degraded to `drop_oldest`) and the fix. The DERIVED partition co-locates every `block` topic's whole flow regardless: crossing a boundary costs a real hop whether or not the edge is correct. |
| `process_group_order:` | Explicit cross-process rank order: the group names from `process_groups:` in the order you want them ranked, first = rank 0. When present it must be a PERMUTATION of the group names: every group exactly once, no unknowns, no duplicates. When absent, rank follows the `process_groups:` listing order, which is the normal case; `graph partition` emits listing order and REMOVES a stale block of this key rather than maintaining two answers. |
| `level_assignments:` | Per-node DAG level index, used INSTEAD of the derived levelization. This is the baked output of the cost-aware refinement `graph partition` emits, and it is what lets you hand-tune the schedule. Hard contract, all checked at build: every key names a real node; EVERY node is covered (a partial map is ambiguous about intent and is rejected; delete the whole block to fall back to the derived levels); every triggering edge is strictly level-increasing; and the levels form a contiguous `0..K` with no gaps (the multi-process barrier advances one generation per level). Within-level fire order is NOT taken from this map; it is always `nodes:` declaration order, exactly as on the derived path. |

### Cross-machine networking: permissive by default + the `network:` block

A real-clock live run is **network-viewable by default**:
`graph run` (monolith OR multi-process), `node run`, and `ros2 attach` each
spawn ONE network GATEWAY process that opens the robot's single zenoh
session, listens on the well-known port **7683** (bind-probing upward on
conflict; override with `CERULION_GATEWAY_PORT`), scouts the LAN, and
announces every produced topic; one loud breadcrumb prints at start. The
optional top-level `network:` block below RESTRICTS this (a Strict
allow-list with verbatim locators) rather than enabling it. Turn networking
off for a run with `--network off` (or `CERULION_NETWORK=off`). See
`docs/networking.md` for the full gateway model + a two-machine quickstart.

```yaml
network:
  mode: peer                     # peer | client | disabled (omitted means disabled; see `mode` below)
  connect:                       # zenoh locators to dial (remote peers/routers)
    - tcp/192.168.123.99:7447
  listen:                        # zenoh locators to bind locally
    - tcp/0.0.0.0:7447
  egress:                        # canonical absolute topics this graph EXPORTS
    - /go2/utlidar/cloud
  ingress:                       # canonical absolute topics this graph IMPORTS
    - /go2/cmd_vel/keyboard
```

- `mode`: the graph's zenoh role. `peer` joins the mesh; `client` connects
  to a router without routing traffic. `disabled` (also what an omitted `mode:`
  means) makes the BLOCK inert, which is NOT the same as turning the network
  off: a run treats a disabled block exactly like no block at all, so under the
  real clock it gets the permissive default above (gateway process, LAN
  scouting, every produced topic announced). The local-only switch is
  `--network off` / `CERULION_NETWORK=off`, nothing in this block. `router` is
  deliberately NOT exposed. An unknown mode string is rejected at parse
  listing the valid values.
- `connect` / `listen`: each entry maps 1:1 onto a zenoh locator string,
  used VERBATIM (a Strict block adds no scouting and no bind ladder), so a
  config-only two-machine deployment needs one side to `listen` and the
  other to `connect`. (Scouting-ON + the 7683 bind ladder are the
  permissive-default behavior, not block knobs.)
- `egress`: topics this graph exports to the network, enforced as a TRUE
  **allow-list**: only declared egress topics ever leave the machine (a
  remote machine's request for any other produced topic is refused at the
  bridge gate: its bridge flag structurally never flips on; an
  ingress-only graph installs a deny-all gate). Each entry must be
  **produced by an in-graph node** (derived names and `topic:` overrides
  both count).
- `ingress`: topics this graph imports from the network. Each must **NOT**
  have an in-graph producer: they are external sources, the same class as
  a producer-less absolute `source:`. Each must be CONSUMED by an in-graph
  node (an ingress nobody reads fails the build), and received frames are
  validated against the consuming input's schema hash before re-injection.

Validation (at graph load, so `graph validate` and `graph run` both apply it). Every
rejection names the offending topic and the fix:

| Rule | Rejected shape |
|---|---|
| Loop safety | The same topic in BOTH `egress` and `ingress` (it would loop back to itself); remove it from one list. |
| Egress ownership | An `egress` topic no in-graph node produces. |
| Ingress externality | An `ingress` topic an in-graph node produces; remove the entry or the producing node. |
| Canonical names | A bare (non-`/`) topic in either list: rejected with the expected `/`-prefixed form, never silently canonicalized. Malformed absolute shapes (trailing `/`, `//`, zenoh-reserved chars) are rejected like every other topic name. |
| Disabled-with-lists | `mode: disabled` (or omitted) with a non-empty `egress`/`ingress`; enable the network (`mode: peer\|client`) or remove the lists. |
| Duplicates | The same topic twice in one list. |

Empty `egress`/`ingress` under `peer`/`client` is valid (an
export-restricted robot). Inspect what remote machines are advertising with
`cerulion topic list` (remote discovery is automagic, scouting ON by
default; add `--connect tcp/<host>:7683`, the well-known permissive-gateway
port, for a peer scouting can't find).

Runtime controls (`cerulion graph run`, `node run`):

- `--network off` (also `CERULION_NETWORK=off`, honored by ANY entry point)
  is the kill-switch: run LOCAL-ONLY (no gateway, no zenoh session; a loud
  notice). The YAML stays untouched. `off` is the only accepted flag value.
- Networked MULTI-PROCESS is FIRST-CLASS:
  the supervisor spawns ONE gateway beside the workers; each worker stays
  network-free. See `docs/multi_process.md`.
- `--record` KEEPS the network (the robot stays visible while recording).
  The ONE exception: `--record` + an explicit block declaring `ingress:` is
  refused (naming both workarounds: recorded ingress re-injection
  is not replay-faithful). Replay is structurally network-inert (the bag
  is the input, Principle #7).
- `--time-source virtual`/`external` keep the network INERT (replay-class);
  `graph profile` is local-only by design.

See `docs/networking.md` for the full model + a two-machine quickstart.

## Node metadata: source code is truth

Per-node metadata (port names, schemas, trigger policy) lives in
`nodes/<type>/src/lib.rs`; there is no sidecar file. The CLI
derives metadata directly from the source: for macro-form nodes
by walking the `#[cerulion_node]` struct's `#[input]` / `#[output]`
field attrs and the node-level macro attribute, and for legacy `--raw-ffi`
nodes by reading the `// CERULION:INFO_START` JSON marker block
(its optional `policy:` field carries the same five-variant shape
the macro emits). `cerulion node info` and `cerulion node list`
print this derived metadata with a `POLICY` column showing the
trigger; `cerulion node modify` mutates the source in place
(splicing new fields into the macro struct, or regenerating the
raw-FFI INFO block).

`schema:` references in graph YAML accept either `pkg/Name` or
`pkg::Name`; the CLI canonicalizes to `/`.

---

## Schema YAML

Generated by `cerulion schema create <NAME>`:

`fields:` is a **mapping**, not a list. Each KEY is the string
`"<type> <name>"`; the value is unused (leave it empty). A list under
`fields:` does not parse as a mapping, so every field is skipped: you get a
schema that loads with ZERO fields, a wrong hash, and no error.

```yaml
schemas:
  Reading:
    description: "An example reading"
    fields:
      # Add fields: "type name" (the value is unused)
      "u32 sequence":
      "f64 value":
      "string source":            # variable
      "u8[] payload":             # variable (dynamic array)
      "f64[3] position":          # fixed array
      "Quaternion orientation":   # nested: fixed here, because Quaternion is
                                  # recursively fixed (see Field types below)
```

A worked in-repo example is `examples/perception/schemas/detections.yaml`.

### Field types

| Type form | Notes |
|---|---|
| `bool`, `i8`, `u8`, `i16`, `u16`, `i32`, `u32`, `i64`, `u64`, `f32`, `f64` | Primitives. Fixed. |
| `string` | UTF-8 string. Variable. |
| `<T>[N]` | Fixed-size array (where `N` is a literal `usize`). Fixed if `T` is fixed. |
| `<T>[]` | Dynamic array. Variable. |
| `string_fixed[N]` | Fixed-capacity string. Fixed. |
| `<SchemaName>` | Nested schema reference. Resolved BOTTOM-UP: if the target is recursively fixed, the reference is **fixed** and its layout is inlined into the parent's fixed section (and folded into the parent's schema hash). Only a reference whose target is variable (or which cannot be resolved) classifies variable. |

A schema is "fixed" iff every field is fixed: a primitive, a `string_fixed[N]`, a `FixedArray` of fixed elements, or a nested reference that resolved fixed. Otherwise it's "variable" and the runtime allocates per-publisher iceoryx2 SHM sized by the 3-tier `max_slice_len` ladder above.

---

## ROS2 `.msg` re-use

Standard ROS2 message types live in the `native_ros2_messages` crate (22 packages, 254 schemas; `cerulion schema list` prints the current set). Reference them by qualified name in graph YAML and node Rust code:

```yaml
schema: sensor_msgs/Image
```

```rust
use native_ros2_messages::sensor_msgs::Image;

#[output]
image: Image,
```

A message type your own robot needs goes in your workspace's `schemas/` directory
(see [Schema YAML](#schema-yaml)), not in this crate. Adding a type to the BUILT-IN
corpus is a change to the Cerulion repository itself, made in a source checkout, and
it takes three steps, not one:

1. Drop the `.msg` file under `crates/native_ros2_messages/msg/<package>/<Name>.msg`.
2. If the schema is **variable**, add a tier arm to
   `crates/cerulion_core/src/codegen/generator/wire_impl.rs::variable_schema_max_slice_len`.
   This is mandatory, not conditional: the silent 128 MiB catch-all is reserved
   for user-defined schemas, and codegen **panics** on a variable schema from an
   in-repo package that has no arm. A brand-new package also needs an
   `IN_REPO_PACKAGES` entry, or that guard cannot see it.
3. Run `tools/scripts/refresh_upstream_msg_manifest.sh`. The upstream-drift gate fails
   closed on a vendored message with no manifest entry, so CI rejects the
   addition until the manifest is refreshed.

---

## Backpressure

Backpressure is handled as a **scheduling** problem, not a buffering one. Cerulion never copies your data into a side buffer and never holds an iceoryx2 `Sample` handle to apply policy; every variant works either with iceoryx2's native subscriber queue or with the scheduler's pre-fire defer. The whole hot path stays zero-copy.

There are **three input-side policies** (`#[input(backpressure = ...)]`) plus **one node-level producer rate cap** (`#[cerulion_node(throttle_ms = N)]`).

### Input-side policies

| Declaration | Behavior |
|---|---|
| `#[input(backpressure = drop_oldest)] image: Image` (default) | iceoryx2-native eviction. When the subscriber's queue is full and a new sample arrives, the oldest unconsumed sample is reclaimed for the new one. "Latest wins": natural for sensor data where stale samples are useless. The data path stays fully iceoryx2-native (no Cerulion buffer/copy), and the subscriber **counts** evictions at drain time via **per-publisher-stream wire-sequence gaps**, keyed by iceoryx2's `sample.origin()` id, so `backpressure_drop_oldest_count` and the `#[on_event(input = "...")]` handler surface the loss **exactly per publisher stream**, with no cap: a consumer lagging by many buffers' worth reports the true loss, multi-publisher topics attribute evictions to the right stream, and a publisher restart simply starts a new stream (its first observation baseline-establishes, uncounted; prior history is unknowable). History replay to late joiners (*backward* sequences within a stream) is recognized as duplicate re-delivery and never counted (re-warned periodically if it persists). Corrupt (undersized) frames and errored drains reset the detector's baselines, and a stream displaced by baseline-capacity eviction under publisher churn re-establishes uncounted for one window; every edge errs toward under-reporting (the conservative direction; never fabricated). This is the default for any input that declares no policy. |
| `#[input(backpressure = sample(N))] lidar: PointCloud` | Subscriber-side **read-gate** (decimation). On each read (a trigger input pops its next queued sample in FIFO order; a latest-value context input drains to the newest), the sample is accepted only if its **wire `timestamp_ns`** (publish clock, replay-deterministic) is ≥ `N` ms after the last accepted read; otherwise it is **decimated** (dropped and counted, returning no data this tick). Caps the *read* rate without copying or buffering. On a per-set **Sync** trigger input the gate runs BEFORE matching, so only admitted frames are eligible to join a set, and a decimated read caps that step's burst at one set (the backlog is still served in full, one set per boundary). A decimated frame still resets that input's `expect_within_ms` watchdog: the arrival is evidence the producer is alive, and decimation is this consumer's own policy. A gate wider than the node's `sync_window_ms` builds with one `warn!` naming both windows; see [Per-set Sync delivery](#per-set-sync-delivery). |
| `#[input(backpressure = block)] image: Image` | Scheduler **pre-fire defer**: no data loss (Principle #6). A real-time `outstanding` counter mirrors the consumer's queue depth (incremented at publish, decremented at drain). The moment `outstanding == depth` (the declared `#[input(depth = N)]`, which IS the input's real iceoryx2 queue: the topic's service is provisioned at `max(transport default, largest consumer depth)`, so the declaration is always honored, never silently capped), the producer's tick is **deferred**, *before* the queue overflows, so no sample is ever dropped. On a data-trigger consumer this is lossless **end to end**: per-message FIFO consumption serves every queued sample to its own fire, so the tick observes the complete, contiguous stream (not just the newest at fire time). On a per-set **Sync** trigger input it is lossless end to end too, and the mirror counts a frame the alignment is HOLDING as unserved, so `depth` stays exact rather than becoming `depth` plus the matcher's slots. The cost is that a starved partner holds the producer at `depth` indefinitely; see [Per-set Sync delivery](#per-set-sync-delivery). Only installed when **every** consumer of the topic is `block` (see degradation below). The topic **must have an in-graph producer** (build-time validated; an external publisher can't be deferred). Under multi-process, the producer and every `block` consumer of the topic are put in ONE process group automatically. A hand-written `process_groups:` may SPLIT them, and is accepted when the edge can carry a **cross-process credit word**, a shared-memory cell both workers operate on, minted per edge by the supervisor for a topic with exactly one in-graph producer and no non-`block` consumers. Splitting any other `block` edge is refused at plan time, before any worker starts, naming which bar it hit (two or more in-graph producers, or a MIXED topic whose `block` consumers are degraded to `drop_oldest`). |

**Default is `drop_oldest`.** Inputs that declare no policy get it.

There is **no `drop_newest`**: it has no real local use case (`block` already covers "ordered, don't skip").

### Node-level producer rate cap: `throttle_ms`

`#[cerulion_node(throttle_ms = N)]` caps how often the **producer node** fires: the scheduler defers the node's tick while `now - last_fire < N` ms. It is a node attribute, **not** a per-input policy and **not** subscriber-side; contrast `sample(N)`, which decimates reads at one specific subscriber. `throttle_ms` stacks with every trigger except `period_ms` (mutually exclusive: period already pins the rate; rejected at compile time) and composes with `block` (the tick defers if **either** gate fires; they share the scheduler's single pre-fire slot). See the node-attribute QoS table above.

### Counter surface: per-input observation

For each `(node, input)` pair the runtime keeps lock-free atomic counters:

| Counter | Counts |
|---|---|
| `backpressure_drop_oldest_count` | `drop_oldest` evictions (EXACT per publisher stream, multi-publisher and restarts included; replays and corrupt drains never fabricate one) |
| `backpressure_sampled_count` | `sample(N)` decimations |
| `backpressure_block_fires_deferred_count` | `block` pre-fire defers (NO data loss) |
| `backpressure_block_defer_regimes_count` | `block` defer REGIMES the producer-side edge opened (each owes one loud regime-opening warn) |

See [Reading the runtime's counters](#reading-the-runtimes-counters) for how they reach you: the log lines below, the `BackpressureEvent` handler, and (for `drop_oldest` on a data-trigger input) the shutdown telemetry.

There is **no `drop_newest` counter** (there is no such policy). Counters are **per-input**, indexed by input field name. The three policy counters are independent: bumping one does NOT bump the others. `backpressure_block_fires_deferred_count` is **not** a data-loss event: it counts how many producer ticks the scheduler deferred so the consumer would never lose a sample. `backpressure_block_defer_regimes_count` counts regime OPENINGS, never steps: it is bumped at the defer edge's re-armed→firing transition (before, and independent of, the warn it then emits), so at quiescence it is `<=` `backpressure_block_fires_deferred_count`; it is what a test compares the loud regime-opening warns against where the sustained `debug!` repeats are compiled out.

The first event of a regime emits a structured `tracing::warn!` carrying `node_id`, `input`, `policy` and the running `total`; sustained repeats carry the same fields at `debug!`, so a steady overflow does not flood the log while the total stays readable.

### `#[on_event]` callbacks

`#[on_event]` is the unified event handler for all reactive node events. Annotate a method in a `#[cerulion_node_impl]` block; the macro reads the **event parameter type** to determine which event to bind and validates the filter at compile time. Handlers dispatch at the **tick tail** (after `tick()` returns `Ok`) in **declaration order**, deterministically. They do NOT fire if `tick()` returns `Err`.

#### Filter and scope rules

The `#[on_event]` attribute **requires** a filter that matches the event type's scope:

| Filter form | Required for event types |
|---|---|
| `#[on_event(input = "<port>")]` | `BackpressureEvent`, `ExpectWithinEvent`, `LivelinessEvent` (input-scoped) |
| `#[on_event(output = "<port>")]` | `PromiseWithinEvent` (output-scoped) |

Mismatched scope (e.g. `BackpressureEvent` with `output =`) is a **compile error**. The filter must name a declared `#[input]` or `#[output]` port (string literal; unknown name = compile error).

Two handlers on the **same port with different event kinds** are allowed: for example a `BackpressureEvent` handler and an `ExpectWithinEvent` handler both on input `"imu"`. Two handlers with the **same (port, event kind)** pair are a **compile error**.

There is **no node-wide (filterless) handler** and **no `TickWithin` event** (`tick_within_ms` is a counter-only knob).

#### Event type 1: `BackpressureEvent` (input-scoped)

Fires when the input's backpressure policy triggers, for **all** policies. Edge-triggered: the first trigger in a regime queues one event; subsequent triggers in the same regime are silent until the policy clears and rearms.

```rust
use cerulion_core::prelude::*;   // BackpressureEvent

#[cerulion_node_impl]
impl TrackerNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        // ... read self.lidar, etc.
        Ok(())
    }

    // Fires (once per regime) after a successful tick when a backpressure
    // event is pending on the "lidar" input, for ANY policy on that input.
    #[on_event(input = "lidar")]
    fn on_lidar_pressure(&mut self, event: BackpressureEvent) {
        // event.policy: BackpressurePolicy   (Sample(N) | DropOldest | Block)
        // event.dropped: u64                 (messages LOST this regime; 0 for Block)
        // event.input_name: Arc<str>
        // event.count_total: u64             (matches handle.backpressure_<variant>_count at event time)
        // event.count_in_regime: u64
        // event.regime_started_at_ns: u64
        // event.buffer_capacity: usize
        tracing::warn!(policy = ?event.policy, dropped = event.dropped, "lidar backpressure");
    }
}
```

> **Fires for ALL backpressure policies**; branch on `event.policy`:
>
> | Input policy | Fires when | `event.dropped` |
> |---|---|---|
> | `sample(N)` | a read arrives within the N-ms window and is decimated | messages decimated |
> | `drop_oldest` | iceoryx2 evicts the oldest sample(s) on overflow (detected by the subscriber via a wire-sequence gap) | messages evicted |
> | `block` | the consumer's queue reaches the defer threshold (producer deferred on its behalf) | `0` (lossless flow-control signal) |
> 
> The `block` row has one exception, on a **per-set Sync trigger input**: the event fires from inside a DRAIN, and while the matcher is HOLDING that input's frame as an aligned set's member the head is re-offered rather than re-drained. So a node whose partner has gone quiet stops emitting this event even though its queue is full and its producer is deferred. The live surface in that regime is the PRODUCER's `backpressure_block_fires_deferred_count`, not the consumer's event.

Manual escape hatch inside `tick`: `ctx.take_backpressure_event("lidar")` returns `Option<BackpressureEvent>` (draining the single pending event). The macro calls this same method in generated code.

#### Event type 2: `ExpectWithinEvent` (input-scoped)

Fires when the `#[input(expect_within_ms = N)]` watchdog window elapses without fresh data on that input; **edge-triggered**, once per silence regime (not once per missed interval).

Deliberately does **not** fire while that input's data sits unserved (a Data node's trigger input with signalled arrivals under a `throttle_ms` cap or a `block` gate, or a per-set Sync node's trigger input whose frame is already the aligned set's member): a handler that fails over to a backup source on staleness must not fire while the source it would abandon is still delivering. Such a window is reported through `expect_within_backlogged_count` instead, and the first genuinely silent window after the backlog drains still fires exactly one event.

```rust
use cerulion_core::prelude::*;   // ExpectWithinEvent

#[cerulion_node_impl]
impl FusionNode {
    fn tick(&mut self) -> Result<(), NodeError> { Ok(()) }

    // Fires after a successful tick when the "imu" input has not received
    // fresh data within its expect_within_ms window.
    #[on_event(input = "imu")]
    fn on_imu_deadline_miss(&mut self, event: ExpectWithinEvent) {
        // event.input_name: Arc<str>
        // event.expect_within_ms: u64
        // event.elapsed_ms: u64          (how long since the last data)
        // event.count_total: u64         (total misses on this input)
        // event.missed_at_ns: u64        (scheduler-clock ts of this miss)
        tracing::warn!(input = %event.input_name, elapsed_ms = event.elapsed_ms, "IMU data overdue");
    }
}
```

Manual escape hatch: `ctx.take_expect_within_event("imu")` returns `Option<ExpectWithinEvent>`.

#### Event type 3: `PromiseWithinEvent` (output-scoped)

Fires when the `#[output(promise_within_ms = N)]` commitment window elapses without a publish on that output; **edge-triggered**, once per silence regime.

```rust
use cerulion_core::prelude::*;   // PromiseWithinEvent

#[cerulion_node_impl]
impl ControlNode {
    fn tick(&mut self) -> Result<(), NodeError> { Ok(()) }

    // Fires after a successful tick when the "cmd_vel" output has not been
    // published within its promise_within_ms window.
    #[on_event(output = "cmd_vel")]
    fn on_cmd_vel_overdue(&mut self, event: PromiseWithinEvent) {
        // event.output_name: Arc<str>
        // event.promise_within_ms: u64
        // event.elapsed_ms: u64
        // event.count_total: u64         (total misses on this output)
        // event.missed_at_ns: u64        (scheduler-clock ts of this miss)
        tracing::warn!(output = %event.output_name, elapsed_ms = event.elapsed_ms, "cmd_vel publish overdue");
    }
}
```

Manual escape hatch: `ctx.take_promise_within_event("cmd_vel")` returns `Option<PromiseWithinEvent>`.

#### Event type 4: `LivelinessEvent` (input-scoped)

Fires when the **publisher set** on an input's topic changes: a publisher (re)connected, or the last one disconnected. **Edge-triggered**: it fires on the transition, not on every step.

```rust
use cerulion_core::prelude::*;   // LivelinessEvent, LivelinessState, LivelinessCause

#[cerulion_node_impl]
impl LocalizerNode {
    fn tick(&mut self) -> Result<(), NodeError> { Ok(()) }

    // Fires after a successful tick when the "scan" input's publisher set
    // transitioned (a publisher connected, or the last one disconnected).
    #[on_event(input = "scan")]
    fn on_scan_liveliness(&mut self, event: LivelinessEvent) {
        // event.input_name: Arc<str>
        // event.state: LivelinessState        (Alive | Lost: the state AFTER the transition)
        // event.cause: LivelinessCause        (PublisherConnected | PublisherDisconnected)
        // event.count_total: u64              (total transitions on this input)
        // event.changed_at_ns: u64            (scheduler-clock ts of detection)
        // event.publisher_count: usize        (live publishers at detection; Lost fires only when this hits 0)
        match event.state {
            LivelinessState::Lost => tracing::warn!(input = %event.input_name, "scan publisher lost"),
            LivelinessState::Alive => tracing::info!(input = %event.input_name, "scan publisher connected"),
            // `LivelinessState` is `#[non_exhaustive]`: the wildcard arm is required.
            _ => {}
        }
    }
}
```

> **Dispatch limitation: read this before relying on a `Lost` handler.** Every `#[on_event]` handler dispatches on the **consuming node's tick Ok-path**. A *purely data-triggered* node stops ticking the moment its input goes silent, so it will **not** fire its `Lost` handler until it next ticks (which may be never). If you need the handler itself to fire on a disconnect, give the node a `period` / `external` trigger (or another live input). The **always-on** record of a disconnect is the runtime's `publisher_disconnects_observed_count` counter (see [Other runtime counters](#other-runtime-counters)), which the liveliness sweep bumps regardless of whether the node ticks. The signal you can SEE without a tick is `#[input(expect_within_ms = N)]` on that input: its watchdog warns while the node is starved.

> **Determinism.** A `LivelinessEvent`'s fields are scheduler-clock / wire data (no `Instant`/wall read), so a *replayed* event is bit-identical. But the live disconnect **observation** itself is **not** bit-reproducible from a free re-run (when a publisher's teardown is observed depends on the OS), so unlike the four backpressure/QoS surfaces, liveliness is replay-deterministic only via the recorded trace, not by free re-execution.

Manual escape hatch: `ctx.take_liveliness_event("scan")` returns `Option<LivelinessEvent>` (`#[must_use]`; discarding it silently loses observability).

#### Combining handlers on one port

Two handlers with **different** event kinds may target the same port; they dispatch in declaration order:

```rust
#[on_event(input = "imu")]
fn on_imu_pressure(&mut self, event: BackpressureEvent) { /* ... */ }

#[on_event(input = "imu")]
fn on_imu_deadline(&mut self, event: ExpectWithinEvent) { /* ... */ }
```

Two handlers with the **same (port, event kind)**, e.g. two `BackpressureEvent` handlers both on `"imu"`, are a compile error.

### Multi-consumer fanout: `block` degrades gracefully

`block` defers the producer, so it is only safe to install when the producer can be deferred without starving someone else:

| Topology | Behavior |
|---|---|
| Every consumer of the topic declares `block` | Pre-fire defer is installed. The producer is deferred when any consumer's queue is full (`backpressure_block_fires_deferred_count` bumps). No data lost. |
| **Mixed**: some `block`, some non-`block` consumers | Deferring the producer would starve the non-`block` siblings, so the `block` consumers are **degraded to `drop_oldest`** with a loud `tracing::warn!`. The producer fires normally. The degrade is end-to-end: the degraded input gets a real eviction detector: `backpressure_drop_oldest_count` counts overflow evictions and its `#[on_event(input = "...")]` handler (with a `BackpressureEvent` parameter) receives **`DropOldest`** events (`dropped > 0`), not `Block` ones. |

This is built-time topology analysis (`is_all_block` on the topic flow), not a runtime decision; it protects against one `block` consumer DOSing every other consumer on the same topic.

### Topic provisioning

Graph topics are provisioned from the topology (no knobs to tune):

| Service setting | Value | What it means for you |
|---|---|---|
| Subscriber buffer | your declared `#[input(depth = N)]` (≤ 64) | the depth IS the real queue, honored even above the transport default |
| `subscriber_max_buffer_size` (ceiling) | `max(transport default, largest consumer depth)` | `cerulion topic echo`/`hz` always attach |
| `max_subscribers` | in-graph subscribers (bodies + trigger drains) + **5 extra slots**: four spare for tools, one budgeted for the gateway's standing topic-liveness observer | up to 4 simultaneous introspection tools per topic (`--record`, `topic echo` / `hz` / `info`, a viz attach) beside the observer; the next one fails loudly with the count + remedy |
| `max_publishers` | **1** (single-writer) | a rogue second publisher (another graph, a raw transport user) is rejected by iceoryx2 itself at port creation; multi-publisher topics need the explicit `multi_publisher_topics:` opt-in. On a service some default opener created FIRST (where the port cap can't enforce: pre-existing services keep their slots), the graph itself refuses to attach its publisher when one is already connected; two graphs publishing the same topic error either way, naming the cross-graph collision |
| `multi_publisher_topics:`-listed topics | publishers **16**, subscribers **16**, buffer ceiling **16** (shared loose constants) | cross-graph topics need graph-INDEPENDENT provisioning on all three knobs (per-graph values would lock the second graph out at open); slots are shared across every graph + tooling on the topic, exhaustion fails loudly at port creation, and a single graph exceeding a cap on its own is rejected at build |
| external topics (absolute `source:`, no in-graph producer) | iceoryx2 defaults (**buffer ceiling only**) | the graph doesn't own the topic: no port requirements imposed (a foreign creator's provisioning can't fail the graph at open), out-of-graph publishers attach freely; the buffer-ceiling requirement stays real (a too-shallow foreign service rejects the graph loudly) |

Event-service listener/notifier caps are provisioned as subscribers + publishers (each attacher of either kind uses one listener + one notifier), so the spare slots hold for the whole attach (data + events). Declaring `history_size` larger than a consumer's `depth` warns at build: that consumer's queue can only retain the newest `depth` replayed frames.

### Determinism (Principle #7)

All policies, and their `BackpressureEvent`s, are deterministic under `VirtualClock`:
- `drop_oldest`: iceoryx2 queue-driven, not clock-driven; deterministic by construction. The eviction *event* is keyed off the **wire `sequence`** gap (publish-side, monotonic per producer), not wall-clock; replay-identical.
- `sample(N)`: read-gate keys off the **wire `timestamp_ns`** (publish clock), not `Instant::now()`; replay-identical to live.
- `block`: pre-fire defer AND the consumer event are driven by the `outstanding` counter vs the threshold (queue depth), not wall-clock; deterministic.
- `throttle_ms`: reads the scheduler's canonical clock (`now_ns`), not `Instant::now()`; replay-identical.

Two runs of the same graph with the same input sequence and `VirtualClock` advance pattern produce bit-identical accept/drop/evict/defer sequences, counter values, and `BackpressureEvent` queues across all four policies.

### Not supported

- **Adaptive backpressure** (policy switch under load, queue-depth auto-grow) is not supported.

---

## Splitting a `block` edge across processes: the credit word

`block` is lossless by declaration (Principle #6): the producer defers its own fire while the
consumer is at depth, so nothing is evicted. Inside one process the producer reads the
consumer's `outstanding` counter directly. Across a process boundary it cannot, so
a `block` edge whose producer and consumer land in different `process_groups:` cannot
use it.

Such a split is accepted **when the edge is CREDITABLE**, and the supervisor mints it a
**cross-process credit word**: a shared-memory page both ranks map, carrying the same
`outstanding` mirror the co-located case reads from the heap.

### What "creditable" means, and the two refusals you can actually hit

An edge is creditable with **exactly one in-graph producer and no non-`block` consumers**.
Every other shape is refused **before any worker spawns**, naming which bar it hit
(`CreditBar` in `crates/cerulion_core/src/graph/topology.rs`).

Two of `CreditBar`'s four variants are reachable as a refusal here. A colocation seed
requires both an in-graph producer and a `block` consumer, so `NoInGraphProducer` and
`NoBlockConsumer` cannot arise on this path; the renderer still renders them rather than
failing, and both are listed below, marked as unreachable:

| bar | what it means | the fix |
|---|---|---|
| `NoInGraphProducer` (**not reachable here**) | nothing in this graph publishes the topic, so there is no fire for the scheduler to defer | not a partition problem: `GraphTopology::validate` already refuses `block` on a producer-less topic with its own message, which is why a seed never carries this bar |
| `MultipleProducers(n)` (always `n >= 2`) | the topic is `multi_publisher_topics`-listed with two or more in-graph writers. The word counts ONE producer's outstanding frames, so two writers would each spend the other's credit | co-locate the flow, or reduce the topic to a single in-graph producer |
| `MixedTopic { first, rest }` | the topic has BOTH `block` consumers and non-`block` siblings, so the `block` ones are already degraded to `drop_oldest`: there is no lossless defer left to credit. The variant carries the siblings because they are the nodes you have to move | move the named siblings into the producer's group, or make them `block` too |
| `NoBlockConsumer` (**not reachable here**) | one producer and no `block` consumer at all (no consumers, or only non-`block` ones) | nothing to credit, and nothing to refuse: a seed requires a `block` consumer, so this edge never reaches the bar |

The DERIVED partition never produces any of these: it co-locates a `block` topic's whole flow
regardless, because crossing a boundary costs a real hop whether or not the edge is creditable.
The bars exist for a HAND-WRITTEN `process_groups:` block.

**`cerulion graph levels` tells you before you run.** Its partition verdict NAMES the creditable
split edges rather than just saying the partition is spawner-consumable, so a credited split
and an ordinary co-located flow never render the same line: the answer to "will my hand-written
split work?" depends on N supervisor-minted credit words, and the verdict says so
(`render_levels_report` in `crates/cerulion_cli_engine/src/graph_cmd.rs`):

```
  partition: spawner-consumable (1 split `block` edge(s) creditable: /p/n1/out -> n2.inp (front -> back); judged on source metadata — `graph run` re-checks the built cdylibs and refuses on drift)
```

(one line as emitted by `render_levels_report`, two-space indented; wrapped here only by your pager.)

Note the caveat the line carries itself: `graph levels` judges on SOURCE metadata, while
`graph run` re-checks the built cdylibs and refuses on drift, so a stale `.so` can still turn
a green `levels` into a refused run.

And if you hand-write a creditable split and then re-derive, you are told that it was REPLACED,
past tense, deliberately: the warn fires strictly AFTER `write_yaml_atomically` has committed the
new block (`warn_creditable_split_overwritten` in
`crates/cerulion_cli_engine/src/partition_emit.rs`, called from
`graph_partition` and from `run_auto_partition_preflight`, so
`graph run --auto-partition` reaches it too, not just the `graph partition` verb). It is a
notification that the file has already changed, not a prompt you can answer; the `.bak` the
write leaves behind is what you recover from.

### When the consumer dies: `--peer-loss continue`

Under `--peer-loss continue` a worker death leaves the survivors running degraded. On a
credited edge that is a specific hazard (the dead rank was the consumer whose `outstanding`
the producer is gating on), so the supervisor reports it. The head is a `warn!`
(`CreditDeathWatch::report_deaths` in `crates/cerulion_cli_engine/src/graph_cmd.rs`) carrying, as structured fields:

```
group  node_id  topic  consumer_input  dead_rank  deferred_producer_ranks
deferred_producer_groups  total_failures
```

with the message *"a cross-process `block` edge lost its CONSUMER — its credit …"*.
`deferred_producer_groups` names the GROUPS, not the batch: it renders a real group name per
producer rank rather than a constant. A rank whose group cannot be resolved renders
`<rank N>` rather than being filtered away: an unknown rank is
reported loudly, because dropping it would under-report who is stuck.

Three further behaviours, all on the shared `FailureRegimeLatch`:

- **The report is flood-latched.** One loud head per regime; repeats are `debug!`
  (`RegimeDecision::Suppressed`); an open regime re-announces at each decade of the
  running total as a `warn!`, *"a cross-process `block` edge is STILL without its consumer"*,
  and `total_failures` is unconditional, never reset by recovery.
- **A producer named alive and then killed is RETRACTED.** If an earlier line reported a
  producer as deferred-but-not-known-dead and that producer later dies, a `warn!` says so
  (field `dead_producer_group`): *"RETRACTION: a producer this edge previously
  reported as deferred (not known dead) has now died"*. The earlier line stays in the log as
  written, and the retraction is the update to read alongside it.
- **Under free-run, the departure line names its dead groups** (`groups=`,
  `warn_free_run_departure`).

### Scope: what is armed and what is not

**The producer park is armed.** A producer deferred at a
cross-process `block` gate whose edge is backed by a MAPPED credit word parks on that word:
the live loop claims its bit via `ParkedEdgeGuard` and blocks on the word's `wake_seq`
(`crates/cerulion_core/src/graph/runtime.rs`, the credit arm of `monitor_wait_block`), and the
consumer's drain rings it. The block is BOUNDED by the park deadline, so a consumer that stops
draining costs latency, never a wedge. Two switches control it, independently of each other and
of the barrier's: `CERULION_CREDIT_WAKE=0` turns the plane off entirely (the watch list stays
empty, so predicate, routing and kernel block are all skipped), and on macOS
`CERULION_CREDIT_OS_SYNC=0` drops just the kernel-wake tier to the sleep-recheck fallback. Both
are documented under [Environment variables](#environment-variables).

So the supervisor's sweep clears a bit production really does set.
Note the qualifier that applies here: `BarrierShared::park_enter`
is a DIFFERENT function on the barrier's own word (`crates/cerulion_core/src/barrier.rs`), and the two
planes keep separate switches on purpose.

One limit on scope: a producer whose edge-local slot is at or beyond
`PARKED_MASK_BITS` (32) claims no bit and degrades to the slice cadence rather than parking on
the word: correct and bounded, simply without the wake-syscall win. Pinned by
`credit_test::a_slot_beyond_the_mask_claims_no_parked_bit` (the bit claim, read while parked),
`credit_block_iox2_test::prc_a_slot_beyond_the_parked_mask_degrades_to_the_slice_cadence`
(the degradation, against an in-mask control) and
`credit_block_iox2_test::prc_a_lockstep_barrier_rank_credit_blocked_at_depth_does_not_wedge`
(a lockstep rank holding a barrier participant AND blocked at depth still returns from the
park).

### Pinned by

| contract | test |
|---|---|
| the credit word's own semantics | `crates/cerulion_core/tests/credit_test.rs` |
| `block` over a real credited edge | `crates/cerulion_core/tests/credit_block_iox2_test.rs` |
| free-run construction | `crates/cerulion_core/tests/free_run_ctor_iox2_test.rs` |
| the death report over REAL processes: the loud head and its field set, the ABSENCE of a repeat, the retraction, the free-run `groups=` | `crates/cerulion_cli/tests/credit_death_e2e_test.rs` |
| the latch's DECADE re-announcement (`"STILL without its consumer"`) | `crates/cerulion_cli_engine/src/graph_cmd.rs` (in-crate tests) |

---

## Environment variables

### accountd Supabase Auth

When `cerulion-accountd` is configured with Supabase Auth, clients exchange a
verified Supabase access token at `POST /v1/auth/supabase/exchange`:

```json
{ "access_token": "<supabase-jwt>", "user_code": "<optional-device-user-code>" }
```

Without `user_code`, the response contains `session_token`, `refresh_token`,
`token_type: "Bearer"`, and `expires_in`. With `user_code`, a valid token
authorizes the pending device login and returns `{ "status": "authorized",
"user_id": "..." }`. Invalid or expired tokens return
`401 Unauthorized` without exposing claims.

Set these variables in the accountd process to enable the provider:

| Variable | Meaning |
|---|---|
| `CERULION_ACCOUNTD_SUPABASE_ISSUER` | Required issuer, typically `https://<ref>.supabase.co/auth/v1`; presence enables Supabase Auth. Surrounding whitespace is trimmed; an empty value is treated as unset. A derived JWKS URL inherits the issuer's scheme. |
| `CERULION_ACCOUNTD_SUPABASE_AUDIENCE` | Accepted JWT audience; defaults to `authenticated`. Surrounding whitespace is trimmed; an empty value is treated as unset. |
| `CERULION_ACCOUNTD_SUPABASE_JWKS_URL` | JWKS endpoint; defaults to `<issuer>/.well-known/jwks.json`. The URL must use `https://`; `http://` is allowed only for `localhost`, `127.0.0.0/8`, or `[::1]`. Surrounding whitespace is trimmed; an empty value is treated as unset. |
| `CERULION_ACCOUNTD_SUPABASE_JWT_SECRET` | Optional legacy HS256 secret. HS256 is unavailable when unset; surrounding whitespace is trimmed; an empty value is treated as unset; fetched JWKS keys never authorize HS256. |

Cached Supabase signing keys are re-validated against the JWKS endpoint at least
hourly of process uptime, so removed keys stop verifying after the cache maximum
age. Wall-clock steps do not affect key caching; they only affect token expiry.

Anonymous Supabase sign-ins (`is_anonymous: true`) are refused; only real
accounts can mint accountd sessions.

An unset issuer leaves Supabase unconfigured and the exchange endpoint refuses
loudly with the same provider-not-configured response as other optional
identity providers.

### Logging

| Variable | Meaning |
|---|---|
| `RUST_LOG` | Standard `tracing` filter, e.g. `RUST_LOG=cerulion=info,iceoryx2=warn`. The workspace's `.cargo/config.toml` sets a sane default. |
| `IOX2_LOG_LEVEL` | iceoryx2's own log filter. The workspace defaults this to `error` so the framework's chatter doesn't drown out user logs. |

With `RUST_LOG` unset the filter is `info`. Lifecycle bookkeeping (startup sweeps, worker drains, exit hygiene and the recorder's clean-finalize accounting) is logged at `debug`, so `recording written to <path>` is the last line of a recorded run and a verification's `replay PASS` is not preceded by paragraphs of accounting. The long-running verbs (`graph run`, the recorder, the daemon) still print their `info` lifecycle lines, such as worker spawn, trace ring and ready-file names, topic provisioning and daemon boot. A release build of `cerulion` (which hosts the recorder, `cerulion bagd`) and of `cerulion-netd` compiles `debug` and `trace` out entirely (`tracing`'s `release_max_level_info`), so the demoted lines do not appear there at any `RUST_LOG`. Binaries built without cargo's `--release` keep them, and `RUST_LOG=debug` shows them there. That is cargo's flag for building `cerulion` itself; the `--release` flag of `cerulion graph run` and `cerulion node build` selects the node crate's profile instead.

### Node (cdylib) logs

A node compiled as a cdylib (the default for `cerulion graph run` / `cerulion node run`) statically links its own copy of the framework and of `tracing`, so its `tracing` dispatcher is separate from the host process's. To make your node's `tracing::info!`/`warn!`/`error!` (and the framework's own loud node-side diagnostics, e.g. the "OutputProxy dropped without writing all declared variable fields" discard error) visible, **macro-generated nodes (`#[cerulion_node]`) install a cdylib-local `stderr` subscriber at node init**. A legacy `--raw-ffi` node does not get this, so its own `tracing` output is not shown: one more reason to stay on the macro. The filter comes from `RUST_LOG` **as captured in the graph's environment snapshot at build time** (not live `std::env`, for determinism), and an absent/empty `RUST_LOG` defaults to `info`. When `RUST_LOG` is set, each loaded cdylib prints one breadcrumb line on stderr naming the filter it applied (`node-side logs -> stderr (filter: ...)`; a spec that fails to parse is reported there too, and falls back to `info`); under the default nothing is announced.

Node-side logs go straight to `stderr`, **bypassing any host-side subscriber's filtering/formatting**: by default the host's own logs go to `stdout`, node-side logs to `stderr`. Setting `RUST_LOG=off` in the graph environment silences node-side logs (the one-line filter announcement still prints, because `RUST_LOG` is set). Note also that release builds compile out `debug`/`trace` node-side (`tracing`'s `release_max_level_info` in `cerulion_core`), so `RUST_LOG=...=debug` on a release dylib graph cannot deliver below `info`; build the node without `--release` to get those levels. The `debug-logging` cargo feature does not lift that ceiling (see [Cargo features](#cargo-features)).

The framework's subscriber is installed **first**, at node init, before any of your node code runs, so it wins. Do **not** install your own global `tracing` default inside node code (e.g. `tracing_subscriber::fmt::init()` in `init()` panics on "global default already set", which fails the node load); use `RUST_LOG` to control output instead. A global default that genuinely pre-exists node init (e.g. installed by a link-time constructor) is respected; the framework's install is then a no-op.

### Live-loop tuning

Live-run tuning knobs read by `cerulion graph run` (and its multi-process
workers). All are optional; the defaults are the recommended production
shape. Values are matched exactly where noted; a malformed value never
silently changes behavior (it warns and falls back to the documented
default: loud over silent).

| Variable | Meaning |
|---|---|
| `CERULION_MONITOR_WAIT` | Live-loop monitor-wait **park** override. `1` forces the park on, `0` disables it (exact match; any other non-empty value warns and uses the auto default). **Auto default: ON for live runs** (`--time-source real`/`external`), off for `--time-source virtual`. On Linux x86_64 (WAITPKG) / Linux aarch64 the park is a real shallow CPU monitor-wait (`UMWAIT`/`WFE`) that slices at 20 µs and **yields the core after every slice** (a monitor-wait-parked thread is RUNNING to the OS scheduler, so without the yield a co-located runnable process waited a scheduler tick, ~7 ms, whenever graph processes outnumber cores). On aarch64 the effective slice is the generic-timer event-stream period (base `WFE` has no timeout operand), so a same-core peer wakes within ~one period there (machine-specific, and on an Orin-class board it is around 130 µs; x86 `UMWAIT` honors the 20 µs slice); cross-core wake latency is unaffected. On targets **without** a CPU monitor-wait primitive (macOS, x86 without WAITPKG) the park runs in the **degraded sleep-recheck tier**; see below. The `--no-monitor-wait` flag beats this env in both directions. **Also read by `rmw_cerulion`'s event-driven `rmw_wait`** through the same classification; there unset means the measured platform default (park on aarch64, off on x86), `1` forces the rmw park on, `0` off; see "ROS 2 rmw wait tuning" below. |
| `CERULION_DOORBELL` | Data-**doorbell** arm of the park: `1` forces it on (a producer's publish rings a shared-memory line that wakes the parked consumer instantly; implies the park), `0` disables it (timer-only park). Exact match; near-misses warn and use the auto default (**ON for live runs**). Effective on Linux only: on no-primitive targets the doorbell ring is a no-op stub, so it is **forced off** with a warning if you request it, and it does not imply the park there. **Native live loop only:** `rmw_cerulion` does NOT read this flag; its publishers always arm the topic doorbell (a single atomic store per publish; inert if nobody parks) and its `rmw_wait` park tier is governed by `CERULION_MONITOR_WAIT` alone. |
| `CERULION_CPU_DMA_LOCK` | CPU C-state cap override (Linux; needs root / `CAP_SYS_NICE` for `/dev/cpu_dma_latency`: a failed acquire warns and the run continues). `1` forces a hard C0 pin, `0` disables the cap (exact match; near-misses warn and use the auto decision). **Auto default:** a graph-derived exit-latency cap computed from the graph's tightest timing, and **skipped entirely while the monitor-wait park is active** (they are alternative shallow-idle mitigations). The `--no-cpu-dma-lock` flag disables the auto cap. No-op off Linux. |
| `CERULION_CPU_DMA_LOCK_US` | Explicit C-state exit-latency cap in µs (e.g. `=10`), used under the auto mode. A valid non-negative integer wins over the graph-derived cap (but **not** over `CERULION_CPU_DMA_LOCK=1`/`=0`); malformed/negative values warn and fall through to the graph-derived cap. |
| `CERULION_BARRIER_SPIN_US` | Multi-process only (Unix): the barrier-boundary **spin-then-block** budget in µs at each cross-process DAG-level boundary: the bounded spin runs before the blocking tier (the futex sleep on Linux; on macOS ≥ 14.4 the `os_sync_wait_on_address` kernel wake, with the chunked ~100 µs sleep-recheck as the fallback on older hosts / under `CERULION_BARRIER_OS_SYNC=0`; one knob, every tier). Default **20** on Linux and on macOS with the os_sync tier active (the kernel-wake knee); **150** on the macOS sleep-recheck fallback (covers its ~100 µs arrival skew). `0` is the kill switch; the bounded spin is skipped entirely: on Linux the hard-bounded 50-iteration read loop + futex block; on macOS straight to the blocking tier (no pre-block iteration loop exists on the macOS paths). Values above **100000** (100 ms) are clamped with a warning so a typo can never turn the bounded spin into a busy-spin; garbage warns and uses the default. |
| `CERULION_BARRIER_OS_SYNC` | macOS only (no-op elsewhere): the barrier's kernel-wake tier (Apple `os_sync_wait_on_address` / `os_sync_wake_by_address_all`, macOS ≥ 14.4, resolved at runtime via `dlsym` so older hosts degrade cleanly instead of failing at launch). `0` disables it: barrier waits take the chunked ~100 µs sleep-recheck fallback (and the spin default above resolves to 150 µs), and the step-start park's barrier-arrival kernel wake (the wake word a parked context blocks on so a peer's arrival wakes it in µs) is disabled with it, restoring the polled ~100 µs cadence. `1`/unset = enabled (the default); anything else warns and stays enabled (exact match; loud over silent). |
| `CERULION_PARK_OS_SYNC` | macOS only (no-op elsewhere): the live-loop **park nap**'s kernel-wait tier. On a host with no CPU monitor-wait primitive the park is a bounded recheck nap, and on macOS ≥ 14.4 each nap is an `os_sync_wait_on_address` timed wait (roughly half `nanosleep`'s timer-coalescing overshoot). `0` disables that tier and restores the plain sleep-recheck nap; `1`/unset = enabled; any other value warns, naming this variable, and keeps it enabled. See `docs/deployment_tuning.md`. |
| `CERULION_CREDIT_WAKE` | Multi-process only (Unix): the **producer-side credit park**. A producer deferred at a cross-process `block` gate blocks on that edge's shared credit word, and the consumer's drain wakes it; `0` turns that off, so the producer keeps the park's recheck cadence instead. Correct and bounded either way (the pre-fire gate is unchanged and re-derives on every pass), so the only thing `0` costs is wake latency. `1`/unset = enabled (the default); anything else warns and stays enabled (exact match; a typo must not silently disable a latency plane, because the result is correct output at collapsed latency, which nothing downstream can notice). Inert on any run that splits no `block` edge: the watch list is empty, so the whole plane costs one cached read. |
| `CERULION_CREDIT_OS_SYNC` | macOS only (no-op elsewhere): the **credit** word's kernel-wake tier, the credit-plane sibling of `CERULION_BARRIER_OS_SYNC` above. `0` disables it: a credit-blocked producer takes the chunked sleep-recheck fallback instead of the `os_sync` kernel block. `1`/unset = enabled; anything else warns and stays enabled. It controls credit waiting independently of `CERULION_BARRIER_OS_SYNC`: both words ride one backend and one unrecoverable-errno latch, but each switch speaks for its own plane, so disabling the barrier's macOS wake tier says nothing about the credit plane. `credit_os_sync_independence_test` drives both switches in both directions in subprocesses (the switches are process-cached, so one process can observe only one combination) and fails if the credit tier consults the barrier's switch. |
| `CERULION_FIRE_THREADS` | Within-level parallel-fire pool size. Read **once at graph build**. Default: `min(available_parallelism, widest DAG level width)`, at least 1. A positive integer is honored; `0` or garbage warns and auto-sizes. Thread count never changes **what** fires or its recorded order (the level structure fixes that), only how a wide level's independent fires are spread across cores. |
| `CERULION_LIVE_SPIN_US` | Live-loop **spin-then-block** budget in µs before the loop parks. **Default (unset): derived**. The loop spins only while a wake is imminent per the graph (a `period_ms` node due within one hop, or the last step just fired), and blocks immediately during predicted idle, so a quiescent graph still gives its core back. `0` disables the spin outright (always block immediately). A positive `N` is a fixed manual ceiling, **clamped to 100 000 (100 ms)** with a one-time warn: the spin is a pre-block latency optimization for an imminent wake, never a polling loop. Garbage warns and falls back to derived. Reach for it when you are measuring the last microseconds of wake latency on a pinned core; leave it alone otherwise. **Also read by `rmw_cerulion`'s event-driven `rmw_wait`** through the same parse and the same 100 ms ceiling, with ONE asymmetry: `unset` there means NO spin (the rmw wait has no graph to derive an imminent wake from, so its default is park-first), not a derived spin; see "ROS 2 rmw wait tuning" below. |

### Sign-in environment variables

| Variable | Meaning |
|---|---|
| `CERULION_ACCOUNT_SERVICE` | Base URL of the account service `cerulion login` (and Cerulion Studio, which shares `~/.cerulion/auth.json`) signs in to. Default `https://app.cerulion.com`, the hosted issuer, so a CLI sign-in and a web sign-in are ONE identity and `account_id` is the web account's own id. Set it to run against another service, e.g. `http://127.0.0.1:8787` for a local `cerulion-accountd`. A trailing slash is trimmed; an empty/whitespace value falls back to the default. (Cerulion Studio applies one extra rule to the same variable: it refuses cleartext `http://` off the loopback literals, because its Team panel posts the session to that origin.) |

A service is only required to serve the sign-in surface (`POST /v1/auth/device/start`,
`/v1/auth/device/poll`, `/v1/auth/refresh`, `/v1/auth/revoke`, `GET /v1/me`). Device
**certificates** (`/v1/devices*`) are `cerulion-accountd`'s surface and the hosted default
does not issue them. Against such a service:

- `cerulion login` takes the account id from `/v1/me` and caches no
  `~/.cerulion/device.cert`. Only a **404** on the device surface reads as "certifies
  nothing": a registration that fails at the transport or with a status of its own (401,
  5xx) leaves the outcome unknown and **refuses the login**, rather than deleting a cert
  the service does issue and committing the switch with no certified key. A cert an
  earlier login cached is **cleared** (a rename to a hidden sibling, described
  below, not a delete), because it
  attests this machine's key under the account that issued it and device-binding
  resolution treats it as authoritative; keeping it would leave the desk bound to the
  previous account while `auth.json` names the new one. The clear happens only **after**
  `/v1/me` names the new account: a login that fails leaves the working cert alone. It
  covers every path a consumer reads: `~/.cerulion/device.cert` plus, when set,
  `CERULION_NETD_DEVICE_CERT` (or the `device.cert` sibling of `CERULION_NETD_DESK_KEY`),
  since `cerulion-netd` resolves the cert from its own environment. Both variables must be
  **absolute**: `cerulion-netd` resolves a relative path against ITS working directory, not
  the one `cerulion login` ran in, so the same value would name two different files and
  neither the clear nor the caching could know which one netd reads. A relative value is
  refused by name rather than resolved against the CLI's own directory.
  Both variables are checked on **every** login, including the ones that clear nothing (a
  re-login to the same account, an issuer that certifies nothing on a machine with no cert
  cached): a relative override is a property of the configuration, not of the branch a
  particular login takes.
  Every path is attempted
  even if one refuses, so no switch leaves some consumers cleared and others not; a cert
  that cannot be cleared fails the login, naming the paths that are still there, and the
  ones already cleared are PUT BACK: the clear is all-or-nothing, since a desk holding
  the netd copy but not its own reads two different answers about which account it is.
  "Cleared" is a **rename** to a hidden sibling of the same name
  (`.device.cert.superseded.<account>`), not a delete, and three properties follow
  from that. A cert this machine cannot READ (a mode, an ACL,
  a type that refuses `read`) is cleared like any other, because a rename never opens it.
  A symlink stays a SYMLINK, at the same target, when the clear is undone, never a
  regular file holding the bytes it resolved to, which would freeze one account's cert at
  the path and take it out of whatever rotates the target. And an interrupted login is
  recoverable: see below. A symlink pointing at nothing is an entry, not an absence, and
  is cleared too; otherwise its target reappearing would make the previous account's cert
  authoritative again. Cert writes take the same store lock, so a certifying login on
  another process cannot land inside the window between a clear and its rollback.
  The clear and the `auth.json` write are ONE commit, taken under the store lock, and
  the clear goes **first**: the two files are separately durable, so one interruption
  window has to exist, and a kill after publishing `auth.json` would leave the new
  account's tokens beside the OLD account's cert, which device-binding resolution reads
  as truth. A kill in that window is then **repaired by the next login**, which is what
  the account tag in the aside's name is for: the next `cerulion login` puts the aside
  back when the store still names the account it certifies (the interrupted login
  published nothing, so that sign-in is still the live one and it keeps its device
  binding), and drops it when the store names anyone else (the cert would bind this desk
  to an account it is not signed in to). A write that then fails puts the certs back, and
  if one cannot be put back the failure says so rather than claiming the previous sign-in
  survived intact.
  The put-back does not wait for a login: any command that RESOLVES the device binding
  runs it when the cert it needs is absent, so a login killed in that window is finished
  by the next command that needs the binding. The certs are also published one path at a
  time, so an interruption there leaves some consumers holding the cert and the rest
  holding nothing; a consumer holding NOTHING is cached to by the next certifying login
  (whether or not that login cleared anything there), and by any command that resolves
  the binding, which caches the cert it has just **verified** (it decoded, it attests
  this machine's device key, and `auth.json` names the account it binds it to) at every
  consumer with none. A cert that fails any of those checks is copied nowhere: spreading
  one on the strength of its bytes is how a previous account's cert would come back after
  a switch. The same holds in the other direction: when the CLI's own path is the one the
  killed login never reached, a binding only a `CERULION_NETD_*`-relocated cache holds is
  **adopted** from there, after the same two checks, rather than reported absent, and
  the propagation then fills the CLI's own path from it. A relocated cache naming another
  account is left where it is and the binding reads as absent, which is what
  `cerulion login` fixes; one that is present but not a regular file (including a
  symlink whose target no longer exists) is named, not reported as an absence
  somewhere else. The account is re-checked against the store
  under the store lock at the moment of the copy, so a `cerulion login` that switches
  accounts while another command is resolving wins: the copy is skipped rather than
  refilling the paths that login just cleared. A consumer holding something (of any kind, including a symlink something
  else rotates) is never rewritten. A cert path that is not a regular file (a named
  pipe, a socket, a device node) is refused by kind, naming what is there, rather than
  opened: reading one can block forever, and a command that hangs silently is worse than
  one that says what is wrong.
- `cerulion account devices list` / `revoke` fail with a message naming this variable
  rather than a bare 404. Point it at a service that registers devices (a local
  `cerulion-accountd`) to use them. `revoke` distinguishes the two 404 readings by asking
  `GET /v1/devices` whether the service registers devices at all; only a 404 (no surface)
  and a success (a surface this id is not on) are answers; a probe that fails, at the
  transport or with a status of its own (401, 5xx), makes the error say the outcome is
  **unknown** rather than reporting a missing device.

Against a service that **does** issue certificates, the same commit rule holds with the
new cert added to the end of it: the previous account's cert is cleared, `auth.json` is
published, and only then is the freshly issued cert cached, all inside ONE hold of the
store lock, so no window, and no interruption inside one, leaves a cert and an `auth.json`
naming different accounts. A store write that fails puts the previous cert back and caches
nothing, so the machine keeps exactly the sign-in it already had. Every path the new cert
is destined for is prepared BEFORE `auth.json` is published, and one that cannot take it
(an unwritable directory, a path that is not a file) **refuses the whole login** with
nothing published: a session published against a cert only SOME consumers hold would leave
`cerulion-netd` reading no binding at all under an account it is signed in to. The prepared
writes are then published one per consumer, and a failure part-way through leaves some
consumers on the new cert and the rest on nothing: the command **reports that as a failure**
naming the path, rather than printing `Signed in as …` for a desk whose consumers are not
all bound. The ones that succeeded **keep** the cert: every one of them names the account
just signed in, so nothing is misbound, and emptying them again would turn a partly bound
desk into an unbound one, including a path that held a perfectly good cert for this same
account before the login touched it. The message names which paths hold it; what is left
empty is filled by the next command that resolves the binding, or by the next
`cerulion login`. The session itself is published and stays (nothing on disk names another
account). `~/.cerulion/device.cert` is the CLI's OWN file: a
symlink there is REPLACED by the cert rather than written through, because following it
would write a secret wherever the link points. The one link left alone is one that already
resolves to **exactly this cert**, a same-account re-login, where the write would change
nothing but the entry's kind and would take the path out of whatever rotates the link's
target. The clear runs only when the account
actually **changes**: re-signing in to the same account rewrites `~/.cerulion/device.cert`
in place and leaves a `CERULION_NETD_*`-relocated copy of that same account's cert alone.
When it does run, the new cert is cached at every path it emptied as well as at
`~/.cerulion/device.cert`, so a `CERULION_NETD_*`-relocated cache holds the new account's
cert rather than nothing: `cerulion-netd` resolves that path itself, and an emptied one
would leave the desk with no device binding after a login that succeeded.

A local `cerulion-accountd` serves the sign-in API only: it advertises `{origin}/v1/auth/device`
as the RFC 8628 verification URI, but **no browser page is served there**: authorizing a
local device code means driving the daemon's own API (`POST /v1/auth/magic-link/start` with
the `user_code`, then the emailed `GET /v1/auth/magic-link/complete` link, or the
`/v1/auth/oauth/{provider}/start` flow). Front it with your own approval page, or sign in
against the hosted default, if you want a URL a user can open. It reads
`CERULION_ACCOUNTD_BIND` (default `127.0.0.1:8787`) and forms that URI under whichever
origin it is **listening** on: move it to another port and the verification URI moves with
it, and an ephemeral bind (`127.0.0.1:0`, where the
kernel picks the port) prints the port it actually got rather than `0`. A wildcard bind
(`0.0.0.0:PORT`, `[::]:PORT`) resolves
to the loopback of that family, since a wildcard is not an address a browser can open. Set
`CERULION_ACCOUNTD_VERIFICATION_BASE_URI` when it is reachable under some other origin (a
tunnel, a LAN address); reachability from elsewhere is the one thing the daemon cannot
infer from its own socket.

### macOS / no-primitive park behavior

On targets without a CPU monitor-wait primitive (macOS, x86 without WAITPKG),
the live-loop park is **on by default** and degrades to a **chunked ~100 µs
bounded sleep-recheck loop**: it can never busy-spin, and the condition is
rechecked roughly every 100 µs, so an event arriving mid-park waits for the next
recheck plus whatever the OS scheduler adds. It is on by default there
because, measured against the plain blocking wait, whose median wake latency
is unstable from run to run, the degraded
park holds a stable and lower one.

The tier is logged once at the first park (`DEGRADED sleep-recheck tier …`).
Opt out with `CERULION_MONITOR_WAIT=0` or `--no-monitor-wait`; both restore
the plain blocking wait.

**Graphs with `external`-policy ingress nodes** (fd / Blocking sources) park
like every other graph, on every tier: the park's recheck loop **polls each
external fd directly**
(level-triggered, non-blocking) alongside its listener and data-doorbell
checks, so fd readiness wakes a parked loop within one recheck slice. The
slice, and so the fd wake bound, is per tier: **20 µs** on the x86
WAITPKG tier (`UMWAIT` honors the requested deadline); **one generic-timer
event-stream period** on the aarch64 `WFE` tier (base `WFE` has no timeout
operand, so the requested 20 µs is quantized up to the period, which is
machine-specific); **~100 µs** on the degraded
sleep-recheck tier.
Without that poll a park would never watch external fds at all and an
ingress node would wake only at the ≤250 ms park timeout, throttling a lidar
to a few Hz. Wake attribution is observable via the `wakes_external_fd` field in
the live-loop exit telemetry (Principle #3). Note the C-state cap table entry
above: the cap is skipped while the park is active, which includes
external-fd graphs on all tiers.

### ROS 2 rmw wait tuning

Read by `rmw_cerulion` (`RMW_IMPLEMENTATION=rmw_cerulion`) inside the ROS 2
process, per `rmw_wait` call. `rmw_wait` blocks on the attached
entities' existing iceoryx2 event listeners (plus an fd doorbell per guard
condition) rather than sleep-polling, and on Linux it goes one
tier shallower: every rmw publisher arms its topic's shared-memory doorbell
at create, and a wait whose subscription topics have mapped doorbell pages
PARKS on them with the same CPU monitor-wait the native live loop uses
(`UMWAIT`/`WFE`; a bounded sleep-recheck where the CPU has no primitive), so
a publish wakes the waiter hardware-instantly. Whether that tier is ON is a
platform default set by measurement (see the `CERULION_MONITOR_WAIT` row
below): aarch64 parks by default, x86 does not: its `ppoll` kernel wait
measured faster at both 2 ms and 10 ms. Where it runs the park is bounded
and yields the core every recheck slice: 20 µs where `UMWAIT` honors the
deadline; on the aarch64 `WFE` tier (the park's platform-default home) one
generic-timer event-stream period per slice, which is machine-specific.
A wake then drains and probes only
the entities that fired (plus the rung topic), not the whole set. There are no rmw-only tuning
knobs: the wait reads the SAME `CERULION_LIVE_SPIN_US` and
`CERULION_MONITOR_WAIT` the native live loop reads (same parse, same ceiling,
the rows in "Live-loop tuning" above; their rmw-side meanings are restated
below), plus the one rmw-only kill-switch. The block-timeout ladder (first
rung 200 µs, doubling after 50 consecutive empty timeouts up to the 20 ms
pump cap; any wake resets it) is internal constants, not knobs. On Linux the
first `rmw_wait` on a thread also sets that thread's kernel timer slack to
1 ns (`prctl(PR_SET_TIMERSLACK, 1)`), deliberately thread-wide (no narrower
scope exists), so the
ladder's 200 µs `ppoll` rung does not pay the default 50 µs slack on every
empty wake.

| Variable | Meaning |
|---|---|
| `CERULION_LIVE_SPIN_US` | The SAME knob as in "Live-loop tuning" above, read by `rmw_wait` through the same parse and the same 100 000 µs (100 ms) ceiling. The ONE asymmetry: **unset means the consumer's own default, and the rmw wait's default is PARK-FIRST, no spin** (it has no graph to derive wake-imminence from; the native loop's derived default spins only while a wake is provably imminent). A positive `N` arms the µs busy-probe in front of the FIRST kernel block of each `rmw_wait` call, at most twice per call: once on entry, and at most one recovery spin after a wake whose probe found nothing (consumed on first use, never re-armed); never after an empty timeout. The entry spin is also bounded ACROSS calls: it is armed only by a previous call that RETURNED READY, and a call that delivered nothing disarms it (however its blocks woke), so neither an idle executor nor a notification-without-readiness storm spins more than once per idle period, even at the 100 ms cap (the value that is safe on the native loop's imminence-gated spin). It yields between probes. `0` disables (what unset already means here); malformed warns once and stays park-first. The measured basis for the park-first default: spin off is best-or-equal in every posture (stock 30 µs, C1 cap 19, C0 pin 17; 64 B ping-pong p50), while spin on put roughly half the processes into a per-lifetime ~123 µs same-core scheduling mode in the stock and C0 postures (never under a C1 cap). Ignored entirely under `CERULION_RMW_EVENT_WAIT=off`. |
| `CERULION_MONITOR_WAIT` | The SAME flag as in "Live-loop tuning" above, resolved by `rmw_wait` through the same classification, applied to a PLATFORM default that is measured, not chosen: **unset ⇒ the platform default: park ON on aarch64 (`WFE`), OFF on x86**; `1` ⇒ force the park on (on x86 the bounded one-rung shape, the experiment hatch); `0` ⇒ off (the wait blocks in `ppoll` on the entities' fds; still event-driven, µs wakes); any other non-empty value warns once and keeps the default. The basis (64 B rmw ping-pong p50): x86-64 2 ms park-on 30.1 vs **off 27.7**, 10 ms park-on 58.8 vs **off 19.8**; aarch64 (Jetson) 2 ms park-on **74.8** vs off 127.1, 10 ms park-on **84.3** vs off 135.7. The park, where it runs, is bounded and yields the core every recheck slice: 20 µs where `UMWAIT` honors the deadline, one generic-timer event-stream period (machine-specific, around 130 µs on an Orin-class board) on the aarch64 `WFE` tier that is the park's default home (a parked thread looks busy to the scheduler, so an unbounded park can starve a thread sharing its core). Elsewhere the topic doorbell is a stub and the flag changes nothing. rmw publishers open the topic bell unowned and never unlink it (a ROS topic may carry two publishers, so a dying one must not pull the page from under the survivor; a re-created publisher joins the same page), and a page replaced by an owned creator is re-mapped automatically on the first frame that arrives without a ring. Residual: one 64-byte page per topic can outlive every publisher on the machine until the next creator. Ignored under `CERULION_RMW_EVENT_WAIT=off`. |
| `CERULION_RMW_EVENT_WAIT` | The one rmw-only knob, the kill-switch. `off` restores the polling wait loop **verbatim**: probe, `sleep(100 µs)`, probe: **no fd block, no park, AND no spin** (`CERULION_LIVE_SPIN_US` is ignored, and the once-only `info!` says so). Everything else (the ready probe, the zero-timeout arm, the 20 ms publisher-event pump cadence) is identical. `on`/unset/empty = event-driven (the default); any other value warns once and stays ON (exact match, no case forgiveness). Independently of this switch, a wait set whose listener drain FAILS mid-call degrades itself to the same sleep pacing for the rest of that call (loud once, counted in `degraded_waits`) and retries the event path on the next call. |

### ROS 2 rmw slice ceilings

How big an SHM slot a bridged VARIABLE-layout topic reserves under
`rmw_cerulion` (`RMW_IMPLEMENTATION=rmw_cerulion`). Fixed-layout types are not
in play; the rmw sizes those exactly (wire header + fixed section). For
variable types the ceiling comes from the same five-tier per-type table
(16 KiB / 256 KiB / 4 MiB / 16 MiB / 128 MiB) that sizes graph-declared
outputs of the same schemas (one judgement, applied to both planes), with the
128 MiB catch-all for types the table does not list. A bridged topic has no
graph YAML to write `max_slice_len:` in, so the per-topic escape hatch is an
environment variable:

| Variable | Meaning |
|---|---|
| `CERULION_RMW_SLICE_CEILING` | Comma-separated `<pkg/Type>:<bytes>` list overriding the per-type slice ceiling for bridged variable topics, e.g. `sensor_msgs/Image:33554432,my_pkg/Big:268435456`. An override wins OUTRIGHT over the table (it can widen or narrow; the operator's explicit word is adopted verbatim). Type names must be EXACTLY `pkg/Type`, two non-empty components: never `pkg::Type` (`:` is the name/bytes separator) and never the rosidl `pkg/msg/Type` (the `msg` segment is not part of the wire name; that entry is skipped with a did-you-mean hint). Any other name shape is skipped loudly rather than stored: a stored name that can never match a bridge topic would be a silently-inert override. Each entry is trimmed of surrounding whitespace (a space after the list comma is fine); a name still carrying whitespace after that trim is skipped loudly. Bytes must be an unsigned integer in `[32, 4294967295]`: the 32-byte wire-header floor below, `WireHeader::total_size`'s `u32` above. A malformed entry is SKIPPED with a loud warn naming it (never a silent zero, never a crash), and the well-formed entries around it still apply; duplicate entries for one type warn and the LAST wins. **The rmw path honors this variable**: `rmw_cerulion` reads it ONCE per process, at the first publisher creation (so a malformed entry warns once per process, never once per publisher, and a value changed after that first creation is not re-read), and resolves every bridged VARIABLE topic's ceiling override-first, then the per-type table, then the 128 MiB catch-all; the resolved value really is the created iceoryx2 slot bound, so an over-ceiling publish fails its SHM loan loudly (`RMW_RET_ERROR`) instead of reserving a blanket slot. An entry naming a FIXED-layout bridged type is IGNORED with a warn at publisher creation: fixed types are always sized exactly at wire header + fixed section, and a narrower slot would refuse every publish. |

### Networking and identity

Read by `graph run`, `node run`, `ros2 attach` and the topic verbs. See
`docs/networking.md` for the model these knobs sit in.

| Variable | Meaning |
|---|---|
| `CERULION_NETWORK` | The network kill-switch. `off` runs LOCAL-ONLY: no gateway process, no zenoh session, a loud notice. `off` is the only accepted value; anything else leaves the permissive default in place. Honored by **any** entry point, so it is the way to silence a machine you do not control the command line of. The `--network off` flag is the same switch. |
| `CERULION_GATEWAY_PORT` | The port the network gateway listens on, instead of the well-known **7683** (which bind-probes upward on conflict). Set it when 7683 is taken by something else on the robot, or to run two gateways on one box. |
| `CERULION_ROBOT_IDENTITY` | The robot's announced network identity: the name that shows up in `cerulion topic list`'s `ROBOTS` section, in the mDNS `_cerulion._tcp` record, and as the first chunk of every announce key. Default: this machine's hostname (`.local` stripped). Set it on a stock-image fleet where every machine boots with the same hostname, or when the hostname is not the name your operators use. Resolved identically by the gateway and the remote-plane daemon, so the LAN and WAN planes can never advertise two different names. |
| `CERULION_PEERS` | Comma-separated `host[:port]` list feeding the discovery ladder's hostname rung, the scripted / CI escape hatch for a robot mDNS and multicast cannot reach. The same list can live in `~/.cerulion/config.toml` under `peers`. Prefer `--connect tcp/<host>:7683` for a one-off. |
| `CERULION_HOME` | The Cerulion config directory, used verbatim instead of `~/.cerulion` (desk key, `device.cert`, `robots.toml`, `peers.json`, `runs/`). The isolation knob for a test rig or a second identity on one machine. |
| `DDS_BRIDGE_CONFIG` | Path to the bridge mapping file a `cerulion ros2 attach` graph's `dds_bridge` node loads. `ros2 attach` sets it absolute when it runs the graph for you; a hand-written `graph run` of an attach graph inherits whatever your shell exported, so export it there. |

### ROS 2 on Cerulion transport

Read by `cerulion ros2 run` / `ros2 launch` and by `graph run` for a graph's
`ros2:` entries; every path stages the identical child environment.

| Variable | Meaning |
|---|---|
| `CERULION_LIB_DIR` | The directory holding `librmw_cerulion.so`, instead of the `cerulion` binary's own directory (where the Linux packages install it and where `cargo build -p rmw_cerulion` puts it in a checkout). It is prepended to the child's `LD_LIBRARY_PATH`; a minimal ament prefix linking it is staged under `~/.cerulion/ros2/` (or used as-is when the directory is already an install's `lib/`) and prepended to `AMENT_PREFIX_PATH`. A missing library is exit 69 with the remediation, never a silent fall-through to the stock rmw. Set but empty is a loud error. |
| `CERULION_ROS2_PRELOAD` | Preload control for the ROS 2 child. **Unset (the default): auto-inject**. When `libcerulion_heaphook.so` exists beside `librmw_cerulion.so` (the Linux packages install both there) it is prepended to the child's `LD_PRELOAD`, with one `info` line naming the injected path. The hook (Linux + glibc + libstdc++ only) interposes the `malloc` family and exports a versioned handshake the rmw uses to fill an unbounded message field directly into a loan slot; while no borrow window is armed it forwards to the real allocator. With the hook active, the rmw offers `borrow_loaned_message` for types carrying at least one FORGEABLE sequence, an unbounded, non-`bool`, default-less primitive sequence (`Image.data`, `PointCloud2.data`, `LaserScan.ranges`; bounded, `bool[]`, and default-valued sequences are excluded and such members always copy), and arms a window over each borrow's slot tail: an adopted fill publishes zero-copy, every escape is copied loudly, and without the hook (or on any handshake skew) the copy path is exactly what runs. **`off` / `none`**: no preload injection at all, the kill switch. **Any other value**: that `.so` is an ADDITION that STACKS `.bashrc`-style, so the child's `LD_PRELOAD` becomes `<your .so>:<hook, when present>:<ambient LD_PRELOAD>` (yours first, so first-wins is yours; the hook kept; the ambient kept); the auto-inject `info` line reports the FULL composed list. A missing path is exit 69, an empty value a loud error. It never silently drops. |
| `CERULION_RMW_ADOPT_TAKE` | Read by `rmw_cerulion` in the ROS 2 CHILD: `1` arms adopt-take for every subscription whose type is forgeable: a plain take serves the shared-memory bytes IN PLACE and the application's own `free()` of each sequence releases the sample. `0`/unset disarms; any other value warns once and disarms. A `cerulion`-staged ROS 2 child never has it set: `cerulion ros2 run|launch --adopt-take` REFUSES (exit 69, because the `ros2` CLI closes the inherited descriptor the hook rides), and setting this variable yourself on those verbs is refused too (exit 2) rather than honoured as a request nothing validated. Set it on a node you launch DIRECTLY, with the hook preloaded. Armed but with the hook inactive in the process, adopt-take stays OFF with one warn, never a silent copy behind a flag that promised adoption. |
| `CERULION_RMW_ADOPT_TAKE_BUDGET` | Read by `rmw_cerulion` in the ROS 2 CHILD: a positive integer capping how many samples ONE subscription may hold adopted at once (each adopted take holds its sample until the application frees the forged pointers). Unset uses the built-in borrow budget; a value that is not a positive integer (or not valid UTF-8) warns and uses the default (once per subscription that reads it, not once per process: creates are rare and each is its own report). Raising it past the topic's provisioned borrow capacity does not raise that capacity, and a take beyond it is a REFUSAL, not a copy: `rmw_take` returns success with `taken = false`, the frame stays queued (nothing is consumed and nothing is delivered), and the refusal is counted and reported. It self-heals (freeing any one adopted message lets the next take succeed), so a caller that holds messages faster than it frees them sees empty takes rather than degraded ones. |
| `CERULION_HEAPHOOK_DEBUG` | When truthy (set, non-empty, not `0`), `libcerulion_heaphook.so` writes a one-line load breadcrumb to stderr naming its ABI version and whether it won `malloc` resolution (`cerulion heap hook loaded: abi vN won_malloc=1`). Diagnostic only: a stock ROS 2 process installs no `tracing` subscriber, so this raw line is how you confirm the preloaded hook actually loaded. Off by default (no output). |

### Recording

Read by the recorder (`cerulion bagd`), including the one `graph run --record`
spawns for you. `graph run --record` builds the recorder's command line
itself, so on that path these variables are the only way to reach the settings
below; that is why they exist.

| Variable | Meaning |
|---|---|
| `CERULION_RECORD_DISCOVERY` | `off` disables live-topic discovery for any recorder in that environment. The rule is whether the topic set was **inferred or named**: discovery is ON for `graph run --record` and for `cerulion bag record --run`, whose universes are both inferred from a run, and OFF whenever you named the topics yourself (`bag record` with positional topics, `--all` or `--regex`). It is on for `--run` because a run's DECLARED outputs need not be what it actually puts on the wire: a bridge graph can declare a handful of topics while many more bridge routes stream, and a bag claiming to describe a run while omitting most of it is the defect discovery exists to close. Reach for `off` when you want a strictly declaration-shaped bag, or when discovery's extra taps are competing for subscriber slots on a busy machine. |
| `CERULION_RECORD_DISCOVERY_SETTLE_MS` | How long the recorder holds bag creation open for late-appearing topics, in ms. Default **2000**, with an absolute floor of 500 ms once discovery is on. `0` disables the hold. Widen it when a bag reports a topic as `appeared_after_bag_creation`; the equivalent flag is `cerulion bagd --discovery-settle-ms`. |
| `CERULION_RECORD_SCHEMA_DEMAND_MS` | The budget, in ms, for resolving a recorded channel's schema over the network when the local corpus does not name it (default **4000**). `0` disables the networked rungs. It matters most on a `ros2 attach` robot, where the bridged routes are exactly the channels a local schema store may not know. Flag equivalent: `cerulion bagd --schema-demand-timeout-ms`. |

One more recording switch is read by the RUN (each process that writes scheduler
trace records), not by the recorder: `CERULION_READ_LOG_FOLD=off` stops runs of
identical read-log records being folded into one counted record, so the bag stamps
trace format 5 instead of 6 and stays readable by an older binary. It changes the
bag's shape only; a `bag play --resim --verify` verdict is the same either way.
Unset, or any other value, leaves folding on. See `docs/read_log_forensics.md`.

### Flashback (the always-on capture window)

Every serving `graph run` holds a rolling window so `cerulion flashback` can
capture the recent past. These size that window and its retention. All are
read ONCE at graph start; turning one up mid-run does nothing, restart the
graph. See `docs/flashback.md`.

| Variable | Meaning |
|---|---|
| `CERULION_FLASHBACK` | The kill switch: `off` disables the whole capture plane for the run, with a loud line saying so. Read once; nothing re-reads it. (`graph run --no-rings` is the separate, orthogonal switch for the scheduler-trace rings.) |
| `CERULION_FLASHBACK_ON_<TRIGGER>` | Per-trigger arming, `on` / `off`. The triggers are `WORKER_DEATH`, `PANIC_DISABLE`, `RUN_VANISHED`, `ESTOP`, `DECLARED`, `STALL`, `RATE` (all default **on**) and `SILENT` (default **off**). An unrecognized value keeps the row's default and prints what you typed; it never guesses a direction. |
| `CERULION_FLASHBACK_DIR` | Where captures land. Default `recordings/flashbacks/`, resolved relative to the directory the graph was run from: a workspace-relative path, like `graphs/` and `nodes/`. |
| `CERULION_FLASHBACK_WINDOW_MS` | How far BACK the window reaches, in ms. Default **30 000**. Not an arbitrary round number: it is the 15 s forward window plus the 15 s anchor cadence, because a capture covering `[T−15s, T+15s]` needs a checkpoint at or before its own start to be re-runnable. Shortening it below that produces captures anchored *after* the thing that went wrong. |
| `CERULION_FLASHBACK_WINDOW_MAX_MB` | The hard BYTE ceiling on the rolling frame window, in MiB: the backstop the time span cannot provide (30 s of a small robot and 30 s of four 4K cameras are not the same number of bytes). **Default: derived from the memory this process may actually use**, that figure divided by 16, floored at 320 MiB and ceilinged at 8 GiB. "May actually use" is the tighter of the machine's own total and the cgroup ceiling binding the process (every cgroup from the process's own up to the mount point, and `memory.high` as well as `memory.max` on cgroup v2), so a container sizes its window from its own share rather than from the host it happens to run on; a machine whose memory cannot be read lands on the 320 MiB floor. An explicit value WINS over the derivation (you know something the machine's RAM does not) and it drags the anchor ceiling with it. Set it when the derived window is bigger than you want to stand, or when you want more window than a share of this machine would buy. Past the ceiling it is the only way up. |
| `CERULION_FLASHBACK_ANCHOR_MAX_MB` | The byte ceiling on retained state ANCHORS, in MiB. Default: the RESOLVED window above, so raising the window raises this with it and the plane stays one priced cost rather than two that drift apart. |
| `CERULION_FLASHBACK_CADENCE_MS` | The anchor cadence, in ms of logical time. Default **15 000**. The one real lever on the plane's standing CPU cost (the duty is roughly `smear / cadence`), which is why it is reachable at all. |
| `CERULION_FLASHBACK_MAX_STATE_MB` | The arm-time ceiling on a node's captured state, in MiB. The default is DERIVED (the largest footprint whose projected first-tick smear still fits the stall budget, ≈ 673 MiB at the default values), so it moves if that budget or the measured rate does. Raise it for a big-state robot whose duty cycle genuinely has room for the longer smear; you will almost always have to raise `CERULION_FLASHBACK_STATE_RING_MB` with it. |
| `CERULION_FLASHBACK_STATE_RING_MB` | Per-rank state-ring size, in MiB (default **64**). The ring must hold the anchors a capture's pre-window needs (`window / cadence + 1` of them), so a big-state robot legitimately needs more and a tiny one is paying for space it can never fill. |
| `CERULION_FLASHBACK_TRACE_MAX_MB` | Byte ceiling on retained scheduler TRACE, in MiB (default **64**). Past it the oldest records go, and a capture that loses its own resume boundary reports itself NOT resimmable rather than shipping a trace that begins mid-step. A capture whose `handoff.trace` names this ceiling is telling you to raise it. |
| `CERULION_FLASHBACK_TAP_BUDGET_MB` | Per-topic budget for the standing window tap, in MiB (default **64**). A property of the machine, not of Cerulion: a desk with 128 GB can afford a deeper standing tap than a Jetson. |
| `CERULION_FLASHBACK_EXCLUDE_TOPICS` | Comma-separated canonical topic names the window does NOT hold, each either exact or a trailing-`*` prefix (`/cam/*`). The lever for buying window SECONDS back on a robot whose state is big: trimming camera topics buys them directly, and it is the axis that still works once the duration knobs are spent. **It governs the WINDOW, never a recording**: `cerulion bag record` and `graph run --record` still record every topic they were asked to. |
| `CERULION_FLASHBACK_MAX_MB` | Total disk retention across all captures, in MiB. Default **2048**. Oldest captures rotate away when the cap is reached; `cerulion flashback --pin` exempts one. |
| `CERULION_FLASHBACK_MAX_CAPTURES` | How many captures are kept (default **20**): the second cap, because a byte budget cannot bound inode churn or keep a listing readable. |
| `CERULION_FLASHBACK_MAX_PER_HOUR` | Rolling-hour cap on captures taken (default **20**). The number every rate-cap refusal quotes; two of the budget are reserved for manual captures. |

### Desk daemons

`cerulion-netd` (the shared network/mirror daemon) and `cerulion-vizd` (the
visualization daemon) are spawned for you on first use and configured entirely
from the environment; the only flags are `--help` (both) and `--version`/`-V`
(`cerulion-netd` only; prints `cerulion-netd <version>` and exits). Set these
before the first command that spawns one; a daemon already running keeps its
boot-time configuration.

`cerulion-wsd` is the standing local workspace-engine daemon for proprietary
Studio clients. It serves workspace, graph and node inspection (workspace schema
NAMES only; there is no schema verb) plus surgical node/graph edits over
versioned NDJSON on a private Unix socket. Build, run, profile and schema
inspection are not part of protocol 1. Every connection is
greeted with `{"hello":"cerulion-wsd","protocol":1}`; a client that reads a
`protocol` it does not know must disconnect. Requests with a field this version
does not know are refused (`bad_request`), never silently ignored. Error codes:
`bad_request`, `unknown_verb`, `workspace_not_found`, `not_found` (graph/node
type/schema), `invalid_request` (the engine refused; the CLI's own message),
`version_conflict`, `engine_error`. `cerulion-wsd --help` documents the flags
and environment; the daemon logs to stderr under `RUST_LOG` (default `info`).

| Variable | Meaning |
|---|---|
| `CERULION_NETD_SOCK` / `CERULION_VIZD_SOCK` | Absolute control-socket path, instead of the default `$XDG_RUNTIME_DIR/cerulion/<netd\|vizd>.sock`, then `~/.cerulion/…`, then `/tmp/cerulion-<euid>/…` (per user; two users on one machine never share a socket directory). For running a second, non-default instance. All three daemons apply the same directory rule. A sticky, world-writable directory (mode `1777`) owned by root or by you is accepted as is (`/tmp` itself, or a `1777` directory of your own): the sticky bit stops every *other* user unlinking your socket, and the one user it does not stop, the directory's own owner, is then root or you (a daemon running as root must also not re-mode the machine's `/tmp` from `1777` to `1755`). A directory of yours that no other user can write into is used as is: nothing is changed and nothing is logged: the `0700` directories the ladder creates, or a `~/.cerulion` another Cerulion command created under a `umask 0022`; one created under Ubuntu's default `umask 0002` is `0775` and is tightened once, with the warn, the first time a daemon uses it. The directory must be readable by you as well as searchable: it is judged and, if need be, tightened through one open descriptor (so a symlink swapped in between the check and the change cannot redirect it), and that open needs the read bit; a search-only `0300` directory of yours is refused with "could not be inspected: Permission denied" (no ladder rung has that shape; only an explicitly configured path can). Otherwise a directory of yours that other users could write into (group or world write bit) is tightened automatically: the write bits are stripped, sticky and setgid bits kept, and one `warn!` line says so; a directory of yours whose bits could **not** be stripped (a filesystem with a fixed mode, such as a CIFS home mounted `dir_mode=0777`) is refused with the reason, as is one that is neither yours nor a root-owned `1777` share. Another user's `1777` share is refused too, not accepted the way `/tmp` is: the sticky bit does not bind that directory's own owner, so its owner can unlink your socket and bind an impostor at the same path. A missing directory is created `0700`. |
| `CERULION_WSD_SOCKET` | Absolute control-socket path for `cerulion-wsd`; defaults to `$XDG_RUNTIME_DIR/cerulion/wsd.sock`, then `~/.cerulion/wsd.sock`, then `/tmp/cerulion-<euid>/wsd.sock`; the same ladder and directory rule as the row above. |
| `CERULION_WSD_HARD_EXIT_MS` | Deadline for `cerulion-wsd`'s graceful shutdown before in-flight requests are aborted, in ms (default **5000**): a request blocked on a workspace lock must never turn SIGTERM into a zombie holding the socket. |
| `CERULION_NETD_NETWORK` / `CERULION_VIZD_NETWORK` | `off` forces a strictly LOCAL-ONLY daemon: no zenoh session ever, and a remote demand surfaces a plain "not network-configured" answer instead of hanging. Mirrors `CERULION_NETWORK=off` but scoped to the daemon, so it never surprises a `graph run` sharing the machine. |
| `CERULION_NETD_CONNECT` / `CERULION_VIZD_CONNECT` | Comma/whitespace-separated zenoh locators to dial (`tcp/192.168.123.99:7683`). Each is where the corresponding verb's `--connect` is threaded when that command STARTS its daemon: `cerulion viz --connect` into the vizd one, a demanding consumer's `--connect` into the netd one. |
| `CERULION_NETD_LISTEN` / `CERULION_VIZD_LISTEN` | Comma/whitespace-separated zenoh locators to bind (`tcp/0.0.0.0:7447`). The `--listen` counterpart of the row above, threaded the same way. |
| `CERULION_NETD_IDLE_GRACE_MS` | How long a consumer-less `cerulion-netd` waits before self-exiting, in ms (default **30 000**). |
| `CERULION_NETD_HARD_EXIT_MS` | Deadline for graceful shutdown before the daemon hard-exits, in ms (default **5000**). A wedged teardown must never leave a zombie holding the control socket. |
| `CERULION_NETD_BIN` | Path to the `cerulion-netd` binary, when it is not beside the `cerulion` executable. If you point this at a wrapper script, the wrapper must `exec` the daemon rather than run it as a child; the readiness wait watches the process it spawned, and a non-`exec` wrapper exits as soon as it has forked, which reads as "the daemon died". |

WSD graph and node reads include a `version` field containing the lowercase
SHA-256 digest of the exact bytes of `graphs/<graph>.yaml` or
`nodes/<node_type>/src/lib.rs`. The `graph.stage_node` and `node.modify`
requests may include `expect_version`. When supplied, the daemon compares it
under the shared workspace-scoped exclusive lock at
`<root>/.cerulion/workspace.lock` and returns `version_conflict` without writing
if the file changed. Successful mutations return the new file-byte version.
A staged node's `outputs:` are its DECLARED ports (name and schema, read from
`nodes/<type>/src/lib.rs`), exactly as `cerulion node stage` writes them; only
the input bindings are supplied by the client.

Every engine writer of a workspace file holds that same lock across its
check-and-write: `cerulion node create/delete/modify`, `node stage`,
`graph create`, `graph partition`, `graph run`'s auto-partition persist,
`ros2 attach`'s consent batch, `ros2 migrate --write`'s write batch,
`schema create/delete`, and every WSD mutation.
So cooperating daemon and CLI mutations serialize **on Unix**. The lock is a
kernel `flock(2)`: a second process waits (and logs one `warn!` naming the lock
file). On a non-Unix build every one of these writers gets a declared-weaker
no-op guard that locks nothing, so they do not serialize there at all. A READ never
creates the lock file; the first MUTATION creates `.cerulion/` and, when the
workspace already has a `.gitignore` without it, appends `.cerulion/` to it;
a workspace with no `.gitignore` gets none. **`ros2 migrate --write` is the one
exception to that append**: it takes a variant of the lock that writes no
tracked file, because a rolled-back migration promises a pristine tree. The lock
is per-directory, so two writers serialize only when they resolve to the same
root; `ros2 migrate`'s `--workspace` is a colcon workspace, which is not
necessarily your Cerulion one. Clients should still send
`expect_version` for read-modify-write operations and re-read after
`version_conflict`.

`graph.validate` runs the engine's full `graph validate`. Its network-ingress
gate needs each consuming node's library, which the daemon reads in a CHILD
process (`cerulion-wsd --inspect-node <lib>`, in its own process group, killed
after 30 s, each pipe capped at 1 MiB, the info document being the whole of the
child's stdout; the child routes load-time chatter to stderr first): a library
that aborts, hangs or floods on load fails the `network ingress` check with the
reason named; anything it spawns that stays in the child's process group dies
with it (a descendant that leaves the group with `setsid()` does not, and is
one of the holders the verdict below names), and the check fails only if the
child's stdout has still not closed 500 ms after the child is reaped. The verdict says exactly that and names the possibilities
without asserting one: the pipe may still be held (by a process outside the
group, or by one the kill could not end in time), or the reader thread was not
scheduled. The child's own copy of the document descriptor is close-on-exec, so
anything the library `exec`s cannot hold that channel; a constructor that
`fork()`s WITHOUT exec'ing (`daemon(3)`) still can, and is one of the holders
the verdict names. A stderr still open, or a stderr the child wrote to at all,
is logged and never fatal; a stderr that FLOODS past the same 1 MiB cap is a
load-time defect and does fail the check.
If the child exits non-zero having said nothing on stderr, whatever it wrote to
stdout is quoted instead, labelled as stdout. The daemon keeps serving
throughout. (`cerulion graph validate` in a terminal still loads libraries in
its own one-shot process.)

### Visualization and video

| Variable | Meaning |
|---|---|
| `CERULION_RERUN_URL` | Makes `cerulion-vizd` a CLIENT of an external Rerun gRPC endpoint instead of hosting its own proxy. **Daemon-side**, read once when the daemon starts; it is inert against an already-running daemon. |
| `CERULION_OPENH264_FETCH` | `off` keeps this desk from ever reaching Cisco's CDN for the H.264 decoder. Video still renders (the desk falls back to letting the viewer decode, slower) unless `CERULION_OPENH264_BLOB` points at a copy. |
| `CERULION_OPENH264_BLOB` | Path to an existing OpenH264 binary: the air-gapped install's answer (fetch once, distribute internally, point every desk at it). Pointing it at a file that is not a Cisco release does not break video: the load is refused, reported once, and the desk falls back to viewer-side decoding. Never a black pane, never silent. |

### Remote access and accounts

For `cerulion connect` / `pair` / `login` and the WAN plane. See
`docs/remote_plane.md`.

| Variable | Meaning |
|---|---|
| `CERULION_RELAY_URL` | A self-hosted iroh relay URL, instead of the public n0 relays. Also the `--relay-url` flag on `connect` and `pair`. |
| `CERULION_EPOCH_DIR` | Where the desk's revocation-epoch cache lives. Deliberately not `CERULION_NETD_*`-prefixed: it is a desk-wide artifact location honored by `cerulion connect`, `cerulion-netd` and the account-page cache writer alike. Default: an `epochs/` directory next to the desk key file. |
| `CERULION_CONNECTD_BIN` | Path to the `cerulion-connectd` sibling binary that `cerulion connect` spawns, for when it does not sit beside `cerulion`. The release archive, which the installer and the apt package unpack, puts it there; it is not published to crates.io, so a CLI installed with `cargo install` has none. In a source checkout `cargo build -p cerulion_connectd` builds it. |
| `CERULION_ACCOUNT_SERVICE` | Base URL of the account service `cerulion login` talks to. For pointing a desk at a self-hosted or development issuer. |
| `CERULION_NETD_DESK_KEY` | ABSOLUTE path to the desk's device key file: the desk's identity and the key a robot access-lists. Unset means an EPHEMERAL key, which an un-paired robot refuses, so a real WAN deployment points this at `~/.cerulion/desk.key` (the same key `cerulion pair` creates). WAN builds only. |
| `CERULION_NETD_DEVICE_CERT` | Explicit ABSOLUTE path to the cached device cert `cerulion login` writes. Unset takes the sibling `device.cert` next to the desk key. A relative value in either variable makes `cerulion login` refuse rather than write a file `cerulion-netd`, which resolves it against its own working directory, may not read. WAN builds only. |
| `CERULION_NETD_WAN_ROBOTS` | `;`-separated `name=eid[@ip:port,...]` entries naming the robots reachable over the iroh WAN plane. Unset means every demand routes to the zenoh LAN plane. WAN builds only. |
| `CERULION_NETD_RELAY_DISABLED` | Any non-empty value disables all iroh relays (LAN direct-dial only), for a locked-down or air-gapped desk. WAN builds only. |
| `CERULION_STATE_ROOT` | The on-robot state root for `cerulion-remoted` (default `/var/lib/cerulion`). Also its `--state-root` flag. |

### Variables that are not part of this surface

The framework reads a handful of other `CERULION_*` variables that are
measurement seams and test fault-injection switches, not knobs. They are named
here so you can recognize one in a log line or a script and know it is not
something to reach for: they may change or disappear in any release, and
setting one can silently cost you a documented guarantee:

`CERULION_DRAIN_DISCIPLINE`, `CERULION_NOTIFY_ELISION`,
`CERULION_TOPIC_LIVENESS`, `CERULION_WAKE_AHEAD_US`,
`CERULION_MW_SINGLE_PARK`, `CERULION_MP_DRAIN_MS`, `CERULION_MP_PEER_LOSS`,
`CERULION_MP_DROP_GRACE_MS`, `CERULION_MP_TRACE_DIR`,
`CERULION_STATE_ARM_TAG`, `CERULION_H264_LAT_PROBE`,
`CERULION_NETD_STALL_SHUTDOWN_FOR_TEST`, `CERULION_EXECUTION_MODE`.

Three are worth a word because they change behavior this document describes:
`CERULION_EXECUTION_MODE=free_run` opts a MULTI-PROCESS `graph
run` into the free-run substrate: no barrier, each rank on its own wall-faithful
clock; the bag's `coordination` stamp and every worker plan follow the ONE
resolved value. It is **experimental**: it is documented here and the benchmark
configurations in [`docs/PERFORMANCE.md`](PERFORMANCE.md) name it, and the
substrate it selects may still change between releases. It is listed with the
seams because it selects the execution substrate rather than tuning it. The
default is barrier lockstep on every route; on a run that
executes no process groups (`--single-process`, a non-Unix host, or an
unpartitioned graph under a non-real clock; under the real clock an
unpartitioned graph is auto-partitioned and honours the opt-in) the variable
is inert and warns; a partitioned graph under `--time-source virtual` still
runs the supervisor (the flag is ignored with a warning) and honours the
opt-in, while `external` is rejected outright. Any other value keeps the
default and warns.
`CERULION_DRAIN_DISCIPLINE=separate` puts every data-trigger node on the
separate drain path (and so onto legacy latest-per-set Sync delivery; see
[Per-set Sync delivery](#per-set-sync-delivery)), and
`CERULION_MP_PEER_LOSS` shadows `--peer-loss` when the flag is absent. Use the
flag.

---

## Cargo features

You generally don't set these; the CLI generates the right feature wiring per node crate. For reference:

| Feature | On `cerulion_core` | When |
|---|---|---|
| `cdylib` | n/a | Set on a node crate's `Cargo.toml`; switches the macro to emit cdylib FFI exports for `cerulion node run` / `cerulion graph run` to load. Each cdylib carries the framework's `CERULION_ABI_VERSION` AND the rustc that built it; a node built against an incompatible `cerulion_core`, OR by a different rustc release or full compiler commit than the `cerulion` host, is rejected loudly at load, even when both meet the crate's MSRV. To select the official stable compiler that built the host: `rustup toolchain install <host's rustc>`, then rebuild with `RUSTUP_TOOLCHAIN=<host's rustc> cerulion node build <type>` (plain `cerulion node build` needs no override when Cargo's effective compiler already matches). For distribution, custom, nightly, or beta hosts, a matching release string alone may not reproduce the fingerprint: use the identical compiler installation for both sides or rebuild the CLI and nodes together with the selected compiler. Also rebuild after upgrading the framework or the toolchain. |
| `debug-logging` | yes | Enables `tracing`'s `max_level_trace`. It does NOT put `debug`/`trace` back into a release build: `cerulion_core` sets `release_max_level_info` on its `tracing` dependency, `tracing` consults every `release_max_level_*` feature before any `max_level_*` one in a build without debug assertions, and Cargo unifies features across the binary, so a release binary stays capped at `info` with or without this feature, no matter what `RUST_LOG` says. To get `debug`/`trace` lines, build without cargo's `--release` (a dev-profile build has no ceiling). |
| `test-helpers` | yes | Exposes test-only constructors and seams (`ClosureNodeEntry`, the `new_for_test` constructors). Used by integration tests; users shouldn't need it. |
| `fuzz-helpers` | yes | Fuzz-test entry points. |

---

## What this document is NOT

- A tour of `cerulion_core`'s internal modules. Use `cargo doc --open` for the rustdoc if you need to read internals.
- A specification of the wire format. See `crates/cerulion_core/src/wire.rs` for the 32-byte header layout.
- A description of the `iceoryx2` or `zenoh` configuration surfaces. Those are framework-internal.
- A guide to writing tests against `cerulion_core::graph::*` directly. The integration-test patterns in `crates/cerulion_core/tests/` are framework-internal: they will continue to be supported but are not the path application authors take.

If something you want to do isn't possible through the surface above, open an issue rather than reaching into internals.
