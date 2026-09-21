# Schema Resolution: Zero-Schema `ros2 attach`, the Acquisition Ladder, and the Wire Rung

A "how it works and why" reference. When you point `cerulion ros2 attach` at a
robot, Cerulion has to turn each discovered DDS topic's ROS type
(`sensor_msgs/PointCloud2`, `unitree_go/SportModeState`, some vendor's custom
`acme_msgs/Widget`) into a schema it can decode. The goal is **zero-schema
attach**: you should not have to hand-write or hunt down a single `.msg` file
for the common case. Cerulion resolves what it can from what it already knows,
then *acquires* the rest automatically, asking the robot itself over the wire
before touching your filesystem, and only ever writes a file into your
workspace with your consent.

Related: `docs/networking.md` (the SEPARATE network-discovery ladder: how
`topic list` FINDS remote robots; this doc is about resolving their message
SCHEMAS), `ros2 attach` discovery, the one resolution chain +
the `.msg` store, wire-native acquisition, robot-local + web
acquisition, and the Cerulion-native path.

## TL;DR

- **Attach resolves each unknown type through a ladder**, in priority order,
  and stops at the first rung that answers.
- **The chain comes first (a predicate, not a rung):** a type already covered by
  the built-in ROS 2 corpus or a `.msg` in your workspace store is resolved with
  no acquisition at all.
- **Then the acquisition rungs run over what is still unresolved**, deduped by
  type:
  1. **[not implemented] The Cerulion-native path.**
  2. **Wire: `~/get_type_description`.** Ask the robot's own node to
     hand back the verbatim `.msg` text over DDS. No local ROS 2 install and
     no pre-supplied `.msg` files are needed. Works on
     **Jazzy / Kilted / Rolling** and
     **patched Iron** (the service was backported in the rclcpp
     21.0.3 / rclpy 4.1.3 Iron patch releases, default-ON); only a frozen
     pre-patch **Iron .0** image advertises the type hashes with no service to
     answer, and **pre-Iron distros are skipped loudly** ("no type hash in
     discovery"). See the per-distro reach table below.
  3. **Local: the ament harvest.** Read the `.msg` from a ROS 2
     install on THIS machine's `$AMENT_PREFIX_PATH` (covers running attach *on*
     the robot, or on a development machine with the packages installed).
  4. **[not implemented] Robot-local / web.** SSH/bridge harvest, then
     `packages.ros.org` / GitHub, consent-gated.
- **Nothing is written before consent.** Acquired schemas are staged IN MEMORY;
  the report says exactly what became resolvable, but `.msg` files land in
  `schemas/<pkg>/msg/` only through the same consent gate as the graph + config
  (`--dry-run` writes nothing; non-TTY needs `--yes`; a TTY previews and asks).

## Why this order

The decision is to prefer the source that is **most authoritative and
least manual**, working outward from the robot:

- **The robot tells you directly.** The wire rung asks the publishing
  node for the exact type description it compiled against. This is the source of
  truth: it cannot be stale relative to what is actually on the wire, and it
  needs nothing from you. (A Cerulion-native rung that would sit above it is not
  implemented.)
- **The schema is on a disk you can read.** The local ament harvest reads a
  definition installed on THIS host's ROS install. It must match the
  publisher's schema, and it depends on the packages being installed where
  attach runs.
- **Fetch it from somewhere else.** SSH/bridge/web reach
  further afield and are consent-gated, because they touch a remote host or the
  network on your behalf.

Closer-to-the-robot wins, so a type the robot will happily describe over the
wire is never chased down on a filesystem or the internet.

## The chain (rungs that resolve *without acquiring*)

Before any acquisition runs, attach partitions the discovered topics against the
resolution **chain**:

1. **Workspace `.msg` store**: `schemas/<pkg>/msg/<Type>.msg`. Every rung below
   materializes into this store, so acquisition is a one-time cost: acquire
   once, commit the store, resolve forever (and deterministically: a build or
   replay never depends on a live robot).
2. **Built-in ROS 2 corpus**: the ~254 types compiled into
   `native_ros2_messages`.

A type the chain already resolves is never handed to the acquisition ladder.
The store also makes hand-dropping a `.msg` a first-class workflow: drop a file
in `schemas/<pkg>/msg/` and it resolves at the next attach.

Store types are first-class across the CLI's schema surfaces, not just at
attach (msg-store parity): they participate in `graph run`'s
schema-hash-divergence check, `graph run --record`'s recorded per-channel
`wire_fixed_size` (as its cross-check and fallback; see "The recorded
layout comes from the node" below), and `cerulion bag play --resim`'s
tolerance field resolution. In `node create` / `node modify` port resolution the store is
a RECOGNIZED tier between workspace YAML and the built-in corpus, but a
store-typed port is refused with a precise error rather than scaffolded:
a store type has no generated Rust type yet, so the scaffolded import
could never compile (see "Port `SCHEMA` resolution" in `docs/user-api.md`).
Everywhere, a workspace YAML schema outranks a same-named store type.

A bare name two store packages define is never silently guessed, but each
surface refuses in its own register. The RESOLVING surfaces (`node
create` / `node modify` port resolution, `schema info`, and `graph
validate`'s existence gate) raise a loud error naming both candidates.
The advisory LOOKUP MAPS cannot error mid-run, so they degrade instead:
`graph run --record`'s wire-size map emits NO bare alias for the
ambiguous name (a graph output declared by that bare string is sized from
its producing node's own declaration, and records `wire_fixed_size` 0 only
when the node declares none: the node is the primary source, this map the
cross-check; see the next section), and `bag play
--resim`'s replay registry resolves
the bare graph `schema:` string to no schema: the topic degrades to
schema-unavailable (no expected hash, field-level tolerance validation
skipped for it) and an unqualified nested reference to the name stays
unresolved. Qualify the name (`pkg/Type`) to get exact behavior on every
surface.

A `.msg` `pkg/Type` beside a workspace YAML definition of the same
`pkg/Type` is the TWIN, and **both spellings of it are refused**: the
qualified one by the workspace lookup, the bare one by propagating that same
refusal, so neither can type a port or be introspected. Binding either would
be a guess: the workspace definition owns that string on every
string-addressed surface (the gateway's served documents, the recorded
channel descriptor, replay's expected schema), while the store definition is
what an `attach` bridge encodes with; a port bound to one would be recorded
and replayed against the other. The lookup maps emit no bare alias for it
either. **The only remedy is to remove one of the two definitions**;
naming one of them without removing the other is not supported.

An AMBIGUOUS workspace spelling is refused outright, naming every source,
never resolved by a precedence. The SPELLING surfaces (`schema info`, port
resolution, the graph validator's identity, `graph run`'s pre-build lookup)
take that verdict from the ONE workspace lookup described in the section
above: a schema entry name declared by more than one workspace YAML file; a
`<name>.yaml` beside another file's entry named `<name>`; a YAML definition
the `.msg` store also spells.

Two further shapes name exactly ONE file, so the spelling surfaces resolve
them (`schema info <name>` renders that file, and a port may carry the
stem) while no map can pick WHICH of the file's entries a channel labelled
`<name>` carries: a `<name>.yaml` declaring several entries and none named
`<name>`, and a `<name>.yaml` whose sole entry's own name is ambiguous
(declared by another file too, or the stem of another file). The ADVISORY
maps therefore bind nothing under such a name and say so: `schema list`
prints the refusal on every row of a refused entry name instead of a hash;
serving serves no workspace document under the name (a `.msg` store twin of
a refused `pkg/Type` stays served for the dependency closure, though no
port can bind it); the recording map and the graph-run hash map carry
nothing under it; replay resolves nothing for it.

A workspace file that fails to read or parse claims nothing (said loudly
by every surface that consults the tier) while a present-but-broken
`<name>.yaml` is the one file the spelling names outright, so it is a loud
`could not be checked` naming that file, never "absent"; a file declaring
no entries claims nothing.

Every size surface (`schema info`'s resolved layout, the gateway's served
hash bindings, the recorded `wire_fixed_size`, the replay registry and the
recorder's schema corpus) applies one wire-representability rule: a schema is representable only if
its smallest frame (the 32-byte wire header, the fixed section, and one
8-byte offset-table entry per variable field) fits the wire's `u32`
`total_size`. A definition past that ceiling is refused by `schema info`
(a `.msg` store definition and a workspace YAML entry alike; the entry is
never shown with its parse-time hash), listed unhashed by `schema list`,
omitted by the recording map and by `graph run`'s hash-divergence map (that
port's check is skipped, loudly), and schema-unavailable at replay, never
saturated or truncated to a size no frame can carry. The recording map's
omission is not the last word on a recorded channel's size: the producing
node is consulted first (next section), and it drops an unrepresentable
value to "no claim" rather than saturating it, by the same rule.

## The recorded layout comes from the node

Everything above describes how a **name** resolves to a definition. A
recording asks a narrower question (what layout is on this channel's
wire) and answers it from the producing node, not from the file.

`graph run --record` stamps a channel's `schema_hash` from the
producing output's own `OutputMeta` ("the exact value stamped on the wire"),
because a re-parsed workspace hash can diverge and would mislabel
every frame. The channel's `wire_fixed_size` is resolved the
same way, from `<T as ShmMessage>::WIRE_FIXED_SIZE` carried on the same
port metadata. The workspace `schemas/` entry the graph's `schema:` names
becomes the **cross-check** and the **fallback**:

| the node says | the workspace says | recorded | said |
|---|---|---|---|
| `N` | `N` | `N` | nothing |
| `N` | `M` | `N` | a `warn!` naming the topic, the port, and both sizes |
| `N` | nothing | `N` | nothing |
| nothing | `M` | `M` | nothing |
| nothing | nothing | `0` | a `debug!` naming both silences |

A node "says nothing" when it carries port metadata with no size on it: a
hand-written `NodeEntry`, a raw-FFI cdylib whose info block omits the key,
or a macro cdylib built before the key existed (it is an additive
info-JSON key, so such a cdylib still loads and simply falls back). A node
with NO port metadata at all is a different case and does not reach the
fallback: the recorder refuses outright rather than record a fabricated
hash. A node saying `0` is a real answer: a purely variable schema has a
zero-byte fixed section.

The `schema:` value is still validated as a
NAME. `graph validate` still refuses a value that names nothing this
workspace can resolve, and still refuses one that disagrees with the name
the producing node declares: this is a resolution SOURCE, not a bypass.
The practical effect is that a workspace may vary a message's layout
between runs (a per-size benchmark type generated into a crate's `OUT_DIR`
the CLI cannot see, a schema edited after the node was built) and the bag
still describes what was published, with the disagreement reported rather
than silently resolved in the stale file's favour.

## The wire rung: `~/get_type_description`

Since **Jazzy**, and on **patched Iron** (the rclcpp 21.0.3 /
rclpy 4.1.3 Iron patch releases backported it, default-ON), a ROS 2 node serves
a `type_description_interfaces/srv/GetTypeDescription` service at
`~/get_type_description` (the RIHS01 type *hashes* this service is keyed on
shipped earlier, in Iron .0). Given a type's **RIHS01 hash**, it returns the
full recursive type description **and the verbatim original `.msg` file
contents** for the type and every dependency. Cerulion speaks this over the
existing `ros2-client`/`rustdds` stack: **no ROS install required**.

How the rung resolves one type:

1. **Consume the attach's discovery.** The wire rung reuses the SAME discovery
   result `ros2 attach` already harvested: it opens **no second discovery
   window**. Discovery harvests the endpoints from a **window-end snapshot of the
   participant's loss-proof internal DiscoveryDB** (not the bounded ros2-client
   status channel, which drops the SEDP startup burst on a large graph), so every
   endpoint, and its SEDP `USER_DATA`, is captured reliably. From each matching
   endpoint the rung reads the RIHS01 hash from its `USER_DATA`
   (`typehash=RIHS01_…;`), **preferring PUBLISHER (writer) endpoints and falling
   back to SUBSCRIBER (reader) endpoints** when no publisher carried a hash (a
   robot-side subscriber, e.g. a `/cmd_vel` reader whose schema the teleop
   direction needs; on a writer/reader hash skew the writer wins). Call
   **targets** are chosen independently of the hash source: every node joined
   from ANY of the type's endpoints via `ros_discovery_info` (writers first,
   then readers) is a candidate, so a subscriber-side node still answers when
   the publisher's node announcement was lost. Each participant's DDS vendor id
   picks the service correlation mapping.

   > Upstream rustdds 0.14.2 provides the DiscoveryDB snapshot accessors and
   > endpoint `USER_DATA`; the published **cerulion-rustdds** fork is retained
   > only for the `participant_lease_duration` builder knob. The published
   > **cerulion-ros2-client** fork selects it because `[patch.crates-io]` cannot
   > reach published consumers. Drop both forks when an upstream rustdds release
   > provides a participant lease duration knob.
2. **Skip loudly, with a distinct reason for each not-callable shape.** A type
   is skipped with exactly one of three precise reasons: (i) it was **never seen**
   on any endpoint in the window; (ii) it was seen but **no endpoint carried a
   hash**: a **pre-Iron distro or a non-ROS DDS peer** (e.g. the Unitree Go2's
   raw CycloneDDS stack), skipped with
   `no type hash in discovery (pre-Iron distro or non-ROS DDS publisher)`; or
   (iii) it **has a hash but no owning-node** join candidate (`ros_discovery_info`
   absent/unparsed). The server looks a type up *by hash*, so without one the wire
   rung cannot be called, and the ladder falls through to the local rung.
3. **Call the owning node.** Pick the request/reply correlation mapping from the
   participant's vendor (Fast DDS ⇒ Enhanced, Cyclone ⇒ Cyclone, unknown ⇒ try
   both), call `/<node>/get_type_description` with the hash and
   `include_type_sources: true`, under the services-default QoS (RELIABLE,
   KEEP_LAST 10, VOLATILE). One call per `(node, hash)`; on failure, retry
   another node holding the same hash.
4. **Materialize the closure.** Every `encoding: "msg"` source **that is not
   already resolvable** becomes a verbatim `schemas/<pkg>/msg/<Type>.msg`,
   byte-identical to the robot's own definition, so it round-trips the generic
   codec bit-for-bit against a native compiled consumer. A closure member the
   store or the built-in corpus already resolves is **served from there, never
   re-materialized** (so a nested dependency like `builtin_interfaces/Time` does
   not litter the store).

Per-distro reach: **Humble / Galactic**: no hash, no service, skipped. **Iron**:
RIHS01 type hashes shipped in Iron **.0**; the
`get_type_description` *service* the rung calls shipped in **Jazzy .0** and was
**backported to Iron patch releases** (rclcpp 21.0.3 / rclpy
4.1.3), where it is **default-on**, so a patched-Iron robot is fully served.
Only a frozen pre-patch Iron .0 image advertises hashes with no node to answer
(the rung's call fails and the ladder falls through to the local rung).
**Jazzy / Kilted / Rolling**: hash **and** service in every release,
default-on, the rung's full functional domain.

## The local rung: the ament harvest

For a type the wire rung cannot serve, the local rung reads the `.msg` from a
ROS 2 install on this host's `$AMENT_PREFIX_PATH`: it consults the
`rosidl_interfaces` ament index marker, reads `share/<pkg>/msg/<Type>.msg`
verbatim, and walks the full nested closure (built-in dependencies are served
from the corpus, never re-materialized). This covers running attach **on** the
robot, or on any dev machine with the packages installed. It skips with a
precise reason when a package or file is absent, and, like every rung, that
reason is surfaced in the report.

## When the local definition is WRONG: the consumption check

The chain above runs **before** the rungs: a type the workspace store or the
built-in corpus already resolves is never handed to the acquisition ladder, and
never checked against what the robot would have said. That is deliberate
(acquire once, resolve forever: deterministic builds and replays that do not
depend on a live robot), but it leaves one residual: **if the local definition
disagrees with what the writer actually serializes, the local one wins
uncontested.**

Nothing downstream catches that on its own. The wire `schema_hash` is minted
from the local entry, and the consumer's reader is built from the *same* local
entry: both sides of the mismatch gate are wrong together, so it never fires.
A local definition that is merely **shorter** than the writer's therefore
decodes "successfully" into garbage: every field after the divergence is read at
the wrong offset, and as long as the byte lengths happen to work out, plausible
values are published.

This shape occurs in practice: a community definition can disagree with what a
vendor's firmware publishes, so the payload is read at the wrong offset.
Whether the divergence surfaces at all depends on the byte lengths involved; it
is not guaranteed to.

So the CDR ingress gates on **consumption**. After the field walk succeeds,
`CdrCodec::decode` checks how much of the body the schema actually accounted
for. A walk that finishes with more unconsumed trailing bytes than an alignment
pad can explain (`MAX_CDR_TRAILING_PAD` = 3, the RTPS ceiling: a
`SerializedPayload` is padded to the enclosing submessage's 4-byte alignment) is
structural evidence that this schema is not the writer's, and becomes a loud
`CdrCodecError::TrailingBytes` (naming the schema, the consumed/total split,
and the remedy) instead of a silently-wrong frame.

The remedy names **both** provenances, because they need different actions.

**If the definition came from the workspace store:** delete the stale
`schemas/<pkg>/msg/<Type>.msg` and re-run `cerulion ros2 attach`, which
materializes the robot's own type description over the wire, *unless* a
compiled-in definition of the same qualified name exists. The chain predicate is
`store ∪ built-ins` and it runs **before** the acquisition ladder, so once the
store file is gone the built-in resolves the type and the wire rung never runs.
In that case you are really in the second branch.

**If it came from a compiled-in corpus** (`native_ros2_messages::BUILTIN_MSGS`,
or the go2 bridge's `UNITREE_MSGS`) there is no file to delete, so *write* a
corrected `schemas/<pkg>/msg/<Type>.msg` instead. The bridge's codec appends
store schemas **last** (`generic::bridge_schema_set_with_store`), so a store
definition wins over a colliding built-in / `UNITREE_MSGS` entry by
last-insert-wins, and every such shadow fires a loud `warn!`: the store wins,
never silently.

That precedence is the **bridge's decode set** only. In the workspace ladder
(`cerulion schema info`, the graph validator's identity and `graph run`; see
"Port `SCHEMA` resolution" in `docs/user-api.md`) a spelling that both a workspace YAML and the store
define is **refused, naming every source**: `'Foo' is
ambiguous in this workspace — defined by: schemas/a.yaml (entry Foo),
schemas/nav/msg/Foo.msg (store).`, for a bare entry or stem beside any
`schemas/<pkg>/msg/<Name>.msg`, and for a nested `schemas/<pkg>/<Type>.yaml`
beside `schemas/<pkg>/msg/<Type>.msg` under the qualified spelling. No tier
outranks another for a spelling you name; precedence decides only a nested
reference nobody spelled (the `schema info` tree: built-ins, then the store,
then the top-level workspace YAML files; a top-level workspace definition
wins; a nested `schemas/<pkg>/<Type>.yaml` is reachable by spelling, not by
that fold, so a nested reference to a name it and the store both spell
renders the store's).

Writing the file is only half of it: **the bridge reads the store only when its
config carries `msg_dirs:`**. `BridgeConfig::effective_msg_dirs` feeds
`bridge_schema_set_with_store`, which early-returns on an empty slice, and
`ros2 attach` emits the key only when the store was already non-empty or the run
materialized `.msg` files into it, neither of which holds when you hand-write
the first file into an empty store. So after writing the corrected file, do one of:

- re-run `cerulion ros2 attach`: the store is now non-empty, so the regenerated
  bridge config emits `msg_dirs:`; or
- add `msg_dirs: [../schemas]` to `graphs/<graph>.bridge.yaml` by hand.

A store copy of a type that is in **no** compiled corpus behaves differently
again: `unitree_go/Go2FrontVideoData` is in neither `BUILTIN_MSGS` nor
`UNITREE_MSGS`, so its store copy at
`examples/go2/schemas/unitree_go/msg/Go2FrontVideoData.msg` pre-empts the robot's
**ament package** copy, because the chain predicate resolves the type before the
acquisition ladder ever reaches the ament harvest. That is
store-before-acquisition, not the store-shadows-a-compiled-in
mechanism described above.

On the bridge the refusal rides the same
per-route flood-latched failure path as any other decode failure (loud first,
repeats at debug, counted unconditionally), and a refused sample burns no wire
sequence.

Two limits:

- **A wrong schema that stops 1 to 3 bytes short is not caught**: it is
  indistinguishable from a real alignment pad. The allowance is not tightened
  using the DDS encapsulation header's declared pad count (XTypes 1.3
  §7.4.3.5), because that count never reaches the decision. On the
  production ingress the header is already gone: rustdds parses the
  representation identifier *and* the options off upstream and hands the bridge
  the bare body, and the `decode_dds_payload` convenience strips the
  4-byte header and discards the options outright. Threading it means
  re-plumbing raw options through every ingress caller. Real XCDR1 writers do
  set the field: the three committed Go2 captures each declare an exact,
  non-zero pad (3, 2 and 3 bytes) matching their real leftover, cross-checked
  and pinned per fixture in
  `examples/go2/nodes/dds_bridge/tests/frontvideostream_wire_test.rs`, and the
  720p capture's declared 2 would sharpen the allowance from 3 to 2 on that
  sample. Threading the field is a tightening this gate forgoes for reach.
- **This is a detector, not a change to the ordering.** The check
  converts silent corruption into a loud, attributable error; it does not change
  which definition wins, and consumption validation does not change
  schema-source precedence.

## Consent: nothing is written before you say so

Acquired schemas are a **third write class** on the same consent ladder as the
generated graph and bridge config (the never-mutate floor):

- **`--dry-run`** shows what *would* resolve and which `.msg` files would be
  written, and writes nothing, not even a `schemas/` directory.
- **`--yes`** writes the graph, the config, and each acquired `.msg` (backing up
  any pre-existing store file it overwrites to `.bak`).
- **Interactive (TTY)** previews the full file list and asks `y/N`.
- **Non-TTY without `--yes`** refuses, having written nothing, and names the
  flags.

## Reading the report

Each resolved-by-acquisition topic is marked in the RESOLVABLE section with the
rung and the file that will be written on consent:

```
RESOLVABLE
  /widget   acme_msgs/Widget   (raw codec; schema acquired — writes schemas/acme_msgs/msg/Widget.msg on consent)
```

Each still-unresolvable topic carries the **loud per-rung reason** every rung
gave, in ladder order, so you can see exactly why automatic acquisition did not
reach it and what to do next:

```
UNRESOLVABLE
  /telemetry  vendor_msgs/Telemetry  — add a schema or exclude the topic
      · acquisition: wire service via get_type_description: no type hash in discovery (pre-Iron distro or non-ROS DDS publisher)
      · acquisition: local ROS install via ament index: vendor_msgs not found in any ament prefix ...
```

The remediation is always concrete: install the package where attach runs, or drop
the `.msg` into `schemas/<pkg>/msg/` by hand. Attach does not harvest it over
SSH or the web.

## The migration section

Every attach report, `--dry-run` included, ends with an automatic `MIGRATION`
section: the on-ramp off the bridge. It groups the discovered **processes** (the
DDS participant is the restart unit) by what a restart under `rmw_cerulion`
would buy:

```
MIGRATION — what could run natively on rmw_cerulion:

RESTARTABLE TODAY (1 process(es)) — every message type these nodes use resolves locally; ...
  /talker  — std_msgs/String

RESTARTABLE AFTER THIS ATTACH WRITES ITS SCHEMAS (1 process(es)) — ...
  /sensors/imu_filter  — vendor_msgs/ImuPacket

STAYS BRIDGED (1 topic(s), 1 process(es)) — the dds_bridge keeps carrying these:
  /utlidar/cloud  (sensor_msgs/PointCloud2)  — no ROS 2 node record seen (vendor/raw DDS, or the node table was not observed this window); the type resolves locally — if this is one of your nodes, restarting it under rmw_cerulion works
  /widget_node  — blocked by unresolvable or excluded type(s): acme_msgs/Widget (see the report above)
```

Unattributed endpoints are counted as **topics** (an orphan has no participant
identity to count); the blocked group is attributed, so it is counted as
**processes**. An orphan is annotated by its type in three shapes, mirroring
the process precedence: a type that resolves **locally** carries the
restart-works hint (a restart under `rmw_cerulion` works; the report just cannot
name the process to restart), a type resolved **only during this attach**
carries the after-write hint (restartable once the schemas are written), and an
**unknown** type gets the plain absence line.

followed by the equivalent commands (`cerulion ros2 launch <your-bringup>` and a
paste-ready `ros2:` graph-entry block with `<package>`/`<executable>`
placeholders: DDS discovery sees endpoints, not launch metadata), the
bridged-vs-native cost facts, and a pointer at `cerulion ros2 migrate`. The
verdicts ride the SAME resolvability partition this document describes: a type
acquired during the attach (any rung, wire or local ament) moves its process
from "stays bridged" to "restartable after this attach writes its schemas"
(unless its only topics were malformed-excluded: a type the bridge will carry
nothing for is never claimed after-write restartable). ROS plumbing endpoints
(parameter services, `rosout`, `parameter_events`, `ros_discovery_info`) are
excluded from the judgment: every ROS 2 node carries those under any rmw. A
robot with no `ros_discovery_info` node table at all gets the headline
that **no restartable process can be named**, worded as absence of evidence
(vendor/raw-DDS processes, or a table the window simply did not observe), and
pointing at the per-topic lines, which still say what a restart would buy where
a type resolves. The generated `ros2:` ids are alphabet-restricted to
`[A-Za-z0-9_]` and double-quoted, so a hostile node name cannot inject YAML
structure into the paste-ready block. A run that discovered
nothing still prints the section in its empty form. The section is pure
rendering over data attach already holds: no extra discovery, no network, no
effect on the consent ladder or exit codes.

## When the workspace cannot CHECK a spelling

Resolution has a third answer besides "resolved" and "not found": **could not
check**. It means the workspace names a `schemas/*.yaml` for the spelling and
something about that file stops it being verified: it cannot be read, it no
longer parses, or it no longer declares the entry the port named. The schema may
be perfectly present; what failed is the check.

The port preflight
(`schema_cmd::workspace_binding_preflight`) REFUSES a binding it cannot verify
(`CliError::SchemaUnchecked`) and NAMES the file, with its three
conditions kept apart because their remedies differ. (Those three are TOCTOU-only:
the file must change between two reads. The routine producer of this answer is
`workspace_lookup`'s own broken-stem arm.)

Two rules follow from it:

- **"Could not check" is never rendered as "does not exist."** `graph validate`
  discriminates the two, so a broken `schemas/Foo.yaml` sends you to that file
  rather than to a hunt for a missing definition.
- **A duplicated identity is all-or-nothing.** `pkg/Type` and `pkg::Type` are one
  identity but two YAML keys, so a single file can declare both. The claims map
  refuses a duplicated identity (`AmbiguityKind::DuplicateEntry`) and the fold
  skips the key entirely, so *neither* bearer is served. `schema info` judges both
  declarations, so an un-carryable one refuses the spelling there too. The
  preflight therefore refuses when **any** bearer is un-carryable: accepting on
  the strength of the healthy one would accept what `schema info` refuses.

One deliberate gap remains: a stem file declaring **no** entries
(`schemas/Foo.yaml` holding `schemas: {}`) still answers the existence probe with
the file. That is deliberate back-compat with workspaces that predate the `.msg`
store, and is pinned as such; no layer judges it, and the fold binds nothing
under the name.

A second, narrower asymmetry sits beside it: where a duplicated identity's
declarations are ALL carryable, the preflight and `schema info` both accept the
spelling while the fold still refuses the key as `DuplicateEntry` and binds no
hash. The gate deliberately does not refuse that shape: only an un-carryable
declaration makes it refuse.

## Not the network discovery ladder

This ladder resolves message **schemas**. It is distinct from the **network
discovery** ladder in `docs/networking.md`, which is how `topic list` FINDS
remote robots (mDNS / cached peers / hostname convention → a zenoh query). One
finds robots; the other decodes their messages. They share no code and run at
different times.
