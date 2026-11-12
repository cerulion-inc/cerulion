# Cross-Machine Networking: Paired by Default, the Gateway, and the `network:` Block

A Cerulion robot defaults to **paired access on every platform**. A live run
makes its exported topics available to authorized desks through the authenticated
remote plane. Robots on your account (and existing saved pairings) are discovered automatically;
iroh takes a direct path when possible, including on one LAN, and uses a relay
when direct connectivity is unavailable. No listen setting, IP address or
`--connect` flag is needed for that path.

The zenoh LAN plane refuses topic demands, catalog and schema queries, and inbound
frames under paired access. Its transport details below describe the explicit
`CERULION_LAN_ACCESS=open` compatibility path. A `network:` block restricts which
topics cross machines; it does not enable networking or authenticate LAN peers.
Turn networking off for a run with `--local` (also spelled `--network off`).

Related: `docs/user-api.md` "Cross-machine networking" (the YAML reference +
validation table), `docs/multi_process.md` (shared authenticated serving for
monoliths and supervised workers), the raw-wire-frame bridge, explicit open LAN
compatibility and pairing.

## TL;DR

- **Paired by default everywhere.** A real-clock `graph run`, `node run` or
  `ros2 attach` registers its produced topics for export. The first live export
  starts authenticated serving automatically; a query-only desk stays a consumer.
  A correct paired start has no open-access warning.
- **Find robots without locators.** `cerulion topic list` lists local topics and
  discovers same-account and explicitly paired robots. `topic echo`, `hz` and
  `info` read their topics through the same authenticated route. Cached identities
  and validated direct addresses support established pairings offline; new
  machines enroll by signing in with `cerulion login`.
- **The `network:` block restricts scope.** An enabled block exports exactly its
  `egress:` list and imports exactly its `ingress:` list. Under paired access,
  each import selects a unique authenticated producer and checks schema, QoS
  and single-writer compatibility. An unsupported graph policy is refused with
  a reason instead of being silently changed. Explicit zenoh locators remain available with open LAN
  access. Monolith and multiprocess graphs use the same rules.
- **One shared remote endpoint.** A machine that serves and consumes topics uses
  its existing authenticated endpoint for both. Graph and worker processes publish
  and receive through local shared memory.
- **Local-only is per run.** `--local`, `--network off` and `CERULION_NETWORK=off`
  keep that run's topics out of every serving plane, including an already-running
  daemon. The run remains locally readable. Query `--local` skips remote discovery
  and demands. See [environment variables](user-api.md#environment-variables) for
  parsing and command-specific behavior.
- **Recording keeps networking.** `--record` keeps allowed exports visible. A
  recording with declared `ingress:` is refused because external frame reinjection
  cannot be replayed faithfully.
- **Open LAN access is explicit and unauthenticated.** Selecting
  `CERULION_LAN_ACCESS=open` lets unpaired peers demand exported topics and publish
  into declared ingress. Strict and permissive export plans both warn:

  > `LAN access OPEN by explicit selection: unpaired peers may read exported topics and publish into declared ingress topics. Unset CERULION_LAN_ACCESS to restore paired access, or run --network off for local-only.`

- **Every plane enforces producer scope.** Topics, schemas and run metadata of a
  local-only run remain unavailable off-machine through an already-running
  gateway, remote server or mirror. Other daemons also refuse outputs omitted
  from that run's declared egress before opening a tap. Local reads and recording
  keep working. Recorder status preserves the caller's directory spelling; resolve relative paths from the run's working directory.
  Recorder status follows the same scope; if another run already
  exports the shared status topic, this recording disables status publication. Opaque `ros2:` children inherit the run scope, including topics
  registered later; config-mapped raw DDS outputs obey it too. A separately
  launched third-party DDS participant has its own transport settings.
- **Producer policy survives a topic restart.** Remote taps, mirrors and open-LAN
  announcements retain admission until withdrawn. If an earlier remote operation
  still holds a topic, a local-only successor refuses startup. Close the demand
  and withdraw the announcement, or restart the serving daemon first.
- **Discovery keeps source and reachability evidence.** Stale-session refresh has
  a two-second request budget; a successful rotation can wait one additional second
  for publication. Account lookup has a two-second deadline; robot catalogs share
  a four-second budget. Offline and
  unknown robot rows remain visible, and online rows show the actual direct or
  relay path. Topic rows name their source. `find` and `viz --robot`
  use these catalogs too; duplicate sources require `--robot NAME` or `--local`
  for local reads. The last observer releases its private shared-memory channel.
  The mDNS/cache/hostname ladder and zenoh gathers describe explicit open LAN,
  locators or `--scan` compatibility; account and saved-pair discovery is the
  default authenticated route. Paired access skips implicit LAN discovery and
  stranger catalog queries. `topic snapshot --robot` uses authenticated ops.

The paired gate stays in front of every installed LAN authorizer. Identity
beacons may remain visible, but topic announcements are withheld while paired.
Opening access after startup does not replay those withheld announcements.
Android apps retain the same default through their `lan_access` runtime key and
host their authenticated remote plane through `remote`; see [Android](android.md).
Authenticated ops keep their capability and e-stop checks; a graph import pulls
an allowed producer stream and does not grant an arbitrary remote publish verb.

Held remote reads survive a serving endpoint start or stop. If a reader cannot be
restored within the transition's five-second budget, netd reports the robot and
topic. CLI observers re-demand their pinned source and schema, or stop with a
named error if recovery fails;
graph imports revalidate the pinned producer before reconnecting, and
visualization re-demands its pinned source and schema outside frame draining. Status checks use separate connections, so a busy daemon does not release healthy readers. Visualization retries each lost source with backoff independently.

## The decision matrix

Every real-clock live run reaching `graph run` / `node run` / `ros2 attach`
resolves to exactly one of these, in evaluation order:

| Condition | Result |
|---|---|
| `--local` / `--network off` on graph/node run, **or** `CERULION_NETWORK=off` | LOCAL-ONLY: no gateway or session; other daemons cannot expose this run's topics, schemas or run metadata. Local observers and recording still work; loud kill-switch notice. |
| `--time-source virtual` **or** `external` | Network INERT (replay-class: a zenoh session is a live side effect that would break replay byte-identity). Silent, no gateway. |
| Enabled `network:` block + `--record` + declared `ingress:` | **Refused**, naming the topics + both workarounds. |
| Enabled `network:` block (egress-only under `--record`, or any non-record) | **Strict export/import scope**: declared egress and ingress use authenticated producer selection under paired access; explicit open LAN access retains verbatim zenoh locators. |
| No / disabled block (incl. `--record`) | **All produced topics eligible for paired export** through the shared daemon. The legacy zenoh announcements, scouting and fallback listen port apply only with explicit open LAN access. |

`graph profile` is the one deliberate exception: a bounded MEASUREMENT run,
not a deployment, so it stays LOCAL-ONLY by design (its transport is built
`network: None`; it never resolves a network). Re-execution
(`cerulion bag play <bag> --resim all`) is likewise network-inert by
construction (the bag is the input, and a replay must match the live run).

## Local inspection and compatibility

`--local` is available in this source version on `graph run`, `node run`,
and `topic list`, `echo`, `hz`, and `info`. CLI 1.0.0 uses `--network off`
for runs, `--no-network` for listing, and `CERULION_NETWORK=off` for
observers. All three compatibility spellings remain supported. See
[Local scope for runs and topic inspection](user-api.md#local-scope-for-runs-and-topic-inspection)
for commands and precedence.

Topic inspection with `--local` creates no remote discovery session and
starts no network daemon. Echo, rate measurement and info read genuine
local shared-memory producers. Their remote schema fallback is also
suppressed, so an unknown type is displayed with the existing hash/hex
fallback. The existing sign-in gate runs first; local scope does not
suppress first-use account sign-in.

Selection wins over explicit list locators and `--scan`. The
`CERULION_NETWORK` environment kill-switch still applies without a local
flag: trimmed, case-insensitive `off` suppresses networking; unset or
whitespace-only has no effect; every other nonempty value warns and fails
closed. No run or inspection flag enables networking over this kill-switch.
The inspection verbs and `schema info` also stay local under the network
daemon's own `CERULION_NETD_NETWORK=off` (exactly `off`, the daemon's
parse), opening no discovery session of their own. `cerulion viz --robot`
refuses under `CERULION_NETWORK`, `CERULION_NETD_NETWORK=off` or
`CERULION_VIZD_NETWORK=off`.

A command does not stop a gateway owned by another process or remove
already mirrored topics. Listing reads mirror provenance locally and
continues to label those rows REMOTE with their source robot, or
`origin unavailable` if attribution failed. Required publisher markers retain
network identity independently of robot attribution. Local observers refuse
mirrors instead of demanding them from `cerulion-netd`. Declared graph ingress
uses the canonical topic channel and retains its remote provenance; it does not
turn a remote source into a genuine local producer. A running local observer
also holds its local source until it exits: a remote injector must refuse that
same topic while the observer retains it, even if the local producer exits.
Concurrent local/remote creation can refuse both attempts; the command reports
that refusal. Native marker listener capacity limits simultaneous explicitly
local observers (default 16 per topic; existing configured limits remain honored).
Upgrade and restart an
older injector before relying on that distinction: an unmarked older producer
with no attribution is indistinguishable from a local source.

## Egress converges onto cerulion-netd

On Unix, live graph exports use the machine's shared `cerulion-netd`. Both a
blockless run and a paired run with an enabled `network:` block register their
export scope there. The first live registration starts the authenticated server;
closing the registration releases its ownership. A desk that only lists or reads
robots does not start serving merely because it queried them.

When a machine both serves and consumes, netd delegates outbound authenticated
demands through the serving endpoint. It does not bind a second endpoint for the
same device key. Graph processes remain network-free and use local shared memory.

The run forwards its resolved iceoryx2 namespace with registration. The daemon
checks the fields that determine shared-memory discovery before it taps or
reinjects a topic: root path, prefix, service/node directories and service file
suffixes. Cleanup flags do not change discovery identity. A namespace mismatch is refused rather than silently reading
an unrelated namespace. Multiprocess graph data uses the resolved shared data
namespace; its coordination namespace remains separate.

Explicit open LAN strict plans keep their verbatim zenoh locators and per-run
gateway routing. A blockless open LAN run uses the shared gateway, with the
existing per-run fallback when netd is unavailable. Open access warns on either
route; paired access cannot silently fall back to an unauthenticated import.

The daemon retains serving ownership until its server has stopped. After the
last producer and consumer release their connections, the idle grace permits
shutdown once active ops sessions, control leases and owed e-stop work settle.

## Making a robot discoverable

A signed-in producer registers itself and starts authenticated serving on its
first live export. A signed-in desk finds robots on that account through the
account directory. Existing saved pairings join the same discovery and read path, including
validated cached direct routes at offline sites. An authenticated reply establishes
presence; a directory row alone does not prove that a robot is reachable.

No `CERULION_NETD_LISTEN`, IP address or `--connect` is required for authenticated
access. A robot with durable pairing state can serve that pairing without a new
sign-in. An existing robot reuses its validated registration offline; first
registration still requires its account issuer.

For an intentionally open zenoh LAN deployment, set `CERULION_LAN_ACCESS=open`
and provide a listen endpoint on the serving machine:

```bash
CERULION_NETD_LISTEN=tcp/0.0.0.0:7683
```

That optional setting binds the legacy LAN plane and permits its mDNS beacon
when the endpoint has a LAN-dialable nonzero fixed TCP port. It is not the switch
for authenticated serving or account discovery. A permissive fallback synthesizes
its usual listen port; an explicit open LAN strict block uses its locators
verbatim, without a bind ladder.

### Unencrypted QUIC first (opt-in)

`CERULION_UNENCRYPTED_QUIC=on` in `cerulion-netd`'s environment, on the robot
and on the machines that dial it, moves netd's links from `tcp/` to zenoh's
unencrypted QUIC carrier, `udp/<host>:<port>?rel=1`, where both ends allow it:

- **Listen.** Every `tcp/<host>:<port>` in `CERULION_NETD_LISTEN` also listens
  on `udp/<host>:<port>?rel=1`. The port number is the same; TCP and UDP ports
  are separate.
- **Advertise.** The mDNS record gains the TXT key `udp_rel=<port>`.
- **Dial.** At start-up netd checks, over UDP, every discovered peer that
  advertises `udp_rel` and answered the usual TCP reachability check, and every
  `CERULION_NETD_CONNECT` locator written as
  `udp/<host>:<port>?rel=1`. A peer that answers is dialled over QUIC. One that
  does not is dialled over `tcp/` on the same host and port, and netd logs one
  warning that starts `QUIC FALLBACK:`, naming the QUIC locator, the reason (the
  UDP port is closed, or no QUIC answer within the bound) and the TCP locator.
  netd lists one locator per peer address, so zenoh does not dial one address
  over both carriers at once, unless `CERULION_NETD_CONNECT` itself lists both.

Explicit `tcp/` connect locators are used as written, and peers taken from
`~/.cerulion/peers.json` stay on `tcp/`. Only the exact value `on` turns the
setting on; any other value leaves it off and logs a warning naming the value.

The check waits at most `CERULION_NETD_QUIC_PROBE_TIMEOUT_MS` (default 1000,
the first loss timeout QUIC itself uses before it has measured a round trip).
A closed port answers at once, so the full wait is paid only where UDP is
dropped without an answer, once per peer address each time netd starts, with every
address checked at the same time. Raise it only for a path whose round trip is above
about 300 ms. Values above 6000 are treated as 6000, with a warning: netd opens
its control socket only after the check and its own start-up, and a command
that starts netd waits 10 seconds for that socket.

**This QUIC is not encrypted and does not authenticate the peer**, the same as
`tcp/`. What it changes is congestion control: a Linux TCP connection that idles
longer than one retransmission timeout restarts from its initial window, so a
large message sent about once a second pays several extra round trips, while
the QUIC stack has no idle reset, so the same message over `udp/...?rel=1`
does not start each send from the initial window. The difference grows with the
round trip and the message size. Cross-host figures will be added once they are
measured on a natively built robot.

Limits: the check runs once, when netd starts. A peer that appears later, or a
QUIC link that later stops passing UDP, is not checked again until netd
restarts. A peer that answers only over UDP fails the TCP reachability check
and is not dialled. With `CERULION_MULTICAST_SCOUTING=on`, peers found by
scouting are dialled by zenoh itself, over whichever locator it picks. The
per-run gateway child and a graph's `network:` block do not read the setting;
a `network:` block names `udp/<host>:<port>?rel=1` itself when it
wants this carrier.

## The gateway

The process and zenoh key-space details in this section describe the explicit
open LAN compatibility path. Paired runs use the shared authenticated serving
and import path described above.

A per-run gateway process owns one run's whole network plane: the single zenoh
session, every egress announce, the demand watch, egress forwarding, and the
ingress re-injection door. It is the **Strict-or-fallback** path everywhere:
spawned for an open LAN Strict posture (an explicit `network:` block, which netd's shared
permissive session cannot represent), when netd registration fails, and on
non-Unix platforms, which have no netd. On a permissive Unix run, robot
included, egress goes to netd instead and no gateway child is spawned. It is spawned by the monolith
`graph run` arm and by the multi-process supervisor alike (a hidden
`cerulion graph run-gateway` subcommand you never invoke directly), handed a
serialized plan + the run's iceoryx2 namespace.

- **Port.** A per-run gateway CHILD listens on the well-known port **7683**
  (IANA-unassigned) and bind-probes UPWARD (up to 8 tries)
  on an address-in-use conflict, so two robots on one host coexist. This does
  NOT apply to netd's optional open zenoh listener, which takes its endpoints
  only from `CERULION_NETD_LISTEN`. Authenticated serving starts automatically
  and needs no zenoh listen endpoint. Override
  the base with `CERULION_GATEWAY_PORT=<n>` (parsed strictly; a non-integer
  is a loud error). A Strict gateway uses the block's `listen` locators
  VERBATIM: no ladder (your config is truth); a stolen port fails the boot
  loudly.
- **Egress is demand-driven, tap-based.** The gateway does NOT dual-publish
  from inside a publisher. It
  registers a bridge flag per announced topic; a REMOTE subscriber's demand
  liveliness token flips that flag ON, and the gateway then attaches a
  listener-less capture subscriber (`DataOnlySubscriber`) to the topic's SHM
  and forwards each drained wire frame to zenoh VERBATIM. When the demand
  ends the flag flips OFF and the tap is dropped: an unwanted topic costs
  nothing. Semantics are drop-to-live (the freshest frames), and the single
  heap copy to zenoh exists identically in every architecture.
- **Runtime topic registration + announce-on-first-serve.** The boot
  plan lists a graph's YAML-declared egress topics, but a `cerulion ros2 attach`
  robot's dds_bridge creates ~90 SHM publishers at RUNTIME (raw routes like
  `/utlidar/cloud`), none of them in the boot plan. So the gateway makes a topic
  BOTH discoverable AND demandable the moment it appears at runtime: discovery
  and demand both tell the truth about what the robot actually publishes:
  - **The control channel.** A network-free worker registers each raw route it
    creates over a fixed iceoryx2 control service (`/__cerulion/gateway_topics`);
    the gateway drains it every drive pass and, for each registered topic,
    registers a bridge flag (makes it demand-GRANTABLE) and declares a hashless
    `cerulion_ann/{robot}{topic}` announce token (makes it DISCOVERABLE), exactly
    as a boot-plan topic. The reserved `/__cerulion/*` control namespace is
    refused as robot egress at both the writer and the gateway. Registration is
    per-PROCESS and PRESENCE-BASED: each registering process holds
    ONE control publisher shared by all its topics (the control service caps
    concurrently-registering processes at 64: process #65 is refused loudly,
    its local pub/sub unaffected), and there is NO withdraw: a destroyed
    producer's topic stays announced until its process exits and the gateway
    restarts, with the catalog row kept accurate by its live `producer_count`
    probe (`Some(0)`) and its `liveness` annotation (the desk dims a dead
    route rather than rendering a phantom stream).
  - **The permissive SHM probe.** When a remote DEMAND names an UNregistered
    topic AND the posture is permissive (`AllowAll` only; Strict never probes),
    the gateway probes local SHM open-only (creates nothing on a miss, TOCTOU-safe)
    and, if the service is LIVE, registers + announces it ON THE SPOT
    (announce-on-first-serve) so the demand grants like a plan-declared producer.
    A topic the gateway ingresses (the echo loop) or a reserved name is
    never probe-served. State is O(1) under a hostile miss/loop flood (one counter
    + a warn-once-per-regime latch each).
  - **The belt feed.** The demand reconciler below re-affirms egress for the
    topics it iterates; that set is SEEDED from the boot plan but REFRESHED from
    the gateway's live registered set each ~1 s pass (a monotonic-count dirty
    check: a bare count read at steady state, a snapshot only on a real
    registration), so a runtime-registered topic is re-affirmed by the belt too,
    on any link where the demand token crosses.

  The control channel `/__cerulion/gateway_topics` is INERT to the user surfaces
  by construction: it has no `/data` suffix so `topic list` never enumerates it,
  it is never announced so a remote `topic list`'s announce harvest never lists
  it, and it is not a graph-declared node output so `graph run --record`'s
  tap plan (derived strictly from node outputs) never records it: nothing for
  replay to read.
- **Demand reconciler: the belt to the subscriber's suspenders.**
  The demand watch above is a liveliness SUBSCRIBER: sub-ms on a
  healthy link. As a belt against a MISSED subscriber wake, the gateway ALSO
  runs a demand RECONCILER: a bounded `cerulion_lv/**` demand query on a ~1 s
  interval whose result reconciles the announce flags with 3-pass absence
  hysteresis (kills flapping + makes removal robust). Its iteration set is the
  gateway's LIVE registered set (boot plan + runtime registrations), so
  a raw route registered after boot is re-affirmed here too. The reconciler never
  widens egress past the allow-list (the SAME gate the subscriber path uses)
  and only lifecycle-manages a topic whose demand it has actually observed; a
  topic another path enabled but the reconciler never saw is left untouched. It
  is a coarse safety net, not the fast path: a failed query is loud once per
  regime (egress flags freeze until it recovers) and a reconciler-caused flip
  is attributable independently of the subscriber. **Where it applies.** The
  reconciler's demand query is an ACCEPTER→DIALER GET of `cerulion_lv/**`, the
  SAME non-forwarding direction the inversion bullet below documents, so on a
  strict connect-only, listen-less peer link it CANNOT observe demand and
  nothing egresses through it. The
  reconciler is therefore the belt for links where the demand token DOES cross
  (loopback, listener-ful peers, multicast-scouted meshes) plus the flap/absence-
  hysteresis + enable-flood guard everywhere; on a strict connect-only link the
  demand path is the demand QUERYABLE inversion below, which moves demand onto
  the two directions that carry.
- **Query surface + demand inversion.** On a strict connect-only, listen-less,
  scouting-off zenoh 1.8 peer link the routing is asymmetric, as measured
  against that version and configuration: the DIALER's declarations do not
  reach the ACCEPTER and the ACCEPTER's GETs do not route to the DIALER. The
  directions that carry are ACCEPTER→DIALER declarations (at connect) and
  DIALER→ACCEPTER queries. An Android app that dials a listening desk with its
  `connect` key is not held back by this on the zenoh this release builds: the
  desk's catalog and demand GETs reach the app over the link the app dialed. A
  demand design on either of the other directions (the dialer DECLARING a
  `cerulion_lv` token, or the accepter GETting `cerulion_lv/**`) stays dark on
  such a link, so demand rides the two directions that carry, folded
  into ONE per-robot, verb-dispatched QUERY SURFACE: the PRODUCER (the
  accepter/listener) DECLARES a single wildcard QUERYABLE at
  `cerulion_q/{robot}/**` (an accepter declaration: reaches the dialer) and
  dispatches inbound GETs by verb. The `demand` verb: the DEMANDER (dialer) runs
  a ~2 s demand-GET loop that GETs `cerulion_q/{robot_or_*}/demand{topic}` (a
  dialer query: reaches the accepter), which pulls that topic's egress ON with a
  synchronous `enabled/w2`/`refused` ack. The GET carries the demander's wire
  header version (two bytes); a producer at another version (or a demander that
  sends none, a release before the version existed) answers
  `refused:wire-header`, enables nothing, and both sides log the mismatch once
  per run of refusals. The `catalog` verb answers a
  `cerulion_q/{robot}/catalog` GET with the robot's full topic catalog (every
  produced/announced topic + its schema hash **when the gateway knows it**
  (the field is `Option<u64>`, absent for a boot-announced topic whose hash the
  gateway has not yet observed) AND its qualified schema NAME
  `pkg/Type` when the CLI resolved one, so a desk knows WHICH type to fetch per
  topic), which `topic list` uses as its
  richer remote-listing data path (falling back to the announce space for a robot
  running an older binary that does not answer). The `schema` verb
  answers a `cerulion_q/{robot}/schema/{pkg}/{Type}` GET (qualified) OR a
  `cerulion_q/{robot}/schema/{Name}` GET (a package-less workspace type from
  `cerulion schema create <Name>`) with the requested
  type's verbatim `.msg`/YAML TEXT plus its full nested-CUSTOM-type closure (each
  reply doc = qualified name + encoding [`msg`/`yaml`] + text + the custom deps it
  references), so a desk with ZERO local knowledge of a robot's custom types can
  seed a decoder IN MEMORY (`topic echo` names + structurally decodes an
  otherwise-opaque frame; `schema info <name>` prints the fetched schema with a
  provenance line) with NO files written. Built-in types every desk already has
  (`std_msgs`, `geometry_msgs`, …) are OMITTED from the served closure; an unknown
  type returns a structured error (never silence). The served docs are
  computed by the CLI from the workspace (`.msg` store + workspace YAML) and handed
  to the gateway in its plan: a network-free gateway process cannot reach the
  store. The catalog also carries a hash→name reverse map so a topic registered at
  RUNTIME (a `ros2 attach` raw route, hash-only over the reg-channel) is NAMED by
  its reg-channel hash. The `schema` selector is ALWAYS explicit (robot AND the
  full type name, `pkg/Type` or a bare `Name`): a mid-key wildcard would compute
  an empty route on a strict connect-only link. The
  key space is DISTINCT from
  the `cerulion_lv`/`cerulion_ann` liveliness spaces, which STAY.
  The queryable handler calls the SAME `enable_bridge` seam (the allow-list gate
  still enforces posture: a GET can never widen egress past the declared list).
  The demander targets a producer's identity harvested from the announce space
  (the working query direction), falling back to the single-chunk `*` wildcard
  (which intersects every producer's queryable) until an identity is learned.
  **Composition + keepalive.** Each granted GET stamps a last-seen instant on
  the producer; the reconciler thread's per-pass EXPIRY sweep disables a
  GET-granted topic once its last GET aged past `DEMAND_TTL` (6 s = 3× the GET
  interval) AND liveliness is ALSO absent: a topic disables only when BOTH
  demand paths (the GET grant and the liveliness view) have released it, so the
  liveliness-query path stays a live additional enable source that never fights
  the expiry. The two paths never fight on removal either: the reconciler
  absence-disables only topics IT enabled, the expiry disables only GET-granted
  topics, and every GET re-affirms egress so any spurious lower self-heals
  within one GET interval. Observable counters (`demand_grant_count` /
  `demand_expiry_count` on the producer; `demand_enabled_ack_count` /
  `harvested_identity_count` on the demander) attribute each flip. Like the
  reconciler, the queryable + GET loop exist ONLY on a gateway (never under
  virtual/replay: nothing that could make a replay differ from the live run).
- **Discovery beacon + announce identity (identity = HOSTNAME).** After a successful bind the gateway
  says WHO it is and WHERE it landed via an ALWAYS-ON mDNS `_cerulion._tcp`
  advertisement (SRV = the actual bound port, TXT `robot=<identity>`, mDNS
  instance = `<identity>`). **The robot's network identity IS the machine
  HOSTNAME** (resolved at runtime, sans `.local`), NOT the graph prefix: the
  announce keys' robot chunk, the `cerulion_q/{robot}/**` query surface, and
  the mDNS instance/TXT ALL derive from this ONE identity, so they agree by
  construction. Identity is network-only (never in topic names or bags), so
  resolving it at runtime is replay-safe and lets one built image announce as
  whatever host it boots as. Override it with the `CERULION_ROBOT_IDENTITY`
  env var (a trimmed, non-empty value wins), needed for a fleet on a **stock
  image where every host is `ubuntu`**: identical hostnames collide, so name
  your robots (via the env override, or by renaming the host). mDNS instance
  disambiguation still applies within one identity. This is the ONE gateway
  beacon. Robot identity ALSO rides the announce keys: every announced
  topic's token is `cerulion_ann/{robot}{topic}` (the robot chunk is the SAME
  hostname identity), and every gateway declares one bare identity token
  `cerulion_ann/{robot}` at boot, so an ingress-only / zero-egress robot
  still surfaces a `ROBOTS` row even where mDNS cannot reach. The announce
  KEY FORMAT is a version boundary (an incompatible shape bumps the
  `cerulion_ann` chunk). The mDNS advertise is additive: a failure is one
  loud warn and the gateway keeps serving. A Strict gateway whose `listen`
  locators carry no parseable `tcp/` port skips the mDNS beacon with one
  warn, never a crash. (The demand `cerulion_lv` space differs: a
  demand token is topic-keyed, no robot chunk.)
- **Death is loud + local-only.** If the gateway exits unexpectedly
  mid-run, the run CONTINUES LOCAL-ONLY with a one-time warn (the gateway is
  not restarted); egress/ingress simply stop crossing the machine
  boundary. A spawn failure at start degrades the same way.
- **Teardown.** On a clean `graph run` (non-record monolith)
  shutdown, its live loop exiting on a node-requested stop or a caught
  SIGINT/SIGTERM/SIGHUP, the graph process forwards a GRACEFUL SIGINT to the
  gateway and gives it a bounded grace window (`GATEWAY_SHUTDOWN_GRACE`, 2s) to
  drop its zenoh session + taps cleanly. This matters for a directed kill: a
  `kill <graph-pid>` / systemd SIGTERM flips only the graph process's `running`
  and would never reach the SEPARATE gateway process on its own. The guard's `Drop`
  SIGKILL is a BACKSTOP only: it fires on the error/`?` exit path (which
  skips the graceful forward) or if the gateway outlives the grace window;
  either way there is no orphan. The `--record` monolith and multi-process
  supervisor arms still rely on that Drop-SIGKILL backstop for the gateway (the
  graceful forward is not implemented there; no orphan either way). The
  gateway is not a DAG worker: its death never trips `--peer-loss` semantics.

## The `network:` block (Strict tightening)

```yaml
network:
  mode: peer            # peer | client | disabled (default when omitted: disabled)
  connect:              # remote zenoh locators to dial (verbatim)
    - tcp/192.168.123.99:7447
  listen:               # local zenoh locators to bind (verbatim)
    - tcp/0.0.0.0:7447
  egress:               # canonical absolute topics this graph EXPORTS (allow-list)
    - /go2/utlidar/cloud
  ingress:              # canonical absolute topics this graph IMPORTS
    - /go2/cmd_vel/keyboard
```

| Field | Meaning |
|---|---|
| `mode: peer` | Join the zenoh mesh as a peer (the normal robot/workstation role). |
| `mode: client` | Connect to a zenoh router; do not route traffic yourself. |
| `mode: disabled` | The block is parsed but INERT: it falls through to the PERMISSIVE default (no strict restriction). This is also the meaning of an omitted `mode:`. Declaring `egress`/`ingress` under `disabled` is a graph-load error (the lists would silently do nothing). |
| `connect` | Zenoh locators to dial, used verbatim (no scouting added). Repeatable. |
| `listen` | Zenoh locators to bind locally, used verbatim (no bind ladder). Repeatable. |
| `egress` | The per-graph export SCOPE (a SAFETY filter, not authorization). Each must be produced by an in-graph node, or be marked `external: true` when a process outside the graph publishes it (see "Topics published outside the graph"); only these leave the machine. WHO may demand them is decided by the account/pairing grant, not this list. An entry is a topic name or a `{topic, reliability, external, send_queue}` map (see "Carriers and per-topic reliability"). |
| `ingress` | Topics this graph imports. Each must NOT have an in-graph producer. |
| `tls` | Certificate files for `quic/` and `tls/` locators (see below). |

`router` mode is deliberately not exposed: a graph declares a participant,
not infrastructure. A Strict block's scouting stays OFF (the mesh is exactly
the locators you declared); gossip, and multicast scouting under
`CERULION_MULTICAST_SCOUTING=on`, are permissive-default behavior, not block
knobs.

### Carriers and per-topic reliability

The carrier is the first segment of each `listen` / `connect` locator, so a
locator string is the whole choice. TCP stays the default.

| Locator | Carrier | Needs |
|---|---|---|
| `tcp/<host>:<port>` | TCP | nothing |
| `quic/<host>:<port>` | QUIC over TLS 1.3, one stream per priority | `tls` files |
| `quic/<host>:<port>?mixed_rel=1` | the same, plus a datagram link on the same connection and port | `tls` files |
| `tls/<host>:<port>` | TLS 1.3 over TCP, for a path where UDP does not get through | the same `tls` files as `quic/` |
| `udp/<host>:<port>?rel=1;mixed_rel=1` | QUIC with NO encryption and NO peer authentication | a WireGuard overlay that supplies both |
| `udp/<host>:<port>`, `udp/...?rel=0`, `quic/...?rel=0` | datagram only: refused (rule 7) | |

**TCP after idle on Linux.** With `net.ipv4.tcp_slow_start_after_idle`
nonzero, the Linux default, the congestion window of a TCP connection that
sends nothing for longer than one retransmission timeout decays toward its
initial size, so each large message sent at a low rate pays a few extra round
trips. The cost grows with the path's round-trip time: small on a LAN, large
across a wide-area link. The window belongs to the sending host, and the
`quic/` carrier's congestion controller has no idle reset. Measured 2026-10-26
over a 31 ms wide-area path with a 301 KB message: at 1 Hz, `tcp/` 164.8 ms
p50 / 257.7 ms p99 and `quic/` 79.3 / 107.5 ms; at 30 Hz both about 63 ms p50.

`graph run`, `node run` and `ros2 attach` log this once per run, when the run
sends topics off the machine over `tcp/` from a host where the sysctl is
nonzero:

- through a per-run gateway (a `network:` block, or an unreachable
  `cerulion-netd`) whose locators include `tcp/`: a warning that names them;
- through the shared `cerulion-netd` session: an info line, since every
  networked run registers there whether or not a remote peer subscribes.

The fixes are `sysctl -w net.ipv4.tcp_slow_start_after_idle=0` on the sending
host, kept in a file under `/etc/sysctl.d/`, or `quic/` on both ends: a
`network:` block with `quic/` locators, their `tls:` files and an `egress:`
list. The sysctl is per network namespace; the run reads it in its own. A
`cerulion-netd` started with only `quic/` locators still gets the info line,
because the run cannot see netd's locators. Other hosts never log it.

Both ends of a link use the same spelling. `mixed_rel=auto` is accepted and
lets the peer decide, which is why it does not satisfy a `best_effort` topic
(the connection may come up with streams only).

`quic/` and `tls/` encrypt the link and let the CONNECTING side check the
listener's certificate. The listener does not check who connects (no client
certificate is requested), so on a robot that listens on `quic/` or `tls/`,
who may pull which topic is still the pairing grant's decision, as on `tcp/`.

```yaml
network:
  mode: peer
  listen:
    - quic/0.0.0.0:7447?mixed_rel=1
  egress:
    - /robot/cmd_status                 # reliable, the default
    - topic: /robot/lidar/points
      reliability: best_effort          # rides QUIC datagrams
  tls:
    certificate: /etc/cerulion/robot.pem   # presented by a quic/ listener
    private_key: /etc/cerulion/robot.key
```

The machine that dials it lists the robot's certificate as its trust anchor:

```yaml
network:
  mode: peer
  connect:
    - quic/192.168.1.10:7447?mixed_rel=1
  ingress:
    - /robot/lidar/points
  tls:
    root_ca: /etc/cerulion/robot.pem
```

- **`reliability`**: `reliable` (in order, retransmitted on loss) or
  `best_effort`; an entry without it is put at `default_reliability`. A
  best-effort topic is put with zenoh's best-effort marker, and on a `mixed_rel=1` QUIC connection zenoh sends it on the datagram
  link: a lost frame is gone, and the frames after it are not held behind it.
  All best-effort topics share that one datagram flow, which has no priority
  order. zenoh sizes datagrams once, when the link comes up: if the path MTU
  later shrinks below that size, a datagram send fails, and zenoh closes the
  datagram link and the stream link with it, so the whole connection
  reconnects. Mark a topic best effort on a path whose MTU is stable. A
  bare-string entry is a map with no `reliability`, and is put at
  `default_reliability`.
- **`default_reliability`**: `reliable` (the default) or `best_effort`, for
  every `egress` entry that sets no `reliability` of its own. An entry's own
  `reliability` always wins, so `{topic: /robot/cmd, reliability: reliable}`
  keeps one topic reliable under a `best_effort` default. A `best_effort`
  default needs a `mixed_rel=1` locator, as a best-effort topic does:

  ```yaml
  network:
    mode: peer
    listen:
      - quic/0.0.0.0:7447?mixed_rel=1
    default_reliability: best_effort
    egress:
      - /robot/lidar/points             # best effort, from the default
      - topic: /robot/cmd_status
        reliability: reliable           # reliable, its own setting wins
    tls:
      certificate: /etc/cerulion/robot.pem
      private_key: /etc/cerulion/robot.key
  ```
- **`send_queue`**: how many frames the gateway holds for the topic while
  the link is congested (a whole number, 1 or more; default: the depth of the
  gateway's tap on the topic). Each egress topic has its own queue and its own
  sender, so a congested link holds up the topics it carries and not the
  gateway's other links (closing LAN access while a put waits on a congested
  link waits for that put); topics sent to the same peer share that peer's
  link. Each queued frame is a heap copy, so a topic's queue can hold up to
  `send_queue` times its frame size.
  The node that publishes the topic writes to shared memory and does not wait
  on the network. A `reliable` frame waits for zenoh's transmit queue instead
  of being dropped at the sender, for up to zenoh's `wait_before_close` (5 s
  by default); then zenoh drops it and closes the link to that peer. When the
  topic's queue is full, a `reliable` topic refuses the new frame, counts it
  as `refused` and logs a warning naming the topic; a `best_effort` topic
  drops its oldest queued frame and counts it as `dropped`. Every 10 s the
  gateway logs a warning for a topic whose `refused` or `failed` rose (an
  info line when only best-effort `dropped` rose), and each topic's totals
  when it stops. A graph with a `network:`
  block using explicit open LAN access runs its own gateway process, so its log
  is where these appear. The
  gateway `cerulion-netd` hosts for runs without a `network:` block also
  reports each topic's `forwarded`, `dropped`, `refused`, `failed`,
  `discarded`, `queued`, `in_flight` and `send_queue` in its `status` reply.
  A sustained `refused` count means the link is slower than the topic: raise
  `send_queue` to ride out longer bursts, lower the publish rate, or mark the
  topic `best_effort`.

  ```yaml
  egress:
    - topic: /robot/camera/image
      send_queue: 32
  ```
- **`tls`**: absolute paths to PEM files, read when the gateway opens its
  session (a missing file fails the open, naming the path). A `quic/` or
  `tls/` listener needs `certificate` and `private_key`; a `quic/` or `tls/`
  connect needs `root_ca`,
  the certificate(s) the peer's certificate chains to. A self-signed peer
  certificate can be its own `root_ca` (ECDSA P-256 and Ed25519 keys both
  work) when it is NOT marked as a CA: `openssl req -x509` marks it
  `CA:TRUE` by default, and a `quic/` or `tls/` listener refuses to open with such a
  certificate, naming the file and the fix. Mint it as an end entity:

  ```sh
  openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes \
    -keyout robot.key -out robot.pem -days 365 -subj /CN=robot \
    -addext subjectAltName=IP:192.168.1.10 \
    -addext basicConstraints=critical,CA:FALSE
  ```

  Put every peer's certificate in that one file to trust several. The
  connecting side always checks that the certificate names the host or IP in
  the locator, so a certificate for a robot dialled by IP carries that IP in
  its subject alternative names. zenoh also trusts the public Web PKI root
  set beside `root_ca`, so a listener holding a publicly issued certificate
  for that name is accepted too.
- **`cerulion-netd`** reads the same files for its own `quic/` and `tls/`
  locators from `CERULION_NETD_TLS_ROOT_CA`, `CERULION_NETD_TLS_CERTIFICATE`
  and `CERULION_NETD_TLS_PRIVATE_KEY`, beside `CERULION_NETD_CONNECT` /
  `CERULION_NETD_LISTEN`. It applies rules 7 to 9 below to those variables
  when it starts, reads each file then, and refuses to start, naming the
  variable, on a setting that cannot work.
- **Open zenoh LAN discovery** speaks TCP: the mDNS beacon advertises a `tcp/` listen
  port (plus `udp_rel=<port>` under `CERULION_UNENCRYPTED_QUIC=on`, below; a
  graph listening on `quic/` alone is not advertised), and
  `topic list --connect` takes `<carrier>/<host>:<port>` with no `?` metadata
  and probes it over TCP before it uses it, so it cannot reach a QUIC-only
  zenoh listener. Explicit `CERULION_NETD_CONNECT` or a graph's `connect:`
  can select that carrier. Authenticated account and paired access uses iroh
  regardless of the optional zenoh listener's carrier.

### The egress/ingress model

Declared graph ingress reserves one canonical local writer at the complete
consumer capacity before an injector opens. Authenticated wire producers
carry explicit Reliable/send_queue metadata: congestion refuses new frames
and preserves queued frames, with `wan_refused` distinct from `wan_dropped`.
A missing policy is unverified; best-effort datagram graph ingress is refused
on the authenticated wire path. Before releasing graph workers, the CLI resolves
each declared ingress topic to exactly one authenticated account or paired
producer and checks its schema, reliable policy and single live writer. The
producer's native service must also allow at most one publisher; a
multi-publisher service with one live writer is refused for graph import.
Missing or ambiguous producers fail startup. Held demands reconnect to the
same pinned producer and release when the run stops. Observer reads retain
their best-effort queue and can read every writer of a multi-publisher topic.
Imported topics retain the source robot's access policy. A serving consumer
exports its local producers; read its imported topics on their source robot.
Metadata discovery and refused demands preserve other running ingress readers.
Account authorization survives a switch to a shared serving endpoint. Restoration
uses one total time budget and reports unresolved readers by name.

The graph file stays the single source of truth: under a
Strict block, which topics cross the machine boundary is declared in YAML.

> **Egress lists are a per-graph SCOPING filter, NOT authorization.**
> A topic being egress-listed limits which of a
> graph's OWN produced topics leave the machine; it never decides WHO may
> access them. Authorization is the account / pairing grant, enforced by the
> `DemandAuthorizer` seam BOTH network planes (zenoh LAN + iroh WAN) consult
> across the WHOLE query surface: before serving a topic's frames (`demand`)
> AND before answering a discovery query (`catalog` / `schema`, the query
> plane). An unauthorized demander is refused at that gate regardless of the
> egress list; it can neither pull frames nor ENUMERATE the robot's topic
> catalog / schemas; the egress list only narrows the surface an authorized
> demander sees. (The default is the deny-nothing `AllowAllAuthorizer`:
> the seam exists but withholds nothing; the account / pairing grant installs the
> `is_allowed(account)` predicate.)

- **Egress = a per-graph export SCOPE.** An `egress` topic must be
  graph-OWNED: produced by an in-graph node, under its derived name
  (`/{prefix}/{node}/{output}`) or a `topic:` override, unless the entry is
  marked `external: true` (below). The gateway
  announces exactly the declared list and taps only a declared topic a
  remote actually demands. A remote demand for an UNdeclared produced topic
  is refused at the bridge scoping filter (its flag structurally never
  flips), logged once per topic at warn. An empty `egress:` is a deny-all
  scope (an ingress-only robot). This is a SAFETY FILTER on the graph's own
  export surface, not the access boundary (see the note above).
- **Ingress = an external network source.** An `ingress` topic is imported
  exactly like a producer-less absolute `source:` reference; the gateway
  validates every inbound frame against the consuming input's schema hash
  before re-injecting it into local SHM (mismatches are counted + dropped).
  `block` backpressure is rejected on an ingress input (the scheduler cannot
  defer a remote publisher).
- **Names are canonical.** Both lists take canonical absolute names (leading
  `/`), declared exactly as the producing graph publishes them: one wire
  name per topic, no per-machine aliasing.
- **Announce backfill.** A late-joining remote that queries after the
  gateway booted still sees the announces: every declared egress topic
  carries a standing announce token, so discovery is not a race.

### Topics published outside the graph

A process outside the graph file can publish on this machine's shared
memory: a ROS 2 program running on `rmw_cerulion`, for example. Under a
Strict block its topic leaves the machine only when `egress:` lists it, and
the entry says it has no in-graph producer:

```yaml
network:
  mode: peer
  listen:
    - tcp/0.0.0.0:7447
  egress:
    - /robot/odom              # produced by a node in this graph
    - topic: /chatter
      external: true           # published by a ROS 2 program on rmw_cerulion
```

The gateway announces `/chatter` from boot and taps it when a remote demands
it. Until the outside program has created the topic, the tap attach fails,
is counted, logs once at warn, and is retried on every gateway pass. An
outside topic that `egress:` does not list is still announced when its
publisher registers it, and a remote demand for it is refused at the bridge
scoping filter, so none of its frames leave the machine.
`external: true` on a topic an in-graph node produces is a graph-load error,
and so is an unmarked entry with no in-graph producer; that error names the
`external: true` spelling.

### Loop-safety rules (graph-load validation)

Every rejection names the offending topic or setting and states the fix:

1. **No topic in both lists** (it would loop back to itself): remove it
   from one list. The gateway ALSO refuses a plan whose announce and ingress
   sets intersect, as a structural backstop.
2. **Egress must be produced in-graph**, unless the entry is `external:
   true`; an `external: true` entry must NOT be produced in-graph.
3. **Ingress must NOT be produced in-graph.**
4. **Canonical absolute names only** (bare names rejected with the expected
   `/`-prefixed form; malformed shapes rejected like any topic name).
5. **No lists under `disabled`.**
6. **No duplicates within a list** (a bare name and a map naming the same
   topic are duplicates).
7. **Known carrier, not datagram only**: `tcp/`, `quic/`, `tls/` or `udp/`,
   with `rel` / `mixed_rel` values zenoh accepts (`mixed_rel` only on `quic/`
   and `udp/...?rel=1`). A datagram-only locator (plain `udp/<host>:<port>`,
   `udp/...?rel=0`, `quic/...?rel=0`) is refused: zenoh sends every message
   on it once with no retransmission, so a lost packet loses a reliable
   topic's frame or one of zenoh's own declarations. Use
   `udp/<host>:<port>?rel=1;mixed_rel=1` or `quic/<host>:<port>?mixed_rel=1`
   to carry best-effort topics on datagrams beside reliable streams.
8. **`quic/` and `tls/` have their files**: `certificate` + `private_key` for
   a `quic/` or `tls/` listener, `root_ca` for a `quic/` or `tls/` connect;
   absolute paths only.
9. **No inert `tls` setting**: `root_ca` without a `quic/` or `tls/` connect
   locator, or `certificate` / `private_key` without a `quic/` or `tls/`
   listen locator, is refused (each is read only by that side), and so is a
   `tls:` block that sets no file.
10. **Best effort needs a datagram link**: a `best_effort` topic, and a
    `default_reliability: best_effort`, each need a `mixed_rel=1` QUIC
    locator (not `auto`) in `listen` or `connect`; without one the topics
    would ride the reliable link with nothing saying so. The default is
    refused even when every entry sets its own `reliability`. When the graph
    also has locators with no datagram link, graph load logs a WARN naming
    them: a peer reached through one of them receives the best-effort topics
    on its reliable link.

Empty `egress`/`ingress` with `peer`/`client` mode is valid.

## Recording keeps the network

`--record` KEEPS the robot network-visible: the gateway runs as a separate
process tapping the recording graph's SHM, so recording and networking
coexist: announces + egress stay ON while the bag is written (the graph
process itself stays network-free, so the network never bleeds into the
bag). This holds for a permissive run and for an egress-only Strict block.

The ONE refusal: `--record` alongside an explicit `network:` block that
declares `ingress:` topics is rejected loudly at run start (before any file
mutation or gateway spawn), because recorded ingress re-injection is not
replay-faithful. The error names the ingress topics and both
workarounds:

- add `--network off` to record LOCAL-ONLY, or
- drop `--record` to keep the network live.

Re-execution (`cerulion bag play <bag> --resim all`) is structurally
network-inert (the bag is the input, and a replay must match the live run).

## CLI introspection: `topic list`

```bash
# Local topics, then same-account and saved paired robots:
cerulion topic list

# Optional open LAN compatibility: reach a peer the LAN ladder cannot find.
# This requires explicit open access and a listener on the serving robot,
# configured with CERULION_NETD_LISTEN or the graph's explicit listen locators:
cerulion topic list --connect tcp/192.168.123.99:7683

# Multicast AND mDNS both blocked? Opt into a unicast /24 sweep (rung 4).
# OFF by default; a horizontal connect sweep reads as port-scan recon:
cerulion topic list --scan

# Scripts / CI / offline (skip the remote query entirely):
cerulion topic list --no-network
```

Authenticated robots and their catalogs join the remote result with their source
identity. A topic name shared by producers requires explicit source selection;
see the `topic` command reference in [user API](user-api.md).

The open LAN compatibility result below uses the legacy announce and mDNS
gather. `topic list` prints the LOCAL `TOPIC` section FIRST and instantly (a slow or
failing remote query never delays or hides it; the framework's own channels,
`/bagd/status` and anything under `/__cerulion/`, are hidden there unless
`--all`, and a count line says when any were), then a `ROBOTS` section
(every LIVE gateway: a robot appears iff its announce tokens arrived,
grouped by the announce keys' EXACT robot chunk, OR an mDNS browse answered
for it; mDNS also enriches announce rows with the locator; each row shows
the robot name, its DISTINCT visible-topic count, the mDNS locator, and the
provenance tag `announce`/`mdns`; non-mDNS ladder finds render on a labeled
`candidates (unverified):` line under the rows) followed by a
`REMOTE TOPICS` section: every topic a networked Cerulion robot is
advertising OR demanding, gathered over TWO CONCURRENT bounded sub-second
liveliness queries (the demand space `cerulion_lv` and the announce space
`cerulion_ann`; announce keys' robot chunks stripped, so a topic two robots
both announce lists once), merged + deduped, sorted canonical. The one
discovery query session HARD-BOUNDS its connect phase at 1 s
(`connect/timeout_ms=1000` (the positive global timeout is zenoh's real
tokio bound) plus `connect/retry/period_init_ms=0` for a single inline
attempt and `exit_on_failure=false`), so a dead / black-hole locator (a
stale cache address, an unreachable `--connect`) can never stall the command
past that bound, while reachable robots connect BEFORE the gathers start.
That 1 s bound is FATAL, not merely slow (endpoints are tried sequentially and
one hanging connect fails the whole open, losing every other reachable robot),
so it is only HALF the defense: EVERY connect endpoint is first TCP-PRE-FILTERED
in parallel and only reachable ones fold in: discovered ladder candidates at a
short 300 ms budget, and explicit `--connect` locators at a 1 s budget equal to
the connect bound. Anything that cannot answer a TCP connect within its budget
could not have contributed within the bound anyway, so it is SKIPPED FOR THE RUN
(a ladder candidate silently, an explicit `--connect` with a loud warn) rather
than folded in, and if the open still fails (a tarpit that accepts TCP but
never speaks zenoh) it is retried ONCE with the reachable-explicit locators only,
so a discovered false positive never sinks the gather. With nothing discovered
the `ROBOTS` section is omitted and the `REMOTE TOPICS` section collapses to one
line, `remote: none discovered in 500 ms (sign in with the robot's account using cerulion login;
account and paired robots are reached automatically)`; a peer that is reachable (a given locator, a discovered
robot) but advertised no topic in time gets `retry` instead.

**Mirror provenance: one data source = one topic.** When a desk
process re-injects a remote robot's topic into local SHM (vizd's remote viz,
`topic echo`'s auto-ingress, `cerulion connect`'s iroh re-inject), the mirror
lands as a canonical `{topic}/data` service for graph ingress and manual connect,
or a source-qualified `/__cerulion/observers/.../data` service for CLI and viewer
observers. Private observer channels are hidden unless `--all` is given, which
marks them internal and names their origin robot. `topic list`'s local
enumeration would otherwise surface as a phantom SECOND LOCAL topic. Each
re-injector records the mirror's origin in an out-of-band provenance registry
(`/__cerulion/mirrors`, a reserved SHM control service with NO `/data` suffix so
it is itself invisible to `topic list`), and `topic list` reads it to render
every topic in one of THREE states:

  - **LOCAL**: a genuine local `{topic}/data` service, absent from the registry.
    Listed under the `TOPIC` section.
  - **REMOTE · streaming**: a local mirror of a remote robot (its canonical name
    is in the registry AND a live mirror publisher is streaming it). Rendered in
    the `REMOTE TOPICS` section as `<topic>  ● streaming  <robot>`, attributed to
    its origin robot, NEVER under LOCAL.
  - **REMOTE · idle**: a topic a remote robot advertises but which the desk does
    NOT mirror locally (from the announce harvest above). A bare `REMOTE TOPICS`
    row, unchanged.

A topic that is BOTH network-announced AND locally mirrored renders ONCE, as
streaming. Crucially, the streaming rows are LOCAL SHM knowledge (read from the
provenance registry, not the network), so they render even under `--no-network`
and even when the remote query fails: a mirror is REMOTE by IDENTITY (it is a
copy of a remote robot's data), independent of any live network query. The wire
name is untouched. Canonical graph consumers subscribe to the mirrored name;
observers open the private channel netd returns, keeping identically named local
and remote sources separate. The `topic list` presentation folds mirrors into REMOTE. The
Studio sidebar parses the same rows: the topic path stays the row's first
whitespace token, with the `● streaming` marker + robot in the trailing columns.

- **Remote discovery is ON by default** for same-account and explicitly paired
  robots on any network. Under explicit open LAN access, locators or `--scan`,
  the LAN ladder also discovers robots offering open access. Multicast scouting
  is enabled only under `CERULION_MULTICAST_SCOUTING=on`.
  `--no-network` opts out of the whole remote half, ladder included.
- **Extra locators are additive**: `--connect` /
  `--listen` add to an explicit open LAN discovery session (both repeatable) to
  reach a peer the ladder cannot find. Account and paired access needs neither.
  Ladder-found gateway locators fold into the
  same session the same way.
- **`--scan` opts into the subnet sweep (rung 4)** for a network that blocks
  BOTH multicast and mDNS reflection. OFF by default and structurally
  unreachable without the flag: a horizontal `/24` connect sweep reads as
  port-scan reconnaissance to corporate IDS. Each open port is only a
  CANDIDATE: it earns a real `ROBOTS` row only if its announce tokens
  actually arrive in the gather; otherwise its address stays visible on the
  labeled `candidates (unverified):` line. See "Finding robots" below.
- **Best-effort, never silently empty.** The local list already printed, so
  a remote session/query failure is a LOUD one-line note on stderr (`remote:
  discovery unavailable (<error>; pass --no-network to skip it)`) and exit 0:
  never a fake-success, never a hang. An empty ladder run names the escape
  (`--connect tcp/<host>:7683`) for explicit open LAN access. Account and paired
  robots use their authenticated remote route automatically, including on an
  isolated LAN; iroh uses a direct path when available and a relay otherwise.

## The catalog updates itself: change PUSHES

The discovery ladder and `topic list` above are what you run when you ASK
"what is out there?". Change pushes add the other half: the desk is TOLD when
the answer changes, so a UI (Cerulion Studio's sidebar, the agent) updates
itself instead of polling or waiting for somebody to click refresh.

**Nothing new goes on the wire.** A robot's produced topics are already
announced as zenoh LIVELINESS tokens (`cerulion_ann/{robot}{topic}` plus
the bare identity token; see "Finding robots" below), and a liveliness
SUBSCRIBER is told the instant a token appears or its declaring session
dies. The feature is a reader for events that were already there:

```
robot: graph run / ros2 attach          declares announce tokens
            │                          (one per produced topic)
            ▼   zenoh liveliness
desk: cerulion-netd                    ONE subscription on the session it
            │                          already owns; folds transitions into
            │                          a view and COALESCES them (250 ms)
            ▼   UDS: {"event":"catalog_changed", …}
      cerulion-vizd                    relays to controllers that subscribed
            ▼   UDS: {"event":"catalog_changed", …}
      Studio / the agent               runs the refresh it already has
```

| Property | Behaviour |
|---|---|
| Opt-in | netd pushes ONLY down a connection that sent `subscribe_catalog`; vizd only down one that sent `subscribe_events`. A consumer that does not ask receives no line it would not have received before, so an unsolicited line can never desync an existing reader. (netd's `Hello` banner does report the new version: that is the handshake; every line after it is unchanged.) |
| Coalescing | 250 ms, anchored on the FIRST change (not the last). An `ros2 attach` graph's ~75-topic announce burst is ONE notification; a flapping robot still notifies once per window rather than deferring forever. |
| Robot death | One session death drops every token at once and is reported as ONE robot-level removal, not N topic removals. |
| Versioning | Every notification carries a monotonic `version`; `subscribe_*` answers with the current one plus the announcing-robot set. A MISSED notification is harmless: the next carries a newer version and the correct response to any of them is a full refresh. |
| Slow consumer | The per-connection slot is capacity ONE with a lossless merge: a lagging consumer gets one line describing everything it missed. The watch/forwarder threads never do I/O, so a consumer that stopped reading stalls only itself. |
| Manual refresh | Unchanged, and still the escape hatch for everything. |
| Skew | The push subscription arrived in netd control protocol **v6** (the banner's `protocol` field reports the version a daemon speaks); a new client against a daemon older than v6 is refused by the per-verb gate and degrades loudly to its own refresh path, and an older client against a v6 or later daemon is simply never subscribed. vizd's protocol version is deliberately UNCHANGED (a new verb is additive, and its controllers compare the banner for exact equality); capability there is negotiated by verb. |

**What is NOT event-driven:** the LOCAL half. The push above covers REMOTE
topics; a LOCAL iceoryx2 service appearing or disappearing on the desk carries
no liveliness token (there is no event to subscribe to), so a locally
produced topic's arrival is still picked up by whatever refresh the
consumer already does. Building a local watcher would mean polling
`list_topics` on a timer, which is the very thing this removes.

Turn it off by pointing the desk at a `cerulion-netd` that predates control
protocol v6 (the version that added the push subscription), or by
running with no network (`CERULION_NETD_NETWORK=off`); in both cases the
subscribe still succeeds and reports `watching`/`connected: false`, so a
consumer is told plainly that no push can arrive rather than waiting on one
that cannot come. (A too-old netd is the one case vizd cannot report as
`watching: false`, because the verb is refused before a subscription exists;
it reports `connected: false` and logs the version + the restart remedy.)

**Lifecycle:** `cerulion-vizd` opens a SECOND netd connection for
this subscription, at STARTUP, and holds it for its whole life, unlike the
demand plane's connection, which is lazy and appears on the first remote
attach. netd counts a live connection as busy, so `cerulion-netd` stays up
for as long as a vizd holds that subscription and exits on its idle grace
once that connection closes.

The standing subscription is what keeps discovery WARM (the sidebar is
current the moment you look, rather than after your first attach), and
netd's idle self-exit is what reclaims it afterwards. Closing Studio does not
itself stop the vizd it spawned, so netd stays up until that vizd exits.

## Finding robots: the discovery ladder

Authenticated discovery combines the signed-in account directory and saved
pairings. Selected routes are checked by authenticated metadata replies or probes;
validated identities and direct addresses may be cached for offline reconnects.
The account directory does not authorize a stranger: the remote endpoint still
checks identity, capability and revocation. See [remote access](remote_plane.md).

The following legacy open LAN ladder answers "which robots exist?" BEFORE any session is dialed. Its
rungs run in PARALLEL under one ~1.5 s ceiling, GATHER-ALL (multiple robots
on one network is the norm: never short-circuit), deduped by resolved
`(ip, port)`; a rung that fails or overruns is dropped loudly without
sinking the others.

| Rung | Mechanism | Notes |
|---|---|---|
| **mDNS** (PRIMARY) | Browse `_cerulion._tcp` | The primary robot-IP discovery mechanism where multicast is available: it is Cerulion-scoped by construction, and SRV/TXT carry the robot name + actual bound port natively. Enterprise APs commonly reflect mDNS across VLANs for AirPlay/Chromecast. The rungs below cover the networks it does not reach. |
| Cached peers | `~/.cerulion/peers.json` | Robots seen before (7-day TTL): every TTL-fresh entry is a CANDIDATE (no per-candidate verify session; its locator folds into the one query session and presence in the gather, announce OR mDNS, confirms it). Post-gather, only mDNS-verified `(robot, locator)` pairs are written back (an announce-only robot has no verified locator: it is logged, and its old entries age toward the TTL). Covers "the same robot as yesterday" instantly. Trust hardening (paired identity) is not implemented. |
| Hostname convention | `CERULION_PEERS` env, `~/.cerulion/config.toml` `peers = [...]`, `<name>.local` | The scripted/CI escape hatch and the lab's standing-robots list; a bare name that fails DNS retries as `<name>.local`. Default port 7683. |
| Subnet sweep (rung 4) | **`topic list --scan` ONLY** | OPT-IN. A unicast `connect` sweep of the local `/24`(s) on the well-known gateway port, the last LAN rung when a network blocks BOTH multicast AND mDNS reflection. Each open-port survivor is a CANDIDATE (robot = the host IP), folded into the one query session; an unrelated open port surfaces no announce token so it earns no ROBOTS row (no per-survivor beacon probe); its address stays visible on the `candidates (unverified):` line. The subnet is clamped to the host's `/24` (never a `/16` sweep); a `/25`+ subnet is respected verbatim. |

Zenoh GOSSIP scouting stays on underneath: a session learns further peers'
locators over the links it already has, and opens no UDP socket for it. Zenoh
MULTICAST scouting (periodic scout packets to UDP `224.0.0.224:7446`) is OFF by
default, so an idle machine sends no discovery traffic after its one mDNS
registration. `CERULION_MULTICAST_SCOUTING=on` (trimmed, any case) turns it
back on for a network where mDNS does not cross; set it on both ends, since a
session with it off neither sends nor answers scouts; its hits then merge into the
same session. Any other non-empty value leaves it off and logs a warning. The
switch is read by `cerulion-netd` and `cerulion-vizd` at daemon start, by
`graph run` for a permissive gateway process it spawns itself, and by the
`topic list` discovery session; a permissive run whose egress rides a running
`cerulion-netd` gets that daemon's boot-time value.

**The `--scan` sweep is opt-in, and structurally so.** A horizontal `connect`
sweep across a `/24` is the exact traffic signature corporate/enterprise IDS
flags as port-scan reconnaissance, so the rung is unreachable on a default run:
it is wired into the ladder ONLY inside the `if scan { … }` branch, and the sole
producer of that flag is `topic list --scan`. No env var, no config key enables
it. On a scan run the ladder ceiling widens (the /24 connect sweep is slower
than the default rungs), and each open-port survivor is only a candidate: it
earns a real `ROBOTS` row only if its announce tokens arrive in the gather (a
real Cerulion gateway); a bare open port stays on the labeled
`candidates (unverified):` line.

**A hard limit:** AP client isolation (common on enterprise/university
wifi) blocks ALL station-to-station traffic and defeats every LAN rung;
only a rendezvous point both sides dial out to survives it. The authenticated
iroh path can use its relay; the open LAN ladder has no rendezvous tier.

### Optional zenoh LAN dial list at daemon boot

Default account and paired discovery uses the authenticated remote plane and
refreshes independently of zenoh's dial list. For open LAN compatibility,
`cerulion-netd` builds its zenoh connect endpoints at boot. With multicast
scouting off, that session dials only those endpoints, in this order:

1. explicit `CERULION_NETD_CONNECT` locators, verbatim;
2. one `_cerulion._tcp` mDNS browse (800 ms: one PTR query per interface and
   address family, plus a resolve query for an answer that arrives without its
   SRV or address records), one locator per IPv4 address in each answer (IPv6
   only for an answer with no IPv4 address);
3. TTL-fresh `peers.json` rows.

Rows 2 and 3 are deduped by address and TCP-probed together; survivors are
folded into the connect endpoints. The browse's mDNS daemon is shut down when
the browse ends, so netd sends no further mDNS queries. This dial list reaches
listening open LAN peers; it grants no access to a paired run. Account robots
and saved pairings need no environment variable or entry in this list.

Two details worth knowing:

- **A discovered row (mDNS or cache) is TCP-probed; your
  `CERULION_NETD_CONNECT` locator is not.** A cached address may be stale, an
  mDNS answer carries the addresses of the interface it was sent on, which may
  not be routable from here, and a dead one would otherwise buy a background
  retry connector for the life of the daemon. Your explicit
  locator is your word: netd's session is long-lived, so zenoh retries it in the
  background and a robot that is merely booting comes up on its own. (This is
  the opposite of `topic list`'s rule, which drops an unreachable explicit
  locator; that command is a one-shot gather under a *fatal* 1 s connect bound,
  where one hanging endpoint sinks every other robot.)
- **The fold happens once, at daemon start.** netd's session config is immutable
  after init, and netd self-exits on idle, so a restart is the refresh. A netd
  held alive for hours by a running Studio keeps its boot-time dial list. Its
  optional zenoh listener and beacon require `CERULION_NETD_LISTEN`.
  An open LAN peer that boots later enters that dial list on the next restart,
  or is reached by background retries of an explicit `CERULION_NETD_CONNECT`
  locator. `CERULION_MULTICAST_SCOUTING=on` on both machines also discovers
  listening zenoh peers, at the cost of periodic scout packets. These refresh
  rules do not limit automatic authenticated discovery or serving.

To see what a running daemon is CONFIGURED to dial:

```bash
# Resolve the control socket exactly as every consumer does: the env override,
# then $XDG_RUNTIME_DIR, then $HOME, then /tmp.
sock="${CERULION_NETD_SOCK:-${XDG_RUNTIME_DIR:+$XDG_RUNTIME_DIR/cerulion/netd.sock}}"
sock="${sock:-${HOME:+$HOME/.cerulion/netd.sock}}"
sock="${sock:-/tmp/cerulion-$(id -u)/netd.sock}"

# One NDJSON request; `connect_endpoints` in the reply is the folded set.
printf '{"method":"status","id":1}\n' | nc -U "$sock"
```

netd writes its `Hello` banner first, so you get TWO lines (the banner, then the
status reply):

```
{"hello":"cerulion-netd","protocol":11,"wire_header":2}
{"id":1,"demands":[],"active_connections":1,"idle":false,"connect_endpoints":[],"lan_access":"paired"}
```

`lan_access` is the shared gateway's LAN posture (`open` or `paired`). A daemon
without a network session, or an older daemon, omits it, and the omission means
unknown, never open.

`egress` appears once the daemon's shared gateway has booted: one entry per
egress topic a remote has demanded, with that topic's `reliability`,
`send_queue`, `queued`, `in_flight`, `forwarded`, `dropped`, `refused`,
`failed` and `discarded` (see `send_queue` under "Carriers and per-topic
reliability"). A daemon with no gateway, or an older daemon, omits it.

Interpreting `connect_endpoints`:

- **`connect_endpoints` is what the daemon was CONFIGURED to dial, not what it
  reached.** netd's zenoh session is lazy (a netd nobody has demanded from has
  opened no session at all), and a fiat-trusted dead locator reads identically to
  a live one. Use it to answer "why is my desk talking to that address?", not
  "is that robot up?".
- **An older daemon omits the field entirely, which decodes as *unknown***, never
  as "this daemon dialed nothing". An explicit `[]` is the real "nothing folded".

## Two-machine quickstart (Go2: Mac ↔ robot)

Sign in with the same account on both machines, or use an established explicit
pairing. Each real-clock graph registers its exports automatically. Add a
`network:` block to restrict exports and declare imports; authenticated producer
selection needs no listen or connect locators, on one LAN or across networks.

The node types in these sketches (`lidar_driver`, `locomotion`,
`keyboard_teleop`, `obstacle_monitor`) stand for your own `nodes/<type>/`
crates. To just LOOK at the robot's topics from the workstation you need no
graph and no node at all: `cerulion viz --robot <name> /go2/utlidar/cloud` demands them and
renders them. The workstation graph below is for a node that computes on the
robot's data.

**Robot: `graphs/go2_driver.yaml`** (exports the lidar cloud, imports teleop):

```yaml
prefix: go2
nodes:
  - id: utlidar
    type: lidar_driver
    outputs:
      - name: cloud
        schema: sensor_msgs/PointCloud2
        topic: /go2/utlidar/cloud       # externally-fixed absolute name
  - id: locomotion
    type: locomotion
    inputs:
      - name: cmd_vel
        source: /go2/cmd_vel/keyboard   # external source: arrives over the network
network:
  mode: peer
  egress:
    - /go2/utlidar/cloud
  ingress:
    - /go2/cmd_vel/keyboard
```

**Mac: `graphs/teleop.yaml`** (imports the cloud, exports commands):

```yaml
prefix: mac
nodes:
  - id: keyboard
    type: keyboard_teleop
    outputs:
      - name: cmd_vel
        schema: geometry_msgs/Twist
        topic: /go2/cmd_vel/keyboard    # publishes the name the robot imports
  - id: monitor
    type: obstacle_monitor
    inputs:
      - name: cloud
        source: /go2/utlidar/cloud      # external source: arrives over the network
network:
  mode: peer
  egress:
    - /go2/cmd_vel/keyboard
  ingress:
    - /go2/utlidar/cloud
```

Sanity-check the link before starting the Mac graph:

```bash
# On the Mac, with the robot graph running:
cerulion topic list
# REMOTE TOPICS
# /go2/utlidar/cloud
```

Then `cerulion graph run go2_driver` on the robot and
`cerulion graph run teleop` on the Mac.

## What crosses the wire

Local delivery stays zero-copy shared memory; only egress topics pay the
network. An exported frame travels as the SAME unified wire format (40-byte
`WireHeader` + payload) VERBATIM as the zenoh payload on the key
`cerulion_w2{topic}`: no envelope, no per-hop translation, so the receiving
side feeds the bytes straight into its subscriber path and schema hashes
validate end-to-end. Discovery rides the separate liveliness key-spaces: the
demand space `cerulion_lv{topic}` (a remote subscriber asking for a topic,
which is what flips a gateway egress flag; topic-keyed, no robot chunk) and the
announce space `cerulion_ann/{robot}{topic}` (a gateway advertising a
produced topic, attributed to its robot; plus the bare
`cerulion_ann/{robot}` identity token). `topic list` queries both.

## Transforms (`/tf` and `/tf_static`) over the link

Transforms are the payload every frame-aware consumer needs: the
robot broadcasts the TF tree on `/tf` and the sensor mounting frames on
`/tf_static`, and a workstation node resolves poses against them. Both ride the link: `/tf`
carries `tf2_msgs/TFMessage`, the worst-case wire shape (a variable array of
nested messages with strings), and it crosses BYTE-IDENTICAL like any other
topic. Under the permissive default `/tf` is already viewable; a `network:`
block restricts it explicitly.

The robot OWNS and egress-lists the transform topics; the workstation
(Mac) ingresses them and feeds a node that consumes the tree. (Viewing them is
again `cerulion viz --robot <name>`, with no graph.)

**Robot: `graphs/go2_driver.yaml`** (add to the quickstart's blocks):

```yaml
nodes:
  - id: tf_broadcaster
    type: tf_broadcaster
    outputs:
      - name: tf
        schema: tf2_msgs/TFMessage
        topic: /tf                       # absolute, externally-fixed name
      - name: tf_static
        schema: tf2_msgs/TFMessage
        topic: /tf_static
network:
  mode: peer
  listen:
    - tcp/0.0.0.0:7447
  egress:
    - /tf
    - /tf_static
```

**Mac: `graphs/teleop.yaml`** (the workstation side ingresses both):

```yaml
nodes:
  - id: tf_listener
    type: tf_listener
    inputs:
      - name: tf
        source: /tf                       # external source: arrives over the network
      - name: tf_static
        source: /tf_static
network:
  mode: peer
  connect:
    - tcp/192.168.123.99:7447
  ingress:
    - /tf
    - /tf_static
```

Notes:

- **Ownership.** `/tf` is a multi-publisher opt-in topic LOCALLY (several
  nodes may broadcast into one tree), but across the link the mirrored `/tf`
  arrives through the ONE ingress publisher, so on the workstation the local
  `/tf` is single-writer: an in-graph consumer reads it like any
  absolute-source topic. The consumer's `TFMessage` input resolves the
  ingress schema hash; a `/tf` with no in-graph consumer is refused at build.
- **`/tf` and `/tf_static` are symmetric**: same schema and wire path; the
  only difference is publishing cadence (dynamic vs latched-once), a node
  concern.
- **Byte-identity.** The `TFMessage` frame (40-byte `WireHeader` + offset
  table + opaque `transforms` bytes) is re-injected verbatim (sequence,
  timestamp, nested transform bytes intact), pinned end-to-end by
  `crates/cerulion_core/tests/network_tf_e2e_test.rs`.
