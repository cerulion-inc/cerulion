# Cross-Machine Networking: Permissive by Default, the Gateway, and the `network:` Block

A "how it works and why" reference. A Cerulion robot is
**network-viewable by default**: every real-clock live run registers its
produced topics with a network GATEWAY, which opens the machine's single
zenoh session, announces those topics, and forwards a topic to the mesh the
moment a remote subscriber asks for it. On Unix a permissive run registers
with the machine's shared `cerulion-netd` gateway; a `network:` block and the
fallback routes use a per-run gateway process instead. You restrict this by
adding a `network:` block (the block is a TIGHTENING, not the on-switch), or
turn it off for a run with `--network off`.

Related: `docs/user-api.md` "Cross-machine networking" (the YAML reference +
validation table), `docs/multi_process.md` (networked multi-process: the
per-run gateway serves a monolith AND a supervised worker split alike),
the transport-layer raw-wire-frame bridge, the permissive
default + gateway, the discovery ladder, and pairing.

## TL;DR

- **Open by default.** A real-clock `graph run` (monolith OR multi-process),
  `node run`, or `ros2 attach` with NO `network:` block runs PERMISSIVE: one
  zenoh session scouts the LAN (multicast + gossip ON) and announces every
  produced topic. One loud breadcrumb prints at start:

  > ``network egress OPEN BY DEFAULT: no `network:` block, so every produced topic is network-viewable by any peer on the LAN, with no pairing required. Restrict with a `network:` block, or run `--network off` for local-only.``

- **The `network:` block TIGHTENS.** With an explicit enabled block the
  gateway runs STRICT: it announces + egresses exactly the declared `egress`
  allow-list, opens exactly the declared `ingress` bridges, and uses the
  block's locators VERBATIM (no scouting, no bind ladder). The block is
  how you RESTRICT a robot, not how you enable networking.
- **One robot = one network peer.** The gateway is a SEPARATE process that
  owns the entire network plane (Principle #8). The graph/worker processes
  stay network-free: they publish into and read from shared memory only.
  The gateway taps their SHM zero-copy (listener-less capture subscribers)
  and forwards frames; the tap adds no copy of its own.
- **Kill-switches.** `cerulion graph run <g> --network off` (also on `node
  run`) or the env knob `CERULION_NETWORK=off` (honored by ANY entry point:
  `graph run`, `node run`, `ros2 attach`) runs LOCAL-ONLY: no gateway, no
  session, loud notice.
- **Networked multi-process is FIRST-CLASS.** An enabled block is accepted on a
  multi-process run exactly as on a monolith. The supervisor spawns ONE
  gateway beside the workers; each worker stays network-free.
- **Recording KEEPS the network.** `--record` does not force local-only:
  the robot stays visible while recording. The ONE exception: `--record`
  plus an explicit block that declares `ingress:` is refused (because
  recorded ingress re-injection is not replay-faithful).
- **`topic list` discovers local and LAN topics.** `cerulion topic list` lists
  LOCAL topics instantly, then discovers REMOTE topics over the LAN by default
  (scouting ON). The remote half is the discovery ladder (~1.5 s ceiling), a
  connect phase hard-bounded at 1 s, and the demand/announce reply gathers
  (~0.5 s after). It completes within ~3 s worst case, ceiling-bounded; the
  LOCAL list has already printed by then. `--no-network` skips the remote
  query; extra `--connect`/`--listen` locators reach peers scouting can't
  find.
- **Robots are FOUND, not typed.** mDNS `_cerulion._tcp` is the ONE
  gateway beacon and the PRIMARY discovery mechanism on every network: "your
  robot shows up like a Chromecast"; SRV = the bound port, TXT `robot=<name>`.
  **The beacon is gated on having a LISTEN endpoint**, so which path you are on
  decides whether it exists: a per-run gateway child binds 7683 and beacons; the
  default netd-hosted plane listens and beacons **only when
  `CERULION_NETD_LISTEN` is set** (see "Making a robot discoverable" below).
  A robot with neither is reachable only through zenoh multicast scouting, where
  the LAN allows it, which is why the failure looks intermittent. The client-side
  discovery LADDER (mDNS browse, cached peers, hostname convention; in
  parallel, ~1.5 s ceiling) produces candidate locators for `topic list`,
  which prints a `ROBOTS` section above the remote topics. A ROBOTS row is
  LIVE PRESENCE: a robot appears iff its ANNOUNCE tokens actually arrive in
  the gather (each announce key carries the producing robot as its first
  chunk, `cerulion_ann/{robot}{topic}`, so even absolute mirror topics
  attribute exactly) OR an mDNS browse answers for it (a browse answer is
  itself a live gateway); mDNS also enriches announce rows with the
  locator. Never a mere candidate. mDNS finds robots; zenoh connects and
  does everything after. (There is no per-candidate verify probe: presence
  IS the verification.)
- **Open by default is unauthenticated.** A robot with no `network:` block is
  viewable by any peer on the LAN; restrict it with a `network:` block or
  `--network off`.

## The decision matrix

Every real-clock live run reaching `graph run` / `node run` / `ros2 attach`
resolves to exactly one of these, in evaluation order:

| Condition | Result |
|---|---|
| `--network off` **or** `CERULION_NETWORK=off` | LOCAL-ONLY: no gateway, no session; loud kill-switch notice. |
| `--time-source virtual` **or** `external` | Network INERT (replay-class: a zenoh session is a live side effect that would break replay byte-identity, Principle #7). Silent, no gateway. |
| Enabled `network:` block + `--record` + declared `ingress:` | **Refused**, naming the topics + both workarounds. |
| Enabled `network:` block (egress-only under `--record`, or any non-record) | **Strict** gateway: verbatim locators, egress allow-list, declared ingress. |
| No / disabled block (incl. `--record`) | **Permissive**: scouting ON, announce every produced topic. Egress rides the machine's shared `cerulion-netd` session on Unix (see below); the synthesized 7683 listen + bind ladder appear only on the permissive FALLBACK child (netd unreachable, or non-Unix). |

`graph profile` is the one deliberate exception: a bounded MEASUREMENT run,
not a deployment, so it stays LOCAL-ONLY by design (its transport is built
`network: None`; it never resolves a network). Re-execution
(`cerulion bag play <bag> --resim all`) is likewise network-inert by
construction (the bag is the input, Principle #7).

## Egress converges onto cerulion-netd

Every PERMISSIVE real-clock live `graph run` on a Unix machine routes its egress
through that machine's ONE `cerulion-netd` session **by default**, NOT a per-run
gateway child. There is no desk/robot branch in the routing: a robot running a
bare permissive `graph run` joins netd exactly like a desk does. This is
Principle #8 applied to the whole machine: N graphs + N remote consumers share ONE
zenoh session instead of one per graph.

A **desk**, a machine that both produces topics and consumes remote ones
(Studio, `topic echo`, a bridge graph), is where the session count would
otherwise multiply fastest. Graphs and remote consumers on one machine share
that one daemon and one zenoh session, whatever the machine is.

- **How.** A producing graph pushes its egress plan into `cerulion-netd` over
  the `register_egress` UDS verb (spawning netd detached + refcounting exactly
  like the ingress consumers do); netd boots ONE shared embedded gateway on the
  first egress plan and grows the egress set at runtime. Closing the connection
  (clean exit, panic, or Ctrl-C: anything that drops the fd) RELEASES the
  registration (the crash-safe refcount). No per-run gateway child is spawned.
- **Namespace verification.** The run forwards its resolved iceoryx2 namespace
  in the register (`ix_config_json`: `Some` for a multi-process deployment, its
  supervisor-minted shared worker namespace; `None` for a monolith, which shares
  netd's default namespace). netd's gateway taps SHM on ITS namespace, so it
  verifies the forwarded SHM-discovery identity matches its shared session before
  tapping. That identity is every field that determines WHICH SHM files a topic
  resolves to (`root_path` + `prefix`, the `service` and `node` directories, and
  the per-file service suffixes), NOT just `(root_path, prefix)`: two configs that
  agree on root+prefix but differ in `service.directory` discover DIFFERENT
  services, so verifying root+prefix alone would let netd tap its own empty
  service directory while the run silently egresses nothing. It deliberately does
  NOT include the dead-node-cleanup flags (netd disables them, a forwarded run
  leaves them on; folding them in would false-mismatch every common-case run). A
  run that resolved a DIFFERENT namespace (a divergent `IOX2_CONFIG_FILE` between
  netd's spawn and the run's, or a custom `service`/`node` directory) is REFUSED:
  the run then falls back to a per-run gateway child on its OWN namespace, never a
  silent no-egress.
- **When a per-run gateway child is used instead.** Two cases, distinguished by
  loudness:
  1. **STRICT posture** (an explicit `network:` block: verbatim locators +
     egress allow-list): a QUIET routing rule. netd's shared session has its own
     locator config and boots its gateway PERMISSIVE, so a Strict run's verbatim
     locators + egress allow-list are un-representable on the shared session.
     **Strict is a permanent child-routing rule**: a Strict run
     spawns a per-run gateway child that honors the block verbatim. This is the
     designed steady state, not a degrade, so it routes on a `tracing::debug!`
     (suppressed at the default `info` level): no loud warn. (A permissive run,
     by contrast, still fires its one-shot `PERMISSIVE_EGRESS_NOTICE` warn when it
     spawns a child, but a Strict run does not.)
  2. **netd unreachable / a namespace mismatch**: LOUD. A netd registration
     failure (netd unspawnable, or the run resolved a DIFFERENT iceoryx2
     namespace than netd, caught by comparing the forwarded `ix_config_json`) is an
     unexpected degrade, so it warns LOUDLY (`tracing::warn!`) before falling back
     to a per-run gateway child on the run's own namespace (the same warn-but-run
     posture as the local-only fallback).

The `cerulion-netd` daemon self-exits after an idle grace, so a machine that
stops producing/consuming leaves nothing behind.

## Making a robot discoverable

On the default permissive Unix path a run's egress rides the machine's shared
`cerulion-netd` session, and **nothing forwards a synthesized 7683 listen to
it**: netd takes its listen endpoints only from `CERULION_NETD_LISTEN`, and the
embedded gateway's mDNS beacon is gated on a parseable `tcp/` listen endpoint.
A stock robot with no env set and no `network:` block is therefore reachable
only through zenoh multicast scouting where the LAN allows it; it does NOT
answer on 7683 and does NOT show up over mDNS.

Give netd a listen endpoint on any machine that should be findable:

```bash
CERULION_NETD_LISTEN=tcp/0.0.0.0:7683
```

Set it in the robot's service environment and restart `cerulion-netd`. With it
set, the shared plane binds the port and the beacon advertises the bound port
plus `robot=<name>`, and the Chromecast-style discovery above applies.

The alternative is a per-run gateway child, and which child you get matters:

- **Permissive fallback** (netd unreachable, or a non-Unix host): synthesizes the
  well-known 7683 listen, bind-probes upward, and beacons on its own.
- **Strict** (an explicit `network:` block): binds the block's `listen` locators
  VERBATIM: no synthesized port, no bind ladder, and a stolen port fails the boot
  loudly. It beacons only if one of those locators is a parseable `tcp/` endpoint.
  Your config is the truth, so give it the port you want advertised.

## The gateway

A per-run gateway process owns one run's whole network plane: the single zenoh
session, every egress announce, the demand watch, egress forwarding, and the
ingress re-injection door. It is the **Strict-or-fallback** path everywhere:
spawned for a Strict posture (an explicit `network:` block, which netd's shared
permissive session cannot represent), when netd registration fails, and on
non-Unix platforms, which have no netd. On a permissive Unix run, robot
included, egress goes to netd instead and no gateway child is spawned. It is spawned by the monolith
`graph run` arm and by the multi-process supervisor alike (a hidden
`cerulion graph run-gateway` subcommand you never invoke directly), handed a
serialized plan + the run's iceoryx2 namespace.

- **Port.** A per-run gateway CHILD listens on the well-known port **7683**
  (IANA-unassigned) and bind-probes UPWARD (up to 8 tries)
  on an address-in-use conflict, so two robots on one host coexist. This does
  NOT apply to the default netd-hosted plane, which takes its listen endpoints
  only from `CERULION_NETD_LISTEN` and synthesizes nothing. Override
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
  (loopback, listener-ful peers, scouting-on meshes) plus the flap/absence-
  hysteresis + enable-flood guard everywhere; on a strict connect-only link the
  demand path is the demand QUERYABLE inversion below, which moves demand onto
  the two directions that carry.
- **Query surface + demand inversion.** On a strict connect-only, listen-less,
  scouting-off zenoh 1.8 peer link the routing is asymmetric, as measured
  against that version and configuration: the DIALER's declarations do not
  reach the ACCEPTER and the ACCEPTER's GETs do not route to the DIALER. The
  directions that carry are ACCEPTER→DIALER declarations (at connect) and
  DIALER→ACCEPTER queries. A
  demand design on either of the other directions (the dialer DECLARING a
  `cerulion_lv` token, or the accepter GETting `cerulion_lv/**`) stays dark on
  such a link, so demand rides the two directions that carry, folded
  into ONE per-robot, verb-dispatched QUERY SURFACE: the PRODUCER (the
  accepter/listener) DECLARES a single wildcard QUERYABLE at
  `cerulion_q/{robot}/**` (an accepter declaration: reaches the dialer) and
  dispatches inbound GETs by verb. The `demand` verb: the DEMANDER (dialer) runs
  a ~2 s demand-GET loop that GETs `cerulion_q/{robot_or_*}/demand{topic}` (a
  dialer query: reaches the accepter), which pulls that topic's egress ON with a
  synchronous `enabled`/`refused` ack. The `catalog` verb answers a
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
  within one GET interval. Principle #3 counters (`demand_grant_count` /
  `demand_expiry_count` on the producer; `demand_enabled_ack_count` /
  `harvested_identity_count` on the demander) attribute each flip. Like the
  reconciler, the queryable + GET loop exist ONLY on a gateway (never under
  virtual/replay: zero Principle-#7 surface).
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
| `egress` | The per-graph export SCOPE (a SAFETY filter, not authorization). Each must be produced by an in-graph node; only these leave the machine. WHO may demand them is decided by the account/pairing grant, not this list. |
| `ingress` | Topics this graph imports. Each must NOT have an in-graph producer. |

`router` mode is deliberately not exposed: a graph declares a participant,
not infrastructure. A Strict block's scouting stays OFF (the mesh is exactly
the locators you declared); scouting-ON is the permissive-default behavior,
not a block knob.

### The egress/ingress model

The graph file stays the single source of truth (Principle #5): under a
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
  (`/{prefix}/{node}/{output}`) or a `topic:` override. The gateway
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

### Loop-safety rules (graph-load validation)

Every rejection names the offending topic and states the fix:

1. **No topic in both lists** (it would loop back to itself): remove it
   from one list. The gateway ALSO refuses a plan whose announce and ingress
   sets intersect, as a structural backstop.
2. **Egress must be produced in-graph.**
3. **Ingress must NOT be produced in-graph.**
4. **Canonical absolute names only** (bare names rejected with the expected
   `/`-prefixed form; malformed shapes rejected like any topic name).
5. **No lists under `disabled`.**
6. **No duplicates within a list.**

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
network-inert (the bag is the input, Principle #7).

## CLI introspection: `topic list`

```bash
# Local topics, then whatever the LAN is advertising (no flags needed):
cerulion topic list

# Reach a peer scouting can't find (another subnet). Works against a robot
# that is actually LISTENING: a per-run gateway child on 7683, or a netd
# plane given CERULION_NETD_LISTEN (see "Making a robot discoverable"):
cerulion topic list --connect tcp/192.168.123.99:7683

# Multicast AND mDNS both blocked? Opt into a unicast /24 sweep (rung 4).
# OFF by default; a horizontal connect sweep reads as port-scan recon:
cerulion topic list --scan

# Scripts / CI / offline (skip the remote query entirely):
cerulion topic list --no-network
```

`topic list` prints the LOCAL `TOPIC` section FIRST and instantly (a slow or
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
line, `remote: none discovered in 500 ms (a robot off the LAN needs --connect
tcp/<host>:7683)`; a peer that is reachable (a given locator, a discovered
robot) but advertised no topic in time gets `retry` instead.

**Mirror provenance: one data source = one topic.** When a desk
process re-injects a remote robot's topic into local SHM (vizd's remote viz,
`topic echo`'s auto-ingress, `cerulion connect`'s iroh re-inject), the mirror
lands as a plain `{topic}/data` service, which `topic list`'s local
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
name is untouched (a consumer still subscribes to the exact mirrored name:
"remote = local"); only the `topic list` presentation folds it into REMOTE. The
Studio sidebar parses the same rows: the topic path stays the row's first
whitespace token, with the `● streaming` marker + robot in the trailing columns.

- **Remote discovery is ON by default** (scouting multicast + gossip, plus
  the ladder): an unpaired robot on the LAN shows up with no flags.
  `--no-network` opts out of the whole remote half, ladder included.
- **Extra locators are additive**: `--connect` /
  `--listen` add to the default scouting session (both repeatable) to reach
  a peer scouting cannot find (remote discovery is the default, so there is
  no flag to turn it on). Ladder-found gateway locators fold into the
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
  (`--connect tcp/<host>:7683`). An isolated network with no LAN discovery has
  no automatic answer: there is no rendezvous tier, so pass the robot's
  locator with `--connect`.

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
| Skew | The push verbs arrived in netd control protocol **v6**, and the daemon speaks **v7** today; a new client against an older daemon is refused by the per-verb gate and degrades loudly to its own refresh path, and an older client against a daemon that has them is simply never subscribed. vizd's protocol version is deliberately UNCHANGED (a new verb is additive, and its controllers compare the banner for exact equality); capability there is negotiated by verb. |

**What is NOT event-driven:** the LOCAL half. The push above covers REMOTE
topics; a LOCAL iceoryx2 service appearing or disappearing on the desk carries
no liveliness token (there is no event to subscribe to), so a locally
produced topic's arrival is still picked up by whatever refresh the
consumer already does. Building a local watcher would mean polling
`list_topics` on a timer, which is the very thing this removes.

Turn it off by pointing the desk at a `cerulion-netd` older than v6, the version that introduced the push verbs, or by
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

The ladder answers "which robots exist?" BEFORE any session is dialed. Its
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

Zenoh scouting stays ON underneath as the TRANSPORT mesh (robot↔robot
links, gossip) and as one more parallel discovery input; its hits merge
into the same query session.

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
only a rendezvous point both sides dial out to survives it. There is no
rendezvous tier.

### `cerulion-netd` folds the cache in too

The ladder above is what `topic list` runs. `cerulion-netd`, the daemon
every OTHER surface rides (`topic hz`/`echo`/`info`, `schema info`, vizd/Studio
attaches), opens its one session with **scouting**, plus the
`CERULION_NETD_CONNECT` escape hatch. On any network where scouting is degraded
(Tailscale, multicast-filtered wifi; the common case) scouting alone would
leave the daemon serving Studio unable to dial a robot whose verified locator
the desk's own `peers.json` already holds, and first contact would need an
operator to hand-export that env var.

**So netd folds the cache in at boot**, from the same sources in the same
order: explicit `CERULION_NETD_CONNECT` locators first, then TTL-fresh
`peers.json` rows. So: run `topic list` once (or `cerulion viz --robot NAME`,
or open Studio), and from then on every netd on that machine reaches that robot
with **no environment variable set**. A desk that has never seen the robot
falls back to scouting, and a plain "not found".

Two details worth knowing:

- **A cached row is TCP-probed; your `CERULION_NETD_CONNECT` locator is not.**
  A cached address is a hint that may be stale, and a dead one would otherwise
  buy a background retry connector for the life of the daemon. Your explicit
  locator is your word: netd's session is long-lived, so zenoh retries it in the
  background and a robot that is merely booting comes up on its own. (This is
  the opposite of `topic list`'s rule, which drops an unreachable explicit
  locator; that command is a one-shot gather under a *fatal* 1 s connect bound,
  where one hanging endpoint sinks every other robot.)
- **The fold happens once, at daemon start.** netd's session config is immutable
  after init, and netd self-exits on idle, so a restart is the refresh. A netd
  held alive for hours by a running Studio keeps its boot-time dial list; if a
  robot changes address mid-session, restart netd (or just let it idle out).

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
{"hello":"cerulion-netd","protocol":7}
{"id":1,"demands":[],"active_connections":1,"idle":false,"connect_endpoints":["tcp/203.0.113.101:7683"]}
```

Interpreting `connect_endpoints`:

- **`connect_endpoints` is what the daemon was CONFIGURED to dial, not what it
  reached.** netd's zenoh session is lazy (a netd nobody has demanded from has
  opened no session at all), and a fiat-trusted dead locator reads identically to
  a live one. Use it to answer "why is my desk talking to that address?", not
  "is that robot up?".
- **An older daemon omits the field entirely, which decodes as *unknown***, never
  as "this daemon dialed nothing". An explicit `[]` is the real "nothing folded".

## Two-machine quickstart (Go2: Mac ↔ robot)

You do not NEED a `network:` block: a bare `cerulion graph run` on
each side is already network-viewable. Add blocks to RESTRICT what crosses
and to pin locators for a subnet scouting can't bridge. The robot (Jetson
Orin, `192.168.123.99`) LISTENS; the workstation (Mac) CONNECTS.

The node types in these sketches (`lidar_driver`, `locomotion`,
`keyboard_teleop`, `obstacle_monitor`) stand for your own `nodes/<type>/`
crates. To just LOOK at the robot's topics from the workstation you need no
graph and no node at all: `cerulion viz --robot <name>` demands them and
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
  listen:
    - tcp/0.0.0.0:7447
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
  connect:
    - tcp/192.168.123.99:7447
  egress:
    - /go2/cmd_vel/keyboard
  ingress:
    - /go2/utlidar/cloud
```

Sanity-check the link before starting the Mac graph:

```bash
# On the Mac, with the robot graph running:
cerulion topic list --connect tcp/192.168.123.99:7447
# REMOTE TOPICS
# /go2/utlidar/cloud
```

Then `cerulion graph run go2_driver` on the robot and
`cerulion graph run teleop` on the Mac.

## What crosses the wire

Local delivery stays zero-copy shared memory; only egress topics pay the
network. An exported frame travels as the SAME unified wire format (32-byte
`WireHeader` + payload) VERBATIM as the zenoh payload on the key
`cerulion{topic}`: no envelope, no per-hop translation, so the receiving
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
- **Byte-identity.** The `TFMessage` frame (32-byte `WireHeader` + offset
  table + opaque `transforms` bytes) is re-injected verbatim (sequence,
  timestamp, nested transform bytes intact), pinned end-to-end by
  `crates/cerulion_core/tests/network_tf_e2e_test.rs`.
