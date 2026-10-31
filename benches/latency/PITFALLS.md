# Pitfalls catalog

Everything that produced a wrong, missing, or misleading number in prior
campaigns, in symptom → cause → fix form. If a sweep is producing zero or
partial `.bin` files for a cell, or a number that looks too good or too
bad to be true, start here before re-deriving any of it.

Two sections:

- **Part 1: ROS 2 / RMW pitfalls** (#1 to #12): distilled from the prior
  campaign across three RMWs, multiple distros, and two SHM
  modes. The fixes are encoded in `ros2/` (configs, `run_bench.sh`,
  `verify_shm.sh`, the per-cell docker flags); the entries explain *why*
  so an upstream package update that re-breaks a cell can be diagnosed
  instead of re-discovered. Distro caveat: the prior campaign ran
  humble/jazzy/kilted; this suite runs humble/jazzy/**lyrical**. Where an
  entry names a version boundary, re-verify it on lyrical before citing it.
- **Part 2: Operational pitfalls** (#13 to #21): transport- and
  harness-level failure modes accumulated since that campaign, several of which
  silently invalidate results rather than loudly failing. These are the
  dangerous ones. (#22 to #25 follow Part 2: a Lyrical build
  break in the middle of a sweep, two classes carried over from the
  prior campaign (the iwlwifi
  host wedge and the Humble 16 MB iceoryx-v1 ceiling) and the
  external-warmer null.)

No latency numbers appear here except inside clearly-labeled
prior-campaign provenance notes.

---

## Part 1: ROS 2 / RMW pitfalls

### 1. FastDDS SHM-only XML kills participant discovery

**Symptom:** `Waiting for at least 1 matching subscription(s)` repeats,
then `failed to get subscription count: publisher's context is invalid`.
Publisher never matches subscriber; the cell produces 0 bins. Seen on
jazzy/kilted-era Fast DDS (2.14 / 3.x); humble's 2.6 tolerated it.

**Cause:** `useBuiltinTransports=false` plus a SHM-only
`<userTransports>` block leaves the participant with no UDP/TCP wire,
and Fast DDS's Participant Discovery Protocol (PDP) **always** uses
UDP/TCP even when SHM carries the data. SHM-only kills PDP; the endpoints
never see each other.

**Fix:** keep `useBuiltinTransports=true` while still registering the SHM
transport descriptor (for segment-size tuning). Data auto-prefers SHM
intra-host; discovery keeps UDP. Encoded in `ros2/configs/fastdds_shm.xml`.
Do **not** fix it by hand-writing a UDP transport descriptor; that loses
Fast DDS's auto-tuned discovery timing defaults.

### 2. `FASTRTPS_DEFAULT_PROFILES_FILE` is deprecated in Fast DDS 3.x

**Symptom:** a deprecation warning from `rmw_fastrtps_shared_cpp`; on
Fast DDS 3.x the old env var still loads the profile file today, but the
back-compat path is slated for removal; a future distro bump silently
stops loading your XML entirely (every QoS/SHM setting reverts to
defaults, which *changes the measurement* without failing anything).

**Cause:** Fast DDS 3.x renamed `FASTRTPS_*` env vars to `FASTDDS_*`.

**Fix:** export **both** names, always:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$XML"   # Fast DDS 2.x (humble)
export FASTDDS_DEFAULT_PROFILES_FILE="$XML"    # Fast DDS 2.14+/3.x (jazzy, lyrical)
```

Encoded in `ros2/run_bench.sh`.

### 3. `/__verify_shm` is an invalid topic name on newer distros

**Symptom:** the SHM verification helper reports `publish failed` for
every RMW × mode combo on jazzy and newer, noise that can mask a real
failure in the same log.

**Cause:** ROS 2 topic naming rules reject *repeated* underscores;
`/__verify_shm` parses as one. Humble's validator was looser; newer
distros enforce the spec strictly.

**Fix:** single underscore: `/_verify_shm` (a valid hidden topic).
Encoded in `ros2/verify_shm.sh`.

### 4. `ros2 topic pub --once` waits for a matching subscriber by default

**Symptom:** the verification publisher visibly starts, then hangs until
timeout; the script reports failure even though the participant is
healthy.

**Cause:** ros2cli changed `--once` to default to `-w 1` (wait for ≥ 1
matching subscription before publishing). A verification publish with no
subscriber waits forever.

**Fix:** pass `-w 0` explicitly. Encoded in `ros2/verify_shm.sh`
(combines with #3).

### 5. rmw_zenoh requires the `rmw_zenohd` router

**Symptom:** `Unable to connect to a Zenoh router` /
`Scouting delay elapsed before start conditions are met`; pub never sees
sub; 0 bins.

**Cause:** rmw_zenoh on jazzy and newer ships with multicast scouting
OFF and gossip-via-router enabled; without a running `rmw_zenohd`,
peers never discover each other.

**Fix:** launch `rmw_zenohd` in the background *before* any rmw_zenoh
process, including the verification helper. Encoded in
`ros2/run_bench.sh`. Sub-pitfall: when a helper starts its own router,
it must track the PID and kill only its own; a blanket
`pkill rmw_zenohd` kills the bench's router out from under the next
cell. Bounce the router between SHM-on and SHM-off modes: SIGKILLed
bench nodes leave ghost subscriptions in the router's discovery state
that confuse the next cell's match counting.

### 6. Zenoh SHM log-marker grep drifts across zenoh versions

**Symptom:** `verify_shm.sh` fails a `zenoh × shm` batch even though SHM
is genuinely engaged; or, in the inverse (pre-hard-gate) world, a stale
grep silently records UDP-fallback rows under an `shm` label.

**Cause:** zenoh's log format for SHM initialization changes across
versions; a marker pattern derived on one version misses the next.

**Fix:** in this suite a failed `verify_shm.sh` is a **hard cell
failure** (the retry / documented-empty path: an `shm` label must be
earned, never assumed; METHODOLOGY §9). When the failure is marker
drift rather than a real fallback: re-derive the marker from
`RUST_LOG=zenoh=debug` output on the new version and update
`verify_shm.sh`. To salvage a run while investigating,
`CER_BENCH_ALLOW_UNVERIFIED_SHM=1` records the cell but writes a
`_logs/<cell>_<size>_SHM_UNVERIFIED` marker and warns loudly in both
the container and the runner; the rows are permanently labeled, never
silently trusted. Diagnostic cross-check either way: the latency curves
themselves; an SHM path and a UDP-loopback fallback are far apart at
small payloads, and flat-vs-climbing at large payloads is unambiguous.

A receiver-side structural trick from the prior campaign, independent of
any log marker: in Rust zenoh code, read the payload via
`sample.payload().as_shm()` instead of `.to_bytes()`; `as_shm()`
explicitly requests the SHM reference and **panics if SHM is not
active**, making the receiver self-verifying per message. (The native
`zenoh_shm_round_trip` pair verifies via the forced
`message_size_threshold: 0` config plus the smoke gates; adopt
`as_shm()` if that ever needs a per-sample proof.)

### 7. SCHED_FIFO + containers can SIGKILL/segfault small-payload cells

**Symptom:** a `chrt=1` cell dies (`Killed` or `Aborted`) at small
payloads while the identical `chrt=0` cell passes.

**Cause (not fully root-caused in the prior campaign):** SCHED_FIFO at
priority 80 inside a container exposes RT-throttling
(`sched_rt_runtime_us`), RT tasks starving non-RT kernel threads, and
cgroup memory edge cases under loaned-buffer SHM allocation.

**Fix:** none known that preserves the cell; the runner's 3-attempt
retry gives transient variants a chance, and a persistently failing
chrt-on cell is recorded as a documented skip, never silently dropped,
never patched around by weakening the chrt setting (that would relabel
the line).

**Bare-host variant:** RT throttling is not
container-only. On a bare host, whole-tree SCHED_FIFO over spinning
worker loops exhausts `sched_rt_runtime_us` and shows up not as a kill
but as a 1.000 s-periodic comb of 12.8 to 29.6 ms latency spikes plus
fallback-ladder steps; see the METHODOLOGY §7 chrt note. Diagnose it by sampling
`/sys/kernel/debug/sched/debug` for `rt_throttled=1` during the cell,
and by the spike grid: magnitudes below `period − runtime` (50 ms at
the default 950000/1000000), spacing exactly one RT period.

**NEVER disable the guard on a remote machine.** Setting
`sched_rt_runtime_us=-1` to "confirm" the mechanism takes the machine
off the network within a minute (the RT cohort starves the NIC softirq
core; tx-only, power-cycle to recover). The throttle comb is the
kernel keeping the machine alive. Dose WITHIN the guard instead:
e.g. 990000/900000 µs move the forced stall to 10/100 ms per second
and the comb magnitude should track it, with the kernel always keeping
an escape.

### 8. Docker: scripts baked into the image don't pick up host edits

**Symptom:** you patch a script on the host; behavior inside the
container is unchanged.

**Cause:** the file was COPYed into the image at build time and the
container runs the baked copy.

**Fix:** bind-mount everything that iterates (`configs/`, `scripts/`,
`run_bench.sh`) read-only into the container; rebuild the image only for
source/toolchain changes. If a new script isn't taking effect, check the
mount list first.

### 9. CycloneDDS 0.10+ renamed XML schema elements

**Symptom:** `//CycloneDDS/Domain/General: NetworkInterface: unknown
element`, then `rmw_create_node: failed to create domain`; all bench
binaries abort at startup on jazzy and newer.

**Cause:** Cyclone DDS 0.10 moved `<NetworkInterface>` inside an
`<Interfaces>` wrapper and moved `<FragmentSize>` from `<Internal>` to
`<General>` (naked integers also deprecation-warn without an explicit
`B` unit suffix).

**Fix:** use the new wrapped form; it is backward-compatible with
Cyclone 0.9 (humble), so one XML serves every distro:

```xml
<General>
  <Interfaces>
    <NetworkInterface autodetermine="true"/>
  </Interfaces>
  <FragmentSize>65500B</FragmentSize>
</General>
```

Do not keep the old XML with per-distro special-casing. Encoded in
`ros2/configs/cyclonedds_no_shm.xml`.

### 10. A cell can report PASS with partial payload coverage

**Symptom:** the sweep summary says PASS but the cell has `.bin` files
for only some of the 10 payload sizes.

**Cause:** the prior campaign's driver accepted a cell at ≥ 50% of sizes,
a deliberate trade-off (partial data beats no data when a cell
predictably fails at 16 MB), but one that reads as complete if you don't
know it.

**Fix:** know the threshold before trusting a PASS; when a claim needs
full coverage, count the `.bin` files (or tighten the driver's threshold
and accept more failed cells). Either way, per-size coverage is checked
from the artifacts, never assumed from the summary line.

**This suite's drivers are strict by default:** `run_bench.sh` exits 13
if *any* requested size fails its exact measured-count gate;
`bench.py`'s per-bin success gate is the exact per-size `.bin` set for
the sizes requested (never `glob > 0`, and stale `.bin`s for a prefix
are removed before the run so a leftover file can't stand in for a
fresh one); `compile_csv.py` exits nonzero listing every missing /
under-count (prefix, payload) row; strict means the schedule's exact
expected measured count per (variant, payload), not merely ≥ 1 sample,
so a truncated `.bin` from a killed run cannot summarize into a
full-looking row (`--allow-partial` demotes that to loud warnings);
and `plot.py`'s strict mode verifies every series carries all expected
payload rows (`--skip-missing` is the documented escape hatch). The
prior campaign's ≥ 50% threshold survives only as history; know it when reading
prior-campaign artifacts.

### 11. `rcl_take_loaned_message` is a NO-OP stub on rmw_zenoh, and the capability flag lies both ways

**Symptom:** every `zenoh × loan` cell produces 0 bins, and the attempt
log balloons (multi-GiB in the prior campaign) with truncated rcutils
error strings repeated per take attempt.

**Cause:** rmw_zenoh's `rmw_take_loaned_message` returns
`RMW_RET_UNSUPPORTED` on **every released version** (tracked upstream in
ros2/rmw_zenoh #175 / #893). A loop that treats the failure as "queue
empty, try again" spins forever, spewing one truncated-error warning per
attempt.

**Fix:** probe `can_loan_messages()` after subscription creation; if
false AND the RMW is the known take-stub RMW (rmw_zenoh), exit rc=77
(the autotools SKIP convention) so the driver records a structural
skip. **Re-verify on new releases**: the skip should be removed the
day the read loan lands.

**The trap inside the fix:** `can_loan_messages()` is NOT a reliable
indicator: **both ways, and for a different reason than the prior
campaign guessed**. The prior-campaign "returns `false` on CycloneDDS while
`rcl_take_loaned_message` works fine at SHM-floor latencies" was NOT a
CycloneDDS quirk: it was the **rcl env gate** at the rclcpp layer
(rclcpp#2335 / rcl#1110: subscription-side loans default OFF unless
`ROS_DISABLE_LOANED_MESSAGES=0`, while `rcl_take_loaned_message`
bypasses the gate entirely), which affects EVERY RMW identically. With
`=0` exported on the loan lane (run_bench.sh does this) the bit reads
the RMW's real capability. Do not generalize the zenoh skip to other
RMWs: let the take loop run everywhere except rmw_zenoh, and keep
FastDDS on the runtime probe. And do not "fix" it by handling
`RCL_RET_UNSUPPORTED` deep in the take loop; that buries a structural
limitation in tactical noise.

**The lyrical inversion (root cause known):** on
lyrical the capability bit is literally **uninitialized memory**:
rmw_cyclonedds 4.1.4 declares `ArrayValueType::m_is_self_contained`
(TypeSupport2.hpp:229) but never initializes it, so
`is_self_contained()` returns an indeterminate bool (UB) for every
message type containing a fixed-size array member (every `Pod<N>` in
this bench; most real sensor types). Both garbage values are fatal:
`false` → `can_loan_messages=0` on pub AND sub, every take-loan returns
UNSUPPORTED per call (the Ready→take-fail 100%-core spin; loaned
publish silently downgrades too; lyrical logs `loaned=0` on all
lanes); `true` (observed under gdb) → the guard passes and `dds_take`'s
loan path hits the rmw's unimplemented sertype op (serdata.cpp:660) →
uncaught `std::logic_error("not implemented")` → SIGABRT inside
`rcl_take_loaned_message`. A minimal one-.cpp plain-rcl repro (PSMX iox
engaged, plain take 200/200 as the control) proves it is not a lane
bug; the gdb backtrace of the repro shows the frames named above.
Bug present on the upstream `lyrical` branch tip; upstream fix is a
one-line ctor init. Nearest upstream issue: ros2/rmw_cyclonedds#585
(variable-size types over PSMX); loan API history: rmw_cyclonedds
PR#297. The suite's structural skip is
`lyrical_cyclonedds_*_loan_*` in `bench.py`; remove it the day the
init fix ships. On Cyclone 11 note also that `rmw_publish_loaned_message`
is literally `return rmw_publish(...)` (a plain publish), so even
`loaned=1` there would not prove zero-copy publish.

**Prior-data caveat (provenance: prior campaign):** early
`humble_zenoh_*_loan` rows in the old trees cannot have come from a
working loan path (the stub is unsupported on humble too) and were
flagged non-comparable. This suite's structural skip prevents the class.

### 12. zenoh + SHM dies with ENOMEM inside containers (RLIMIT_MEMLOCK)

**Symptom:** every `zenoh × shm` cell produces 0 rows;
`Unable to create POSIX shm segment: OS error 12` during `rmw_init`.

**Cause:** the error text points at `shm_open`/`ftruncate`, but the
actual failure is the `mlock()` zenoh's POSIX SHM provider issues on its
pool: Docker's default `RLIMIT_MEMLOCK` (8 MB) is smaller than zenoh's
pool (64 MB), so `mlock` returns ENOMEM and session construction throws.
`--shm-size` is a red herring: the tmpfs budget is fine; the pinned-
memory limit is the bottleneck.

**Fix:** `--ulimit memlock=-1` on every ROS 2 cell's `docker run`
(encoded in the per-cell flags). The related FastDDS large-payload
failure is different: 16 MB round trips need the SHM **segment** raised
(256 MiB `segment_size` in `fastdds_shm.xml`); both directions of a
16 MB exchange must fit in flight.

**On a bare host the same limit hangs instead of erroring.** Under the
stock 8 MiB host `RLIMIT_MEMLOCK`, the native `zenoh_shm` line hangs
deterministically at its 5th payload size (16 KB): no error, no crash,
the initiator's `recv()` never returns, and only the watchdog ends the
cell. The same binary completes all 10 sizes with `memlock unlimited`,
so the limit is the cause and not the code. `bench.py` preflights this
(`ensure_memlock_for_zenoh`): soft→hard raise, then
`sudo -n prlimit --memlock=unlimited` on itself, and REFUSES the zenoh
cells loudly if neither works. Permanent host fix:
`<user> - memlock unlimited` in `/etc/security/limits.d/` (note:
SSH sessions arriving over the overlay VPN inherit the VPN daemon's systemd limits, not PAM's;
a `LimitMEMLOCK=infinity` drop-in on the service covers those).

---

## Part 2: Operational pitfalls

### 13. Stale binaries silently attribute the old build's behavior to your change

**Symptom:** a fix or regression "has no effect", or an entire A/B
tells a coherent, wrong story. Nothing fails; the numbers are simply
about the wrong code.

**Cause:** a runner that rebuilds conditionally (`if [ ! -x ]`) or not
at all reuses whatever binary/cdylib is lying in `target/`. In a prior
campaign, every aarch64 run that was meant to measure
a code change had silently reused
a build from before that change; the entire
result set was voided when the staleness was discovered, and validation
had to be redone.

**Fix:** **rebuild always, unconditionally, immediately before
measuring**: the binary AND the cdylibs, from the same checkout, in the
same pass. Every runner in this suite does; if you drive a binary by
hand, `cargo build --release` first, every time. Also remove stale
debug-profile cdylibs before workspace runs: the CLI's freshest-wins
cdylib resolution can pick a stale debug artifact over your fresh
release one.

### 14. fd exhaustion masquerades as `ServiceInCorruptedState`

**Symptom:** graph build fails with
`PublishSubscribeOpenError::ServiceInCorruptedState`, which reads like
stale/corrupt SHM state, sending you down a cleanup rabbit hole that
never helps.

**Cause:** it is neither corruption nor stale state. iceoryx2 holds file
descriptors per service/port; under the default `ulimit -n 1024`, a
graph of moderate size exhausts the fd table mid-build and iceoryx2
surfaces the failure with this misleading error.

**Fix:** `ulimit -n 65536` in the shell (or runner) before any workspace
run. Encoded in `workspace/run_workspace.sh`. If you see this error,
check `ulimit -n` before touching `/dev/shm`.

### 15. iceoryx2 sub-crate version skew = silent data-plane death

**Symptom:** everything builds, everything starts, no errors, and no
node ever fires. The data plane is simply dead.

**Cause:** the iceoryx2 dependency family must resolve to **exactly one
version** across every artifact that shares an SHM connection (the host
binary and every node cdylib). A skew in any `iceoryx2-*` sub-crate
(e.g. `iceoryx2-bb-elementary` 0.10.0 vs 0.10.1) changes the internal
`PackageVersion` handshake, iceoryx2 refuses the zero-copy connection
(`ZeroCopyCreationError::VersionMismatch`), and the failure is silent
at the application level.

**Fix:** exact-pin (`=0.10.0`) every `iceoryx2*` dependency; after any
lockfile regeneration, verify one version per lockfile:

```bash
grep '^name = "iceoryx2' -A1 native/Cargo.lock workspace/Cargo.lock
```

All hits must show the same version. The workspace runner's
rebuild-everything-from-one-checkout rule (#13) is the runtime guard for
the binary↔cdylib pair; the lockfile check is the build-time one.

### 16. Deterministic stamps are the platform default: a gated node with `CER_BENCH_WALL_STAMP` unset collects zero samples

**Symptom:** a graph-based latency leg runs to completion, delivery
accounting looks healthy, and the `.bin` is missing/empty, or every
RTT computes to zero.

**Cause:** Cerulion nodes stamp the deterministic gating
clock (`now_ns()`) by default so `graph run --record` replays
byte-exact. A deterministic stamp is useless for wall latency (under
`VirtualClock`, publish- and receive-side stamps inside one scheduler
step are identical: RTT = 0). Graphs built as record and replay assets
therefore gate their entire wall-latency measurement path on
`CER_BENCH_WALL_STAMP=1`; with it unset, they collect zero samples by
design (those graphs double as byte-exact record/replay assets).

**Fix in this suite:** the `workspace/` nodes stamp `real_ns()`
**unconditionally**: there is no record/replay use of this workspace,
so there is nothing for the deterministic mode to protect
(METHODOLOGY §4). `run_workspace.sh` still exports
`CER_BENCH_WALL_STAMP=1` for env-contract parity, and the per-size
sample-count gate turns a missing/short `.bin` into a hard failure
either way. The pitfall stays live for anyone porting a robotics-style
gated node into a latency harness: export the variable, or your leg
measures nothing. Inverse rule: when *recording* a robotics bench graph
for replay, leave it unset; wall stamps break byte-exact replay.

### 17. Undeclared run shape / network posture

**Symptom:** a "monolith" latency number that was actually measured
across process boundaries; ambient network activity in a supposedly
hermetic run; unexplained extra processes in `ps` during a leg.

**Cause:** two product defaults changed after the prior campaign.
(1) An unpartitioned graph on Linux under the real clock is
auto-partitioned and runs **multi-process by default**. (2)
Every real-clock live run spawns a **network gateway** process (zenoh
session, LAN scouting, announce traffic) by default. A bench that
inherits either default is measuring a different system than its label
claims.

**Fix:** make the shape part of the leg's DEFINITION, never
happenstance, and let the row label BE the invocation. The `split`
leg runs a DECLARED 2-group `process_groups:` graph
(`cerulion graph run rtt_bench_split`, zero flags, zero env overrides),
so the multi-process shape is the measured shape ON PURPOSE and
documented as such; the `mono` leg runs the same 3-node chain with
exactly `--single-process`. (The flagless `default` leg, the
derived partition, is retired while the park-wake bug inflates it; the runner
refuses it loudly. See README § workspace lines.) Neither leg
suppresses the gateway: it parks at zero
demand outside the measured SHM chain, and suppressing it would measure
a shape no flagless user gets. The leg name in the `raw_prefix` carries
the declared shape into every artifact. When adding a new leg, state
its exact invocation in its definition.

### 18. Cross-day comparisons are not A/Bs (~12% day drift)

**Symptom:** a "regression" (or "win") appears with no plausible cause,
and evaporates on a re-run.

**Cause:** same-host absolute numbers drift day to day: ambient load,
thermals, kernel/firmware updates. Between same-host sweeps on different
days, the prior campaign observed drift on the order of ~12% with no
code change (prior-campaign observation, not a constant of nature;
budget for it, don't cite it).

**Fix:** A/B comparisons run interleaved in one window, one host, one
build session. Anything cross-day/cross-host is trend data and must
carry the machine hash + date of both sides. On a suspect host, relative
same-window A/Bs stay valid; absolutes need a fresh baseline.

### 19. The DMA lock can silently no-op, and then only your tails are wrong

**Symptom:** medians look normal; p99/max are inflated and noisy. The
run "succeeded".

**Cause:** if `/dev/cpu_dma_latency` is missing or not writable, the
CPU-idle-state lock soft-fails with a warning and the run continues,
and every wait-then-wake receive path then pays C-state exit latency in
its tail percentiles. The failure mode is precisely a wrong *tail* with
a healthy-looking median, which no eyeball check catches.

**Fix:** the native binaries REFUSE to run when the device exists but
is not writable (`acquire_dma_lock()` panics with the remedy; a
reboot resets the device to 0600).
Make it writable with `sudo chmod 666 /dev/cpu_dma_latency` until reboot,
or permanently via a udev rule:
`KERNEL=="cpu_dma_latency", MODE="0666"` in
`/etc/udev/rules.d/99-cerulion-bench-dma.rules`. Rootless hosts can set
`CER_BENCH_ALLOW_NO_DMA_LOCK=1` to run anyway, loudly, with
`cpu_dma_lock: SKIPPED` in the log and tails not citable. In
containers, the per-cell stdout smoke check aborts a cell where the
device is present but the lock failed (exit 12; METHODOLOGY §9's
exit-code table); a host without the device soft-warns and continues,
unlocked. The docker cells get `--device /dev/cpu_dma_latency`
when present. Never compare a locked run's tails against an unlocked
run's.

### 20. Plot/prefix coupling: a renamed line silently vanishes from plots

**Symptom:** a plot is missing a line; nothing errored anywhere.

**Cause:** the `raw_prefix` is the coupling key across the whole
pipeline: `.bin` filename stem → CSV key → plot-line key. Renaming a
prefix in one place (a runner, an env var, a plot spec) doesn't break
anything; the pipeline just stops finding that line's data and plots
what remains.

**Fix:** the line inventory in `README.md` is **pinned**; treat any
`raw_prefix` change as an interface change touching binaries, runners,
`bench.py`'s enumeration, and the plot specs together. After any rename,
diff the plot's line count against the inventory before trusting it.

### 21. The native zenoh pair rendezvous on a fixed loopback locator: one zenoh bench pair per host

**Symptom:** a native `zenoh_shm_*` cell fails at session open (the
pong can't bind its listener), hangs waiting for its peer; or, worse,
produces plausible-looking numbers while ping is connected to a *stale*
pong left over from a previous run.

**Cause:** the native zenoh pair uses a fixed rendezvous:
`zenoh_shm_round_trip_pong` **listens** on `tcp/127.0.0.1:7447` and the
ping side **connects** to that same locator
(`native/src/lib.rs::zenoh_shm_config`); multicast and gossip scouting
are deliberately disabled, so the fixed locator IS the discovery. The
port is baked in; two concurrent zenoh bench pairs on one host
collide: the second pong fails to bind (address in use), and a ping
happily talks to whichever pong owns the port, including an orphan from
a killed run that's answering with the wrong build's behavior.

**Fix:** one zenoh bench pair per host at a time; never run two
variants' (or two checkouts') zenoh cells concurrently on one machine.
Before a zenoh cell, make sure no stray pong is alive
(`pkill -x zenoh_shm_round_trip_pong`). `bench.py`'s native watchdog
does a best-effort pkill of the pong whenever it kills a timed-out
native bench, but a pong orphaned by a hand-driven run is yours to
clean.

---

### 22. `ament_target_dependencies` is REMOVED on Lyrical

**Symptom:** the Lyrical image build fails in colcon with
`Unknown CMake command "ament_target_dependencies"` while the identical
source builds clean on Humble/Jazzy.

**Cause:** ROS 2's modern-CMake migration deprecated
`ament_target_dependencies()` in Kilted and removed it in Lyrical (hit
live building `latency_bench:lyrical`).

**Fix:** link modern namespaced targets, which are portable back to
Humble: `rclcpp::rclcpp` +
`<msg_pkg>::<msg_pkg>__rosidl_typesupport_cpp`. `rcl`/`rmw`
symbols and headers arrive transitively through `rclcpp::rclcpp`'s
PUBLIC dependencies: no extra link line for rcl-lane binaries. Any
new package added to this suite must use `target_link_libraries` from
day one.

### 23. Docker-bridge veth churn + avahi can wedge a WiFi-only host's iwlwifi mid-sweep

**Symptom:** hours into an unattended sweep, the host loses ALL
network; the sweep stalls; `pkill -9` of bench processes is a no-op;
monitoring tools freeze when they touch `/proc`. The host stays wedged
until a hard reboot (~20 h in the prior campaign).

**Cause:** each per-cell `docker run --rm` on the default bridge
creates and destroys a veth pair; each veth-create makes the host
`avahi-daemon` join mDNS multicast groups on the new interface; ROS 2
multicast discovery inside the container adds more. On a host whose
only uplink is Intel WiFi, that sustained churn can deadlock the
iwlwifi firmware command queue
(`kernel: iwlwifi ...: Queue N is stuck`; no clean userspace recovery
short of `rmmod iwlwifi && modprobe iwlwifi`). Tasks blocked on network
sockets then enter D state, where SIGKILL is only delivered on syscall
return (never, for these), hence the unkillable cascade.

**Detection:**

```bash
journalctl -k --since "1 hour ago" | grep -i "iwlwifi.*stuck"
```

**Fix / mitigations:** run long docker sweeps on **wired ethernet**
(the class simply does not apply to a wired uplink); use
`--network none` for cells that need no transport and `--network host`
where the suite already does (both avoid the veth pair; this suite's
shm cells still use the bridge, so the class is live); for unattended
WiFi-only runs, wrap the sweep with a kernel-log watchdog that greps
for the stuck-queue signature between cells and aborts via a sentinel
file.

### 24. Humble CycloneDDS+SHM has a hard 16 MB ceiling (iceoryx v1 max chunk)

**Symptom:** on **Humble only**, the `cyclonedds × shm` cell runs
64 B → 4 MB cleanly, then the 16 MB size dies as a mystery `Killed`
(publisher abort), reproducibly, in both chrt modes, even with a roudi
mempool config that reserves 16 MB-class blocks.

**Cause:** the iceoryx **v1** build shipped in Humble's apt archive has
a max-chunk-size limit below 16 MB + headers; the mempool retrofit
(the prior campaign sized pools up to 16,842,752 B, ~357 MB reserved in
`/dev/shm`) does not lift it. Jazzy-era and newer images carry newer
iceoryx and cover the full sweep; the ceiling is iceoryx-v1-specific,
not a CycloneDDS or QoS problem.

**Fix:** none inside Humble's apt packages; expect and document an
8-of-10-sizes result for `humble × cyclonedds × shm` rather than
burning a retry budget on it. Distinguish it from #7's RT-throttle
`Killed` by distro + size signature (Humble, exactly the 16 MB cell,
both chrt modes).

---

### 25. An external "warmer" process does not warm a low-rate cell

**Symptom:** a low-rate cell's inflated p50 looks like core idle-state
cost, so you park a 1 kHz busy/tickler process on the bench core
expecting the tax to vanish. It doesn't move.

**Cause:** the low-rate tax is per-wake coldness of the measured
process's OWN software path, not the core's power state. In a
diagnostic ladder,
a 1 ms-cadence nanosleep tickler pinned to the bench core moved p50 by zero (measured under
whole-process core pinning, itself ~+3 µs; an unpinned replication has
not been run), while making the measured process tick at 1 kHz
in-process recovered ~5 to 6 µs, and even that is only ~40 to 45 % of the
tax; the rest is the message path's µarch footprint evicted during the
gap, which nothing external can hold warm.

**Fix:** don't add warmer processes; they change the cell's co-tenancy
without changing its verdict. Hold rate constant instead (`fixed100`,
§17 of METHODOLOGY.md); if a production graph needs warm low-rate
wakes, note that any ≥100 Hz node sharing the process already provides
them.

## Debugging a new failure: the checklist

1. **Read the cell's attempt log first.** The first error line is
   usually the actionable one.
2. **`Killed`** → kernel-side (OOM, RT throttling: #7; the Humble
   cyclonedds×shm 16 MB cell specifically: #24).
   **`Aborted (core dumped)`** → binary-side (config schema: #9).
   **`Waiting for at least 1 matching subscription(s)`** → discovery
   never matched (#1, #5). **`context is invalid`** → participant init
   failed or was torn down underneath the publisher.
3. **Compare across distros.** Newer-distro-only failures are usually an
   API/config rename (#1, #2, #3, #4, #5, #9).
4. **Compare across chrt modes.** chrt-1-only failures are #7.
5. **Zero samples with a healthy-looking run** on a graph-based leg →
   #16 (gated stamps); on any leg, check the `.bin` sample-count gate
   output.
6. **No node ever fires, no errors** → #15 (version skew), then #14
   (fd cap) if the build failed with `ServiceInCorruptedState`.
7. **Numbers exist but look wrong** → #13 (stale binary), #18
   (cross-day comparison), #19 (unlocked tails), #10 (partial
   coverage), #20 (missing line), #21 (a stale pong owning the fixed
   zenoh locator).
8. **Search upstream** (ROS 2 release notes, the RMW's issue tracker)
   for the error string: this bench does nothing exotic; most failures
   are documented ROS 2 user-facing breakage.
