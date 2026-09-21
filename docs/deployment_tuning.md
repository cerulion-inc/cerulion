# Deployment Tuning: Taming the Host OS Tail for Latency-Critical Cerulion Graphs

An operator's guide to Linux host tuning for latency-critical Cerulion
deployments. Cerulion's floors, medians, and p99s are excellent on a
stock desktop kernel with no tuning, but the **millisecond-class MAX
tail** you see on an untuned host is dominated by ambient kernel
scheduling: the run queue holding your ready-to-run node off-CPU
while an unrelated task runs. This guide is how you reduce that tail.

If you are shipping a robot on a general-purpose Linux install and your
control loop's worst-case matters, read this. If you only care about
median throughput, you can skip it: the defaults are already fast.

Related: `docs/multi_process.md` (the deployment surface these knobs
tune), `docs/PERFORMANCE.md` (headline benchmarks), the "Environment
variables" section of `docs/user-api.md` (the in-framework knobs
below).

## TL;DR

- **Untuned, the floor, p50 and p99 are already excellent; the MAX is
  dominated by host scheduling.** On a stock desktop-kernel x86 host a real
  2-process split holds single-digit-microsecond medians (the README's
  performance table), while its worst sample is millisecond-class, which a
  scheduler trace attributes to the kernel run queue rather than to Cerulion.
- **The single highest-leverage knob is CPU isolation.** Give Cerulion a
  set of cores the general scheduler will not touch (`isolcpus=` /
  `nohz_full=` / `rcu_nocbs=`, or cgroup-v2 `AllowedCPUs=`) and pin the
  deployment there. Run-queue delay on those cores drops sharply.
- **Do NOT just `chrt` your processes.** Plain SCHED_FIFO on shared cores
  without RT-throttle tuning makes p50 *worse* and injects stalls tens of
  milliseconds long (measured). SCHED_FIFO only pays off *after* isolation, and only with
  the RT throttle handled.
- **The shallow-idle wake path is already Cerulion's job, not yours.**
  Monitor-wait parks (UMWAIT/WFE) with SHM doorbell wakes, a graph-derived
  C-state cap, and the barrier-boundary spin are all in the framework. You
  do not tune C-states by hand.

## What you get untuned

The published reference is the fixed100 package under
`docs/benchmarks/results/` (rendered in `docs/PERFORMANCE.md`): a real
`cerulion graph run` of a 2-process split graph on a stock desktop-kernel x86
host, nothing tuned, full sample delivery. Its multi-process row holds a p50
of 4.07 to 4.43 µs and a p99 of 6.61 to 10.45 µs from 64 B to 16 MiB (measured
with free run enabled, at a nominal 100 Hz).

The floor, p50, and p99 are the zero-copy transport doing its job. The
**max is orders of magnitude above the p99**, and on the traced runs that gap
is dominated by host scheduling. A `perf sched record -a` over such a run catches the kernel holding
a **RUNNABLE** Cerulion thread in the run queue for milliseconds while it
schedules other work. The same trace shows every other process on the host
eating the same latency class: kernel worker threads, a remote-desktop
server, the load-balancer's migration threads, `systemd-journald`.

**This is ambient-OS class.** Every process on the
host shows a millisecond-class max in the same trace, and other stacks
measured on the same hosts show the same millisecond-class maxes. That
tail is *kernel run-queue delay* (the time your ready thread waits
for a CPU), which userspace code inside Cerulion cannot shorten.
The rest of this guide addresses it at the source: stop the kernel
from putting anything else on Cerulion's cores.

### What Cerulion already handles (no operator action)

You do **not** tune the shallow-idle wake path. The framework owns it:

| Mechanism | What it does |
|---|---|
| Monitor-wait park (UMWAIT / WFE) + SHM doorbell wake | Shallow CPU park between messages; a peer's shared-memory doorbell wakes it in ns to µs. On by default for live runs (`CERULION_MONITOR_WAIT`). |
| Graph-derived C-state cap | Where no CPU park primitive exists, the runtime caps C-state exit latency from the graph's tightest timing so a core never enters a deep sleep it can't leave in time (`CERULION_CPU_DMA_LOCK`, Linux). |
| Barrier-boundary spin-then-block | Cross-process DAG-level boundaries spin for a bounded budget (default 20 µs on the kernel-wake tiers: Linux futex, macOS ≥ 14.4 os_sync; 150 µs on the macOS sleep-recheck fallback) before the blocking tier (futex on Linux; `os_sync_wait_on_address` on macOS ≥ 14.4, chunked sleep-recheck below), so lockstep peers don't pay a kernel round-trip per level (`CERULION_BARRIER_SPIN_US`; `CERULION_BARRIER_OS_SYNC=0` disables the macOS kernel-wake tier). |
| macOS degraded park | On hosts without a CPU monitor-wait primitive, a bounded recheck-nap park (never a busy-spin). On macOS ≥ 14.4 each ~100 µs nap is an `os_sync_wait_on_address` timed wait, roughly half `nanosleep`'s timer-coalescing overshoot; `CERULION_PARK_OS_SYNC=0` disables that tier and restores the plain sleep-recheck nap (macOS < 14.4 and every other OS keep it unconditionally). |

Deep C-state cold-wakes are handled for you. **Run-queue delay is not**, and
that is what the ladder below addresses.

## The tuning ladder

In order of leverage. Most deployments get the entire tail from steps
(a) to (d). Take (d) whenever you take (b), they pair; (e) to (g) are for
hard real-time targets.

### a. CPU isolation: the highest-leverage knob

Remove a set of cores from the general scheduler and hand them to
Cerulion. Boot argument (in `/etc/default/grub`, then `update-grub` /
`grub2-mkconfig`):

```
isolcpus=4-7
```

`isolcpus=` takes the listed cores out of the kernel's automatic load
balancer: nothing is scheduled there unless explicitly pinned. Then pin
the Cerulion deployment onto exactly that set (`taskset`, a systemd
`AllowedCPUs=`, or a cpuset-v2 cgroup; see the worked example). Modern
alternative to the boot arg: cgroup-v2 cpusets at runtime, no reboot;
but note `AllowedCPUs=` on the Cerulion unit only confines Cerulion *to*
those cores; it does not move other userspace *off* them. Full runtime
isolation needs a real cpuset partition (`cpuset.cpus.partition=root` on
the Cerulion cgroup) or constraining every other slice's `AllowedCPUs=`
to the housekeeping cores, and it still excludes kernel-internal
threads less thoroughly than the boot arg.

**Expected effect: run-queue delay on the isolated cores drops sharply**;
there is little else runnable to displace your node, so the
millisecond-class "held runnable" wait has little to hold it. On most hosts
this step alone removes most of the millisecond-class max.

### b. `nohz_full=`: stop the periodic tick

On the *same* isolated cores:

```
nohz_full=4-7
```

By default the kernel fires a scheduler-tick timer interrupt ~100 to 1000×/s
on every core to drive time-slicing and accounting. On a core running a
single Cerulion thread that tick is pure jitter, a periodic preemption
you don't need. `nohz_full=` disables it while one task is runnable, so
your node runs uninterrupted between messages.

### c. IRQ steering: keep device interrupts off robot cores

Hardware interrupts land wherever the kernel (or `irqbalance`) steers
them, and a NIC or disk IRQ firing on a Cerulion core is a direct
preemption. Disable the balancer and steer IRQs to the housekeeping
cores:

```bash
systemctl disable --now irqbalance
# Route every IRQ to cores 0-3 (mask 0x0f); repeat per /proc/irq/<N>.
for irq in /proc/irq/*/smp_affinity; do echo 0f > "$irq" 2>/dev/null; done
echo 0f > /proc/irq/default_smp_affinity   # new IRQs default to housekeeping
```

The `/proc/irq/*` and `/proc/sys/kernel/*` writes here and in step (e)
need root: run them via `sudo sh -c '...'` or from a root-owned boot
service. Some managed IRQs refuse re-steering: that's expected; steer
what you can. The goal is zero device interrupts on cores 4 to 7.

### d. `rcu_nocbs=`: offload RCU callbacks

Pairs with `nohz_full=`. On the same cores:

```
rcu_nocbs=4-7
```

RCU (read-copy-update) callback processing is otherwise driven from the
scheduler tick you just disabled; `rcu_nocbs=` offloads those callbacks to
dedicated kthreads on the housekeeping cores, so RCU work never wakes your
isolated cores. On current kernels `nohz_full=` already implies
`rcu_nocbs=` for the same CPUs; keep the explicit argument anyway:
belt-and-suspenders across kernel versions, and it documents intent in
the cmdline.

### e. SCHED_FIFO: done RIGHT

Real-time scheduling policy raises a thread above all normal tasks, but
**only helps once the cores are isolated, and only with the RT throttle
handled.** Applied blindly on shared cores it makes things *worse* (see
"What NOT to do"). The order is: isolate first (a), then:

1. **Confirm the deployment is pinned to isolated cores.** SCHED_FIFO on a
   shared core just lets your thread starve the kernel's own housekeeping.
2. **Handle the RT throttle.** The kernel caps aggregate RT time per core
   at `sched_rt_runtime_us` / `sched_rt_period_us`, default
   **950000 / 1000000**, i.e. 95% of every 1 s window. When an RT thread
   exceeds that budget the kernel *forces it off-CPU for the remaining
   ~50 ms*, which is exactly the stall produced by bare
   `chrt`. Two ways to deal with it:

   | Option | How | Trade-off |
   |---|---|---|
   | Disable the throttle | `echo -1 > /proc/sys/kernel/sched_rt_runtime_us` (a **global** sysctl: it affects every core, not just the isolated ones) | Removes the ~50 ms forced-idle window entirely. **Watchdog risk:** the throttle is the safety valve that stops a runaway RT thread from wedging a core / locking out the soft-lockup watchdog; disabling it globally is only safe when your RT threads are pinned to isolated cores where nothing else needs the CPU. |
   | Widen the budget | Raise `sched_rt_runtime_us` toward `sched_rt_period_us` (e.g. `999000 / 1000000`) | Keeps a small safety slack so a runaway task is still eventually throttled; the forced-idle window shrinks to ~1 ms instead of ~50 ms. The conservative default choice. |

3. **Then** set the policy, via the systemd unit (preferred, see the
   worked example) or `chrt -f 80 <pid>`. A mid-range priority (80) sits
   above normal tasks without competing with the kernel's own top-priority
   RT threads.

### f. PREEMPT_RT kernel: for hard deadlines

Everything above shortens the tail; a `PREEMPT_RT` kernel reduces the
non-preemptible kernel work that produces it. RT
makes almost all kernel code preemptible (threaded IRQs, sleeping
spinlocks, priority inheritance), so the intervals during which the kernel
holds a ready thread get much shorter. If your robot has a hard
deadline, not just a good average, build on a PREEMPT_RT kernel,
apply (a) to (e) on top, and validate the worst case on your own hardware
and workload before relying on a deadline. For soft-real-time control loops the stock kernel
plus isolation is usually enough.

### g. Supporting knobs

| Knob | How | Why |
|---|---|---|
| Performance governor | `cpupower frequency-set -g performance` (or write `performance` to each `/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`) | Stops the CPU downclocking during the idle gaps between messages: a downclocked core pays a frequency-ramp penalty on the next wake, adding µs-class jitter. |
| Locked-memory limit | `LimitMEMLOCK=infinity` in the unit, or `ulimit -l` / `/etc/security/limits.conf` | SHM-heavy graphs (large or many topics) mlock their shared-memory pools so they never page. Set the limit to cover the pools your graph configures; a graph whose pools run to gigabytes fails at startup against the default 64 KB ceiling. |
| SMT-sibling isolation | Isolate *both* hyperthreads of each physical core (e.g. if 4 and its sibling 12 share a core, isolate both) | A noisy task on the sibling logical CPU contends for the shared physical core's execution units and L1/L2; partial isolation leaves a jitter path open. Check pairings via `lscpu -e` / `/sys/devices/system/cpu/cpu*/topology/thread_siblings_list`. |

## Worked example: 8-core robot host, 4 cores to Cerulion

A generic recipe: an 8-core host, cores **0 to 3** for the OS and everything
else, cores **4 to 7** isolated for one Cerulion deployment. Replace core
numbers, the binary path, and the graph name with your own.

**1. Boot arguments**: `/etc/default/grub`, append to
`GRUB_CMDLINE_LINUX_DEFAULT`, then `update-grub` (Debian/Ubuntu) or
`grub2-mkconfig -o /boot/grub2/grub.cfg` (Fedora/RHEL) and reboot:

```
isolcpus=4-7 nohz_full=4-7 rcu_nocbs=4-7
```

If SMT is enabled, isolate each isolated core's sibling too (step g):
check pairings with `lscpu -e`; e.g. if cores 4 to 7's siblings are 12 to 15,
the lists become `4-7,12-15`.

**2. IRQ steering**: a boot-time service or your provisioning script:

```bash
systemctl disable --now irqbalance
for irq in /proc/irq/*/smp_affinity; do echo 0f > "$irq" 2>/dev/null; done
echo 0f > /proc/irq/default_smp_affinity
echo -1 > /proc/sys/kernel/sched_rt_runtime_us   # global sysctl; safe only because the RT threads are pinned to isolated cores
```

**3. systemd unit**: `/etc/systemd/system/cerulion-perception.service`:

```ini
[Unit]
Description=Cerulion perception deployment
After=network.target

[Service]
ExecStart=/usr/local/bin/cerulion graph run perception_stack

# Pin the whole deployment to the isolated cores (cgroup-v2 cpuset).
AllowedCPUs=4-7

# SCHED_FIFO: safe here because these cores are isolated AND the RT
# throttle is disabled (step 2). See "SCHED_FIFO: done RIGHT".
CPUSchedulingPolicy=fifo
CPUSchedulingPriority=80

# SHM-heavy graphs: lift the locked-memory ceiling so pools never page.
LimitMEMLOCK=infinity

Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
systemctl daemon-reload
systemctl enable --now cerulion-perception
```

Set the performance governor on the isolated cores at boot (a oneshot
unit, or your provisioning tool). That is the whole stack: isolated,
tick-free, IRQ-quiet, RT-scheduled cores running one Cerulion deployment.

## What NOT to do

- **Do not just `chrt` your processes on shared cores.** Plain
  SCHED_FIFO without isolation and without RT-throttle tuning is a measured
  trap:

  | Config | Result vs untuned |
  |---|---|
  | `chrt -f 80`, no RT-throttle tuning | p50 slightly **worse**, plus worst-case stalls tens of milliseconds long (the RT-throttle signature, `sched_rt_runtime_us` 950000/1000000) in most tested configs |
  | Affinity alone (pin, no isolation) | a **worse** split p99 than untuned |

  SCHED_FIFO is the *last* rung of the ladder for a reason: it only helps
  after the cores are isolated (a) and the throttle is handled (e).
  Applied first, it hurts.

- **Do not expect userspace to fix run-queue delay.** No environment
  variable, thread-priority tweak, or code change *inside* Cerulion can
  shorten the time the kernel holds a runnable thread off-CPU. That is a
  scheduling-policy problem, and it is solved by keeping other work off
  Cerulion's cores (a) to (d), not by tuning the framework. The in-framework
  knobs (`CERULION_MONITOR_WAIT`, `CERULION_CPU_DMA_LOCK`,
  `CERULION_BARRIER_SPIN_US`) address the *wake* path, which is already
  handled by default; they are not a substitute for host isolation.

## Verifying your tuning

Two measurements, before and after.

**1. Re-measure end-to-end latency.** Run your graph the same way both
times and compare the max. Instrument your sink to stamp `CLOCK_MONOTONIC`
at publish and diff at receive (the pattern the public latency suite's
workspace nodes use: see `benches/latency/workspace/nodes/ping_node`), and emit
floor / p50 / p99 / max:

```
floor=<min>  p50=<>  p99=<>  max=<>
```

Tuning should barely move the floor, p50 and p99, and should bring the
**max** down from millisecond-class toward the p99.

**2. Confirm the scheduler delay collapsed**: the direct evidence, on
the cores you isolated. During a live run:

```bash
perf sched record -a -- sleep 60
perf sched latency --sort max
```

Look at the **Maximum delay** column for your Cerulion threads. Untuned it
sits in the milliseconds; after isolation it should
drop to the µs class on the isolated cores, where the run queue has nothing
else to schedule there. If the max delay is still ms-class, something is
still landing on your cores: re-check the `isolcpus`/`nohz_full` cpu list
matches where you pinned, and that IRQs and the balancer are steered away.

---

*The published figures this doc leans on are in `docs/benchmarks/results/`.
The tuning effects described here are qualitative; reproduce them with your
own graph and host per "Verifying your tuning" above.*
