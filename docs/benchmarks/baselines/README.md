# Baselines

A baseline report captures the full environmental context of one machine at
one point in time, before any middleware is installed. It is optional extra
context for a bench host: the packages under `../results/` carry their host
identity in their own `run.json`, and none of the published packages depends on
a report here.

## How they're produced

```
sudo ./tools/scripts/benchmarks/capture_baseline.sh --cyclictest-minutes 10
```

See `tools/scripts/benchmarks/capture_baseline.sh`. That script writes
`baseline-<timestamp>/baseline-report.md`; once you have checked it, copy it
into this directory as `<machine_hash>-<YYYYMMDD>.md`, using the same
machine hash the packages under `../results/` are named with.

## Required contents

Every baseline report MUST include a `## Summary card` section with these
lines (exact labels, one per line):

```
- CPU model: <string>
- CPU threads: <int>
- Kernel: <kernel version string>
- Governor: <cpufreq governor, e.g. performance | powersave | schedutil>
- PREEMPT_RT: <yes | no>
```

This section is parsed by `tools/scripts/benchmarks/lib/machine_hash.sh` to
compute the `machine_hash` that downstream CSVs reference. Deviate from
these labels and the hash computation breaks.

## Policy

- Do NOT edit a committed baseline after the fact. Machines change; file
  a new baseline with a fresh date.
- Do NOT hand-tune the kernel or governor for a published result; the
  published packages are STOCK posture.
- Red flags (non-`performance` governor, non-invariant TSC, PREEMPT_RT
  kernel, cyclictest Max > 1000us, hwlatdetect SMI hits) must be called
  out in the PR that adds the baseline. Silent acceptance is not allowed.
