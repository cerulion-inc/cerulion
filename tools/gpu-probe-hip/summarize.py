#!/usr/bin/env python3
"""Summarize a HIP S0 results dir (raw.log + env.txt) into Markdown on stdout.

    summarize.py <results dir>            print SUMMARY.md
    summarize.py --check <results dir>    smoke gate: exit 1 on any mismatch, an
                                          unparsable row, or no cpu/copy latency row

Table shapes follow tools/gpu-probe (the CUDA S0 probe): p50 / p99 us per size per
mode, the median across reps of each rep's p50 and p99.
"""
import collections
import json
import pathlib
import statistics as st
import sys

GPU_GPU = ["floor-device", "copy", "unified-cpusync", "unified-event", "device-ipcmem-cpusync",
           "device-ipcmem-event", "device-vmm-cpusync", "device-vmm-event", "device-dmabuf-event"]
GPU_CPU = ["copy-cpuread", "unified-cpuread"]
CPU = ["cpu"]
FLOOR = ["floor-device", "floor-registered-shm", "floor-hipHostMalloc"]


def load(out):
    """(rows, walls, unparsed) from raw.log. Every RESULT row carries its rep (run.sh adds it)."""
    rows, walls, bad = [], [], []
    for line in (pathlib.Path(out) / "raw.log").read_text().splitlines():
        for prefix, sink in (("RESULT ", rows), ("WALL ", walls)):
            if line.startswith(prefix):
                try:
                    sink.append(json.loads(line[len(prefix):]))
                except json.JSONDecodeError:
                    bad.append(line[:200])
    for r in rows:
        r.setdefault("rep", 0)
    return rows, walls, bad


def kib(n):
    return f"{n // 1024} KiB" if n < 1 << 20 else f"{n >> 20} MiB"


def num(v):
    return f"{v:.1f}" if v < 10 else f"{v:.0f}"


def latency_index(rows):
    lat = collections.defaultdict(list)
    for r in rows:
        if r.get("kind") == "latency":
            lat[(r["mode"], r["size"])].append(r)
    return lat


def cell(lat, mode, size):
    v = lat.get((mode, size))
    if not v:
        return "-"
    return f"{num(st.median(x['p50_us'] for x in v))} / {num(st.median(x['p99_us'] for x in v))}"


def p50(lat, mode, size):
    v = lat.get((mode, size))
    return st.median(x["p50_us"] for x in v) if v else None


def table(lines, lat, sizes, title, modes):
    lines.append(f"## {title} (p50 / p99 us, median over reps)\n")
    lines.append("| size | " + " | ".join(modes) + " |")
    lines.append("|---" * (len(modes) + 1) + "|")
    for s in sizes:
        lines.append(f"| {kib(s)} | " + " | ".join(cell(lat, m, s) for m in modes) + " |")
    lines.append("")


def ratio_table(lines, lat, sizes, base, modes, title):
    present = [m for m in modes if any((m, s) in lat for s in sizes)]
    if not present or not any((base, s) in lat for s in sizes):
        return
    lines.append(f"## {title} (p50 of {base} / p50 of mode; above 1 means faster than {base})\n")
    lines.append("| size | " + " | ".join(present) + " |")
    lines.append("|---" * (len(present) + 1) + "|")
    for s in sizes:
        b = p50(lat, base, s)
        cells = []
        for m in present:
            v = p50(lat, m, s)
            cells.append(f"{b / v:.2f}x" if b and v else "-")
        lines.append(f"| {kib(s)} | " + " | ".join(cells) + " |")
    lines.append("")


def mismatch_totals(rows):
    mm, frames = collections.Counter(), collections.Counter()
    for r in rows:
        if r.get("kind") == "latency":
            mm[r["mode"]] += r.get("gpu_mismatch", 0) + r.get("cpu_mismatch", 0)
            frames[r["mode"]] += r.get("frames", r.get("iters", 0) + r.get("warmup", 0))
    return mm, frames


def med(v, k, fmt="{:.1f}"):
    xs = [x[k] for x in v if isinstance(x.get(k), (int, float))]
    return fmt.format(st.median(xs)) if xs else ""


def first(v, k):
    for x in v:
        if k in x and x[k] not in ("", None):
            return str(x[k])
    return ""


def summary(out):
    out = pathlib.Path(out)
    rows, walls, bad = load(out)
    lines = ["# S0 HIP GPU IPC probe results\n", "```"]
    env = out / "env.txt"
    lines.append(env.read_text().strip() if env.exists() else "(no env.txt)")
    lines += ["```", ""]
    for r in rows:
        if r.get("kind") == "probe":
            keys = [k for k in r if k not in ("kind", "align", "rep")]
            lines.append("Device probe: " + ", ".join(f"{k}={r[k]}" for k in keys) + "\n")
            break

    lat = latency_index(rows)
    sizes = sorted({k[1] for k in lat})
    if sizes:
        table(lines, lat, sizes, "GPU producer -> GPU consumer", GPU_GPU)
        table(lines, lat, sizes, "GPU producer -> CPU consumer", GPU_CPU)
        table(lines, lat, sizes, "CPU producer -> CPU consumer (today's memcpy path)", CPU)
        table(lines, lat, sizes, "In-process GPU floor by memory kind", FLOOR)
        ratio_table(lines, lat, sizes, "copy", [m for m in GPU_GPU if m not in ("copy",)], "Speedup over the copy path, GPU consumer")
        ratio_table(lines, lat, sizes, "copy-cpuread", ["unified-cpuread", "cpu"], "Speedup over copy-cpuread, CPU consumer")

        lines.append("## GPU clock at mid-run (MHz from amdgpu pp_dpm_sclk, per rep)\n")
        lines.append("| mode | " + " | ".join(kib(s) for s in sizes) + " |")
        lines.append("|---" * (len(sizes) + 1) + "|")
        for m in sorted({k[0] for k in lat}):
            cells = ["/".join(str(x.get("gpu_mhz_mid", "?")) for x in sorted(lat.get((m, s), []), key=lambda x: x["rep"])) or "-" for s in sizes]
            lines.append(f"| {m} | " + " | ".join(cells) + " |")
        lines.append("")
        smi = [(r["mode"], r["size"], r["rep"], r["smi_sclk_post"]) for r in rows if r.get("kind") == "latency" and r.get("smi_sclk_post")]
        if smi:
            distinct = collections.Counter(x[3] for x in smi)
            lines.append("rocm-smi sclk after each row (distinct readings, count): " + "; ".join(f"`{k}` x{v}" for k, v in distinct.most_common(8)) + "\n")

    mm, frames = mismatch_totals(rows)
    lines.append("## Mismatches (GPU + CPU word mismatches, all reps; must be 0)\n")
    lines.append("| mode | frames | mismatches |")
    lines.append("|---|---|---|")
    for k in sorted(frames):
        lines.append(f"| {k} | {frames[k]} | {mm[k]} |")
    lines.append(f"\nTotal mismatches: {sum(mm.values())} over {sum(frames.values())} frames\n")

    ot = collections.defaultdict(list)
    for r in rows:
        if r.get("kind") == "onetime":
            ot[(r["mode"], r.get("side", "?"), r.get("size", 0))].append(r)

    uni = {k: v for k, v in ot.items() if k[0].startswith("unified")}
    if uni:
        lines.append("## One-time costs: UNIFIED registration (median over reps)\n")
        lines.append("| mode | side | size | segment MiB | register us | us per MiB | flags | path | dptr == host |")
        lines.append("|---|---|---|---|---|---|---|---|---|")
        for (m, side, s), v in sorted(uni.items()):
            if not any("register_us" in x for x in v):
                continue
            seg = med(v, "segment", "{:.0f}")
            seg = f"{float(seg) / (1 << 20):.2f}" if seg else ""
            lines.append(f"| {m} | {side} | {kib(s)} | {seg} | {med(v, 'register_us')} | {med(v, 'register_us_per_mib')} | {first(v, 'flags')} | {first(v, 'reg_path')} | {first(v, 'dptr_eq_host')} |")
        tried = sorted({x["reg_tried"] for v in uni.values() for x in v if x.get("reg_tried")})
        if tried:
            lines.append("\nRegistration paths that failed before the one used:\n")
            lines += [f"- `{t}`" for t in tried]
        lines.append("")

    dev = {k: v for k, v in ot.items() if k[0].startswith("device")}
    if dev:
        keys = ["block", "create_map_us", "export_us", "malloc_us", "ipc_get_mem_handle_us", "dmabuf_export_us",
                "handle_rtt_us", "handle_one_way_us", "import_us", "map_access_us", "ipc_open_mem_handle_us"]
        lines.append("## One-time costs: DEVICE handles (us, median over reps)\n")
        lines.append("| mode | side | size | " + " | ".join(keys) + " | import path |")
        lines.append("|---" * (len(keys) + 4) + "|")
        for (m, side, s), v in sorted(dev.items()):
            v = [x for x in v if "ipc_event_open_us" not in x]
            if not v:
                continue
            path = first(v, "dmabuf_import") or first(v, "import_arg")
            lines.append(f"| {m} | {side} | {kib(s)} | " + " | ".join(med(v, k, "{:.0f}" if k == "block" else "{:.1f}") for k in keys) + f" | {path} |")
        lines.append("")

    ev = [r for r in rows if r.get("kind") == "onetime" and "ipc_event_open_us" in r]
    if ev:
        by = collections.defaultdict(list)
        for r in ev:
            by[r["mode"]].append(r["ipc_event_open_us"])
        lines.append("IPC event open (hipIpcOpenEventHandle, us, median): " + ", ".join(f"{m} {st.median(v):.1f}" for m, v in sorted(by.items())) + "\n")

    bw = collections.defaultdict(list)
    for r in rows:
        if r.get("kind") == "onetime" and "cpu_read_gbps_shm_registered" in r and r.get("size", 0) >= 1 << 20:
            bw[r["size"]].append(r)
    if bw:
        lines.append("## CPU read speed on pages (reader, GB/s, median over reps)\n")
        lines.append("| size | shm unregistered | shm registered | malloc | hipHostMalloc |")
        lines.append("|---|---|---|---|---|")
        for s in sorted(bw):
            v = bw[s]
            lines.append(f"| {kib(s)} | {med(v, 'cpu_read_gbps_shm_unregistered', '{:.2f}')} | {med(v, 'cpu_read_gbps_shm_registered', '{:.2f}')} | {med(v, 'cpu_read_gbps_malloc', '{:.2f}')} | {med(v, 'cpu_read_gbps_hipHostMalloc', '{:.2f}')} |")
        lines.append("")

    sk = [r for r in rows if r.get("kind") in ("skip", "failed")]
    if sk:
        lines.append("## Skips and failures\n")
        seen = collections.OrderedDict()
        for r in sk:
            t = (r["kind"], r.get("mode", "?"), r.get("side", ""), r.get("reason", ""))
            seen.setdefault(t, []).append(r.get("size"))
        for (kind, m, side, reason), szs in seen.items():
            szs = sorted({s for s in szs if s})
            where = (" sizes " + ",".join(kib(s) for s in szs)) if szs else ""
            lines.append(f"- **{kind}** {m}{(' (' + side + ')') if side else ''}{where}: {reason}")
        lines.append("")

    if walls:
        by = collections.defaultdict(list)
        for w in walls:
            by[w["unit"]].append(w["secs"])
        lines.append("## Wall time per unit (s)\n")
        lines.append("| unit | runs | median | total |")
        lines.append("|---|---|---|---|")
        for u, v in by.items():
            lines.append(f"| {u} | {len(v)} | {st.median(v):.1f} | {sum(v):.1f} |")
        lines.append(f"\nCampaign wall time over all units: {sum(w['secs'] for w in walls) / 60:.1f} min\n")

    if bad:
        lines.append("## Unparsable lines\n")
        lines += [f"- `{b}`" for b in bad]
        lines.append("")
    return "\n".join(lines) + "\n"


def check(out):
    """Smoke gate. Returns (ok, messages)."""
    rows, _, bad = load(out)
    msgs, ok = [], True
    mm, frames = mismatch_totals(rows)
    for m in sorted(frames):
        msgs.append(f"{m}: {frames[m]} frames, {mm[m]} mismatches")
    if sum(mm.values()):
        ok = False
        msgs.append("FAIL: mismatches are not 0")
    for need in ("cpu", "copy"):
        if not frames.get(need):
            ok = False
            msgs.append(f"FAIL: no latency row for {need}")
    if bad:
        ok = False
        msgs.append(f"FAIL: {len(bad)} unparsable RESULT/WALL lines")
    for r in rows:
        if r.get("kind") in ("skip", "failed"):
            msgs.append(f"{r['kind'].upper()}: {r.get('mode', '?')} {r.get('side', '')} {r.get('reason', '')}"[:400])
    msgs.append("SMOKE PASS" if ok else "SMOKE FAIL")
    return ok, msgs


def main(argv):
    if len(argv) == 3 and argv[1] == "--check":
        ok, msgs = check(argv[2])
        print("\n".join(msgs))
        return 0 if ok else 1
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    sys.stdout.write(summary(argv[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
