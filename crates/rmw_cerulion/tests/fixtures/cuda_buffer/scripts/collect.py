#!/usr/bin/env python3
"""Copy the reviewable capture outputs into golden/, normalizing what varies
between runs (pids, socket inodes, the container hostname, timestamps in
log lines).

usage: collect.py <out_dir> <golden_dir>
"""
import glob
import csv
import json
import os
import re
import shutil
import sys

OUT, GOLD = sys.argv[1], sys.argv[2]
WARMUP = 100  # latency.sh S2_WARMUP default


def cp(src, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copyfile(src, dst)


def strace_transcript(path, pid_names):
    """Keep the AF_UNIX socket calls and the /dev/shm opens; rename pids."""
    keep = []
    for line in open(path, errors="replace"):
        m = re.match(r"^(\d+)\s+(\w+)\((.*)$", line.rstrip("\n"))
        if not m:
            continue
        pid, call, rest = m.groups()
        unix = "UNIX" in rest or "AF_UNIX" in rest or "@cuda_vmm" in rest
        shm = "/dev/shm/" in rest
        if call in ("socket", "bind", "listen", "accept", "accept4", "connect", "sendmsg",
                    "recvmsg") and unix:
            pass
        elif call in ("openat", "ftruncate", "unlink") and shm and "cuda_vmm" in rest:
            # ipc_probe reads the segment O_RDONLY to report it; the backend
            # opens it O_RDWR. Keep only the backend's opens.
            if call == "openat" and "O_RDONLY" in rest:
                continue
        else:
            continue
        rest = re.sub(r"(socket|UNIX|UNIX-STREAM):\[[^\]]*\]", r"\1:[INODE]", rest)
        keep.append(f"{call}({rest}")
    text = "\n".join(keep) + "\n"
    for pid, name in pid_names.items():
        text = re.sub(rf"(?<!\d){pid}(?!\d)", name, text)
    return text


def main():
    # Both latency runs are inputs; refuse before golden/ is cleared.
    for src_name in ("latency", "latency_ready"):
        if not os.path.isdir(os.path.join(OUT, src_name)):
            sys.exit(f"collect: {os.path.join(OUT, src_name)} is missing; run the latency stage")
    if os.path.isdir(GOLD):
        shutil.rmtree(GOLD)
    os.makedirs(GOLD)

    # Descriptor CDR.
    for f in glob.glob(os.path.join(OUT, "descriptor", "*.cdr")):
        cp(f, os.path.join(GOLD, "descriptor", os.path.basename(f)))
    for f in ("probe.json", "decoded.json"):
        cp(os.path.join(OUT, "descriptor", f), os.path.join(GOLD, "descriptor", f))

    # IPC protocol, metadata, grace window, stale import.
    ipc = os.path.join(OUT, "ipc")
    events = [json.loads(l) for l in open(os.path.join(ipc, "events.jsonl"))]
    pids = {}
    for e in events:
        if e.get("event") == "ready":
            pids[str(e["pid"])] = "<" + e["_src"].upper() + "_PID>"
    cp(os.path.join(ipc, "checks.json"), os.path.join(GOLD, "ipc", "checks.json"))
    with open(os.path.join(GOLD, "ipc", "events.jsonl"), "w") as fh:
        def scrub(v):
            if isinstance(v, dict):
                return {k: scrub(x) for k, x in v.items()}
            if isinstance(v, list):
                return [scrub(x) for x in v]
            if isinstance(v, int) and not isinstance(v, bool) and str(v) in pids:
                return pids[str(v)]
            if isinstance(v, str):
                for pid, name in pids.items():
                    v = re.sub(rf"(?<!\d){pid}(?!\d)", name, v)
            return v
        for e in events:
            fh.write(json.dumps(scrub(e), sort_keys=True) + "\n")
    for name in ("serve", "importd", "fetch"):
        p = os.path.join(ipc, name + ".strace")
        if os.path.exists(p):
            with open(os.path.join(GOLD, "ipc", name + ".unix_shm.strace.txt"), "w") as fh:
                fh.write(strace_transcript(p, pids))

    for name in ("serve", "importd"):
        p = os.path.join(ipc, name + ".stderr")
        if os.path.exists(p):
            txt = open(p, errors="replace").read()
            txt = re.sub(r"\[\d{10}\.\d+\]", "[T]", txt)
            txt = re.sub(r"on host \w+", "on host <HOST>", txt)
            txt = re.sub(r"uid \d+", "uid <UID>", txt)
            for pid, nm in pids.items():
                txt = re.sub(rf"(?<!\d){pid}(?!\d)", nm, txt)
            with open(os.path.join(GOLD, "ipc", name + ".stderr.txt"), "w") as fh:
                fh.write(txt)

    # Upstream samples.
    cp(os.path.join(OUT, "samples", "summary.tsv"), os.path.join(GOLD, "samples", "summary.tsv"))

    # CPU fallback and wire captures.
    fb = os.path.join(OUT, "fallback")
    cp(os.path.join(fb, "summary.tsv"), os.path.join(GOLD, "fallback", "summary.tsv"))
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from decode_cdr import decode_wire_hit
    for w in glob.glob(os.path.join(fb, "*", "*", "wire.json")):
        rmw, case = w.split(os.sep)[-3:-1]
        wire = json.load(open(w))
        wire["first_hits_decoded"] = [decode_wire_hit(h) for h in wire["first_hits"]]
        os.makedirs(os.path.join(GOLD, "fallback", rmw, case), exist_ok=True)
        with open(os.path.join(GOLD, "fallback", rmw, case, "wire.json"), "w") as fh:
            json.dump(wire, fh, indent=2)
        for log in ("pub.log", "sub.log"):
            src = os.path.join(os.path.dirname(w), log)
            if os.path.exists(src):
                txt = open(src, errors="replace").read()
                txt = re.sub(r"\[\d{10}\.\d+\]", "[T]", txt)
                txt = re.sub(r"on host \w+", "on host <HOST>", txt)
                with open(os.path.join(GOLD, "fallback", rmw, case, log), "w") as fh:
                    fh.write(txt)

    # Latency baseline: latency/ is stamped after the fill is enqueued,
    # latency_ready/ after the producer's stream is synchronized.
    for src_name, dst_name in (("latency", "enqueued"), ("latency_ready", "ready")):
        lat = os.path.join(OUT, src_name)
        dst = os.path.join(GOLD, "latency", dst_name)
        for f in ("summary.tsv", "summary.md", "params.txt", "cells.log"):
            if os.path.exists(os.path.join(lat, f)):
                cp(os.path.join(lat, f), os.path.join(dst, f))
        # One line per cell: rmw, mode, size, then every post-warmup sample's
        # latency in whole microseconds, in arrival order. Plain text: the
        # leak guard refuses containers it cannot parse.
        with open(os.path.join(dst, "samples.tsv"), "w") as fh:
            fh.write("# rmw\tmode\tsize\tlat_us (comma separated, post-warmup, arrival order)\n")
            for f in sorted(glob.glob(os.path.join(lat, "*__*__*.csv"))):
                rmw, mode, size = os.path.basename(f)[:-4].split("__")
                recs = [r for r in csv.DictReader(open(f)) if int(r["seq"]) >= WARMUP]
                fh.write(f"{rmw}\t{mode}\t{size}\t"
                         + ",".join(str(int(r["lat_ns"]) // 1000) for r in recs) + "\n")

    # Environment.
    env = os.path.join(OUT, "env")
    if os.path.isdir(env):
        for f in os.listdir(env):
            cp(os.path.join(env, f), os.path.join(GOLD, "env", f))
    print(json.dumps(sorted(os.path.relpath(os.path.join(dp, f), GOLD)
                            for dp, _, fs in os.walk(GOLD) for f in fs), indent=1))


if __name__ == "__main__":
    main()
