#!/usr/bin/env python3
"""Drive ipc_probe (serve, importd, fetch) through the cuda_buffer_backend
socket protocol, the IPCMetadata segment, the 100 ms reuse grace window and
the stale-descriptor import. Every process runs under strace.

usage: ipc_lifecycle.py <probe_bin_dir> <out_dir>
Writes events.jsonl (every probe line, in order, tagged), checks.json
(expected vs observed per check), and *.strace.
"""
import json
import os
import subprocess
import sys
import time

BIN, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
PROBE = os.path.join(BIN, "ipc_probe")
TRACE = ["strace", "-f", "-qq", "-yy", "-x", "-s", "128", "-e", "signal=none",
         "-e", "trace=socket,bind,listen,accept,accept4,connect,sendmsg,recvmsg,"
               "openat,ftruncate,unlink"]
events = []
checks = []


def log(src, obj):
    obj = dict(obj)
    obj["_src"] = src
    events.append(obj)
    return obj


def spawn(name, args):
    return subprocess.Popen(TRACE + ["-o", os.path.join(OUT, name + ".strace"), PROBE] + args,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
                            stderr=open(os.path.join(OUT, name + ".stderr"), "w"))


def read(p, src):
    while True:
        line = p.stdout.readline()
        if not line:
            raise RuntimeError(f"{src} exited")
        line = line.strip()
        if line.startswith("{"):
            return log(src, json.loads(line))


def cmd(p, src, text):
    log("driver", {"cmd": f"{src}: {text}", "steady_us": time.monotonic_ns() // 1000})
    p.stdin.write(text + "\n")
    p.stdin.flush()
    return read(p, src)


def check(name, expected, observed, note=""):
    checks.append({"check": name, "expected": expected, "observed": observed,
                   "pass": expected == observed, "note": note})


def imp_args(d):
    return (f'{d["size"]} {d["vmm_socket_path"]} {d["vmm_pid"]} {d["vmm_block_id"]} '
            f'{d["vmm_block_size"]} {d["ipc_uid"]}')


serve = spawn("serve", ["serve"])
read(serve, "serve")
imp = spawn("importd", ["importd"])
read(imp, "importd")

# A. Socket protocol + IPCMetadata, by hand and through the plugin.
a = cmd(serve, "serve", "alloc 0 4096 90")
d0 = a["descriptor"]
f = subprocess.run(TRACE + ["-o", os.path.join(OUT, "fetch.strace"), PROBE, "fetch",
                            d0["vmm_socket_path"], str(d0["vmm_pid"]), str(d0["vmm_block_id"]),
                            str(d0["vmm_block_size"])], capture_output=True, text=True)
fetch = log("fetch", json.loads(f.stdout.strip().splitlines()[-1]))
check("socket name", f'@cuda_vmm_{d0["vmm_pid"]}_{d0["vmm_block_id"]}', fetch["socket"])
check("payload byte", "58", fetch["recv_0"]["data"], "one byte 'X' carries the cmsg")
check("fds per message", 1, fetch["recv_0"]["fds"])
check("server closes after one message", 0, fetch["recv_0"]["read_after"])
check("second connect served", 1, fetch["recv_1"]["fds"])
check("fd imports as VMM handle", 0, fetch["import_result"])
check("block head is the publisher's fill", "5a5a5a5a5a5a5a5a", fetch.get("block_head"))
check("IPCMetadata size", 24, fetch["sizeof_IPCMetadata"])
check("IPCMetadata offsets", [0, 8, 16],
      [fetch["offsetof_refcount"], fetch["offsetof_uid"], fetch["offsetof_publish_timestamp_us"]])
check("shm segment size", 24, fetch["meta"]["file_size"])
check("shm uid == descriptor ipc_uid", d0["ipc_uid"], fetch["meta"]["uid"])
i0 = cmd(imp, "importd", f"import 0 {imp_args(d0)}")
check("plugin import backend", "cuda", i0["backend"])
check("plugin import refcount while held", 1, i0["meta_holding"]["refcount"])
dr0 = cmd(imp, "importd", "drop 0")
check("refcount after drop", 0, dr0["meta_after"]["refcount"])
cmd(serve, "serve", "release 0")

# B. Inside the grace window: a released block is not handed out again, and
# a late import of its descriptor still reads the original bytes.
time.sleep(0.25)
b1 = cmd(serve, "serve", "alloc 1 4096 17")
d1 = b1["descriptor"]
cmd(serve, "serve", "release 1")
b2 = cmd(serve, "serve", "alloc 2 4096 34")
d2 = b2["descriptor"]
check("realloc inside grace gets a new block", True, d2["vmm_block_id"] != d1["vmm_block_id"],
      f'describe->realloc {b2["steady_us"] - b1["steady_us"]} us')
i1 = cmd(imp, "importd", f"import 1 {imp_args(d1)}")
check("late import inside grace", ["cuda", 4096, "1111111111111111"],
      [i1["backend"], i1["size"], i1.get("first_bytes")],
      f'describe->import {i1["steady_us"] - b1["steady_us"]} us')
cmd(imp, "importd", "drop 1")

# C. After the grace window: the block is reused. Before the new owner builds
# its descriptor the old uid still matches; after it, the old descriptor is
# stale and the plugin returns an empty buffer.
cmd(serve, "serve", "release 2")
time.sleep(0.25)
c3 = cmd(serve, "serve", "allocnd 3 4096 51")
check("reuse after grace picks the first released block", d1["vmm_block_id"], c3["block_id"])
check("uid in shm unchanged until the next descriptor", d1["ipc_uid"], c3["meta"]["uid"])
i10 = cmd(imp, "importd", f"import 10 {imp_args(d1)}")
check("old descriptor, reused block, before re-describe",
      ["cuda", 4096, "3333333333333333"], [i10["backend"], i10["size"], i10.get("first_bytes")],
      "imports and reads the NEW owner's bytes: the uid guard has not moved yet")
cmd(imp, "importd", "drop 10")
c3d = cmd(serve, "serve", "describe 3")
check("re-describe writes a new uid", True, c3d["descriptor"]["ipc_uid"] != d1["ipc_uid"])
i11 = cmd(imp, "importd", f"import 11 {imp_args(d1)}")
check("stale descriptor after re-describe", ["cuda", 0], [i11["backend"], i11["size"]],
      "empty CUDA buffer, no error to the caller")
cmd(imp, "importd", "drop 11")

# D. A held import pins the block past the grace window.
d4e = cmd(serve, "serve", "alloc 4 8388608 68")
d4 = d4e["descriptor"]
i20 = cmd(imp, "importd", f"import 20 {imp_args(d4)}")
cmd(serve, "serve", "release 4")
time.sleep(0.25)
d5 = cmd(serve, "serve", "alloc 5 8388608 85")["descriptor"]
check("held block not reused after grace", True, d5["vmm_block_id"] != d4["vmm_block_id"],
      f'refcount while held {i20["meta_holding"]["refcount"]}')
cmd(imp, "importd", "drop 20")
time.sleep(0.25)
cmd(serve, "serve", "release 5")
time.sleep(0.25)
d6 = cmd(serve, "serve", "alloc 6 8388608 102")["descriptor"]
check("block reused once the reader dropped it", d4["vmm_block_id"], d6["vmm_block_id"])

for p in (serve, imp):
    p.stdin.write("quit\n")
    p.stdin.flush()
    p.wait(timeout=30)

with open(os.path.join(OUT, "events.jsonl"), "w") as fh:
    for e in events:
        fh.write(json.dumps(e, sort_keys=True) + "\n")
with open(os.path.join(OUT, "checks.json"), "w") as fh:
    json.dump(checks, fh, indent=2)
bad = [c for c in checks if not c["pass"]]
print(json.dumps({"checks": len(checks), "failed": [c["check"] for c in bad]}))
sys.exit(1 if bad else 0)
