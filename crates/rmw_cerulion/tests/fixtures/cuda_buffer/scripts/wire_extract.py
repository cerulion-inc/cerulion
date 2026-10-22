#!/usr/bin/env python3
"""Find descriptor-path Buffer payloads in an strace -xx capture of a
publisher's socket sends.

usage: wire_extract.py <strace_file> <out_json>
A hit is the 8-byte marker ff ff ff ff 32 53 4f 52 (uint32 0xFFFFFFFF, then
uint32 0x524F5332 "ROS2", little-endian). For each hit the 160 bytes from the
marker on are kept. The legacy form of the same Image is counted too: uint32
length 4096, then 16 equal bytes (s2_lat fills each payload with one value),
so a CPU-fallback run shows zero marker hits and nonzero legacy hits.
"""
import json
import re
import sys

src, out = sys.argv[1], sys.argv[2]
MARK = "ffffffff32534f52"
hits, legacy, sends = [], 0, 0
for line in open(src, errors="replace"):
    if not re.search(r"\b(sendto|sendmsg|sendmmsg|write|writev)\(", line):
        continue
    sends += 1
    blob = "".join(re.findall(r"\\x([0-9a-f]{2})", line))
    i = blob.find(MARK)
    while i != -1:
        if i % 2 == 0:
            hits.append(blob[i:i + 320])
        i = blob.find(MARK, i + 2)
    legacy += len(re.findall(r"00100000(?:([0-9a-f]{2})\1{15})", blob))
json.dump({"sends": sends, "marker_hits": len(hits), "first_hits": hits[:3],
           "legacy_4096_hits": legacy}, open(out, "w"), indent=2)
print(json.dumps({"sends": sends, "marker_hits": len(hits), "legacy_4096_hits": legacy}))
