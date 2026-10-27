#!/usr/bin/env python3
"""Independent decoder for the captured CDR fixtures.

It knows only the .msg layouts (sensor_msgs/Image, CudaBufferDescriptor at
the pinned commit) and plain little-endian CDR, and none of the C++ code
that produced the bytes. It decodes every .cdr file in a descriptor capture
directory, writes a field table with byte offsets, and cross-checks the
values against the probe's own JSON report.

usage: decode_cdr.py <descriptor_capture_dir> <out_json>
exit 1 on any mismatch.
"""
import json
import os
import struct
import sys

MARKER = (0xFFFFFFFF, 0x524F5332)


class Reader:
    def __init__(self, data):
        if data[:4] != b"\x00\x01\x00\x00":
            raise ValueError(f"encapsulation {data[:4].hex()} is not CDR_LE")
        self.d = data
        self.base = 4  # CDR alignment counts from the end of the encapsulation
        self.p = 4
        self.fields = []

    def align(self, n):
        rel = self.p - self.base
        self.p += (-rel) % n

    def prim(self, name, fmt, size):
        self.align(size)
        off = self.p
        (v,) = struct.unpack_from("<" + fmt, self.d, self.p)
        self.p += size
        self.fields.append({"field": name, "offset": off, "len": size, "value": v})
        return v

    def string(self, name):
        self.align(4)
        off = self.p
        (n,) = struct.unpack_from("<I", self.d, self.p)
        raw = self.d[self.p + 4:self.p + 4 + n]
        if n == 0 or raw[-1:] != b"\x00":
            raise ValueError(f"{name}: string not NUL terminated")
        self.p += 4 + n
        v = raw[:-1].decode()
        self.fields.append({"field": name, "offset": off, "len": 4 + n, "value": v})
        return v

    def octets(self, name, n):
        off = self.p
        v = self.d[self.p:self.p + n]
        self.p += n
        self.fields.append({"field": name, "offset": off, "len": n, "value": v.hex()})
        return v

    def peek_u32x2(self):
        self.align(4)
        return struct.unpack_from("<II", self.d, self.p) if self.p + 8 <= len(self.d) else (0, 0)


def descriptor(r, prefix=""):
    out = {}
    out["size"] = r.prim(prefix + "size", "Q", 8)
    out["element_type_name"] = r.string(prefix + "element_type_name")
    out["device_id"] = r.prim(prefix + "device_id", "i", 4)
    out["use_ipc"] = bool(r.prim(prefix + "use_ipc", "B", 1))
    out["vmm_pid"] = r.prim(prefix + "vmm_pid", "i", 4)
    out["vmm_block_id"] = r.prim(prefix + "vmm_block_id", "I", 4)
    out["vmm_block_size"] = r.prim(prefix + "vmm_block_size", "Q", 8)
    out["vmm_socket_path"] = r.string(prefix + "vmm_socket_path")
    out["ipc_uid"] = r.prim(prefix + "ipc_uid", "Q", 8)
    out["ipc_event_handle"] = r.octets(prefix + "ipc_event_handle", 64).hex()
    return out


def image(r):
    out = {"sec": r.prim("header.stamp.sec", "i", 4),
           "nanosec": r.prim("header.stamp.nanosec", "I", 4),
           "frame_id": r.string("header.frame_id"),
           "height": r.prim("height", "I", 4),
           "width": r.prim("width", "I", 4),
           "encoding": r.string("encoding"),
           "is_bigendian": r.prim("is_bigendian", "B", 1),
           "step": r.prim("step", "I", 4)}
    if r.peek_u32x2() == MARKER:
        out["data_form"] = "descriptor"
        r.prim("data.marker1", "I", 4)
        r.prim("data.marker2", "I", 4)
        out["backend_type"] = r.string("data.backend_type")
        out["descriptor"] = descriptor(r, "data.descriptor.")
    else:
        out["data_form"] = "legacy_sequence"
        n = r.prim("data.length", "I", 4)
        out["data"] = r.octets("data.bytes", n).hex()
    return out


def decode(path, kind):
    data = open(path, "rb").read()
    r = Reader(data)
    val = descriptor(r) if kind == "descriptor" else image(r)
    if r.p != len(data):
        raise ValueError(f"{path}: {len(data) - r.p} trailing bytes")
    return {"file": os.path.basename(path), "bytes": len(data), "value": val,
            "fields": r.fields}


def decode_wire_hit(hexstr):
    """Decode a captured marker-path payload (marker first, origin unknown).

    The CDR alignment origin is wherever the sample's body began, so both
    phases modulo 8 are tried; the one that yields a NUL-terminated backend
    name and a cuda_vmm_ socket name is kept. Padding bytes are not checked:
    rmw_fastrtps_cpp leaves earlier buffer contents in them.
    """
    data = bytes.fromhex(hexstr)
    for phase in (0, 4):
        r = Reader.__new__(Reader)
        r.d, r.base, r.p, r.fields = data, -phase, 0, []
        try:
            m = (r.prim("marker1", "I", 4), r.prim("marker2", "I", 4))
            backend = r.string("backend_type")
            desc = descriptor(r)
        except (ValueError, struct.error, UnicodeDecodeError):
            continue
        if m == MARKER and desc["vmm_socket_path"].startswith("cuda_vmm_"):
            return {"phase": phase, "backend_type": backend, "descriptor": desc}
    return None


def main():
    cap, out = sys.argv[1], sys.argv[2]
    probe = json.load(open(os.path.join(cap, "probe.json")))
    errors = []
    res = {}
    for name in sorted(os.listdir(cap)):
        if not name.endswith(".cdr"):
            continue
        kind = "descriptor" if name.startswith("descriptor") else "image"
        res[name] = decode(os.path.join(cap, name), kind)
        key = name[:-4].replace(".", "_")
        if probe.get(key) != open(os.path.join(cap, name), "rb").read().hex():
            errors.append(f"{name}: bytes differ from probe.json[{key}]")

    fixed = res["descriptor_fixed.xcdr1.cdr"]["value"]
    want = {"size": 48, "element_type_name": "h", "device_id": 0, "use_ipc": True,
            "vmm_pid": 4242, "vmm_block_id": 7, "vmm_block_size": 2097152,
            "vmm_socket_path": "cuda_vmm_4242_7", "ipc_uid": 0x0123456789ABCDEF,
            "ipc_event_handle": bytes(range(64)).hex()}
    if fixed != want:
        errors.append(f"fixed descriptor decodes to {fixed}")

    live = res["image_cuda_descriptor.xcdr1.cdr"]["value"]
    if live.get("data_form") != "descriptor" or live.get("backend_type") != "cuda":
        errors.append("live image is not on the descriptor path")
    if live.get("descriptor") != probe["live_descriptor"]:
        errors.append(f'live descriptor {live.get("descriptor")} != probe {probe["live_descriptor"]}')
    marker_field = [f for f in res["image_cuda_descriptor.xcdr1.cdr"]["fields"]
                    if f["field"] == "data.marker1"][0]
    if marker_field["offset"] != probe["marker_offset_xcdr1"]:
        errors.append("marker offset disagrees with the probe")
    for a, b in (("image_cpu_legacy.xcdr1.cdr", "image_cuda_plain.xcdr1.cdr"),
                 ("image_cuda_descriptor.xcdr1.cdr", "image_cuda_descriptor.ddscdr.cdr"),
                 ("descriptor_fixed.xcdr1.cdr", "descriptor_fixed.ddscdr.cdr")):
        if open(os.path.join(cap, a), "rb").read() != open(os.path.join(cap, b), "rb").read():
            errors.append(f"{a} and {b} differ")
    leg = res["image_cpu_legacy.xcdr1.cdr"]["value"]
    if leg["data_form"] != "legacy_sequence" or leg["data"] != "ab" * 48:
        errors.append("legacy image payload wrong")

    json.dump({"decoded": res, "errors": errors}, open(out, "w"), indent=2)
    print(json.dumps({"files": len(res), "errors": errors}))
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
