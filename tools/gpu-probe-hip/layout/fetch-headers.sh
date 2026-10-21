#!/usr/bin/env bash
# Fetch the HIP header trees for ROCm 6.2.0 and 7.0.0 into $1 (a fresh directory),
# build dump.c and dump2.c against each, and print the results that layout/rocm-*.txt hold.
# Needs curl and a C compiler; no ROCm install.
set -euo pipefail
OUT=${1:?usage: fetch-headers.sh <fresh dir>}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
for tag in rocm-6.2.0 rocm-7.0.0; do
  I="$OUT/$tag/include/hip"; mkdir -p "$I/amd_detail"
  for f in hip_runtime_api.h hip_common.h driver_types.h texture_types.h surface_types.h channel_descriptor.h hiprtc.h linker_types.h; do
    curl -sfL -o "$I/$f" "https://raw.githubusercontent.com/ROCm/HIP/$tag/include/hip/$f" || rm -f "$I/$f"
  done
  for f in host_defines.h amd_hip_runtime_pt_api.h hip_prof_str.h amd_channel_descriptor.h amd_hip_vector_types.h hip_api_trace.hpp; do
    curl -sfL -o "$I/amd_detail/$f" "https://raw.githubusercontent.com/ROCm/clr/$tag/hipamd/include/hip/amd_detail/$f" || rm -f "$I/amd_detail/$f"
  done
  # hip_version.h is generated at ROCm build time; the dumped values do not depend on it.
  printf '#pragma once\n#define HIP_VERSION_MAJOR 0\n#define HIP_VERSION_MINOR 0\n#define HIP_VERSION_PATCH 0\n#define HIP_VERSION 0\n' > "$I/hip_version.h"
  for p in dump dump2; do cc -D__HIP_PLATFORM_AMD__ -I"$OUT/$tag/include" -o "$OUT/$p-$tag" "$HERE/$p.c"; done
  { "$OUT/dump-$tag"; "$OUT/dump2-$tag"; } > "$OUT/$tag.txt"
  if diff -q "$OUT/$tag.txt" "$HERE/$tag.txt" >/dev/null; then echo "$tag: matches layout/$tag.txt"; else echo "$tag: DIFFERS from layout/$tag.txt"; diff "$OUT/$tag.txt" "$HERE/$tag.txt" || true; fi
done
