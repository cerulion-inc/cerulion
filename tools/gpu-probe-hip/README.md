# gpu-probe-hip: S0 GPU IPC proof on AMD (MI300X, HIP/ROCm)

This is scratch code for slice S0 of the design doc "native GPU residence", on AMD. It is the HIP twin of `tools/gpu-probe` on branch `spike/gpu-s0`, the CUDA probe that ran on Jetson Orin NX, and it keeps that probe's modes, its JSON-line `raw.log`, and its `SUMMARY.md` and table shapes. It is not a member of the root workspace.

Two processes on one host share an iceoryx2 0.10 `[u8]` service. Every frame starts with a 256-byte descriptor at the next 256-byte boundary inside the slot. That is the in-slot pad (`GPU_S0_ALIGN=pad` in the CUDA probe). iceoryx2 `payload_alignment(256)` is never used, because in 0.10 it pads every `[u8]` element 256 times.

HIP is opened with dlopen at run time, so the build needs no ROCm SDK. The UNIFIED register-and-translate path goes through `cerulion_device::Hip` and `SegmentRegistry`. Streams, events, IPC, VMM, external memory, modules and hiprtc go through the probe's own table in `src/hipapi.rs`. The kernels are compiled at startup with hiprtc and cached in `target/kcache`. On a machine without ROCm, every GPU subcommand prints `HIP library not found`, lists each library name it tried, and exits 3.

**Status: built and unit tested on a macOS machine and on an x86 test box. It has not run on AMD hardware.** The results section below is empty until the MI300X run.

`crates/cerulion_device` in this branch is a snapshot of `~/cerulion/worktrees/gpu-core-s3/crates/cerulion_device` (uncommitted work there), copied at 2026-10-21T19:19:05Z on top of `origin/main` `08dc1b15`. It is added to the root `members` so that its `version.workspace` keys resolve.

## Reproduce

From the desk, once the droplet's IP is known (root login, fresh ROCm Ubuntu image):

```sh
cd ~/cerulion/worktrees/gpu-s0-hip/tools/gpu-probe-hip && ./ship.sh <droplet-ip>
```

`ship.sh` rsyncs this worktree (without `target/`, `.git` or results) to `/root/cerulion-hip/`. It starts `bootstrap.sh` detached, so an ssh drop does not stop it. It tails `/root/cerulion-hip/bootstrap.out` every 60 s until `BOOTSTRAP DONE` or `BOOTSTRAP FAILED`, then pulls `results/` and the bootstrap logs back into `./results/`. Its exit code is 0 only on DONE. The stages can also be run one at a time: `./ship.sh <ip> check|push|start|watch|pull`.

On the droplet itself the whole run is one command:

```sh
bash /root/cerulion-hip/tools/gpu-probe-hip/bootstrap.sh
```

`bootstrap.sh` runs four stages (`STAGES`, default `setup build smoke campaign`):

1. **setup.** Installs only missing packages (build-essential, pkg-config, git, rsync, python3, curl), installs rustup into `$HOME` (`--profile minimal`, stable, at least 1.95) if no new-enough rustc is present, then checks ROCm: `/dev/kfd`, `rocminfo`, `rocm-smi`, and `libamdhip64`, `libhiprtc` and `libhsa-runtime64` under `/opt/rocm/lib`. It prints the GPU and the partition modes. If ROCm has no unversioned `libamdhip64.so` (cerulion_device tries `.so.6`, `.so.5` and `.so`), it adds a symlink in `target/libshim` and puts that directory on `LD_LIBRARY_PATH`. Nothing under `/opt/rocm` changes.
2. **build.** `cargo build --release`, the 22 unit tests, the 6 summarize tests, then `gpu-probe-hip probe`.
3. **smoke.** `SMOKE=1 run.sh`: the probe, the floor, and every mode at 64 KiB for 200 frames. Any mismatch, or a missing `cpu` or `copy` row, fails the smoke and stops before the campaign (`FORCE_CAMPAIGN=1` overrides this). A mode whose smoke unit hits its timeout is left out of the campaign, and the log says so.
4. **campaign.** `run.sh` with `REPS` (default 3) writes `results/<UTC stamp>/{env.txt,raw.log,SUMMARY.md}` and prints the path.

Every stage is idempotent. `RESUME=results/<stamp>` continues an interrupted campaign, and units already done are skipped. Logs go to `bootstrap-logs/`. Other entry points:

- `DRY_CPU=1 ./run.sh` runs only the cpu rows. It needs no GPU and calibrates the timing loop.
- `./run.sh` runs the campaign by itself.
- `target/release/gpu-probe-hip writer <mode> [sizes|all] [iters]` runs one mode.

## What is measured

Latency is one-way CLOCK_MONOTONIC time. t0 is taken in the writer before the producing kernel or copy. t1 is taken in the reader once the verify result is on the host. The reader acks every frame, so the run is ping-pong. Sizes are 4 KiB to 64 MiB in steps of 4x (8 sizes). Each size runs 100 warmup + 2000 frames up to 1 MiB, 30 + 500 up to 16 MiB, and 20 + 300 at 64 MiB, the same plan as the CUDA probe.

The fill and verify kernels use the CUDA probe's `pat()` word pattern, so the two probes' results are comparable. The verify kernel counts words that differ from the seeded pattern; this is the mismatch count, and it must be 0. Its reduction goes through shared memory for a fixed 256-thread block, so it does not depend on the wavefront width (64 on MI300X). In the UNIFIED GPU-reader modes the reader also re-checks every 64th frame with a zero-copy CPU read. In the cpuread modes it checks every frame.

| mode | path |
|---|---|
| copy | today's GPU path: fill kernel, D2H into the slot, send, H2D, verify kernel |
| unified-event / unified-cpusync | the iceoryx2 segment is `hipHostRegister`ed in both processes. The fill kernel writes the slot itself. A HIP interprocess event in the descriptor gates the reader (`hipStreamWaitEvent`); in cpusync the writer syncs its stream before send |
| device-ipcmem-event / -cpusync | legacy `hipIpcGetMemHandle` on a `hipMalloc` block; the reader runs `hipIpcOpenMemHandle` once per size |
| device-vmm-event / -cpusync | `hipMemCreate` with POSIX fd handles, exported with `hipMemExportToShareableHandle`, sent once per size over SCM_RIGHTS, then imported (`hipMemImportFromShareableHandle`), reserved, mapped and access-set. Only the descriptor travels |
| device-dmabuf-event | `hipMalloc` block exported with `hsa_amd_portable_export_dmabuf`, sent over SCM_RIGHTS, imported as external memory (`hipImportExternalMemory`, opaque fd); if that fails, through the VMM import |
| copy-cpuread / unified-cpuread | GPU producer, CPU consumer (the CPU verify is timed) |
| cpu | today's CPU path: memcpy into the slot, CPU verify |
| floor-* | one process, no transport: fill and verify on `hipMalloc` memory, on a registered memfd, and on `hipHostMalloc` memory |

UNIFIED registration tries three paths and records the one that worked (`reg_path`) together with the errors of the paths before it (`reg_tried`):

1. `cerulion_device::SegmentRegistry`. It registers with Portable|Mapped, plus `hipHostRegisterReadOnly` for the reader's read-only mapping.
2. A direct `hipHostRegister` with Portable|Mapped.
3. Reader only: a second, writable `MAP_SHARED` mapping of the same `/dev/shm` file, registered in place of the read-only one.

Each row also records the segment size, the time in us and in us per MiB, the flags, and whether the device pointer equals the host pointer.

The writer adds the GPU clock to every latency row: `gpu_mhz_mid` is read from amdgpu `pp_dpm_sclk` at mid-run, and `smi_sclk_post` is rocm-smi's sclk line after the row. `env.txt` records the ROCm version, the amdgpu module, the kernel command line (IOMMU), the GPU, the gfx arch, the compute and memory partition modes, the perf level and the clocks. The `probe` row dumps 26 `hipDeviceGetAttribute` values, the `hipDeviceProp_tR0600` fields the design depends on (`hostRegisterSupported`, `hostRegisterReadOnlySupported`, `canUseHostPointerForRegisteredMem`, `ipcEventSupported`, `gcnArchName`) and the cerulion_device `Caps`. It then tries, in one process, an IPC event handle, an IPC mem handle, a VMM create and export, and a dma-buf export.

Header values: every flag, enum value and struct layout in `src/hipapi.rs` was printed by `layout/dump.c` and `layout/dump2.c` compiled against the ROCm 6.2.0 and 7.0.0 HIP headers. The two releases agree (`layout/rocm-*.txt`), and a unit test checks the constants against both files. `layout/fetch-headers.sh <dir>` reproduces the dumps.

## Expected runtime

**Activate the credit and expect results in about 25 minutes.** Allow 45 minutes before suspecting a hang. Every unit has a 600 s timeout (180 s in the smoke).

| stage | estimate | calibrated on |
|---|---|---|
| setup | 1 to 3 min | rustup-init (minimal, stable 1.97.0) took 22.8 s on an x86 test box with an empty RUSTUP_HOME. The apt step only runs if packages are missing; not measured |
| build + tests | about 1 min | A cold release build with an empty CARGO_HOME (download + 78 crates, 20 jobs) took 10.9 s on an x86 test box (Core Ultra 9 285K); bootstrap's build stage took 10 s. Allowing 3x for droplet vCPUs gives about 35 s |
| smoke | about 2 min | 13 units, each two process starts with HIP init and 300 frames at 64 KiB. HIP init time on MI300X is not measured; 5 to 10 s per unit assumed |
| campaign, 3 reps | about 19 min | see below |

Campaign per rep, about 6.2 min:

- **Frame time, 122 s.** For the GPU modes this is frames x mean latency per mode from the CUDA S0 Orin campaign (`spike/gpu-s0`, `results/20261021T070858Z`, per rep): copy 19.0 s, copy-cpuread 27.3, unified-cpuread 19.9, unified-cpusync 6.7, unified-event 5.3, floor 5.4. The Orin device-cpusync (5.8 s) stands in for each of the two cpusync device modes, and device-event (5.0 s) for each of the three event device modes. cpu is the 11.4 s unit wall measured on an x86 test box (`calibration/an x86 test box-dry-cpu-20261021T192602Z`, median of 3).
- **Untimed work, x1.4: 170 s.** On an x86 test box the cpu unit's wall time was 1.38 times its frames x mean, the extra being the untimed pattern fill and fixed costs.
- **Fixed cost per unit, about 17 s x 12 units: 204 s.** This is two HIP inits plus, per size, service setup, 130 ms of handshake sleeps and one rocm-smi call (about 1.5 s assumed).

The Orin numbers are a conservative stand-in. The MI300X is a discrete GPU on PCIe, so the UNIFIED modes read and write host memory over the link; no number here comes from AMD hardware. `row_wall_ms` on every row and the `WALL` line per unit record the real times, and SUMMARY.md totals them.

## Risks for the MI300X run

1. **Read-only registration.** `hip_runtime_api.h` gives `hipHostRegisterReadOnly` and `hipExtHostRegisterCoarseGrained` the same value (0x8), and documents ReadOnly as "ignored on AMD devices". clr 6.2 stores the flags without reading them, and hard-codes `hostRegisterReadOnlySupported = 0`. Whether KFD can pin the reader's `r--s` iceoryx2 mapping is unknown. The reader falls back to a direct registration, then to a writable alias of the same file. If only the alias works, the AMD reader cannot keep its mapping read-only.
2. **cerulion_device reports `read_only_register = true` on HIP.** It derives this from `hipDeviceAttributeHostRegisterSupported`, which clr hard-codes to 1, while the property clr exposes for read-only registration is 0. The probe row shows both values.
3. **VMM import argument.** ROCm 6.2 through 7.0.2 (clr `rocdevice.cpp`) read the fd as `*(int*)osHandle`. clr `develop` reads `(int)(uintptr_t)osHandle`, the CUDA convention. A wrong guess either fails or faults. The probe passes one pointer that reads as the fd under both conventions (a page mapped at a 4 GiB-aligned base plus the fd, holding the fd); it is unit tested.
4. **VMM and dma-buf need the kernel driver's support.** The VMM handle is a dma-buf fd from `hsa_amd_vmem_export_shareable_handle`, and the dma-buf mode uses `hsa_amd_portable_export_dmabuf`. Both need an amdgpu/KFD with dma-buf export. An older or inbox kernel driver fails here; the rows say which call failed.
5. **Legacy IPC mem handles.** `hipIpcOpenMemHandle` accepts only `hipIpcMemLazyEnablePeerAccess`, so the probe passes it. Small `hipMalloc` blocks may be suballocated (clr carries an offset in the handle). Partition modes (CPX/NPS4) split one MI300X into several logical devices; IPC between them is not tested here. env.txt records the mode.
6. **Interprocess events.** clr implements them with a shared-memory signal array that it registers with `hipHostRegister`, so they depend on host registration working at all.
7. **IOMMU.** A non-passthrough IOMMU adds translation to every PCIe access to registered host memory, which slows the UNIFIED modes. env.txt records `/proc/cmdline` and the IOMMU groups.
8. **Stale GPU caches.** A GPU can keep host memory lines in L2 across kernels. The reader's verify kernel reads slots that the other process's GPU work rewrites. A stale read shows as mismatches, which is what the verify is for.
9. **The library name on ROCm 7.** ROCm 7 ships `libamdhip64.so.7`, and cerulion_device does not try `.so.7`. bootstrap adds a shim symlink when the unversioned name is missing. The probe's own table also tries `.so.7` and `/opt/rocm/lib`.
10. **hiprtc.** hiprtc needs comgr; it is in the runtime image, and bootstrap checks for `libhiprtc`. Without an `--offload-arch`, hiprtc compiles for the current device (`hiprtcInternal.cpp`). `GPU_S0_HIPRTC_OPTS` adds options.
11. **VM telemetry.** On a virtual machine, amdgpu sysfs (`pp_dpm_sclk`) or rocm-smi clocks may be missing. The clock fields are then 0 or n/a, and the latency rows are unaffected.
12. **Dead-node cleanup.** After a killed unit, run.sh removes dead iceoryx2 nodes (`GPU_S0_CLEANUP=1`, which bootstrap sets), because a 64 MiB topic's segment is about 320 MiB of `/dev/shm`. That cleans every dead node on the machine, so ship.sh users on a shared box must set `GPU_S0_CLEANUP=0`.

## Verified before the run

- **desk (macOS, M3 Max).** `cargo build --release` and `cargo test --release`. The probe exits 3 with the library list when no ROCm is present.
- **an x86 test box (Linux x86_64, RTX 5090, no ROCm).** `cargo build --release` with no warnings, and 22 of 22 unit tests: descriptor layout and offsets, pad math, pattern, iteration plan, JSON row escaping, the dual fd handle, `pp_dpm_sclk` and maps parsing, struct layouts and constants against both header dumps, symbol tables, and the missing-library message. cerulion_device's lib tests (13) and conformance suite (15) pass. Every GPU subcommand exits 3 with `HIP library not found` and the five names tried, plus cerulion_device's three. `DRY_CPU=1 REPS=3 ./run.sh` ran clean: 35,640 frames, 0 mismatches. `SMOKE=1 ./run.sh` fails its gate with exit 1 when HIP is absent. `ship.sh` ran end to end against an x86 test box (push, detached start, watch, pull; `STAGES="build smoke"`, cleanup off), and bootstrap stopped at the smoke stage with `BOOTSTRAP FAILED`, as it should without a GPU.

## Results

Not run yet. To be filled from `results/<stamp>/SUMMARY.md` after the MI300X run: GPU producer to GPU consumer (p50 / p99 us per size, every mode), GPU to CPU, CPU to CPU, the in-process floor by memory kind, speedup over the copy path, mismatches, one-time costs (registration us per MiB, VMM create/export/import, IPC mem get/open, dma-buf export/import, IPC event open), CPU read speed on registered pages, device attributes, and skips and failures with their reasons.
