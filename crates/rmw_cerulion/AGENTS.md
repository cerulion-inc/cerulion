# rmw_cerulion - agent notes

ROS 2/MoveIt 2 on zero-copy SHM (cdylib + rlib); C ABI tests run in-process, only
`rclpy_xproc_test` proves `.so`. CLI `ros2 run/launch` and graph `ros2:` stage `RMW_IMPLEMENTATION`.

## Invariants

- Every `extern "C"` uses `ffi::ffi_guard`; an escaping panic aborts the host.
- `#[repr(C)]` on every seam type; codecs zero padding byte-ranges.
- Allocate and free on the SAME side with the SAME allocator: rcl out-params via the caller's `rcutils_allocator_t`, bridge/rosidl containers via libc `malloc`/`free`.
- No `Box<dyn>`/`Arc<dyn>` across the cdylib seam - dispatch via `AnyBridge`.
- Lock entity mutexes with `runtime::lock_unpoisoned` (poison wedges the entity - torn SHM bookkeeping risks use-after-free); `unsafe impl Send` newtypes hold under theirs.
- Reject hostile/over-bound counts BEFORE any resize/alloc in a codec.
- NEVER write a destination the build cannot fill: takes and `rmw_deserialize` read the nested census first (`rmw_schema_mismatch_test`).
- Failure paths: unconditional counters + flood-latched logs + decade re-announcements.
- ROS names are verbatim (`/chatter` ⇔ `/chatter/data`), relative refused, no alias. Eligible publishers register best-effort, never unregister. Claim excluded topic/service-family guards before ports; retain for process lifetime. Ingress admits subscriptions only; only egress admits publishers.
- Windowed borrows: NEVER link `cerulion_heaphook` (2nd malloc interposer); dlsym per-symbol (`src/heaphook.rs`). Only a thread's FIRST borrow owns its window; a loan finished on the WRONG thread copies + HOLDS its slot till that thread borrows again (destroy/poisoned LEAKS slot + `PublisherData`: freeing it wedges the dead-node sweep); quarantine retires at slot REUSE, never publish. Every degrade = the copy path, never a failed publish.
- NEVER export an entry point whose types a distro's headers lack: cfg the `extern "C"` fn out WHOLE, never stub it (`rmw_absent_export_table_test`, lane `nm` audit).
- NEVER strip `ROS_DISTRO` around a fixture: a Kilted-and-later `.so` refuses it (`every_ros_distro_strip_is_declared`).

## Testing

Default `-- --test-threads=1`: SHM singleton, global registries, one `#[traced_test]` slot (each installer needs its own binary). Transport-free bridge/codec suites are parallel. The two `*_linux_test` binaries (real `LD_PRELOAD`) and `rclpy_xproc_test` (Jazzy) use `--ignored`. Add binary contracts/serial reasons/prereqs to the Test map in docs/internals/rmw.md, not here.

## Gotchas

- build.rs bindings: bindgen over ROS headers (`AMENT_PREFIX_PATH`/`CERULION_RMW_SYS_INCLUDE`), else vendored (headerless warns; a SET prefix var with no usable headers ERRORS); `RMW_RET_*` consts shadow bindgen's. Era gate: `wrapper.h` includes, `cerulion_has_*` cfgs + the claim derive from the SELECTED bindings; `era_check.rs` FAILS the build on a claim/fingerprint contradiction or the reserved marker; init exports refuse first.
- `install_tracing()` installs the stderr subscriber (`rmw_cerulion=warn,cerulion_core=warn`), never a 2nd.
- Element-body framing has NO wire version signal: publisher + desk walker deploy TOGETHER; a skew degrades nested arrays to opaque text, never a wrong decode.
- A forged take's shadow aims container headers INTO a held SHM sample - un-forge before release/`fini`. Only compiled C++: `shim/cppstring_shim.cpp`.
- `rmw_deserialize`: only its refusal arm is pinned.
- Adopt-take is NOT armed below a service borrow ceiling of 2 (one held, one received): below it a reusing caller wedges - the release freeing its borrow runs inside the take that borrow blocks. Copying take instead.
- Accepted residual: an adopted take racing the FINAL `rmw_shutdown` can return a sample whose release callback is gone - ONE borrow + pool slot per such take, shutdown-only, bounded (later takes copy). No check closes it, only moves it; rcl stops executors before `rmw_shutdown`. Pin: `the_lost_registration_window_is_shutdown_only_and_bounded`.

Deep reference: docs/internals/rmw.md, read before FFI, bridge/codec or latch work.
