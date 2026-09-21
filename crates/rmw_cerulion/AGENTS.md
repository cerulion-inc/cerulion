# rmw_cerulion - agent notes

Runs ROS 2/MoveIt 2 unmodified on zero-copy SHM (cdylib + rlib; tests call the C ABI in-process;
only `rclpy_xproc_test` proves the `.so`). `cerulion ros2 run/launch` and `ros2:` graph entries
stage `RMW_IMPLEMENTATION` for the user.

## Invariants

- Wrap every `extern "C"` entry point in `ffi::ffi_guard`: a panic across the C boundary aborts
  the process.
- `#[repr(C)]` on every seam type; codecs zero padding byte-ranges (wire determinism).
- Allocate and free on the SAME side with the SAME allocator: rcl-owned out-params via the
  caller's `rcutils_allocator_t`; bridge/rosidl containers via raw libc `malloc`/`free` (rosidl
  `fini()` frees them).
- No `Box<dyn>`/`Arc<dyn>` across the cdylib seam - dispatch via `AnyBridge`.
- Lock entity mutexes with `runtime::lock_unpoisoned` (poison wedges the entity - torn SHM
  bookkeeping risks use-after-free); raw-pointer `unsafe impl Send` newtypes are sound only under
  their discipline.
- Reject hostile/over-bound counts BEFORE any resize/alloc in a codec.
- Failure paths: unconditional counters + flood-latched logs + decade re-announcements (the log is
  a ROS user's only window).
- ROS names are Cerulion names VERBATIM (`/chatter` ⇔ `/chatter/data`; relative refused, no alias);
  every publisher registers for egress (best-effort); no unregister.
- Windowed borrows: NEVER link `cerulion_heaphook` (a 2nd malloc interposer); dlsym per-symbol
  (`src/heaphook.rs`). Only the FIRST outstanding borrow on a thread owns its window; a loan
  finished on the WRONG thread copies + HOLDS its slot until that thread borrows again
  (destroy/poisoned LEAKS such slots AND the whole `PublisherData`: ports registered, slot held,
  no disconnect edge for the process life, else the dead-node sweep wedges); quarantine retires at
  slot REUSE, never at publish. Every degrade = the latched copy path, never a failed publish.

## Testing

Default `-- --test-threads=1` (SHM singleton, global registries, the one `#[traced_test]`
subscriber slot - a suite installing it needs its own binary); transport-free bridge/codec suites
run parallel; the two `*_linux_test` binaries (real `LD_PRELOAD`) and `rclpy_xproc_test` (Jazzy
container) are `--ignored`. Per-binary map - what each pins, why serial, its fixture prereqs - is
the Test map in docs/internals/rmw.md; update THAT when a binary is added, not a list here.

## Gotchas

- build.rs bindings: bindgen over ROS headers (`AMENT_PREFIX_PATH`/`CERULION_RMW_SYS_INCLUDE`),
  else vendored (headerless warns; a SET prefix var with no usable headers ERRORS); `RMW_RET_*`
  consts shadow bindgen's. Era gate: `wrapper.h` includes, `cerulion_has_*` cfgs + the distro claim
  derive from the SELECTED bindings; `era_check.rs` FAILS the build on a claim/fingerprint
  contradiction or the reserved `vendored-dev` marker; init exports refuse a mismatch first.
- `install_tracing()` installs the stderr subscriber (`rmw_cerulion=warn,cerulion_core=warn`); never a 2nd.
- Element-body framing has NO wire version signal: publisher + desk walker deploy TOGETHER; a skew
  degrades nested arrays to opaque text, never a wrong decode.
- A forged take's shadow aims container headers INTO a held SHM sample - un-forge before
  release/`fini`. Only compiled C++: `shim/cppstring_shim.cpp`.
- `rmw_deserialize` has no coverage: pin when touched.
- Adopt-take is NOT armed below a service borrow ceiling of 2 (one unit for the held message, one
  for the receive): below it a reusing caller wedges - the release freeing its borrow runs inside
  the take that borrow blocks. Copying take instead.
- Accepted residual: an adopted take racing the FINAL `rmw_shutdown` can return a sample whose
  release callback is gone - at most ONE borrow + pool slot per such take, shutdown-only, bounded
  (later takes degrade to copies). No check inside the take closes it, only moves it; rcl stops
  executors before `rmw_shutdown`. Pin: `the_lost_registration_window_is_shutdown_only_and_bounded`.

Deep reference: docs/internals/rmw.md - read before FFI, bridge/codec or latch work.
