// SPDX-License-Identifier: AGPL-3.0-only
//! The Metal backend, over `Metal.framework` and the Objective-C runtime,
//! both opened at run time.
//!
//! Apple silicon has one DRAM for CPU and GPU, so Metal only does UNIFIED
//! residence: `newBufferWithBytesNoCopy:length:options:deallocator:` wraps a
//! whole host mapping in one shared-storage `MTLBuffer`, and a message's
//! device address is that buffer plus the message's offset into it. The
//! wrap costs microseconds and copies nothing.
//!
//! The calls go through `objc_msgSend` cast to each method's exact
//! signature, which is how the Objective-C ABI is called from C.

use std::ffi::{c_char, c_void, CStr};

use crate::backend::{
    BackendKind, BufferHandle, Caps, DeviceAddress, DeviceBackend, DeviceUuid, Fence,
    SegmentDevice, Stream,
};
use crate::error::{DeviceError, DeviceResult};
use crate::loader::Library;
use crate::segment::SegmentSpan;

type Id = *mut c_void;
type Sel = *mut c_void;

const METAL: &str = "/System/Library/Frameworks/Metal.framework/Metal";
const OBJC: &str = "/usr/lib/libobjc.A.dylib";
/// Loaded only for its side effect: a command-line process gets no system
/// default Metal device unless CoreGraphics is loaded.
const CORE_GRAPHICS: &str = "/System/Library/Frameworks/CoreGraphics.framework/CoreGraphics";

/// `MTLResourceStorageModeShared`: CPU and GPU see the same memory.
const STORAGE_MODE_SHARED: usize = 0;
/// `MTLCommandBufferStatusCompleted`.
const STATUS_COMPLETED: usize = 4;

/// The selectors this backend sends, registered once at probe time.
#[derive(Debug, Clone, Copy)]
struct Selectors {
    name: Sel,
    utf8_string: Sel,
    registry_id: Sel,
    has_unified_memory: Sel,
    new_command_queue: Sel,
    new_buffer_no_copy: Sel,
    new_buffer_with_length: Sel,
    new_buffer_with_bytes: Sel,
    contents: Sel,
    command_buffer: Sel,
    blit_command_encoder: Sel,
    copy_buffer: Sel,
    end_encoding: Sel,
    commit: Sel,
    wait_until_completed: Sel,
    status: Sel,
    release: Sel,
}

/// The system default Metal device and one command queue on it.
#[derive(Debug)]
pub struct Metal {
    msg_send: usize,
    pool_push: unsafe extern "C" fn() -> *mut c_void,
    pool_pop: unsafe extern "C" fn(*mut c_void),
    sel: Selectors,
    device: usize,
    queue: usize,
    caps: Caps,
    // Kept open for the life of the backend: the pointers above point in.
    _libs: [Library; 2],
    _core_graphics: Option<Library>,
}

// SAFETY: MTLDevice and MTLCommandQueue are documented thread-safe; the
// handles are retained for the life of the backend and only messaged.
unsafe impl Send for Metal {}
// SAFETY: as for Send; no method mutates backend state.
unsafe impl Sync for Metal {}

/// Cast `objc_msgSend` to a method's exact C signature.
macro_rules! send {
    ($self:expr, $obj:expr, $sel:ident, fn($($arg:ty),*) -> $ret:ty $(, $val:expr)*) => {{
        // SAFETY: `msg_send` is objc_msgSend, and the caller names the
        // receiving method's exact argument and return types.
        let f: unsafe extern "C" fn(Id, Sel $(, $arg)*) -> $ret =
            unsafe { std::mem::transmute::<usize, _>($self.msg_send) };
        unsafe { f($obj, $self.sel.$sel $(, $val)*) }
    }};
}

impl Metal {
    /// Open Metal and the Objective-C runtime and take the system default
    /// device.
    pub fn probe() -> DeviceResult<Self> {
        let metal = Library::open(BackendKind::Metal, &[METAL])?;
        let objc = Library::open(BackendKind::Metal, &[OBJC])?;
        let core_graphics = Library::open(BackendKind::Metal, &[CORE_GRAPHICS])
            .inspect_err(|error| {
                tracing::debug!(%error, "CoreGraphics did not load; the default device may be absent");
            })
            .ok();
        // SAFETY: each type is the C declaration of the symbol it reads.
        let (create_default, register, msg_send, pool_push, pool_pop) = unsafe {
            (
                metal.get::<unsafe extern "C" fn() -> Id>("MTLCreateSystemDefaultDevice")?,
                objc.get::<unsafe extern "C" fn(*const c_char) -> Sel>("sel_registerName")?,
                objc.get::<unsafe extern "C" fn()>("objc_msgSend")?,
                objc.get::<unsafe extern "C" fn() -> *mut c_void>("objc_autoreleasePoolPush")?,
                objc.get::<unsafe extern "C" fn(*mut c_void)>("objc_autoreleasePoolPop")?,
            )
        };
        // SAFETY: sel_registerName takes a NUL-terminated name.
        let s = |name: &CStr| unsafe { register(name.as_ptr()) };
        let sel = Selectors {
            name: s(c"name"),
            utf8_string: s(c"UTF8String"),
            registry_id: s(c"registryID"),
            has_unified_memory: s(c"hasUnifiedMemory"),
            new_command_queue: s(c"newCommandQueue"),
            new_buffer_no_copy: s(c"newBufferWithBytesNoCopy:length:options:deallocator:"),
            new_buffer_with_length: s(c"newBufferWithLength:options:"),
            new_buffer_with_bytes: s(c"newBufferWithBytes:length:options:"),
            contents: s(c"contents"),
            command_buffer: s(c"commandBuffer"),
            blit_command_encoder: s(c"blitCommandEncoder"),
            copy_buffer: s(c"copyFromBuffer:sourceOffset:toBuffer:destinationOffset:size:"),
            end_encoding: s(c"endEncoding"),
            commit: s(c"commit"),
            wait_until_completed: s(c"waitUntilCompleted"),
            status: s(c"status"),
            release: s(c"release"),
        };
        // SAFETY: MTLCreateSystemDefaultDevice returns a retained device or nil.
        let device = unsafe { create_default() };
        if device.is_null() {
            return Err(DeviceError::NoDevice {
                backend: BackendKind::Metal,
                detail: "MTLCreateSystemDefaultDevice returned nil".to_string(),
            });
        }
        let mut this = Self {
            msg_send: msg_send as usize,
            pool_push,
            pool_pop,
            sel,
            device: device as usize,
            queue: 0,
            caps: Caps {
                kind: BackendKind::Metal,
                name: String::new(),
                uuid: DeviceUuid::default(),
                integrated: false,
                host_register: false,
                read_only_register: false,
                host_pointer_is_device_pointer: false,
                posix_fd_export: false,
                ipc_event: false,
                mps: None,
            },
            _libs: [metal, objc],
            _core_graphics: core_graphics,
        };
        let queue = send!(this, device, new_command_queue, fn() -> Id);
        if queue.is_null() {
            return Err(DeviceError::NoDevice {
                backend: BackendKind::Metal,
                detail: "newCommandQueue returned nil".to_string(),
            });
        }
        this.queue = queue as usize;
        this.caps.name = {
            let _pool = this.pool();
            let name = send!(this, device, name, fn() -> Id);
            let text = send!(this, name, utf8_string, fn() -> *const c_char);
            if text.is_null() {
                String::new()
            } else {
                // SAFETY: UTF8String returns a NUL-terminated string that
                // lives until the enclosing autorelease pool drains.
                unsafe { CStr::from_ptr(text) }
                    .to_string_lossy()
                    .into_owned()
            }
        };
        let registry_id = send!(this, device, registry_id, fn() -> u64);
        this.caps.uuid.0[..8].copy_from_slice(&registry_id.to_le_bytes());
        let unified = send!(this, device, has_unified_memory, fn() -> i8) != 0;
        this.caps.integrated = unified;
        this.caps.host_register = unified;
        this.caps.read_only_register = unified;
        tracing::info!(
            backend = "metal",
            device = %this.caps.name,
            unified,
            "probed device"
        );
        Ok(this)
    }

    fn pool(&self) -> Pool<'_> {
        // SAFETY: objc_autoreleasePoolPush has no preconditions.
        Pool {
            metal: self,
            token: unsafe { (self.pool_push)() },
        }
    }

    /// Encode one blit copy, commit it and wait for it.
    fn blit(
        &self,
        src: Id,
        src_offset: usize,
        dst: Id,
        dst_offset: usize,
        n: usize,
    ) -> DeviceResult<()> {
        let _pool = self.pool();
        let cb = send!(self, self.queue as Id, command_buffer, fn() -> Id);
        let enc = send!(self, cb, blit_command_encoder, fn() -> Id);
        if cb.is_null() || enc.is_null() {
            return Err(self.failed("commandBuffer/blitCommandEncoder", 0));
        }
        send!(
            self,
            enc,
            copy_buffer,
            fn(Id, usize, Id, usize, usize) -> (),
            src,
            src_offset,
            dst,
            dst_offset,
            n
        );
        send!(self, enc, end_encoding, fn() -> ());
        send!(self, cb, commit, fn() -> ());
        send!(self, cb, wait_until_completed, fn() -> ());
        let status = send!(self, cb, status, fn() -> usize);
        if status != STATUS_COMPLETED {
            return Err(self.failed("MTLCommandBuffer", status as i64));
        }
        Ok(())
    }

    fn failed(&self, call: &'static str, code: i64) -> DeviceError {
        DeviceError::Driver {
            backend: BackendKind::Metal,
            call,
            code,
            message: if code == 0 {
                "returned nil".to_string()
            } else {
                "command buffer did not complete".to_string()
            },
        }
    }

    fn scratch(&self, n: usize) -> DeviceResult<Owned<'_>> {
        let buf = send!(
            self,
            self.device as Id,
            new_buffer_with_length,
            fn(usize, usize) -> Id,
            n,
            STORAGE_MODE_SHARED
        );
        if buf.is_null() {
            return Err(self.failed("newBufferWithLength:options:", 0));
        }
        Ok(Owned {
            metal: self,
            obj: buf,
        })
    }
}

/// An autorelease pool scope.
struct Pool<'a> {
    metal: &'a Metal,
    token: *mut c_void,
}

impl Drop for Pool<'_> {
    fn drop(&mut self) {
        // SAFETY: pops the pool pushed when this scope was made.
        unsafe { (self.metal.pool_pop)(self.token) }
    }
}

/// A retained object released on drop.
struct Owned<'a> {
    metal: &'a Metal,
    obj: Id,
}

impl Drop for Owned<'_> {
    fn drop(&mut self) {
        send!(self.metal, self.obj, release, fn() -> ());
    }
}

/// Metal's blit encoder on macOS needs offsets and sizes that are
/// multiples of 4.
fn blit_aligned(offset: usize, n: usize) -> DeviceResult<()> {
    if offset.is_multiple_of(4) && n.is_multiple_of(4) {
        Ok(())
    } else {
        Err(DeviceError::Unsupported {
            backend: BackendKind::Metal,
            what: "a blit copy whose offset or length is not a multiple of 4",
        })
    }
}

fn buffer(address: DeviceAddress) -> DeviceResult<(Id, usize)> {
    match address {
        DeviceAddress::Buffer { buffer, offset } => Ok((buffer.as_ptr(), offset)),
        DeviceAddress::Flat(_) => Err(DeviceError::Unsupported {
            backend: BackendKind::Metal,
            what: "a flat device address",
        }),
    }
}

impl DeviceBackend for Metal {
    fn caps(&self) -> &Caps {
        &self.caps
    }

    unsafe fn register_segment(&self, span: &SegmentSpan) -> DeviceResult<SegmentDevice> {
        if !self.caps.host_register {
            return Err(DeviceError::Unsupported {
                backend: BackendKind::Metal,
                what: "wrapping host memory (the device has no unified memory)",
            });
        }
        // The deallocator is nil: the transport owns the mapping, and the
        // buffer is released before the mapping goes away.
        let buf = send!(
            self,
            self.device as Id,
            new_buffer_no_copy,
            fn(*mut c_void, usize, usize, Id) -> Id,
            span.base as *mut c_void,
            span.len,
            STORAGE_MODE_SHARED,
            std::ptr::null_mut()
        );
        if buf.is_null() {
            return Err(self.failed("newBufferWithBytesNoCopy:length:options:deallocator:", 0));
        }
        Ok(SegmentDevice::Buffer(BufferHandle::from_ptr(buf)))
    }

    unsafe fn unregister_segment(
        &self,
        _span: &SegmentSpan,
        device: SegmentDevice,
    ) -> DeviceResult<()> {
        match device {
            SegmentDevice::Buffer(buf) => {
                send!(self, buf.as_ptr(), release, fn() -> ());
                Ok(())
            }
            SegmentDevice::Flat { .. } => Err(DeviceError::Unsupported {
                backend: BackendKind::Metal,
                what: "a flat device address",
            }),
        }
    }

    unsafe fn copy_to_host(&self, src: DeviceAddress, dst: &mut [u8]) -> DeviceResult<()> {
        let (src, offset) = buffer(src)?;
        if dst.is_empty() {
            return Ok(());
        }
        blit_aligned(offset, dst.len())?;
        let scratch = self.scratch(dst.len())?;
        self.blit(src, offset, scratch.obj, 0, dst.len())?;
        let contents = send!(self, scratch.obj, contents, fn() -> *mut c_void);
        // SAFETY: a shared-storage buffer of dst.len() bytes, written by the
        // completed blit.
        unsafe {
            std::ptr::copy_nonoverlapping(contents.cast::<u8>(), dst.as_mut_ptr(), dst.len())
        };
        Ok(())
    }

    unsafe fn copy_from_host(&self, dst: DeviceAddress, src: &[u8]) -> DeviceResult<()> {
        let (dst, offset) = buffer(dst)?;
        if src.is_empty() {
            return Ok(());
        }
        blit_aligned(offset, src.len())?;
        let staged = send!(
            self,
            self.device as Id,
            new_buffer_with_bytes,
            fn(*const c_void, usize, usize) -> Id,
            src.as_ptr().cast(),
            src.len(),
            STORAGE_MODE_SHARED
        );
        if staged.is_null() {
            return Err(self.failed("newBufferWithBytes:length:options:", 0));
        }
        let staged = Owned {
            metal: self,
            obj: staged,
        };
        self.blit(staged.obj, 0, dst, offset, src.len())
    }

    /// Metal has no stream this backend can synchronize: the caller waits on
    /// its own command buffer, then records on [`Stream::DEFAULT`]. Any
    /// other stream is refused, so a fence never claims work it did not see.
    fn fence_record(&self, stream: Stream) -> DeviceResult<Fence> {
        if stream != Stream::DEFAULT {
            return Err(DeviceError::Unsupported {
                backend: BackendKind::Metal,
                what: "a fence on a caller stream (wait on the command buffer, then record on the default stream)",
            });
        }
        Ok(Fence::CpuComplete)
    }

    fn fence_wait(&self, fence: &Fence, _stream: Stream) -> DeviceResult<()> {
        match fence {
            Fence::CpuComplete => Ok(()),
        }
    }
}

impl Drop for Metal {
    fn drop(&mut self) {
        if self.queue != 0 {
            send!(self, self.queue as Id, release, fn() -> ());
        }
        send!(self, self.device as Id, release, fn() -> ());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn blits_refuse_offsets_and_lengths_off_a_four_byte_grid() {
        assert!(blit_aligned(0, 4).is_ok());
        assert!(blit_aligned(256, 4096).is_ok());
        assert!(blit_aligned(2, 4).is_err());
        assert!(blit_aligned(0, 6).is_err());
    }
}
