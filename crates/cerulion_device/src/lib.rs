// SPDX-License-Identifier: AGPL-3.0-only
//! GPU residence for Cerulion message payloads.
//!
//! A payload is *resident* when a GPU can address it where it already lies.
//! This crate defines one [`DeviceBackend`] trait and four backends:
//!
//! | Backend | Library (opened at run time) | Status |
//! |---|---|---|
//! | [`HostShm`] | none | always available; the oracle the others are tested against |
//! | [`Cuda`] | `libcuda.so.1` | run on Jetson Orin NX |
//! | [`Hip`] | `libamdhip64.so` | built, not run: no AMD device has run it |
//! | [`Metal`] | `Metal.framework` | run on Apple silicon |
//!
//! No vendor library is linked. A machine without one gets an error value
//! from that backend's `probe`, so one binary serves every machine and a
//! build needs no vendor SDK.
//!
//! # UNIFIED residence
//!
//! On a device that shares DRAM with the CPU (Jetson, Apple silicon, AMD
//! APUs) the transport's shared-memory segment itself becomes device
//! memory. [`SegmentRegistry::attach`] registers the mapping that holds a
//! slot, once per mapping; [`SegmentRegistry::device_address`] then turns
//! any slot address into the address the device uses, without allocating.
//! A subscriber's mapping is read-only: CUDA and HIP register it with their
//! read-only flag, and on every backend `device_address_mut` and
//! `copy_from_host` refuse a read-only segment.
//!
//! Which residence a topic gets is decided by the caller from the
//! probed [`Caps`]; this crate only reports what a device can do.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod backend;
mod cuda;
mod error;
mod hip;
mod host_shm;
mod loader;
mod metal;
mod segment;

pub use backend::{
    Access, BackendKind, BufferHandle, Caps, DeviceAddress, DeviceBackend, DeviceUuid, Fence,
    SegmentDevice, Stream,
};
pub use cuda::Cuda;
pub use error::{DeviceError, DeviceResult};
pub use hip::Hip;
pub use host_shm::HostShm;
pub use metal::Metal;
pub use segment::{parse_maps_line, span_of, SegmentRegistry, SegmentSpan};
