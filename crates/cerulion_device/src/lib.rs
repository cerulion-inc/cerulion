// SPDX-License-Identifier: AGPL-3.0-only
//! GPU residence for Cerulion message payloads.
//!
//! A payload is *resident* when a GPU can address it where it already lies.
//! This crate defines one [`DeviceBackend`] trait and the [`HostShm`]
//! backend: host shared memory as its own device, which needs no GPU and
//! runs everywhere.
//!
//! # UNIFIED residence
//!
//! On a device that shares DRAM with the CPU (Jetson, Apple silicon, AMD
//! APUs) the transport's shared-memory segment itself becomes device
//! memory. [`SegmentRegistry::attach`] registers the mapping that holds a
//! slot, once per mapping; [`SegmentRegistry::device_address`] then turns
//! any slot address into the address the device uses, without allocating.
//! A subscriber's mapping is read-only, and `device_address_mut` and
//! `copy_from_host` refuse a read-only segment.
//!
//! Which residence a topic gets is decided by the caller from the
//! probed [`Caps`]; this crate only reports what a device can do.
#![cfg_attr(not(test), deny(clippy::print_stdout, clippy::print_stderr))]

mod backend;
mod error;
mod host_shm;
mod segment;

pub use backend::{
    Access, BackendKind, BufferHandle, Caps, DeviceAddress, DeviceBackend, DeviceUuid, Fence,
    SegmentDevice, Stream,
};
pub use error::{DeviceError, DeviceResult};
pub use host_shm::HostShm;
pub use segment::{parse_maps_line, span_of, SegmentRegistry, SegmentSpan};
