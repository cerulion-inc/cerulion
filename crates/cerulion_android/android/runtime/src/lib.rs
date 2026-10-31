// SPDX-License-Identifier: AGPL-3.0-only
//! The sample app's runtime library. Java loads it with
//! `CerulionRuntime.loadLibrary("cerulion_sample_runtime")`, which registers
//! the graph node types listed here before any runtime call.

cerulion_android::embed_nodes! {
    "timer" => timer::TimerNodeEntry,
    "relay" => relay::RelayNodeEntry,
    "printer" => printer::PrinterNodeEntry,
}
