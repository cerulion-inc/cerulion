// SPDX-License-Identifier: AGPL-3.0-only
//! `gpu.cuda`: the NVIDIA driver, devices and CUDA runtime this host exposes,
//! found from files alone (no CUDA library is loaded, no tool is run).
//!
//! Reads `/proc/driver/nvidia/version` (the loaded kernel driver), the
//! `/dev/nvidia<N>` device nodes, `/etc/nv_tegra_release` (Jetson L4T),
//! `/usr/local/cuda/version.json` or `version.txt` (the CUDA toolkit) and
//! whether `/etc/ld.so.cache` names `libcuda.so.1`, the driver library a CUDA
//! program resolves through the loader.

// The decision and the readers compile and are tested on every host; only a
// Linux build calls them from `run`.
#![cfg_attr(not(target_os = "linux"), allow(dead_code))]

use std::path::Path;

use super::{Check, Context, Outcome, Reading, Status};

pub(super) const CUDA: Check = Check {
    id: "gpu.cuda",
    why: "a node that uses the GPU finds the driver through the loader and the device nodes; \
          a driver that is loaded but missing from the loader cache fails at the node's first \
          CUDA call.",
    network: false,
    run,
};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub(super) struct GpuReading {
    /// The kernel module version from `/proc/driver/nvidia/version`.
    pub(super) driver: Option<String>,
    /// `/dev/nvidia<N>` nodes.
    pub(super) devices: usize,
    /// The L4T release from `/etc/nv_tegra_release` (`R35.4.1`).
    pub(super) tegra: Option<String>,
    /// The CUDA toolkit version string, when a toolkit is installed.
    pub(super) cuda_toolkit: Option<String>,
    /// Whether `/etc/ld.so.cache` names `libcuda.so.1`; `None` when there is
    /// no cache file (musl and NixOS hosts resolve libraries without one).
    pub(super) libcuda_in_cache: Option<bool>,
}

pub(super) fn decide(r: &GpuReading) -> Outcome {
    let mut parts = Vec::new();
    if let Some(t) = &r.tegra {
        parts.push(format!("Jetson L4T {t}"));
    }
    if let Some(d) = &r.driver {
        parts.push(format!("driver {d}"));
    }
    parts.push(format!(
        "{} /dev/nvidia device{}",
        r.devices,
        if r.devices == 1 { "" } else { "s" }
    ));
    parts.push(match &r.cuda_toolkit {
        Some(v) => format!("CUDA toolkit {v}"),
        None => "no CUDA toolkit in /usr/local/cuda".to_string(),
    });
    parts.push(match r.libcuda_in_cache {
        Some(true) => "libcuda.so.1 in the loader cache".to_string(),
        Some(false) => "libcuda.so.1 not in the loader cache".to_string(),
        None => "no /etc/ld.so.cache to read".to_string(),
    });
    let summary = parts.join("; ");
    // A GPU this host can use: device nodes, or a Jetson, whose GPU has no
    // /dev/nvidia<N> node. A driver with neither (a container on a GPU host
    // without GPU passthrough) is reported, not judged.
    let usable_gpu = r.devices > 0 || r.tegra.is_some();
    let o = if r.driver.is_none() && r.tegra.is_none() && r.devices == 0 {
        Outcome::new(Status::Info, "no NVIDIA driver is loaded")
    } else if usable_gpu && r.libcuda_in_cache == Some(false) {
        Outcome::new(Status::Warn, summary).with_fix(if r.tegra.is_some() {
            "run `sudo ldconfig`, or install the L4T CUDA library package \
             (`sudo apt install nvidia-l4t-cuda`)"
        } else {
            "run `sudo ldconfig` after installing the driver, or install the driver's CUDA \
             library package (`libnvidia-compute-<version>` on Ubuntu)"
        })
    } else {
        Outcome::new(Status::Info, summary)
    };
    let mut o = o.reading(
        "devices",
        Reading::Int(u64::try_from(r.devices).unwrap_or(u64::MAX)),
    );
    if let Some(in_cache) = r.libcuda_in_cache {
        o = o.reading("libcuda_in_cache", Reading::Bool(in_cache));
    }
    o
}

/// The kernel module version in the first line of
/// `/proc/driver/nvidia/version`
/// (`NVRM version: NVIDIA UNIX x86_64 Kernel Module  550.54.14  Thu Feb 22 ...`,
/// or `NVRM version: NVIDIA UNIX Open Kernel Module for x86_64  580.95.05 ...`):
/// the first word of digits and dots holding a dot, else the line with its
/// spaces collapsed.
pub(super) fn driver_version(text: &str) -> Option<String> {
    let line = first_line(text)?;
    let words: Vec<&str> = line.split_whitespace().collect();
    Some(
        words
            .iter()
            .find(|w| w.contains('.') && w.bytes().all(|b| b.is_ascii_digit() || b == b'.'))
            .map_or_else(|| words.join(" "), |v| (*v).to_string()),
    )
}

/// The L4T release in `/etc/nv_tegra_release`
/// (`# R35 (release), REVISION: 4.1, GCID: ...`) as `R35.4.1`, else the first
/// line without its `#`.
pub(super) fn tegra_release(text: &str) -> Option<String> {
    let line = first_line(text)?;
    let line = line.trim_start_matches('#').trim();
    let major = line
        .split_whitespace()
        .next()
        .filter(|m| m.starts_with('R'));
    let revision = line
        .split_once("REVISION:")
        .map(|(_, rest)| rest.trim().split(',').next().unwrap_or("").trim());
    Some(match (major, revision) {
        (Some(m), Some(r)) if !r.is_empty() => format!("{m}.{r}"),
        _ => line.to_string(),
    })
}

/// The first non-empty line of `text`, trimmed.
fn first_line(text: &str) -> Option<String> {
    text.lines()
        .map(str::trim)
        .find(|l| !l.is_empty())
        .map(str::to_string)
}

/// The CUDA toolkit version from `version.json` (`{"cuda": {"version":
/// "12.2.140"}}`) or the older `version.txt` (`CUDA Version 10.2.89`).
pub(super) fn cuda_version(json: Option<&str>, txt: Option<&str>) -> Option<String> {
    if let Some(v) = json
        .and_then(|j| serde_json::from_str::<serde_json::Value>(j).ok())
        .and_then(|v| v["cuda"]["version"].as_str().map(str::to_string))
    {
        return Some(v);
    }
    txt.and_then(|t| {
        t.trim()
            .strip_prefix("CUDA Version ")
            .map(|v| v.trim().to_string())
    })
}

/// Whether the loader cache's bytes name `libcuda.so.1`.
pub(super) fn cache_names_libcuda(cache: &[u8]) -> bool {
    const NAME: &[u8] = b"libcuda.so.1";
    cache.windows(NAME.len()).any(|w| w == NAME)
}

/// `/dev/nvidia<N>` entries in `dev`.
pub(super) fn count_devices(dev: &Path) -> usize {
    std::fs::read_dir(dev).map_or(0, |entries| {
        entries
            .filter_map(Result::ok)
            .filter(|e| {
                e.file_name()
                    .to_str()
                    .and_then(|n| n.strip_prefix("nvidia"))
                    .is_some_and(|rest| {
                        !rest.is_empty() && rest.bytes().all(|b| b.is_ascii_digit())
                    })
            })
            .count()
    })
}

#[cfg(target_os = "linux")]
fn run(_ctx: &Context) -> Outcome {
    let text = |p: &str| std::fs::read_to_string(p).ok();
    decide(&GpuReading {
        driver: text("/proc/driver/nvidia/version")
            .as_deref()
            .and_then(driver_version),
        devices: count_devices(Path::new("/dev")),
        tegra: text("/etc/nv_tegra_release")
            .as_deref()
            .and_then(tegra_release),
        cuda_toolkit: cuda_version(
            text("/usr/local/cuda/version.json").as_deref(),
            text("/usr/local/cuda/version.txt").as_deref(),
        ),
        libcuda_in_cache: std::fs::read("/etc/ld.so.cache")
            .ok()
            .map(|c| cache_names_libcuda(&c)),
    })
}

#[cfg(not(target_os = "linux"))]
fn run(_ctx: &Context) -> Outcome {
    Outcome::not_applicable("NVIDIA drivers and CUDA are read from Linux driver files")
}

#[cfg(test)]
mod tests {
    use super::*;

    const NVRM: &str = "NVRM version: NVIDIA UNIX x86_64 Kernel Module  550.54.14  Thu Feb 22 \
                        01:44:30 UTC 2024\nGCC version:  gcc version 12.3.0\n";
    const TEGRA: &str = "# R35 (release), REVISION: 4.1, GCID: 33958178, BOARD: t186ref, EABI: \
                         aarch64, DATE: Tue Aug  1 19:57:35 UTC 2023\n";

    fn desktop(devices: usize, in_cache: Option<bool>) -> GpuReading {
        GpuReading {
            driver: driver_version(NVRM),
            devices,
            tegra: None,
            cuda_toolkit: Some("12.4.1".to_string()),
            libcuda_in_cache: in_cache,
        }
    }

    #[test]
    fn the_readers_take_the_version_tokens_from_real_lines() {
        assert_eq!(driver_version(NVRM).as_deref(), Some("550.54.14"));
        assert_eq!(
            driver_version(
                "NVRM version: NVIDIA UNIX Open Kernel Module for x86_64  580.95.05  Release \
                 Build  (dvs-builder@U16-I3-B03-4-3)  Tue Sep 23 09:55:41 UTC 2025\n"
            )
            .as_deref(),
            Some("580.95.05")
        );
        assert_eq!(
            driver_version("odd line  here\n").as_deref(),
            Some("odd line here")
        );
        assert_eq!(tegra_release(TEGRA).as_deref(), Some("R35.4.1"));
        assert_eq!(
            tegra_release("# something else\n").as_deref(),
            Some("something else")
        );
        assert_eq!(driver_version("\n"), None);
    }

    #[test]
    fn no_driver_is_info() {
        let o = decide(&GpuReading::default());
        assert_eq!(o.status, Status::Info);
        assert_eq!(o.summary, "no NVIDIA driver is loaded");
    }

    #[test]
    fn a_driver_with_libcuda_in_the_cache_is_info() {
        let o = decide(&desktop(1, Some(true)));
        assert_eq!(o.status, Status::Info);
        assert_eq!(
            o.summary,
            "driver 550.54.14; 1 /dev/nvidia device; CUDA toolkit 12.4.1; libcuda.so.1 in the \
             loader cache"
        );
        assert_eq!(o.fix, None);
    }

    #[test]
    fn a_usable_gpu_missing_from_the_loader_cache_warns() {
        let o = decide(&desktop(2, Some(false)));
        assert_eq!(o.status, Status::Warn);
        assert!(o.fix.unwrap().contains("libnvidia-compute-<version>"));
        let jetson = decide(&GpuReading {
            tegra: tegra_release(TEGRA),
            libcuda_in_cache: Some(false),
            ..GpuReading::default()
        });
        assert_eq!(jetson.status, Status::Warn);
        assert!(
            jetson
                .summary
                .starts_with("Jetson L4T R35.4.1; 0 /dev/nvidia devices"),
            "{}",
            jetson.summary
        );
        assert!(jetson.fix.unwrap().contains("nvidia-l4t-cuda"));
    }

    #[test]
    fn a_driver_without_device_nodes_or_a_cache_file_is_reported_not_judged() {
        // A container on a GPU host without passthrough sees the driver only.
        assert_eq!(decide(&desktop(0, Some(false))).status, Status::Info);
        // A host with no /etc/ld.so.cache (musl, NixOS).
        let o = decide(&desktop(1, None));
        assert_eq!(o.status, Status::Info);
        assert!(
            o.summary.ends_with("no /etc/ld.so.cache to read"),
            "{}",
            o.summary
        );
        assert_eq!(o.readings.get("libcuda_in_cache"), None);
    }

    #[test]
    fn cuda_version_reads_json_then_txt() {
        assert_eq!(
            cuda_version(
                Some(r#"{"cuda": {"name": "CUDA SDK", "version": "12.2.140"}}"#),
                None
            ),
            Some("12.2.140".to_string())
        );
        assert_eq!(
            cuda_version(Some("not json"), Some("CUDA Version 10.2.89\n")),
            Some("10.2.89".to_string())
        );
        assert_eq!(cuda_version(None, None), None);
    }

    #[test]
    fn the_cache_scan_finds_the_exact_soname() {
        assert!(cache_names_libcuda(b"\0libfoo.so\0libcuda.so.1\0"));
        assert!(!cache_names_libcuda(b"\0libcudart.so.12\0libcuda.so\0"));
    }

    #[test]
    fn count_devices_counts_numbered_nodes_only() {
        let dev = tempfile::tempdir().unwrap();
        for n in ["nvidia0", "nvidia1", "nvidiactl", "nvidia-uvm", "nvidia"] {
            std::fs::write(dev.path().join(n), "").unwrap();
        }
        assert_eq!(count_devices(dev.path()), 2);
    }
}
